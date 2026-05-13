import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3
from sensor_msgs.msg import LaserScan
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import numpy as np
from sklearn.neighbors import KDTree

import math

HEADER_FRAME = 'lidar_odom' # static world frame
CHILD_FRAME = 'base_link' # robot frame

class LidarOdometry(Node):

    def __init__(self):
        super().__init__('lidar_odometry')
        self.lidar_odom_publisher = self.create_publisher(Odometry, 'lidar_odom', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.prev_points_tree = None

        # EKF STATE
        # mu is [x, y, theta]
        self.mu = np.zeros(3)

        # P: 3x3 covariance matrix (uncertainty in mu)
        self.P = np.eye(3) * 0.01

        # process noise Q (estimated error in wheel odom motion model).
        # TODO: finalize these values
        self.Q = np.diag([0.01, 0.01, 0.005])

        # measurement noise R (estimated error in ICP result)
        # TODO: finalize these values
        self.R_meas = np.diag([0.05, 0.05, 0.02])

        self.icp_x = 0.0
        self.icp_y = 0.0
        self.icp_theta = 0.0

        self.last_odom_time = None

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

    # EKF predict step
    def odom_callback(self, msg: Odometry):
        now = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.last_odom_time is None:
            self.last_odom_time = now
            return
        dt = now - self.last_odom_time
        self.last_odom_time = now
        if dt <= 0.0:
            return

        v = msg.twist.twist.linear.x   # linear velocity  (m/s)
        w = msg.twist.twist.angular.z  # angular velocity (rad/s)
        theta = self.mu[2]

        # store latest wheel odom pose for the lidar_odom -> odom correction TF
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.odom_theta = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

        # propagate state with differential-drive motion model
        self.mu[0] += v * math.cos(theta) * dt # robot x pos
        self.mu[1] += v * math.sin(theta) * dt # robot y pos
        self.mu[2] += w * dt # robot rotation (theta)
        self.mu[2] = math.atan2(math.sin(self.mu[2]), math.cos(self.mu[2]))

        # jacobian of motion model wrt state [x, y, theta]
        F = np.array([
            [1.0, 0.0, -v * math.sin(theta) * dt],
            [0.0, 1.0,  v * math.cos(theta) * dt],
            [0.0, 0.0,  1.0]
        ])

        # grow covariance with process (encoder) noise
        self.P = F @ self.P @ F.T + self.Q

    # EKF update/ correction step
    def scan_callback(self, msg: LaserScan):
        points = self.scan_to_points(msg)
        if self.prev_points_tree is None:
            self.prev_points_tree = KDTree(points, leaf_size=2)
            return

        MAX_ITER = 30
        CONVERGENCE_T = 1e-4
        CONVERGENCE_R = 1e-5

        R_total = np.eye(2)
        t_total = np.zeros(2)
        source_pts = points.copy()

        # iteration loop for ICP
        for _ in range(MAX_ITER):
            matched_source_pts, matched_target_pts = self.get_matches(source_pts)
            if matched_source_pts is None:
                break

            # 1. calculate centroids
            source_centroid = self.get_centroid(matched_source_pts)
            target_centroid = self.get_centroid(matched_target_pts)

            # 2. center target and source scans
            source_centered = matched_source_pts - source_centroid
            target_centered = matched_target_pts - target_centroid

            # 3. create covariance matrix
            M = np.dot(source_centered.T, target_centered)

            # 4. SVD to get rotation & translation
            U, W, V_t = np.linalg.svd(M)

            # guard against reflection
            if np.linalg.det(V_t.T @ U.T) < 0:
                V_t[-1, :] *= -1
            R = V_t.T @ U.T
            t = target_centroid - R @ source_centroid

            # accumulate transform
            t_total = R @ t_total + t
            R_total = R @ R_total

            # apply to source for next iteration
            source_pts = (R @ source_pts.T).T + t

            # convergence check
            if np.linalg.norm(t) < CONVERGENCE_T and abs(math.atan2(R[1, 0], R[0, 0])) < CONVERGENCE_R:
                break

        # reset previous tree to current raw scan
        self.prev_points_tree = KDTree(points, leaf_size=2)

        dtheta = math.atan2(R_total[1, 0], R_total[0, 0])
        dx = float(t_total[0])
        dy = float(t_total[1])

        # minimum motion threshold (don't update EFK when stationary)
        MIN_T = 1e-3
        MIN_R = 5e-4
        if abs(dx) > MIN_T or abs(dy) > MIN_T or abs(dtheta) > MIN_R:
            # advance the independent ICP pose accumulator to form measurement z
            c_icp = math.cos(self.icp_theta)
            s_icp = math.sin(self.icp_theta)
            self.icp_x += c_icp * dx - s_icp * dy
            self.icp_y += s_icp * dx + c_icp * dy
            self.icp_theta = math.atan2(
                math.sin(self.icp_theta + dtheta),
                math.cos(self.icp_theta + dtheta)
            )

            # EKF update step
            # ICP measurement: z
            z = np.array([self.icp_x, self.icp_y, self.icp_theta])

            # innovation (wrap angle component)
            innovation = z - self.mu # innovation is diff between ICP measurement and current mu location
            innovation[2] = math.atan2(math.sin(innovation[2]), math.cos(innovation[2]))

            S = self.P + self.R_meas # TODO: when implementing SLAM actually incorperate the observation matrix
            K = self.P @ np.linalg.inv(S) # kalman gain

            self.mu = self.mu + K @ innovation
            self.mu[2] = math.atan2(math.sin(self.mu[2]), math.cos(self.mu[2]))
            self.P = (np.eye(3) - K) @ self.P # shrink uncertainty based on kalman gain

        x, y, theta = self.mu[0], self.mu[1], self.mu[2]

        # per REP-105 (https://www.ros.org/reps/rep-0105.html) we want to publish the correction between lidar_odom and odom
        theta_corr = math.atan2(math.sin(theta - self.odom_theta),
                                math.cos(theta - self.odom_theta))
        c, s = math.cos(theta_corr), math.sin(theta_corr)
        x_corr = x - (c * self.odom_x - s * self.odom_y)
        y_corr = y - (s * self.odom_x + c * self.odom_y)

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = HEADER_FRAME  # lidar_odom
        tf.child_frame_id = 'odom'         # lidar_odom -> odom (correction on top of odom)
        tf.transform.translation.x = x_corr
        tf.transform.translation.y = y_corr
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = math.sin(theta_corr / 2.0)
        tf.transform.rotation.w = math.cos(theta_corr / 2.0)

        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = HEADER_FRAME
        odom.child_frame_id = CHILD_FRAME

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)

        # TODO: add later (do we need this?)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        self.lidar_odom_publisher.publish(odom)
            

    def scan_to_points(self, scan_msg):
        points = []
        angle = scan_msg.angle_min

        for range in scan_msg.ranges:
            if math.isinf(range) or math.isnan(range) or range > scan_msg.range_max or range < scan_msg.range_min:
                angle += scan_msg.angle_increment
                continue
            
            x = range * math.cos(angle)
            y = range * math.sin(angle)
            points.append([x, y])
            angle += scan_msg.angle_increment

        return np.array(points, dtype=float)

    def get_centroid(self, pts):
        point_sum = np.sum(pts, axis=0)
        return point_sum / float(len(pts))
    
    def get_matches(self, source_pts, distance_threshold=0.3):
        matched_src = []
        matched_tgt = []
        for pt in source_pts:
            dist, nearest_idx = self.prev_points_tree.query(pt.reshape(1, -1), k=1)
            if dist[0][0] < distance_threshold:
                matched_src.append(pt)
                matched_tgt.append(self.prev_points_tree.data[nearest_idx[0][0]])
        if len(matched_src) < 3:
            return None, None
        return np.array(matched_src, dtype=float), np.array(matched_tgt, dtype=float)

def main(args=None):
    try:
        with rclpy.init(args=args):
            odom_publisher = LidarOdometry()
            rclpy.spin(odom_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

