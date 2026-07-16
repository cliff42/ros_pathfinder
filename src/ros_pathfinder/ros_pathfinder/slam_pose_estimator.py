import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

import numpy as np
from sklearn.neighbors import KDTree

import math

HEADER_FRAME = 'slam_odom' # corrected odometry frame from encoder prediction + lidar correction
CHILD_FRAME = 'base_link' # robot frame
RAW_ODOM_FRAME = 'raw_odom'


class LidarOdometry(Node):

    def __init__(self):
        super().__init__('lidar_odometry')
        self.slam_odom_publisher = self.create_publisher(Odometry, 'slam_odom', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'raw_odom', self.odom_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.prev_points_tree = None
        self.prev_points = None

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
        self.latest_odom_pose = None
        self.prev_scan_odom_pose = None

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        self.corr_x = 0.0
        self.corr_y = 0.0
        self.corr_theta = 0.0

        # Last commanded/measured odom velocities after deadbanding.
        # Used by scan_callback to reject stationary ICP noise.
        self.last_v = 0.0
        self.last_w = 0.0

    # EKF predict step
    def odom_callback(self, msg: Odometry):
        now = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.last_odom_time is None:
            self.last_odom_time = now
            self.store_latest_odom_pose(msg)
            self.mu = self.latest_odom_pose.copy()
            self.icp_x = float(self.mu[0])
            self.icp_y = float(self.mu[1])
            self.icp_theta = float(self.mu[2])
            self.publish_slam_odom(msg.header.stamp)
            return
        dt = now - self.last_odom_time
        self.last_odom_time = now
        if dt <= 0.0:
            self.store_latest_odom_pose(msg)
            self.publish_slam_odom(msg.header.stamp)
            return

        v = msg.twist.twist.linear.x   # linear velocity  (m/s)
        w = msg.twist.twist.angular.z  # angular velocity (rad/s)

        # Treat tiny encoder/odom jitter as zero so it cannot integrate forever.
        V_DEADBAND = 0.005  # m/s
        W_DEADBAND = 0.005  # rad/s
        if abs(v) < V_DEADBAND:
            v = 0.0
        if abs(w) < W_DEADBAND:
            w = 0.0

        theta = self.mu[2]

        # Store deadbanded velocities for logging and stationary ICP rejection.
        self.last_v = v
        self.last_w = w

        self.store_latest_odom_pose(msg)

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

        # Grow covariance with process (encoder) noise.
        # self.Q is interpreted as noise per second, so scale by dt.
        # While stationary, keep process noise very small so the EKF does not
        # become eager to trust tiny noisy ICP corrections.
        if v == 0.0 and w == 0.0:
            Q = np.diag([1e-6, 1e-6, 1e-7])
        else:
            Q = self.Q * dt
        self.P = F @ self.P @ F.T + Q

        corr_tf = TransformStamped()
        corr_tf.header.stamp = msg.header.stamp
        corr_tf.header.frame_id = HEADER_FRAME
        corr_tf.child_frame_id = RAW_ODOM_FRAME
        corr_tf.transform.translation.x = self.corr_x
        corr_tf.transform.translation.y = self.corr_y
        corr_tf.transform.translation.z = 0.0
        corr_tf.transform.rotation.x = 0.0
        corr_tf.transform.rotation.y = 0.0
        corr_tf.transform.rotation.z = math.sin(self.corr_theta / 2.0)
        corr_tf.transform.rotation.w = math.cos(self.corr_theta / 2.0)
        self.tf_broadcaster.sendTransform(corr_tf)
        self.publish_slam_odom(msg.header.stamp)

    # EKF update/ correction step
    def scan_callback(self, msg: LaserScan):
        points = self.scan_to_points(msg)
        if points.shape[0] < 3:
            return

        if self.prev_points_tree is None:
            self.prev_points = points
            self.prev_points_tree = KDTree(points, leaf_size=10)
            if self.latest_odom_pose is not None:
                self.prev_scan_odom_pose = self.latest_odom_pose.copy()
            return

        MAX_ITER = 10
        CONVERGENCE_T = 1e-4
        CONVERGENCE_R = 1e-5

        odom_delta = self.get_scan_to_scan_odom_delta()
        R_total, t_total = self.pose_to_matrix(odom_delta)
        source_pts = self.transform_points(points, R_total, t_total)

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
            source_pts = self.transform_points(points, R_total, t_total)

            # convergence check
            if np.linalg.norm(t) < CONVERGENCE_T and abs(math.atan2(R[1, 0], R[0, 0])) < CONVERGENCE_R:
                break

        self.prev_points = points
        self.prev_points_tree = KDTree(points, leaf_size=10)
        if self.latest_odom_pose is not None:
            self.prev_scan_odom_pose = self.latest_odom_pose.copy()

        dtheta = math.atan2(R_total[1, 0], R_total[0, 0])
        dx = float(t_total[0])
        dy = float(t_total[1])

        self.get_logger().info(
            f"v={self.last_v:.5f}, w={self.last_w:.5f}, "
            f"icp_dx={dx:.5f}, icp_dy={dy:.5f}, icp_dtheta={dtheta:.5f}, "
            f"odom=({self.odom_x:.3f}, {self.odom_y:.3f}, {self.odom_theta:.3f}), "
            f"mu=({self.mu[0]:.3f}, {self.mu[1]:.3f}, {self.mu[2]:.3f})"
        )

        # If odom says the robot is stationary, do not let scan-to-scan ICP
        # accumulate LaserScan jitter as fake motion. We already updated
        # prev_points above, so the next scan still compares against the most
        # recent scan.
        STILL_V = 0.005  # m/s
        STILL_W = 0.005  # rad/s
        if abs(self.last_v) < STILL_V and abs(self.last_w) < STILL_W:
            self.get_logger().info(
                f"STATIONARY: rejecting ICP dx={dx:.5f}, "
                f"dy={dy:.5f}, dtheta={dtheta:.5f}"
            )
            self.publish_slam_odom(msg.header.stamp)
            return

        # Minimum accepted ICP motion once the robot is actually moving.
        MIN_T = 0.01   # 1 cm
        MIN_R = 0.005  # ~0.29 degrees
        if math.hypot(dx, dy) > MIN_T or abs(dtheta) > MIN_R:
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

            S = self.P + self.R_meas # TODO: when implementing SLAM actually incorporate the observation matrix
            K = self.P @ np.linalg.inv(S) # kalman gain

            self.mu = self.mu + K @ innovation
            self.mu[2] = math.atan2(math.sin(self.mu[2]), math.cos(self.mu[2]))
            self.P = (np.eye(3) - K) @ self.P # shrink uncertainty based on kalman gain

        x, y, theta = self.mu[0], self.mu[1], self.mu[2]

        # Recompute and store the slam_odom->raw_odom correction so odom_callback
        # can re-publish it at 50 Hz to keep the TF buffer fresh.
        theta_corr = math.atan2(math.sin(theta - self.odom_theta),
                                math.cos(theta - self.odom_theta))
        c, s = math.cos(theta_corr), math.sin(theta_corr)
        self.corr_x = x - (c * self.odom_x - s * self.odom_y)
        self.corr_y = y - (s * self.odom_x + c * self.odom_y)
        self.corr_theta = theta_corr

        self.publish_slam_odom(msg.header.stamp)

    def publish_slam_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = HEADER_FRAME
        odom.child_frame_id = CHILD_FRAME

        odom.pose.pose.position.x = float(self.mu[0])
        odom.pose.pose.position.y = float(self.mu[1])
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.mu[2] / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.mu[2] / 2.0)

        odom.twist.twist.linear.x = self.last_v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.last_w

        self.slam_odom_publisher.publish(odom)

    def store_latest_odom_pose(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.odom_theta = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        self.latest_odom_pose = np.array([self.odom_x, self.odom_y, self.odom_theta])

    def get_scan_to_scan_odom_delta(self):
        if self.prev_scan_odom_pose is None or self.latest_odom_pose is None:
            return np.zeros(3)
        return self.relative_pose(self.prev_scan_odom_pose, self.latest_odom_pose)

    # convert global diff into robot local frame
    def relative_pose(self, previous_pose, current_pose):
        dx = current_pose[0] - previous_pose[0]
        dy = current_pose[1] - previous_pose[1]
        theta = previous_pose[2]

        c = math.cos(theta)
        s = math.sin(theta)

        local_dx = c * dx + s * dy
        local_dy = -s * dx + c * dy
        local_dtheta = math.atan2(
            math.sin(current_pose[2] - previous_pose[2]),
            math.cos(current_pose[2] - previous_pose[2])
        )
        return np.array([local_dx, local_dy, local_dtheta])

    def pose_to_matrix(self, pose):
        c = math.cos(pose[2])
        s = math.sin(pose[2])
        R = np.array([[c, -s], [s, c]]) # rot matrix
        t = np.array([pose[0], pose[1]]) # translation vector
        return R, t

    def transform_points(self, points, R, t):
        return (R @ points.T).T + t

    def scan_to_points(self, scan_msg):
        ranges = np.array(scan_msg.ranges, dtype=float)
        angles = (scan_msg.angle_min
                  + np.arange(len(ranges)) * scan_msg.angle_increment)
        valid = (np.isfinite(ranges)
                 & (ranges >= scan_msg.range_min)
                 & (ranges <= scan_msg.range_max))
        r = ranges[valid]
        a = angles[valid]
        if r.size == 0:
            return np.empty((0, 2), dtype=float)
        return np.column_stack((r * np.cos(a), r * np.sin(a)))

    def get_centroid(self, pts):
        return pts.mean(axis=0)

    def get_matches(self, source_pts, distance_threshold=0.3):
        dists, idxs = self.prev_points_tree.query(source_pts, k=1)
        dists = dists[:, 0]
        idxs = idxs[:, 0].astype(int)
        mask = dists < distance_threshold
        if mask.sum() < 3:
            return None, None
        matched_src = source_pts[mask]
        matched_tgt = self.prev_points[idxs[mask]]
        return matched_src, matched_tgt


def main(args=None):
    try:
        with rclpy.init(args=args):
            odom_publisher = LidarOdometry()
            rclpy.spin(odom_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
