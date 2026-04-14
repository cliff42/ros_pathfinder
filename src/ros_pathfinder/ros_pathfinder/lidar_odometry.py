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
    # TODO: start with wheel odom as initial state

    def __init__(self):
        super().__init__('lidar_odometry')
        self.lidar_odom_publisher = self.create_publisher(Odometry, 'lidar_odom', 10)
        self.scan_subscriber = self.create_subscription(LaserScan,'scan', self.scan_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.odom_callback)
        self.prev_points_tree = None # TODO: use this as target points in ICP algorithm
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0

    def scan_callback(self, msg: LaserScan):
        try:
            odom_to_laser_tf = self.tf_buffer.lookup_transform(
                'odom',                  
                msg.header.frame_id, # laser
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('could not look up odom->laser transform')
            return
        
        points = self.scan_to_points(msg)
        if self.prev_points_tree is None:
            self.prev_points_tree = KDTree(points, leaf_size=2)
            return
        
        # TODO: find corresponding pts via nearest neighbour
        matched_source_pts, matched_target_pts = self.get_matches(points, self.prev_points)
        
        # 1. calculate centroids
        source_centroid = self.get_centroid(matched_source_pts)
        target_centroid = self.get_centroid(matched_target_pts)

        # 2. center target and source scans
        source_centered = matched_source_pts - source_centroid
        target_centered = matched_target_pts - target_centroid

        # 3. create covariance matrix
        M = np.dot(source_centered.T, target_centered)

        # 4. SVD to get rotation
        U,W,V_t = np.linalg.svd(M)
        R = np.dot(V_t.T,U.T)
        t = target_centroid - source_centroid
        
  
        dtheta = math.atan2(R[1, 0], R[0, 0])
        dx = float(t[0])
        dy = float(t[1])

        c = math.cos(self.theta)
        s = math.sin(self.theta)

        self.x += c * dx - s * dy
        self.y += s * dx + c * dy
        self.theta += dtheta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = HEADER_FRAME
        tf.child_frame_id = CHILD_FRAME
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = math.sin(self.theta / 2.0)
        tf.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = HEADER_FRAME
        odom.child_frame_id = CHILD_FRAME

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = tf.transform.rotation

        # TODO: add later (do we need this?)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        self.lidar_odom_publisher.publish(odom)

        # reset previous tree
        self.prev_points_tree = KDTree(points, leaf_size=2)
            

    def scan_to_points(self, scan_msg):
        points = []
        angle = scan_msg.min_angle

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
    
    def get_matches(self, source_pts):
        target_pts = []
        for pt in source_pts:
            _, nearest_idx = self.prev_points_tree.query(pt, k=1) 
            target_pts.append(self.prev_points_tree.data[nearest_idx])
        return source_pts, target_pts

def main(args=None):
    try:
        with rclpy.init(args=args):
            odom_publisher = LidarOdometry()
            rclpy.spin(odom_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

