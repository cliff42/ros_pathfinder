import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from visualization_msgs.msg import MarkerArray, Marker

from tf2_ros import Buffer, TransformListener

import heapq
import numpy as np
from collections import deque
import math

class ConversionTest(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.goal = None

        self.marker_publisher = self.create_publisher(Marker,"path",10)
        self.create_subscription(PoseStamped,"goal_pose",self.goal_callback,10)
    def goal_callback(self,pose:PoseStamped):
        indx = self.convertToGrid(pose.pose)
        marker = self.create_marker(indx)
        self.get_logger().info('Actual Location on Map: "%s","%s"' % (pose.pose.position.x,pose.pose.position.y))
        self.get_logger().info('Printed Location on Map: "%s","%s"' % (marker.pose.position.x,marker.pose.position.y))
        self.marker_publisher.publish(marker)
    def convertToGrid(self,pose: Pose):
        y = int((pose.position.y + 10)/0.05)
        x = int((pose.position.x + 10)/0.05)

        indx = x + (y * 400)
        return indx
    def create_marker(self,indx):
        r,c = int(indx/400), indx % 400
        y = (r - 200) * 0.05
        x = (c - 200) * 0.05

        landmark_marker = Marker()
        landmark_marker.header.stamp = self.get_clock().now().to_msg()
        landmark_marker.header.frame_id = 'odom'
        landmark_marker.type = Marker.SPHERE
        landmark_marker.scale.x = 0.1
        landmark_marker.scale.y = 0.1
        landmark_marker.scale.z = 0.1
        landmark_marker.color.r = 1.0
        landmark_marker.color.g = 0.0
        landmark_marker.color.b = 0.0
        landmark_marker.color.a = 1.0
        landmark_marker.id = 1

        landmark_marker.pose.position.x = x
        landmark_marker.pose.position.y = y
        landmark_marker.pose.position.z = 0.0
        landmark_marker.pose.orientation.w = 1.0
        return landmark_marker


def main(args=None):
    try:
        with rclpy.init(args=args):
            path_planner = ConversionTest()
            rclpy.spin(path_planner)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()