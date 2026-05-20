import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_interfaces.action import FollowPath
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self._action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            self.execute_callback)
        
        self.create_subscription(Odometry, 'lidar_odom', self._odom_cb, 10,
                                 callback_group=self.cb_group)
        self.twist_publisher = self.create_publisher(Twist,'cmd_vel',10)
        self.odom_x: float
        self.odom_y: float
        self.odom_yaw: float

        self.LINEAR_VEL = 0.5
        self.GOAL_TOL = 0.05**2
        self.L_DIST = 0.15**2

    def _odom_cb(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = 2*math.atan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

    def execute_callback(self,goal_handle):
        path = goal_handle.request.path

        while rclpy.ok():
            feedback_msg = goal_handle.Feedback()

            #pure pursuit
            min_dist = float('inf')
            best_pose = None
            best_indx = 0
            for i,pose in enumerate(path):
                x = pose.pose.position.x
                y = pose.pose.position.y
                dist = (x-self.odom_x)**2 + (y-self.odom_y)**2
                if dist < min_dist and dist >= self.L_DIST:
                    min_dist = dist
                    best_pose = pose
                    best_indx = i
            
            turning_radius = math.sqrt(min_dist)/(2*math.sin(self.odom_yaw))

            angular_velocity = self.LINEAR_VEL / turning_radius

            msg: Twist
            msg.linear.x = self.LINEAR_VEL
            msg.angular.z = angular_velocity
            self.twist_publisher.publish(msg)

            #publish twist

            goal_x = best_pose.pose.position.X
            goal_y = best_pose.pose.position.Y

            while (goal_x-self.odom_x)**2 + (goal_y-self.odom_y)**2 < self.GOAL_TOL:
                pass

            feedback_msg.current_waypoint = best_indx 
            goal_handle.publish_feedback(feedback_msg)

            if feedback_msg.current_waypoint == feedback_msg.total_waypoints - 1:
                goal_handle.result.success = True
                break