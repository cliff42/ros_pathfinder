import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_interfaces.action import FollowPath
import math
import threading
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            self.execute_callback)
        
        self.create_subscription(Odometry, 'slam_odom', self._odom_cb, 10, callback_group=self.cb_group)
        self.twist_publisher = self.create_publisher(Twist,'cmd_vel',10)
        self.pose_lock = threading.Lock()
        self.goal_lock = threading.Lock()
        self.goal_active = False
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None

        self.LINEAR_VEL = 0.12
        self.GOAL_TOL = 0.08
        self.LOOKAHEAD_DIST = 0.20
        self.ANGULAR_GAIN = 1.5
        self.MAX_ANGULAR_VEL = 0.7

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        with self.pose_lock:
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y
            self.odom_yaw = yaw

    def execute_callback(self,goal_handle):
        path = goal_handle.request.path.poses
        result = FollowPath.Result()

        with self.goal_lock:
            if self.goal_active:
                result.success = False
                result.message = 'another path is already active'
                goal_handle.abort()
                self.get_logger().warn('rejected follow_path goal: another path is active')
                return result
            self.goal_active = True

        try:
            if not path:
                result.success = False
                result.message = 'empty path'
                goal_handle.abort()
                self.publish_stop()
                return result

            while rclpy.ok() and self.current_pose() is None:
                self.get_logger().warn('waiting for slam_odom before following path')
                time.sleep(0.05)

            start_pose = self.current_pose()
            first = path[0].pose.position
            final = path[-1].pose.position
            if start_pose is not None:
                x, y, yaw = start_pose
                first_heading = math.atan2(first.y - y, first.x - x)
                final_heading = math.atan2(final.y - y, final.x - x)
                self.get_logger().info(
                    f'accepted path: poses={len(path)}, '
                    f'current=({x:.3f}, {y:.3f}, yaw={yaw:.3f}), '
                    f'first=({first.x:.3f}, {first.y:.3f}), '
                    f'final=({final.x:.3f}, {final.y:.3f}), '
                    f'first_heading={first_heading:.3f}, '
                    f'final_heading={final_heading:.3f}'
                )

            target_idx = 0
            rate_sec = 0.05

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.publish_stop()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'path following canceled'
                    return result

                pose = self.current_pose()
                if pose is None:
                    time.sleep(rate_sec)
                    continue

                x, y, yaw = pose
                final_pose = path[-1].pose.position
                distance_to_goal = math.hypot(final_pose.x - x, final_pose.y - y)

                if distance_to_goal <= self.GOAL_TOL:
                    self.publish_stop()
                    goal_handle.succeed()
                    result.success = True
                    result.message = 'reached goal'
                    return result

                target_idx = self.select_target_index(path, target_idx, x, y)
                target = path[target_idx].pose.position

                dx = target.x - x
                dy = target.y - y
                target_distance = math.hypot(dx, dy)
                target_heading = math.atan2(dy, dx)
                heading_error = self.wrap_angle(target_heading - yaw)

                angular_velocity = self.clamp(
                    self.ANGULAR_GAIN * heading_error,
                    -self.MAX_ANGULAR_VEL,
                    self.MAX_ANGULAR_VEL
                )

                if abs(heading_error) > math.pi / 3.0:
                    linear_velocity = 0.0
                else:
                    linear_velocity = self.LINEAR_VEL * max(0.25, math.cos(heading_error))

                msg = Twist()
                msg.linear.x = linear_velocity
                msg.angular.z = angular_velocity
                self.twist_publisher.publish(msg)

                feedback_msg = FollowPath.Feedback()
                feedback_msg.current_waypoint = target_idx
                feedback_msg.total_waypoints = len(path)
                feedback_msg.distance_to_goal = distance_to_goal
                goal_handle.publish_feedback(feedback_msg)

                self.get_logger().info(
                    f'follow target={target_idx}/{len(path) - 1}, '
                    f'pose=({x:.3f}, {y:.3f}, yaw={yaw:.3f}), '
                    f'target=({target.x:.3f}, {target.y:.3f}), '
                    f'target_heading={target_heading:.3f}, '
                    f'target_dist={target_distance:.3f}, goal_dist={distance_to_goal:.3f}, '
                    f'heading_error={heading_error:.3f}, v={linear_velocity:.3f}, '
                    f'w={angular_velocity:.3f}'
                )

                time.sleep(rate_sec)

            self.publish_stop()
            goal_handle.abort()
            result.success = False
            result.message = 'path following stopped'
            return result
        finally:
            self.finish_goal()

    def finish_goal(self):
        with self.goal_lock:
            self.goal_active = False

    def current_pose(self):
        with self.pose_lock:
            if self.odom_x is None:
                return None
            return self.odom_x, self.odom_y, self.odom_yaw

    def select_target_index(self, path, start_idx, x, y):
        last_idx = len(path) - 1
        idx = min(start_idx, last_idx)

        while idx < last_idx:
            point = path[idx].pose.position
            if math.hypot(point.x - x, point.y - y) >= self.LOOKAHEAD_DIST:
                break
            idx += 1

        return idx

    def publish_stop(self):
        self.twist_publisher.publish(Twist())

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, value, low, high):
        return max(low, min(high, value))

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = PathFollower()
            executor = MultiThreadedExecutor()
            executor.add_node(node)
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
