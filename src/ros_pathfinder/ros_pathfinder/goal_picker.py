import math
import random

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from action_interfaces.action import FollowPath
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Empty


class GoalPicker(Node):
    def __init__(self):
        super().__init__('goal_picker')

        self.map_topic = self.declare_parameter('map_topic', '/map').value
        self.odom_topic = self.declare_parameter('odom_topic', 'slam_odom').value
        self.trigger_topic = self.declare_parameter('trigger_topic', '/pick_goal').value
        self.forward_trigger_topic = self.declare_parameter(
            'forward_trigger_topic',
            '/go_forward_3m',
        ).value
        self.goal_topic = self.declare_parameter('goal_topic', '/goal_pose').value
        self.forward_action = self.declare_parameter(
            'forward_action',
            'follow_path',
        ).value
        self.min_goal_distance = self.declare_parameter('min_goal_distance', 0.5).value
        self.forward_distance = self.declare_parameter('forward_distance', 3.0).value
        self.forward_path_step = self.declare_parameter('forward_path_step', 0.2).value

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos,
        )
        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.trigger_subscriber = self.create_subscription(
            Empty,
            self.trigger_topic,
            self.pick_goal_callback,
            10,
        )
        self.forward_trigger_subscriber = self.create_subscription(
            Empty,
            self.forward_trigger_topic,
            self.forward_goal_callback,
            10,
        )
        self.goal_publisher = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.forward_path_client = ActionClient(self, FollowPath, self.forward_action)

        self.latest_map = None
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.odom_frame = 'slam_odom'
        self.pending_forward_goal = False
        self.forward_goal_in_flight = False

        self.get_logger().info(
            f'Goal picker ready: publish std_msgs/Empty on {self.trigger_topic} '
            f'to pick a map goal. Publish std_msgs/Empty on '
            f'{self.forward_trigger_topic} to follow a straight '
            f'{self.forward_distance:.1f} m forward path.'
        )

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        if msg.header.frame_id:
            self.odom_frame = msg.header.frame_id
        q = msg.pose.pose.orientation
        self.robot_yaw = self.yaw_from_quaternion(q.x, q.y, q.z, q.w)

        if self.pending_forward_goal:
            self.pending_forward_goal = False
            self.send_forward_path()

    def pick_goal_callback(self, _msg: Empty):
        if self.latest_map is None:
            self.get_logger().warn('cannot pick goal: waiting for map')
            return

        free_indices = self.free_goal_indices(self.latest_map)
        if not free_indices:
            self.get_logger().warn('cannot pick goal: no free cells in map')
            return

        index = random.choice(free_indices)
        x, y = self.grid_index_to_world(self.latest_map, index)
        yaw = self.goal_yaw(x, y)

        goal = self.make_goal(x, y, yaw)

        self.goal_publisher.publish(goal)
        self.get_logger().info(
            f'published picked goal: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}, '
            f'frame={goal.header.frame_id}'
        )

    def forward_goal_callback(self, _msg: Empty):
        if self.robot_x is None:
            self.pending_forward_goal = True
            self.get_logger().warn(
                'go-forward requested before slam_odom arrived; will publish '
                'the forward path after the first odom message'
            )
            return

        self.send_forward_path()

    def send_forward_path(self):
        if self.forward_goal_in_flight:
            self.get_logger().warn('forward path already in flight')
            return

        if not self.forward_path_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(
                f'cannot send forward path: {self.forward_action} action '
                'server is not available'
            )
            return

        path = self.make_forward_path()
        goal = FollowPath.Goal()
        goal.path = path

        self.forward_goal_in_flight = True
        future = self.forward_path_client.send_goal_async(goal)
        future.add_done_callback(self.forward_path_goal_response)

        final_pose = path.poses[-1].pose.position
        first_pose = path.poses[0].pose.position
        self.get_logger().info(
            f'sent straight forward path: distance={self.forward_distance:.3f}, '
            f'waypoints={len(path.poses)}, '
            f'from=({self.robot_x:.3f}, {self.robot_y:.3f}, yaw={self.robot_yaw:.3f}), '
            f'first=({first_pose.x:.3f}, {first_pose.y:.3f}), '
            f'final=({final_pose.x:.3f}, {final_pose.y:.3f}), '
            f'frame={path.header.frame_id}'
        )

    def make_forward_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.odom_frame

        steps = max(1, int(math.ceil(self.forward_distance / self.forward_path_step)))
        for step in range(1, steps + 1):
            distance = min(step * self.forward_path_step, self.forward_distance)
            pose = self.make_pose_stamped(
                self.robot_x + distance * math.cos(self.robot_yaw),
                self.robot_y + distance * math.sin(self.robot_yaw),
                self.robot_yaw,
                self.odom_frame,
            )
            path.poses.append(pose)

        return path

    def forward_path_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f'forward path request failed: {exc}')
            self.forward_goal_in_flight = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn('forward path rejected')
            self.forward_goal_in_flight = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.forward_path_result)

    def forward_path_result(self, future):
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().warn(f'forward path result failed: {exc}')
            self.forward_goal_in_flight = False
            return

        self.get_logger().info(
            f'forward path result: success={result.success}, '
            f'message="{result.message}"'
        )
        self.forward_goal_in_flight = False

    def make_goal(self, x, y, yaw):
        return self.make_pose_stamped(x, y, yaw, self.goal_frame())

    def make_pose_stamped(self, x, y, yaw, frame_id):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = frame_id
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def goal_frame(self):
        if self.latest_map is not None and self.latest_map.header.frame_id:
            return self.latest_map.header.frame_id
        return 'slam_odom'

    def free_goal_indices(self, grid: OccupancyGrid):
        indices = []
        for index, value in enumerate(grid.data):
            if value != 0:
                continue

            x, y = self.grid_index_to_world(grid, index)
            if self.robot_x is not None:
                distance = math.hypot(x - self.robot_x, y - self.robot_y)
                if distance < self.min_goal_distance:
                    continue

            indices.append(index)

        return indices

    def grid_index_to_world(self, grid: OccupancyGrid, index: int):
        width = grid.info.width
        resolution = grid.info.resolution
        column = index % width
        row = index // width

        local_x = (column + 0.5) * resolution
        local_y = (row + 0.5) * resolution

        origin = grid.info.origin
        yaw = self.yaw_from_quaternion(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        c = math.cos(yaw)
        s = math.sin(yaw)

        world_x = origin.position.x + c * local_x - s * local_y
        world_y = origin.position.y + s * local_x + c * local_y
        return world_x, world_y

    def goal_yaw(self, goal_x, goal_y):
        if self.robot_x is None:
            return 0.0
        return math.atan2(goal_y - self.robot_y, goal_x - self.robot_x)

    def yaw_from_quaternion(self, x, y, z, w):
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = GoalPicker()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
