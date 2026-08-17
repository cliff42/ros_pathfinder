import math
from typing import Optional

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from tf2_ros import Buffer, TransformException, TransformListener

from ros_pathfinder.geometry.pose2d import Pose2d
from ros_pathfinder.planning.a_star import AStarPlanner
from ros_pathfinder.planning.costmap import Costmap2d, CostmapConfig
from ros_pathfinder.planning.grid_geometry import GridGeometry2d
from ros_pathfinder.planning.path_simplification import simplify_grid_path
from ros_pathfinder.planning.path_validation import validate_remaining_path


class PathPlannerNode(Node):

    MAP_TOPIC = "map"
    COSTMAP_TOPIC = "global_costmap"
    GOAL_TOPIC = "goal_pose"
    PATH_TOPIC = "path"
    FOLLOW_PATH_ACTION = "follow_path"

    def __init__(self):
        super().__init__("path_planner")

        self._declare_parameters()
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._replan_on_blocked_path = bool(
            self.get_parameter("replan_on_blocked_path").value
        )
        self._blocked_path_confirmations = int(
            self.get_parameter("blocked_path_confirmations").value
        )
        self._replan_cooldown_s = float(
            self.get_parameter("replan_cooldown_s").value
        )
        if not self._base_frame:
            raise ValueError("base_frame cannot be empty")
        if self._blocked_path_confirmations <= 0:
            raise ValueError("blocked_path_confirmations must be positive")
        if self._replan_cooldown_s < 0.0:
            raise ValueError("replan_cooldown_s must be non-negative")
        self._costmap_config = CostmapConfig(
            robot_radius_m=float(
                self.get_parameter("robot_radius_m").value
            ),
            safety_margin_m=float(
                self.get_parameter("safety_margin_m").value
            ),
            occupied_threshold=int(
                self.get_parameter("occupied_threshold").value
            ),
            allow_unknown=bool(self.get_parameter("allow_unknown").value),
            unknown_cost_multiplier=float(
                self.get_parameter("unknown_cost_multiplier").value
            ),
        )
        self._costmap: Optional[Costmap2d] = None
        self._grid_geometry: Optional[GridGeometry2d] = None

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            self.MAP_TOPIC,
            self.map_callback,
            map_qos
        )

        self._costmap_publisher = self.create_publisher(
            OccupancyGrid,
            self.COSTMAP_TOPIC,
            map_qos
        )

        self._planner = AStarPlanner()

        self._transform_buffer = Buffer()
        self._transform_listener = TransformListener(
            self._transform_buffer,
            self,
        )

        self._map_frame: Optional[str] = None
        self._goal: Optional[PoseStamped] = None
        self._plan_requested = False
        self._earliest_replan_ns = 0

        # goal comes from rviz goal pose
        self._goal_subscription = self.create_subscription(
            PoseStamped,
            self.GOAL_TOPIC,
            self.goal_callback,
            10,
        )

        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._path_publisher = self.create_publisher(
            Path,
            self.PATH_TOPIC,
            path_qos,
        )

        self._follow_path_client = ActionClient(
            self,
            FollowPath,
            self.FOLLOW_PATH_ACTION,
        )
        self._pending_follow_path: Optional[Path] = None
        self._pending_follow_generation: Optional[int] = None
        self._follow_goal_handle = None
        self._follow_goal_generation: Optional[int] = None
        self._follow_goal_request_future = None
        self._follow_goal_request_path: Optional[Path] = None
        self._follow_goal_request_generation: Optional[int] = None
        self._follow_cancel_future = None
        self._follow_stop_requested = False
        self._last_action_server_warning_ns = 0
        self._goal_generation = 0
        self._active_follow_path: Optional[Path] = None
        self._current_waypoint_index = 0
        self._blocked_path_observations = 0
        self._last_path_dispatch_ns = 0
        self._follow_dispatch_timer = self.create_timer(
            0.5,
            self._dispatch_pending_path,
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("robot_radius_m", 0.53)
        self.declare_parameter("safety_margin_m", 0.05)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("allow_unknown", False)
        self.declare_parameter("unknown_cost_multiplier", 3.0)
        self.declare_parameter("replan_on_blocked_path", True)
        self.declare_parameter("blocked_path_confirmations", 2)
        self.declare_parameter("replan_cooldown_s", 1.5)

    def map_callback(self, map_msg: OccupancyGrid) -> None:
        expected_size = map_msg.info.width * map_msg.info.height
        if len(map_msg.data) != expected_size:
            self.get_logger().error(
                "map data length does not match its dimensions"
            )
            return

        occupancy_values = np.asarray(
            map_msg.data,
            dtype=np.int16,
        ).reshape(map_msg.info.height, map_msg.info.width)

        try:
            self._costmap = Costmap2d.from_occupancy(
                occupancy_values=occupancy_values,
                resolution_m=map_msg.info.resolution,
                config=self._costmap_config,
            )
            self._grid_geometry = GridGeometry2d(
                width=map_msg.info.width,
                height=map_msg.info.height,
                resolution_m=map_msg.info.resolution,
                origin_in_world=self._pose_from_ros_pose(map_msg.info.origin),
            )
        except ValueError as error:
            self.get_logger().error(f"cannot build costmap: {error}")
            return

        self._map_frame = map_msg.header.frame_id or "map"

        costmap_msg = OccupancyGrid()
        costmap_msg.header.frame_id = map_msg.header.frame_id
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.info = map_msg.info
        costmap_msg.data = (
            self._costmap.values.reshape(-1).tolist()
        )

        self._costmap_publisher.publish(costmap_msg)

        self._check_active_path()
        if self._plan_requested:
            self._try_plan()

    def goal_callback(self, msg: PoseStamped) -> None:
        self._goal_generation += 1
        self._goal = msg
        self._plan_requested = True
        self._earliest_replan_ns = 0
        self._pending_follow_path = None
        self._pending_follow_generation = None
        self._follow_stop_requested = True
        self._active_follow_path = None
        self._current_waypoint_index = 0
        self._blocked_path_observations = 0
        self._cancel_active_following()
        self._try_plan()

    def _try_plan(self) -> None:
        if self.get_clock().now().nanoseconds < self._earliest_replan_ns:
            return
        if (
            self._costmap is None
            or self._grid_geometry is None
            or self._map_frame is None
            or self._goal is None
        ):
            return

        if self._goal.header.frame_id != self._map_frame:
            self.get_logger().warning("goal is not expressed in the map frame")
            self._finish_failed_plan()
            return

        try:
            map_to_base = self._transform_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"cannot obtain planning start pose: {error}"
            )
            return

        start = self._grid_geometry.world_to_cell(
            map_to_base.transform.translation.x,
            map_to_base.transform.translation.y,
        )

        goal = self._grid_geometry.world_to_cell(
            self._goal.pose.position.x,
            self._goal.pose.position.y,
        )

        if start is None:
            self.get_logger().warning("robot pose is outside the costmap")
            self._finish_failed_plan()
            return

        if goal is None:
            self.get_logger().warning("goal is outside the costmap")
            self._finish_failed_plan()
            return

        result = self._planner.plan(
            costmap=self._costmap,
            start=start,
            goal=goal,
        )

        if result is None or not result.path:
            self.get_logger().warning("no path could be found")
            self._finish_failed_plan()
            return

        raw_path = list(result.path)
        try:
            path = simplify_grid_path(self._costmap, raw_path)
        except ValueError as error:
            self.get_logger().error(
                f"planner returned an invalid path: {error}"
            )
            self._finish_failed_plan()
            return

        path_msg = self._publish_path(path)
        self._plan_requested = False
        self._earliest_replan_ns = 0
        self._queue_path_for_following(path_msg)
        self.get_logger().info(
            f"published path with {len(path)} poses "
            f"({len(raw_path)} raw A* cells)"
        )

    def _publish_path(
        self,
        cells: list[tuple[int, int]],
    ) -> Path:
        stamp = self.get_clock().now().to_msg()

        path_msg = Path()
        path_msg.header.frame_id = self._map_frame
        path_msg.header.stamp = stamp

        world_points = [
            self._grid_geometry.cell_center_in_world(cell)
            for cell in cells
        ]

        for index, (x_m, y_m) in enumerate(world_points):
            pose = PoseStamped()
            pose.header.frame_id = self._map_frame
            pose.header.stamp = stamp

            pose.pose.position.x = x_m
            pose.pose.position.y = y_m
            pose.pose.position.z = 0.0

            if index + 1 < len(world_points):
                next_x, next_y = world_points[index + 1]
                yaw = math.atan2(
                    next_y - y_m,
                    next_x - x_m,
                )

                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                pose.pose.orientation = self._goal.pose.orientation

            path_msg.poses.append(pose)

        self._path_publisher.publish(path_msg)
        return path_msg

    # clear bad path from rviz
    def _publish_empty_path(self) -> None:
        msg = Path()
        msg.header.frame_id = self._map_frame or "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        self._path_publisher.publish(msg)
        self._pending_follow_path = None
        self._pending_follow_generation = None
        self._follow_stop_requested = True
        self._cancel_active_following()

    def _finish_failed_plan(self) -> None:
        self._plan_requested = False
        self._earliest_replan_ns = 0
        self._publish_empty_path()

    def _queue_path_for_following(self, path: Path) -> None:
        self._pending_follow_path = path
        self._pending_follow_generation = self._goal_generation
        self._follow_stop_requested = False
        self._blocked_path_observations = 0
        self._cancel_active_following()

    def _dispatch_pending_path(self) -> None:
        if self._pending_follow_path is None:
            return
        if self._follow_goal_request_future is not None:
            return
        if self._follow_goal_handle is not None:
            self._cancel_active_following()
            return

        if not self._follow_path_client.server_is_ready():
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_action_server_warning_ns >= 2000000000:
                self.get_logger().warning(
                    "waiting for the follow_path action server"
                )
                self._last_action_server_warning_ns = now_ns
            return

        goal = FollowPath.Goal()
        goal.path = self._pending_follow_path
        goal.controller_id = ""
        goal.goal_checker_id = ""
        goal.progress_checker_id = ""
        generation = self._pending_follow_generation
        self._follow_goal_request_path = self._pending_follow_path
        self._follow_goal_request_generation = generation
        self._pending_follow_path = None
        self._pending_follow_generation = None
        self._follow_stop_requested = False
        self._follow_goal_request_future = (
            self._follow_path_client.send_goal_async(goal)
        )
        self._follow_goal_request_future.add_done_callback(
            self._follow_goal_response_callback
        )

    def _follow_goal_response_callback(self, future) -> None:
        self._follow_goal_request_future = None
        path = self._follow_goal_request_path
        generation = self._follow_goal_request_generation
        self._follow_goal_request_path = None
        self._follow_goal_request_generation = None
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(
                f"could not send path to follower: {error}"
            )
            return

        if not goal_handle.accepted:
            self.get_logger().warning("path follower rejected the path")
            return

        self._follow_goal_handle = goal_handle
        self._follow_goal_generation = generation
        self._active_follow_path = path
        self._current_waypoint_index = 0
        self._blocked_path_observations = 0
        self._last_path_dispatch_ns = self.get_clock().now().nanoseconds
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, goal_handle=goal_handle, generation=generation: (
                self._follow_result_callback(
                    result,
                    goal_handle,
                    generation,
                )
            )
        )

        if (
            self._follow_stop_requested
            or self._pending_follow_path is not None
        ):
            self._cancel_active_following()
        else:
            self.get_logger().info("path follower accepted the path")

    def _follow_result_callback(
        self,
        future,
        goal_handle,
        generation,
    ) -> None:
        if goal_handle is not self._follow_goal_handle:
            return

        self._follow_goal_handle = None
        self._follow_goal_generation = None
        self._follow_cancel_future = None
        self._active_follow_path = None
        self._current_waypoint_index = 0
        self._blocked_path_observations = 0
        try:
            response = future.result()
            result = response.result
        except Exception as error:
            self.get_logger().error(
                f"could not obtain path-following result: {error}"
            )
            return

        succeeded = response.status == GoalStatus.STATUS_SUCCEEDED
        canceled = response.status == GoalStatus.STATUS_CANCELED
        message = result.error_msg or (
            "goal reached" if succeeded else "no error message provided"
        )
        result_summary = (
            f"path follower finished: {message} "
            f"(status={response.status}, error_code={result.error_code})"
        )
        if succeeded or canceled:
            self.get_logger().info(result_summary)
        else:
            self.get_logger().warning(result_summary)

        if (
            self._replan_on_blocked_path
            and generation == self._goal_generation
            and response.status == GoalStatus.STATUS_ABORTED
            and result.error_code == FollowPath.Result.NO_VALID_CONTROL
        ):
            self._request_replan(
                "follower detected a local obstacle",
                delay_s=self._replan_cooldown_s,
            )

    def _check_active_path(self) -> None:
        if not self._replan_on_blocked_path:
            return
        if self._active_follow_path is None:
            return
        if self._costmap is None or self._grid_geometry is None:
            return
        if self._map_frame is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        path_age_s = (now_ns - self._last_path_dispatch_ns) / 1e9
        if path_age_s < self._replan_cooldown_s:
            return

        try:
            map_to_base = self._transform_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"cannot validate active path without robot pose: {error}"
            )
            return

        points = np.array(
            [
                [pose.pose.position.x, pose.pose.position.y]
                for pose in self._active_follow_path.poses
            ],
            dtype=float,
        )
        try:
            validity = validate_remaining_path(
                costmap=self._costmap,
                grid_geometry=self._grid_geometry,
                robot_position_world=(
                    map_to_base.transform.translation.x,
                    map_to_base.transform.translation.y,
                ),
                path_points_world=points,
                previous_waypoint_index=self._current_waypoint_index,
            )
        except ValueError as error:
            self.get_logger().error(f"cannot validate active path: {error}")
            return

        self._current_waypoint_index = validity.current_waypoint_index

        if validity.is_valid:
            self._blocked_path_observations = 0
            return

        self._blocked_path_observations += 1
        self.get_logger().warning(
            f"active path appears blocked "
            f"({self._blocked_path_observations}/"
            f"{self._blocked_path_confirmations}): {validity.reason}"
        )
        if self._blocked_path_observations < self._blocked_path_confirmations:
            return

        self._blocked_path_observations = 0
        self._request_replan(validity.reason)

    def _request_replan(self, reason: str, delay_s: float = 0.0) -> None:
        if self._goal is None:
            return
        if self._plan_requested:
            return

        self.get_logger().warning(f"requesting replan: {reason}")
        self._plan_requested = True
        self._earliest_replan_ns = (
            self.get_clock().now().nanoseconds + int(delay_s * 1e9)
        )
        self._pending_follow_path = None
        self._pending_follow_generation = None
        self._follow_stop_requested = True
        self._cancel_active_following()

    def _cancel_active_following(self) -> None:
        if self._follow_goal_handle is None:
            return
        if self._follow_cancel_future is not None:
            return

        self._follow_cancel_future = (
            self._follow_goal_handle.cancel_goal_async()
        )
        self._follow_cancel_future.add_done_callback(
            self._follow_cancel_response_callback
        )

    def _follow_cancel_response_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self._follow_cancel_future = None
            self.get_logger().error(
                f"could not cancel active path: {error}"
            )
            return

        if not response.goals_canceling:
            self._follow_cancel_future = None
            self.get_logger().warning(
                "path follower did not accept the cancel request"
            )

    @staticmethod
    def _pose_from_ros_pose(pose) -> Pose2d:
        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        return Pose2d(
            x_m=pose.position.x,
            y_m=pose.position.y,
            yaw_rad=yaw,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = PathPlannerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
