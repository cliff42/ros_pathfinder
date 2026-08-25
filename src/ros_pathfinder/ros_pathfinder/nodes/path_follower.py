import math

from dataclasses import dataclass
from threading import Lock
from typing import Any, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from ros_pathfinder.following.path_tracking import (
    PathTracker,
    PathTrackingCommand,
    PathTrackingConfig,
)
from ros_pathfinder.following.trajectory_collision import (
    TrajectoryCollisionChecker,
    TrajectoryCollisionConfig,
)
from ros_pathfinder.geometry.footprint import FootprintBox2d
from ros_pathfinder.geometry.pose2d import Pose2d
from ros_pathfinder.localization.scan_projection import scan_to_observation


@dataclass
class _ActiveGoal:
    goal_handle: Any
    completion_future: Future
    path_frame: str
    path_points: np.ndarray
    final_yaw_rad: float
    target_index: int
    goal_position_reached: bool
    previous_angular_velocity_rad_s: float
    started_time_ns: int
    last_transform_time_ns: int


# https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit
class PathFollowerNode(Node):
    ACTION_NAME = "follow_path"
    CMD_VEL_TOPIC = "cmd_vel"
    SCAN_TOPIC = "scan"
    ODOM_TOPIC = "odom"
    ODOM_FRAME = "odom"

    def __init__(self) -> None:
        super().__init__("path_follower")

        self._declare_parameters()
        self._load_parameters()

        self._action_callback_group = ReentrantCallbackGroup()
        self._control_callback_group = MutuallyExclusiveCallbackGroup()
        self._state_lock = Lock()
        self._goal_reserved = False
        self._active_goal: Optional[_ActiveGoal] = None
        self._last_transform_warning_ns = 0
        self._last_scan_transform_warning_ns = 0
        self._last_tracking_diagnostic_ns = 0
        self._latest_scan_points_base: Optional[np.ndarray] = None
        self._latest_scan_received_time_ns: Optional[int] = None
        self._latest_linear_velocity_m_s = 0.0
        self._latest_angular_velocity_rad_s = 0.0
        self._latest_odom_yaw_rad = 0.0

        self._tracker = PathTracker(self._tracking_config)
        self._collision_checker = TrajectoryCollisionChecker(
            self._collision_config
        )
        physical_footprint = FootprintBox2d(
            min_x_m=self._collision_config.footprint_min_x_m,
            max_x_m=self._collision_config.footprint_max_x_m,
            min_y_m=self._collision_config.footprint_min_y_m,
            max_y_m=self._collision_config.footprint_max_y_m,
        )
        self._self_filter_footprint = physical_footprint.expanded(
            self._self_filter_padding_m
        )
        self._transform_buffer = Buffer()
        self._transform_listener = TransformListener(
            self._transform_buffer,
            self,
            spin_thread=False,
        )

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            self.CMD_VEL_TOPIC,
            command_qos,
        )
        self._scan_subscription = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            self.ODOM_TOPIC,
            self._odom_callback,
            qos_profile_sensor_data,
        )

        self._action_server = ActionServer(
            self,
            FollowPath,
            self.ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_callback_group,
        )
        self._control_timer = self.create_timer(
            1.0 / self._control_rate_hz,
            self._control_callback,
            callback_group=self._control_callback_group,
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("transform_timeout_s", 1.0)
        self.declare_parameter("desired_linear_velocity_m_s", 0.18)
        self.declare_parameter("minimum_lookahead_distance_m", 0.10)
        self.declare_parameter("lookahead_time_s", 0.30)
        self.declare_parameter("maximum_lookahead_distance_m", 0.18)
        self.declare_parameter("goal_position_tolerance_m", 0.12)
        self.declare_parameter("goal_position_tolerance_buffer_m", 0.05)
        self.declare_parameter("goal_yaw_tolerance_rad", 0.25)
        self.declare_parameter("rotate_in_place_threshold_rad", 0.85)
        self.declare_parameter("angular_gain", 1.0)
        self.declare_parameter("max_angular_velocity_rad_s", 0.65)
        self.declare_parameter("minimum_linear_speed_ratio", 0.20)
        self.declare_parameter("angular_smoothing", 0.20)
        self.declare_parameter("angular_deadband_rad_s", 0.015)
        self.declare_parameter("diagnostic_logging_enabled", True)
        self.declare_parameter("diagnostic_log_period_s", 0.25)
        self.declare_parameter("collision_monitor_enabled", True)
        self.declare_parameter("scan_timeout_s", 0.5)
        self.declare_parameter("footprint_min_x_m", -0.15)
        self.declare_parameter("footprint_max_x_m", 0.50)
        self.declare_parameter("footprint_min_y_m", -0.30)
        self.declare_parameter("footprint_max_y_m", 0.30)
        self.declare_parameter("self_filter_padding_m", 0.02)
        self.declare_parameter("collision_margin_m", 0.05)
        self.declare_parameter("collision_prediction_horizon_s", 1.5)
        self.declare_parameter("collision_prediction_step_s", 0.05)

    def _load_parameters(self) -> None:
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._control_rate_hz = float(
            self.get_parameter("control_rate_hz").value
        )
        self._transform_timeout_s = float(
            self.get_parameter("transform_timeout_s").value
        )
        if not self._base_frame:
            raise ValueError("base_frame cannot be empty")
        if self._control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive")
        if self._transform_timeout_s <= 0.0:
            raise ValueError("transform_timeout_s must be positive")
        self._diagnostic_logging_enabled = bool(
            self.get_parameter("diagnostic_logging_enabled").value
        )
        self._diagnostic_log_period_s = float(
            self.get_parameter("diagnostic_log_period_s").value
        )
        if self._diagnostic_log_period_s <= 0.0:
            raise ValueError("diagnostic_log_period_s must be positive")
        self._collision_monitor_enabled = bool(
            self.get_parameter("collision_monitor_enabled").value
        )
        self._scan_timeout_s = float(
            self.get_parameter("scan_timeout_s").value
        )
        if self._scan_timeout_s <= 0.0:
            raise ValueError("scan_timeout_s must be positive")
        self._self_filter_padding_m = float(
            self.get_parameter("self_filter_padding_m").value
        )
        if (
            not math.isfinite(self._self_filter_padding_m)
            or self._self_filter_padding_m < 0.0
        ):
            raise ValueError(
                "self_filter_padding_m must be finite and non-negative"
            )

        self._tracking_config = PathTrackingConfig(
            desired_linear_velocity_m_s=float(
                self.get_parameter("desired_linear_velocity_m_s").value
            ),
            minimum_lookahead_distance_m=float(
                self.get_parameter("minimum_lookahead_distance_m").value
            ),
            lookahead_time_s=float(
                self.get_parameter("lookahead_time_s").value
            ),
            maximum_lookahead_distance_m=float(
                self.get_parameter("maximum_lookahead_distance_m").value
            ),
            goal_position_tolerance_m=float(
                self.get_parameter("goal_position_tolerance_m").value
            ),
            goal_position_tolerance_buffer_m=float(
                self.get_parameter(
                    "goal_position_tolerance_buffer_m"
                ).value
            ),
            goal_yaw_tolerance_rad=float(
                self.get_parameter("goal_yaw_tolerance_rad").value
            ),
            rotate_in_place_threshold_rad=float(
                self.get_parameter("rotate_in_place_threshold_rad").value
            ),
            angular_gain=float(self.get_parameter("angular_gain").value),
            max_angular_velocity_rad_s=float(
                self.get_parameter("max_angular_velocity_rad_s").value
            ),
            minimum_linear_speed_ratio=float(
                self.get_parameter("minimum_linear_speed_ratio").value
            ),
            angular_smoothing=float(
                self.get_parameter("angular_smoothing").value
            ),
            angular_deadband_rad_s=float(
                self.get_parameter("angular_deadband_rad_s").value
            ),
        )
        self._collision_config = TrajectoryCollisionConfig(
            footprint_min_x_m=float(
                self.get_parameter("footprint_min_x_m").value
            ),
            footprint_max_x_m=float(
                self.get_parameter("footprint_max_x_m").value
            ),
            footprint_min_y_m=float(
                self.get_parameter("footprint_min_y_m").value
            ),
            footprint_max_y_m=float(
                self.get_parameter("footprint_max_y_m").value
            ),
            collision_margin_m=float(
                self.get_parameter("collision_margin_m").value
            ),
            prediction_horizon_s=float(
                self.get_parameter("collision_prediction_horizon_s").value
            ),
            prediction_step_s=float(
                self.get_parameter("collision_prediction_step_s").value
            ),
        )

    def _goal_callback(self, goal_request) -> GoalResponse:
        if (
            goal_request.controller_id
            or goal_request.goal_checker_id
            or goal_request.progress_checker_id
        ):
            self.get_logger().warning(
                "rejecting path because this follower does not support "
                "controller, goal-checker, or progress-checker plugins"
            )
            return GoalResponse.REJECT

        path = goal_request.path
        error = self._path_validation_error(path)
        if error is not None:
            self.get_logger().warning(f"rejecting path: {error}")
            return GoalResponse.REJECT

        with self._state_lock:
            if self._goal_reserved:
                self.get_logger().warning(
                    "rejecting path because another path is active"
                )
                return GoalResponse.REJECT
            self._goal_reserved = True

        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        path = goal_handle.request.path
        points = np.array(
            [
                [pose.pose.position.x, pose.pose.position.y]
                for pose in path.poses
            ],
            dtype=float,
        )
        final_yaw = self._yaw_from_orientation(
            path.poses[-1].pose.orientation
        )
        completion_future = Future()
        now_ns = self.get_clock().now().nanoseconds
        active_goal = _ActiveGoal(
            goal_handle=goal_handle,
            completion_future=completion_future,
            path_frame=path.header.frame_id,
            path_points=points,
            final_yaw_rad=final_yaw,
            target_index=0,
            goal_position_reached=False,
            previous_angular_velocity_rad_s=0.0,
            started_time_ns=now_ns,
            last_transform_time_ns=now_ns,
        )

        with self._state_lock:
            self._active_goal = active_goal

        self._publish_stop()
        self.get_logger().info(
            f"following path with {len(points)} poses in "
            f"'{active_goal.path_frame}'"
        )
        return await completion_future

    def _control_callback(self) -> None:
        with self._state_lock:
            active_goal = self._active_goal
            current_linear_velocity_m_s = self._latest_linear_velocity_m_s
            current_angular_velocity_rad_s = (
                self._latest_angular_velocity_rad_s
            )
            current_odom_yaw_rad = self._latest_odom_yaw_rad

        if active_goal is None:
            return

        if active_goal.goal_handle.is_cancel_requested:
            self._finish_goal(
                active_goal,
                outcome="canceled",
                error_code=FollowPath.Result.NONE,
                message="path following canceled",
            )
            return

        now_ns = self.get_clock().now().nanoseconds
        try:
            path_to_base = self._transform_buffer.lookup_transform(
                active_goal.path_frame,
                self._base_frame,
                Time(),
            )
        except TransformException as error:
            self._publish_stop()
            elapsed_s = (
                now_ns - active_goal.last_transform_time_ns
            ) / 1e9
            if now_ns - self._last_transform_warning_ns >= 1000000000:
                self.get_logger().warning(
                    f"cannot obtain robot pose for path following: {error}"
                )
                self._last_transform_warning_ns = now_ns
            if elapsed_s >= self._transform_timeout_s:
                self._finish_goal(
                    active_goal,
                    outcome="aborted",
                    error_code=FollowPath.Result.TF_ERROR,
                    message="robot pose transform timed out",
                )
            return

        transform = path_to_base.transform
        robot_pose = (
            transform.translation.x,
            transform.translation.y,
            self._yaw_from_orientation(transform.rotation),
        )
        active_goal.last_transform_time_ns = now_ns

        try:
            command = self._tracker.update(
                robot_pose=robot_pose,
                path_points=active_goal.path_points,
                final_yaw_rad=active_goal.final_yaw_rad,
                current_linear_velocity_m_s=current_linear_velocity_m_s,
                previous_target_index=active_goal.target_index,
                previous_angular_velocity_rad_s=(
                    active_goal.previous_angular_velocity_rad_s
                ),
                goal_position_reached=(
                    active_goal.goal_position_reached
                ),
            )
        except ValueError as error:
            self._finish_goal(
                active_goal,
                outcome="aborted",
                error_code=FollowPath.Result.INVALID_PATH,
                message=f"path tracking failed: {error}",
            )
            return

        active_goal.target_index = command.target_index
        active_goal.goal_position_reached = command.goal_position_reached
        active_goal.previous_angular_velocity_rad_s = (
            command.angular_velocity_rad_s
        )

        self._log_tracking_diagnostic(
            now_ns=now_ns,
            active_goal=active_goal,
            robot_pose=robot_pose,
            command=command,
            measured_linear_velocity_m_s=current_linear_velocity_m_s,
            measured_angular_velocity_rad_s=(
                current_angular_velocity_rad_s
            ),
            odom_yaw_rad=current_odom_yaw_rad,
            force=command.goal_reached,
        )

        feedback = FollowPath.Feedback()
        feedback.distance_to_goal = command.distance_to_goal_m
        feedback.speed = abs(command.linear_velocity_m_s)
        active_goal.goal_handle.publish_feedback(feedback)

        if command.goal_reached:
            self._finish_goal(
                active_goal,
                outcome="succeeded",
                error_code=FollowPath.Result.NONE,
                message="goal reached",
            )
            return

        if not self._command_is_collision_safe(
            active_goal,
            command.linear_velocity_m_s,
            command.angular_velocity_rad_s,
            command.goal_position_reached,
        ):
            return

        self._publish_command(
            command.linear_velocity_m_s,
            command.angular_velocity_rad_s,
        )

    def _odom_callback(self, msg: Odometry) -> None:
        linear_velocity_m_s = float(msg.twist.twist.linear.x)
        angular_velocity_rad_s = float(msg.twist.twist.angular.z)
        if not (
            math.isfinite(linear_velocity_m_s)
            and math.isfinite(angular_velocity_rad_s)
        ):
            return

        with self._state_lock:
            self._latest_linear_velocity_m_s = linear_velocity_m_s
            self._latest_angular_velocity_rad_s = angular_velocity_rad_s
            self._latest_odom_yaw_rad = self._yaw_from_orientation(
                msg.pose.pose.orientation
            )

    def _log_tracking_diagnostic(
        self,
        now_ns: int,
        active_goal: _ActiveGoal,
        robot_pose: tuple[float, float, float],
        command: PathTrackingCommand,
        measured_linear_velocity_m_s: float,
        measured_angular_velocity_rad_s: float,
        odom_yaw_rad: float,
        force: bool = False,
    ) -> None:
        if not self._diagnostic_logging_enabled:
            return

        period_ns = int(self._diagnostic_log_period_s * 1e9)
        if (
            not force
            and now_ns - self._last_tracking_diagnostic_ns < period_ns
        ):
            return
        self._last_tracking_diagnostic_ns = now_ns

        robot_x, robot_y, robot_yaw = robot_pose
        goal_x, goal_y = active_goal.path_points[-1]
        goal_yaw_error = math.atan2(
            math.sin(active_goal.final_yaw_rad - robot_yaw),
            math.cos(active_goal.final_yaw_rad - robot_yaw),
        )
        diagnostics = command.diagnostics
        map_to_odom_yaw_text = "unavailable"
        try:
            path_to_odom = self._transform_buffer.lookup_transform(
                active_goal.path_frame,
                self.ODOM_FRAME,
                Time(),
            )
            map_to_odom_yaw = self._yaw_from_orientation(
                path_to_odom.transform.rotation
            )
            map_to_odom_yaw_text = (
                f"{math.degrees(map_to_odom_yaw):.1f}deg"
            )
        except TransformException:
            pass
        if abs(command.angular_velocity_rad_s) > 1e-6:
            command_radius_m = abs(
                command.linear_velocity_m_s
                / command.angular_velocity_rad_s
            )
            command_radius_text = f"{command_radius_m:.3f}"
        else:
            command_radius_text = "inf"

        self.get_logger().info(
            "tracking_diag "
            f"mode={diagnostics.control_mode} "
            f"pose=({robot_x:.3f},{robot_y:.3f},"
            f"{math.degrees(robot_yaw):.1f}deg) "
            f"odom_yaw={math.degrees(odom_yaw_rad):.1f}deg "
            f"map_to_odom_yaw={map_to_odom_yaw_text} "
            f"goal=({goal_x:.3f},{goal_y:.3f},"
            f"{math.degrees(active_goal.final_yaw_rad):.1f}deg) "
            f"xy_error={command.distance_to_goal_m:.3f}m "
            f"goal_yaw_error={math.degrees(goal_yaw_error):.1f}deg "
            f"target={command.target_index + 1}/"
            f"{len(active_goal.path_points)}@"
            f"({diagnostics.target_x_m:.3f},"
            f"{diagnostics.target_y_m:.3f}) "
            f"target_heading="
            f"{math.degrees(diagnostics.target_heading_rad):.1f}deg "
            f"heading_error="
            f"{math.degrees(diagnostics.heading_error_rad):.1f}deg "
            f"lookahead={diagnostics.lookahead_distance_m:.3f}m "
            f"curvature={diagnostics.curvature_m_inv:.3f}m^-1 "
            f"raw_w_rad_s="
            f"{diagnostics.unconstrained_angular_velocity_rad_s:.3f} "
            f"saturated={diagnostics.angular_velocity_saturated} "
            f"cmd=(v={command.linear_velocity_m_s:.3f},"
            f"w={command.angular_velocity_rad_s:.3f},"
            f"radius={command_radius_text}) "
            f"odom=(v={measured_linear_velocity_m_s:.3f},"
            f"w={measured_angular_velocity_rad_s:.3f})"
        )

    def _finish_goal(
        self,
        active_goal: _ActiveGoal,
        outcome: str,
        error_code: int,
        message: str,
    ) -> None:
        with self._state_lock:
            if self._active_goal is not active_goal:
                return
            self._active_goal = None
            self._goal_reserved = False

        self._publish_stop()
        if outcome == "succeeded":
            active_goal.goal_handle.succeed()
            self.get_logger().info(message)
        elif outcome == "canceled":
            active_goal.goal_handle.canceled()
            self.get_logger().info(message)
        else:
            active_goal.goal_handle.abort()
            self.get_logger().error(message)

        result = FollowPath.Result()
        result.error_code = error_code
        result.error_msg = message
        active_goal.completion_future.set_result(result)

    def _scan_callback(self, msg: LaserScan) -> None:
        if not self._collision_monitor_enabled:
            return
        if not msg.header.frame_id:
            return

        try:
            base_from_laser = self._transform_buffer.lookup_transform(
                self._base_frame,
                msg.header.frame_id,
                Time(),
            )
        except TransformException as error:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_scan_transform_warning_ns >= 1000000000:
                self.get_logger().warning(
                    f"cannot transform scan for collision checking: {error}"
                )
                self._last_scan_transform_warning_ns = now_ns
            return

        transform = base_from_laser.transform
        laser_pose_in_base = Pose2d(
            x_m=transform.translation.x,
            y_m=transform.translation.y,
            yaw_rad=self._yaw_from_orientation(transform.rotation),
        )
        observation = scan_to_observation(
            ranges=msg.ranges,
            angle_min_rad=msg.angle_min,
            angle_increment_rad=msg.angle_increment,
            range_min_m=msg.range_min,
            range_max_m=msg.range_max,
            laser_pose_in_base=laser_pose_in_base,
            filter_footprint=self._self_filter_footprint,
        )

        with self._state_lock:
            self._latest_scan_points_base = observation.hit_points_base
            self._latest_scan_received_time_ns = (
                self.get_clock().now().nanoseconds
            )

    def _command_is_collision_safe(
        self,
        active_goal: _ActiveGoal,
        linear_velocity_m_s: float,
        angular_velocity_rad_s: float,
        goal_position_reached: bool,
    ) -> bool:
        if not self._collision_monitor_enabled:
            return True

        now_ns = self.get_clock().now().nanoseconds
        with self._state_lock:
            points = self._latest_scan_points_base
            scan_time_ns = self._latest_scan_received_time_ns

        if scan_time_ns is None or points is None:
            self._publish_stop()
            elapsed_s = (now_ns - active_goal.started_time_ns) / 1e9
            if elapsed_s < self._scan_timeout_s:
                return False
            self._finish_goal(
                active_goal,
                outcome="aborted",
                error_code=FollowPath.Result.CONTROLLER_TIMED_OUT,
                message="no usable laser scan received",
            )
            return False

        scan_age_s = (now_ns - scan_time_ns) / 1e9
        if scan_age_s > self._scan_timeout_s:
            self._finish_goal(
                active_goal,
                outcome="aborted",
                error_code=FollowPath.Result.CONTROLLER_TIMED_OUT,
                message=f"laser scan is stale ({scan_age_s:.2f} s)",
            )
            return False

        try:
            collision = self._collision_checker.check(
                obstacle_points_base=points,
                linear_velocity_m_s=linear_velocity_m_s,
                angular_velocity_rad_s=angular_velocity_rad_s,
            )
        except ValueError as error:
            self._finish_goal(
                active_goal,
                outcome="aborted",
                error_code=FollowPath.Result.UNKNOWN,
                message=f"collision checking failed: {error}",
            )
            return False

        if not collision.collision_detected:
            return True

        point_x, point_y = collision.collision_point_base
        collision_time_s = collision.time_to_collision_s
        if goal_position_reached:
            error_code = FollowPath.Result.FAILED_TO_MAKE_PROGRESS
            message = (
                "goal position reached, but final orientation is blocked by "
                "a local obstacle"
            )
        else:
            error_code = FollowPath.Result.NO_VALID_CONTROL
            message = "local obstacle intersects commanded trajectory"

        self._finish_goal(
            active_goal,
            outcome="aborted",
            error_code=error_code,
            message=(
                f"{message} at ({point_x:.2f}, {point_y:.2f}) m in "
                f"base_link (predicted in {collision_time_s:.2f} s)"
            ),
        )
        return False

    def _publish_command(
        self,
        linear_velocity_m_s: float,
        angular_velocity_rad_s: float,
    ) -> None:
        command = Twist()
        command.linear.x = linear_velocity_m_s
        command.angular.z = angular_velocity_rad_s
        self._cmd_vel_publisher.publish(command)

    def _publish_stop(self) -> None:
        self._cmd_vel_publisher.publish(Twist())

    @staticmethod
    def _path_validation_error(path) -> Optional[str]:
        if not path.header.frame_id:
            return "path header has no frame_id"
        if not path.poses:
            return "path contains no poses"

        for pose in path.poses:
            if (
                pose.header.frame_id
                and pose.header.frame_id != path.header.frame_id
            ):
                return "path poses use inconsistent frame_ids"
            if not (
                math.isfinite(pose.pose.position.x)
                and math.isfinite(pose.pose.position.y)
            ):
                return "path contains a non-finite position"

        orientation = path.poses[-1].pose.orientation
        orientation_values = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(math.isfinite(value) for value in orientation_values):
            return "final path orientation is not finite"
        if sum(value * value for value in orientation_values) < 1e-12:
            return "final path orientation is not a valid quaternion"
        return None

    @staticmethod
    def _yaw_from_orientation(orientation) -> float:
        return math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

    def destroy_node(self):
        with self._state_lock:
            active_goal = self._active_goal

        if active_goal is not None:
            self._finish_goal(
                active_goal,
                outcome="aborted",
                error_code=FollowPath.Result.UNKNOWN,
                message="path follower shut down",
            )
        else:
            self._publish_stop()

        self._action_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)

    try:
        node = PathFollowerNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
