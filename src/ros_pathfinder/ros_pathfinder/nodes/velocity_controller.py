import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ros_pathfinder.control.differential_drive import (
    wheel_angular_velocities_from_twist,
)
from ros_pathfinder.control.wheel_velocity_control import (
    WheelVelocityControlConfig,
    WheelVelocityController,
)


class VelocityControllerNode(Node):
    CMD_VEL_TOPIC = "cmd_vel"
    JOINT_STATES_TOPIC = "joint_states"
    MOTOR_COMMAND_TOPIC = "motor_commands"

    LEFT_WHEEL_NAME = "left_wheel_joint"
    RIGHT_WHEEL_NAME = "right_wheel_joint"

    def __init__(self) -> None:
        super().__init__("velocity_controller")
        self._closed = False

        self._declare_parameters()
        self._load_parameters()

        self._desired_left_rad_s = 0.0
        self._desired_right_rad_s = 0.0
        self._measured_left_rad_s: Optional[float] = None
        self._measured_right_rad_s: Optional[float] = None
        self._last_command_time_ns: Optional[int] = None
        self._last_feedback_time_ns: Optional[int] = None
        self._last_control_time_ns: Optional[int] = None
        self._last_feedback_warning_ns = 0
        self._last_diagnostic_ns = 0

        self._left_controller = WheelVelocityController(
            WheelVelocityControlConfig(
                proportional_gain=self._proportional_gain,
                integral_gain=self._integral_gain,
                feedforward_gain=self._left_feedforward_gain,
                integral_limit_rad=self._integral_limit_rad,
                max_abs_effort=self._max_abs_effort,
                max_acceleration_rad_s2=(
                    self._max_wheel_acceleration_rad_s2
                ),
                stopped_velocity_tolerance_rad_s=(
                    self._stopped_velocity_tolerance_rad_s
                ),
            )
        )
        self._right_controller = WheelVelocityController(
            WheelVelocityControlConfig(
                proportional_gain=self._proportional_gain,
                integral_gain=self._integral_gain,
                feedforward_gain=self._right_feedforward_gain,
                integral_limit_rad=self._integral_limit_rad,
                max_abs_effort=self._max_abs_effort,
                max_acceleration_rad_s2=(
                    self._max_wheel_acceleration_rad_s2
                ),
                stopped_velocity_tolerance_rad_s=(
                    self._stopped_velocity_tolerance_rad_s
                ),
            )
        )

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        feedback_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._cmd_vel_subscription = self.create_subscription(
            Twist,
            self.CMD_VEL_TOPIC,
            self._cmd_vel_callback,
            command_qos,
        )
        self._joint_state_subscription = self.create_subscription(
            JointState,
            self.JOINT_STATES_TOPIC,
            self._joint_state_callback,
            feedback_qos,
        )
        self._motor_command_publisher = self.create_publisher(
            Float64MultiArray,
            self.MOTOR_COMMAND_TOPIC,
            command_qos,
        )
        self._control_timer = self.create_timer(
            1.0 / self._control_rate_hz,
            self._control_callback,
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        linear_velocity_m_s = float(msg.linear.x)
        angular_velocity_rad_s = float(msg.angular.z)
        if not (
            math.isfinite(linear_velocity_m_s)
            and math.isfinite(angular_velocity_rad_s)
        ):
            self.get_logger().warning("cmd_vel values must be finite")
            return

        left_rad_s, right_rad_s = wheel_angular_velocities_from_twist(
            linear_velocity_m_s,
            angular_velocity_rad_s,
            self._wheel_radius_m,
            self._wheel_separation_m,
        )
        speed_scale = max(
            1.0,
            abs(left_rad_s) / self._max_wheel_rad_s,
            abs(right_rad_s) / self._max_wheel_rad_s,
        )
        self._desired_left_rad_s = left_rad_s / speed_scale
        self._desired_right_rad_s = right_rad_s / speed_scale
        self._last_command_time_ns = self.get_clock().now().nanoseconds

    def _joint_state_callback(self, msg: JointState) -> None:
        try:
            left_index = msg.name.index(self.LEFT_WHEEL_NAME)
            right_index = msg.name.index(self.RIGHT_WHEEL_NAME)
            left_rad_s = float(msg.velocity[left_index])
            right_rad_s = float(msg.velocity[right_index])
        except (ValueError, IndexError):
            self.get_logger().warning(
                "joint_states is missing wheel names or velocities"
            )
            return

        if not (
            math.isfinite(left_rad_s) and math.isfinite(right_rad_s)
        ):
            self.get_logger().warning(
                "joint_states wheel velocities must be finite"
            )
            return

        alpha = self._measured_velocity_filter_alpha
        if self._measured_left_rad_s is None:
            self._measured_left_rad_s = left_rad_s
            self._measured_right_rad_s = right_rad_s
        else:
            assert self._measured_right_rad_s is not None
            self._measured_left_rad_s = (
                alpha * left_rad_s
                + (1.0 - alpha) * self._measured_left_rad_s
            )
            self._measured_right_rad_s = (
                alpha * right_rad_s
                + (1.0 - alpha) * self._measured_right_rad_s
            )
        self._last_feedback_time_ns = self.get_clock().now().nanoseconds

    def _control_callback(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_control_time_ns is None:
            elapsed_s = 1.0 / self._control_rate_hz
        else:
            elapsed_s = (now_ns - self._last_control_time_ns) * 1e-9
            elapsed_s = min(elapsed_s, 2.0 / self._control_rate_hz)
        self._last_control_time_ns = now_ns
        if elapsed_s <= 0.0:
            self._stop_and_reset()
            return

        if self._command_is_stale(now_ns):
            self._stop_and_reset()
            return
        if self._feedback_is_stale(now_ns):
            self._stop_and_reset()
            if now_ns - self._last_feedback_warning_ns >= 1_000_000_000:
                self.get_logger().warning(
                    "stopping motors because wheel-velocity feedback is "
                    "missing or stale"
                )
                self._last_feedback_warning_ns = now_ns
            return

        assert self._measured_left_rad_s is not None
        assert self._measured_right_rad_s is not None
        left_effort = self._left_controller.update(
            self._desired_left_rad_s,
            self._measured_left_rad_s,
            elapsed_s,
        )
        right_effort = self._right_controller.update(
            self._desired_right_rad_s,
            self._measured_right_rad_s,
            elapsed_s,
        )
        self._publish_efforts(left_effort, right_effort)
        self._log_diagnostic(now_ns, left_effort, right_effort)

    def _command_is_stale(self, now_ns: int) -> bool:
        if self._last_command_time_ns is None:
            return True
        age_s = (now_ns - self._last_command_time_ns) * 1e-9
        return age_s > self._command_timeout_s

    def _feedback_is_stale(self, now_ns: int) -> bool:
        if (
            self._last_feedback_time_ns is None
            or self._measured_left_rad_s is None
            or self._measured_right_rad_s is None
        ):
            return True
        age_s = (now_ns - self._last_feedback_time_ns) * 1e-9
        return age_s > self._feedback_timeout_s

    def _stop_and_reset(self) -> None:
        self._left_controller.reset()
        self._right_controller.reset()
        self._publish_efforts(0.0, 0.0)

    def _publish_efforts(
        self,
        left_effort: float,
        right_effort: float,
    ) -> None:
        command = Float64MultiArray()
        command.data = [left_effort, right_effort]
        self._motor_command_publisher.publish(command)

    def _log_diagnostic(
        self,
        now_ns: int,
        left_effort: float,
        right_effort: float,
    ) -> None:
        if not self._diagnostic_logging_enabled:
            return
        period_ns = int(self._diagnostic_log_period_s * 1e9)
        if now_ns - self._last_diagnostic_ns < period_ns:
            return
        self._last_diagnostic_ns = now_ns

        self.get_logger().info(
            "wheel_control_diag "
            f"requested=({self._desired_left_rad_s:.3f},"
            f"{self._desired_right_rad_s:.3f})rad/s "
            f"limited=({self._left_controller.limited_target_rad_s:.3f},"
            f"{self._right_controller.limited_target_rad_s:.3f})rad/s "
            f"measured=({self._measured_left_rad_s:.3f},"
            f"{self._measured_right_rad_s:.3f})rad/s "
            f"effort=({left_effort:.3f},{right_effort:.3f})"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("feedback_timeout_s", 0.25)
        self.declare_parameter("max_wheel_rad_s", 5.0)
        self.declare_parameter("max_wheel_acceleration_rad_s2", 4.0)
        self.declare_parameter("max_abs_effort", 0.40)
        self.declare_parameter("wheel_radius_m", 0.1016)
        self.declare_parameter("wheel_separation_m", 0.24)
        self.declare_parameter("proportional_gain", 0.04)
        self.declare_parameter("integral_gain", 0.08)
        self.declare_parameter("left_feedforward_gain", 0.02)
        self.declare_parameter("right_feedforward_gain", 0.02)
        self.declare_parameter("integral_limit_rad", 2.0)
        self.declare_parameter("stopped_velocity_tolerance_rad_s", 0.10)
        self.declare_parameter("measured_velocity_filter_alpha", 0.35)
        self.declare_parameter("diagnostic_logging_enabled", True)
        self.declare_parameter("diagnostic_log_period_s", 0.5)

    def _load_parameters(self) -> None:
        parameter_names = (
            "control_rate_hz",
            "command_timeout_s",
            "feedback_timeout_s",
            "max_wheel_rad_s",
            "max_wheel_acceleration_rad_s2",
            "max_abs_effort",
            "wheel_radius_m",
            "wheel_separation_m",
            "proportional_gain",
            "integral_gain",
            "left_feedforward_gain",
            "right_feedforward_gain",
            "integral_limit_rad",
            "stopped_velocity_tolerance_rad_s",
            "measured_velocity_filter_alpha",
            "diagnostic_log_period_s",
        )
        values = {
            name: float(self.get_parameter(name).value)
            for name in parameter_names
        }
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError("velocity-controller parameters must be finite")

        self._control_rate_hz = values["control_rate_hz"]
        self._command_timeout_s = values["command_timeout_s"]
        self._feedback_timeout_s = values["feedback_timeout_s"]
        self._max_wheel_rad_s = values["max_wheel_rad_s"]
        self._max_wheel_acceleration_rad_s2 = values[
            "max_wheel_acceleration_rad_s2"
        ]
        self._max_abs_effort = values["max_abs_effort"]
        self._wheel_radius_m = values["wheel_radius_m"]
        self._wheel_separation_m = values["wheel_separation_m"]
        self._proportional_gain = values["proportional_gain"]
        self._integral_gain = values["integral_gain"]
        self._left_feedforward_gain = values["left_feedforward_gain"]
        self._right_feedforward_gain = values["right_feedforward_gain"]
        self._integral_limit_rad = values["integral_limit_rad"]
        self._stopped_velocity_tolerance_rad_s = values[
            "stopped_velocity_tolerance_rad_s"
        ]
        self._measured_velocity_filter_alpha = values[
            "measured_velocity_filter_alpha"
        ]
        self._diagnostic_log_period_s = values[
            "diagnostic_log_period_s"
        ]
        self._diagnostic_logging_enabled = bool(
            self.get_parameter("diagnostic_logging_enabled").value
        )

        positive_parameters = {
            "control_rate_hz": self._control_rate_hz,
            "command_timeout_s": self._command_timeout_s,
            "feedback_timeout_s": self._feedback_timeout_s,
            "max_wheel_rad_s": self._max_wheel_rad_s,
            "max_wheel_acceleration_rad_s2": (
                self._max_wheel_acceleration_rad_s2
            ),
            "wheel_radius_m": self._wheel_radius_m,
            "wheel_separation_m": self._wheel_separation_m,
            "diagnostic_log_period_s": self._diagnostic_log_period_s,
        }
        for name, value in positive_parameters.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if not 0.0 < self._max_abs_effort <= 1.0:
            raise ValueError("max_abs_effort must be in (0, 1]")
        if not 0.0 < self._measured_velocity_filter_alpha <= 1.0:
            raise ValueError(
                "measured_velocity_filter_alpha must be in (0, 1]"
            )

    def destroy_node(self):
        if not self._closed:
            self._closed = True
            self._stop_and_reset()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = VelocityControllerNode()
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
