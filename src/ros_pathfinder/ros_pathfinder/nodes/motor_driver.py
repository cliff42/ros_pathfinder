import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float64MultiArray

from ros_pathfinder.hardware.gpio_motor import GpioPhaseEnableMotor
from ros_pathfinder.hardware.base_motor import MotorPair

# ros motor commands -> physical gpio output
class MotorDriverNode(Node):

    COMMAND_TOPIC = "motor_commands"

    def __init__(self):
        super().__init__("motor_driver")
        self._closed = False

        self._declare_parameters()
        self._get_parameters()

        self._motors = self.create_motor_pair()

        self.command_subscription = self.create_subscription(
            Float64MultiArray,
            self.COMMAND_TOPIC,
            self.command_callback,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE
            )
        )

    def create_motor_pair(self) -> MotorPair:
        left_motor = GpioPhaseEnableMotor(
            self.left_phase_pin, 
            self.left_enable_pin,
            self.deadband, 
            self.max_abs_effort, 
            self.left_inverted
        )

        right_motor = GpioPhaseEnableMotor(
            self.right_phase_pin, 
            self.right_enable_pin, 
            self.deadband, 
            self.max_abs_effort, 
            self.right_inverted
        )

        return MotorPair(left_motor, right_motor)

    def command_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 2:
            self.get_logger().warning("invalid motor command, expected: [left_speed, right_speed]")
            self._motors.stop()
            return

        left_effort = float(msg.data[0])
        right_effort = float(msg.data[1])

        if not (math.isfinite(left_effort) and math.isfinite(right_effort)):
            self.get_logger().warning("effort commands must be finite")
            self._motors.stop()
            return

        try:
            _set_left_effort, _set_right_effort = self._motors.set_efforts(left_effort, right_effort)
        except Exception as e:
            self.get_logger().error(f"error setting motor efforts: {e}")


    def destroy_node(self):
        if not self._closed:
            self._closed = True
            try:
                self._motors.close()
            except Exception as e:
                self.get_logger().error(
                    f"error while closing motors: {e}"
                )

        return super().destroy_node()

    def _declare_parameters(self) -> None:
        self.declare_parameter("deadband", 0.002)
        self.declare_parameter("max_abs_effort", 0.40)

        # left motor
        self.declare_parameter("left_motor.phase_pin", 20)
        self.declare_parameter("left_motor.enable_pin", 21)
        self.declare_parameter("left_motor.inverted", False)

        # right motor
        self.declare_parameter("right_motor.phase_pin", 23)
        self.declare_parameter("right_motor.enable_pin", 24)
        self.declare_parameter("right_motor.inverted", False)

    def _get_parameters(self) -> None:
        self.deadband = self.get_parameter("deadband").value
        self.max_abs_effort = self.get_parameter("max_abs_effort").value

        #left motor
        self.left_phase_pin = self.get_parameter("left_motor.phase_pin").value
        self.left_enable_pin = self.get_parameter("left_motor.enable_pin").value
        self.left_inverted = self.get_parameter("left_motor.inverted").value

        #right motor
        self.right_phase_pin = self.get_parameter("right_motor.phase_pin").value
        self.right_enable_pin = self.get_parameter("right_motor.enable_pin").value
        self.right_inverted = self.get_parameter("right_motor.inverted").value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = MotorDriverNode()
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