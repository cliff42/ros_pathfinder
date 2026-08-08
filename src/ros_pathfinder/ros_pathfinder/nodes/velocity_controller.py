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
from geometry_msgs.msg import Twist

from ros_pathfinder.control.differential_drive import wheel_angular_velocities_from_twist
from ros_pathfinder.control.wheel_effort import wheel_efforts_from_angular_velocities


# twist (/cmd_vel) -> motor commands
class VelocityControllerNode(Node):

    CMD_VEL_TOPIC = "cmd_vel"
    MOTOR_COMMAND_TOPIC = "motor_commands"

    def __init__(self):
        super().__init__("velocity_controller")
        self._closed = False

        self._declare_parameters()
        self._get_parameters()

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            self.CMD_VEL_TOPIC,
            self.cmd_vel_callback,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE
            )
        )

        self.motor_command_publisher = self.create_publisher(
            Float64MultiArray,
            self.MOTOR_COMMAND_TOPIC,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE
            )
        )

    # TODO: add timer so we stop motors when we stop getting new commands
    def cmd_vel_callback(self, msg: Twist) -> None:
        # diff drive can only move forward/backward along x axis
        linear_motion = msg.linear.x

        # diff drive can only rotate about z axis
        rotation_motion = msg.angular.z

        left_angular_vel, right_angular_vel = wheel_angular_velocities_from_twist(linear_motion, rotation_motion, self.wheel_radius_m, self.wheel_separation_m)

        left_effort, right_effort = wheel_efforts_from_angular_velocities(left_angular_vel, right_angular_vel, self.max_wheel_rad_s, self.max_abs_effort)

        out = Float64MultiArray()
        out.data = [left_effort, right_effort]

        self.motor_command_publisher.publish(out)

    def destroy_node(self):
        return super().destroy_node()

    def _declare_parameters(self) -> None:
        self.declare_parameter("max_wheel_rad_s", 5.0)
        self.declare_parameter("max_abs_effort", 0.40)
        self.declare_parameter("wheel_radius_m", 0.1016)
        self.declare_parameter("wheel_separation_m", 0.24)

    def _get_parameters(self) -> None:
        self.max_wheel_rad_s = self.get_parameter("max_wheel_rad_s").value
        self.max_abs_effort = self.get_parameter("max_abs_effort").value
        self.wheel_radius_m = self.get_parameter("wheel_radius_m").value
        self.wheel_separation_m = self.get_parameter("wheel_separation_m").value


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