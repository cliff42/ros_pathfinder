import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState

from ros_pathfinder.hardware.as5600_encoder import AS5600Encoder
from ros_pathfinder.hardware.wheel_encoder import WheelEncoder


class WheelState(Node):
    JOINT_STATES_TOPIC = "joint_states"

    def __init__(self) -> None:
        super().__init__("wheel_state")
        self._closed = False

        self._declare_parameters()
        self._get_parameters()

        left_sensor = AS5600Encoder(self._left_sensor_bus)
        right_sensor = AS5600Encoder(self._right_sensor_bus)

        self._left_encoder = WheelEncoder(
            sensor=left_sensor,
            gear_ratio=self._gear_ratio,
            direction=-1
        )
        self._right_encoder = WheelEncoder(
            sensor=right_sensor,
            gear_ratio=self._gear_ratio,
            direction=1
        )

        self.joint_state_publisher = self.create_publisher(
            JointState,
            self.JOINT_STATES_TOPIC,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE
            )
        )

        self.timer = self.create_timer(
            (1.0 / self._publish_rate_hz),
            self.wheel_state_callback
        )

    def wheel_state_callback(self) -> None:
        now = self.get_clock().now()

        left_sample = self._left_encoder.sample(now.nanoseconds)
        right_sample = self._right_encoder.sample(now.nanoseconds)

        if (left_sample.velocity_rad_s is None or right_sample.velocity_rad_s is None):
            return

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = ["left_wheel_joint", "right_wheel_joint"]
        msg.position = [left_sample.position_rad, right_sample.position_rad]
        msg.velocity = [left_sample.velocity_rad_s, right_sample.velocity_rad_s]

        self.joint_state_publisher.publish(msg)


    def destroy_node(self):
        if not self._closed:
            self._closed = True
            try:
                self._left_encoder.close()
                self._right_encoder.close()
            except Exception as e:
                self.get_logger().error(
                    f"error while closing encoder sensors: {e}"
                )

        return super().destroy_node()

    def _declare_parameters(self) -> None:
        # the left encoder is on /dev/i2c-2 and the right encoder is on /dev/i2c-1.
        self.declare_parameter("left_sensor_bus", 2)
        self.declare_parameter("right_sensor_bus", 1)
        self.declare_parameter("gear_ratio", 7.0)
        self.declare_parameter("publish_rate_hz", 20.0)

    def _get_parameters(self) -> None:
        self._left_sensor_bus = self.get_parameter("left_sensor_bus").value
        self._right_sensor_bus = self.get_parameter("right_sensor_bus").value
        self._gear_ratio = self.get_parameter("gear_ratio").value
        self._publish_rate_hz = self.get_parameter("publish_rate_hz").value

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = WheelState()
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
