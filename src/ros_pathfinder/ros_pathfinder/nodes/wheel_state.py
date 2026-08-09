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

from sensor_msgs.msg import JointState

from ros_pathfinder.hardware.as5600_encoder import AS5600Encoder
from ros_pathfinder.hardware.wheel_encoder import WheelEncoder

# encoder values -> JointState
class WheelState(Node):

    JOINT_STATES_TOPIC = "/joint_states"

    # TODO: add these values to configs
    GEAR_RATIO = 7.0
    # The left encoder is on /dev/i2c-2 and the right encoder is on /dev/i2c-1.
    LEFT_SENSOR_BUS = 1
    RIGHT_SENSOR_BUS = 2
    TIMER_PERIOD = 0.05 # 20 hz


    def __init__(self):
        super().__init__("wheel_state")
        self._closed = False
        
        left_sensor = AS5600Encoder(self.LEFT_SENSOR_BUS)
        right_sensor = AS5600Encoder(self.RIGHT_SENSOR_BUS)

        self._left_encoder = WheelEncoder(
            sensor=left_sensor,
            gear_ratio=self.GEAR_RATIO,
            direction=-1
        )
        self._right_encoder = WheelEncoder(
            sensor=right_sensor,
            gear_ratio=self.GEAR_RATIO,
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
            self.TIMER_PERIOD,
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