import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu

import board
import busio

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
)
from adafruit_bno08x.i2c import BNO08X_I2C


class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')

        self.topic = str(self.declare_parameter(
            'topic', 'imu/data_raw').value)
        self.frame_id = str(self.declare_parameter(
            'frame_id', 'imu_link').value)
        address = int(self.declare_parameter('i2c_address', 0x4a).value)
        publish_rate_hz = float(self.declare_parameter(
            'publish_rate_hz', 100.0).value)
        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')

        self.i2c = busio.I2C(
            board.SCL,
            board.SDA,
            frequency=400_000,
        )
        self.bno = BNO08X_I2C(self.i2c, address=address)
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)

        self.publisher = self.create_publisher(
            Imu,
            self.topic,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.timer_callback,
        )
        self.last_error_log_ns = None

        self.get_logger().info(
            f'publishing BNO085 IMU data on {self.topic} at '
            f'{publish_rate_hz:.1f} Hz in frame {self.frame_id}'
        )

    def timer_callback(self):
        try:
            acceleration = self.bno.acceleration
            gyro = self.bno.gyro
            if acceleration is None or gyro is None:
                return

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # The BNO085 driver reports acceleration in m/s^2 and gyro rates
            # in rad/s. Orientation is intentionally left unavailable.
            msg.linear_acceleration.x = float(acceleration[0])
            msg.linear_acceleration.y = float(acceleration[1])
            msg.linear_acceleration.z = float(acceleration[2])
            msg.angular_velocity.x = float(gyro[0])
            msg.angular_velocity.y = float(gyro[1])
            msg.angular_velocity.z = float(gyro[2])

            # No orientation estimate is being published. Zero covariance
            # arrays mean the acceleration/rate covariances are unknown.
            msg.orientation_covariance[0] = -1.0

            self.publisher.publish(msg)
        except (OSError, RuntimeError, TypeError) as exc:
            now_ns = self.get_clock().now().nanoseconds
            if (
                self.last_error_log_ns is None
                or now_ns - self.last_error_log_ns >= 1_000_000_000
            ):
                self.get_logger().warning(f'failed to read BNO085: {exc}')
                self.last_error_log_ns = now_ns


def main(args=None):
    try:
        with rclpy.init(args=args):
            imu_publisher = ImuPublisher()
            rclpy.spin(imu_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
