import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu

from ros_pathfinder.hardware.bno08x_imu import BNO08XIMU

class IMU(Node):

    IMU_TOPIC = "imu/data_raw"
    IMU_HEADER_FRAME_ID = "imu_link"

    def __init__(self):
        super().__init__("imu")

        self._declare_parameters()
        self._get_parameters()

        self._closed = False

        self._imu_sensor = BNO08XIMU(address=self.i2c_address, i2c_frequency_hz=self.i2c_frequency_hz)

        self.imu_publisher = self.create_publisher(
            Imu,
            self.IMU_TOPIC,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(
            (1.0 / self.publish_rate_hz),
            self.imu_callback
        )

    def imu_callback(self) -> None:
        now = self.get_clock().now()

        imu_data = self._imu_sensor.read_angular_velocity()

        if imu_data is None:
            return

        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.IMU_HEADER_FRAME_ID
        msg.angular_velocity.x = imu_data.x_rad_s
        msg.angular_velocity.y = imu_data.y_rad_s
        msg.angular_velocity.z = imu_data.z_rad_s
        msg.angular_velocity_covariance = self.angular_velocity_covariance.copy()

        # per ros standards for Imu msg: https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html
        msg.orientation_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.imu_publisher.publish(msg)


    def destroy_node(self):
        if not self._closed:
            self._closed = True
            try:
                self._imu_sensor.close()
            except Exception as e:
                self.get_logger().error(
                    f"error while closing imu sensor: {e}"
                )

        return super().destroy_node()


    def _declare_parameters(self) -> None:
        # TODO: run sudo i2cdetect -y 1 to confirm this
        self.declare_parameter("i2c_address", 0x4A)
        self.declare_parameter("i2c_frequency_hz", 400000)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter(
            "angular_velocity_covariance",
            [0.0] * 9,
        )


    def _get_parameters(self) -> None:
        self.i2c_address = self.get_parameter("i2c_address").value
        self.i2c_frequency_hz = self.get_parameter("i2c_frequency_hz").value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.angular_velocity_covariance = self.get_parameter("angular_velocity_covariance").value

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = IMU()
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