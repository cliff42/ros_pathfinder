import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from smbus2 import SMBus, i2c_msg

as5600 = 0x36
icm29408 = 0x69
bno085 = 0x4a

bus1 = SMBus(1)   # /dev/i2c-1
bus2 = SMBus(2)   # /dev/i2c-2

REG_STATUS = 0x0B
REG_RAW_ANGLE_H = 0x0C  # read 2 bytes: 0x0C,0x0D

class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(String, 'test_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'iteration: %d, status (1): %d, raw_angle (1): %d, status (2): %d, raw_angle (2): %d' % (self.i, self.get_status(bus1), self.get_raw_angle(bus1), self.get_status(bus2), self.get_raw_angle(bus2))
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def get_raw_angle(self, bus):
        data = bus.read_i2c_block_data(as5600, REG_RAW_ANGLE_H, 2)
        raw = ((data[0] << 8) | data[1]) & 0x0FFF
        return (raw * 360.0) / 4096.0
    
    def get_status(self, bus):
        status = bus.read_byte_data(as5600, REG_STATUS)
        return status


def main(args=None):
    try:
        with rclpy.init(args=args):
            imu_publisher = IMUPublisher()

            rclpy.spin(imu_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()