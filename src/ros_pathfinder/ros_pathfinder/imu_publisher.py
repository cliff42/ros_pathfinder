import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
import busio
import board

as5600 = 0x36
icm29408 = 0x69
bno085 = 0x4a

i2c_1 = busio.I2C(board.D1, board.D0)
# i2c_2 = busio.I2C(board.D1, board.D0)

class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(String, 'test_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'iteration: %d, status (1): %d, raw_angle (1): %d, status (2): %d, raw_angle (2): %d' % (self.i, self.get_status(i2c_1), self.get_raw_angle(i2c_1), self.get_status(i2c_1), self.get_raw_angle(i2c_1))
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def get_raw_angle(self, i2c):
        i2c.writeto(as5600, bytes([0x0C]))
        result = bytearray(2)
        i2c.readfrom_into(as5600, result)
        raw_angle = (result[0] << 8) | result[1]   # combine into 16-bit value
        raw_angle &= 0x0FFF                    # mask out 12-bit angle (0–4095)
        print("Raw angle:", (raw_angle * 360.0) / 4096.0) # convert to angle
        return (raw_angle * 360.0) / 4096.0
    
    def get_status(self, i2c):
        i2c.writeto(as5600, bytes([0x0B]))
        result = bytearray(1)
        i2c.readfrom_into(as5600, result)
        status = result[0]
        print("as5600 status:", status)
        bit3_set = bool(status & (1 << 3))
        print("bit 3 set? (too strong)", bit3_set)
        bit4_set = bool(status & (1 << 4))
        print("bit 4 set? (too weak)", bit4_set)
        bit5_set = bool(status & (1 << 5))
        print("bit 5 set? (magnet detected)", bit5_set)
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