import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String, Float64MultiArray
from smbus2 import SMBus, i2c_msg
import board
import busio


from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
bno = BNO08X_I2C(i2c, address=0x4a)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)



as5600 = 0x36
icm29408 = 0x69
bno085 = 0x4a

bus1 = SMBus(1)   # /dev/i2c-1
bus2 = SMBus(2)   # /dev/i2c-2



REG_STATUS = 0x0B
REG_RAW_ANGLE_H = 0x0C  # read 2 bytes: 0x0C,0x0D

class EncoderPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'test_topic', 10)
        self.publisher2_ = self.create_publisher(Float64MultiArray,'imu_topic',10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [self.get_raw_angle(bus1), self.get_raw_angle(bus2),self.timer_period]
        msg2 = Float64MultiArray()
        msg2.data = [bno.acceleration[0],bno.acceleration[1],bno.acceleration[2],bno.gyro[0],bno.gyro[1],bno.gyro[2],self.timer_period]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Encoder Data: "%s"' % msg.data)
        self.publisher2_.publish(msg2)
        self.get_logger().info('Publishing IMU Data: "%s"' % msg2.data)

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
            encoder_publisher = EncoderPublisher()
            rclpy.spin(encoder_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()