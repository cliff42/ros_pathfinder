import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3

from smbus2 import SMBus
import board
import busio

import math

HEADER_FRAME = 'odom' # static world frame
CHILD_FRAME = 'base_link' # robot frame

i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)

as5600 = 0x36

bus1 = SMBus(1)   # /dev/i2c-1
bus2 = SMBus(2)   # /dev/i2c-2

REG_STATUS = 0x0B
REG_RAW_ANGLE_H = 0x0C  # read 2 bytes: 0x0C,0x0D

class OdometryPublisher(Node):
    prev_angle_l = 0
    prev_angle_r = 0
    distance_l = 0
    distance_r = 0
    prev_distance_l = 0
    prev_distance_r = 0
    vel_l = 0
    vel_r = 0
    prev_vel_l = 0
    prev_vel_r = 0
    init_angle = True

    def __init__(self):
        super().__init__('odometry_publisher')
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.odom_callback)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0

    def odom_callback(self):
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = HEADER_FRAME
        tf.child_frame_id = CHILD_FRAME

        angle_l = self.get_raw_angle(bus2)
        angle_r = self.get_raw_angle(bus1)

        if self.init_angle:
            self.prev_angle_l = angle_l
            self.prev_angle_r = angle_r
            self.init_angle = False

        self.distance_l, self.prev_angle_l = self.get_distance(angle_l, self.prev_angle_l, self.distance_l)
        self.distance_r, self.prev_angle_r = self.get_distance(angle_r, self.prev_angle_r, self.distance_r)

        self.vel_l, self.prev_distance_l = self.get_velocity(self.distance_l, self.prev_distance_l, self.timer_period)
        self.vel_r, self.prev_distance_r = self.get_velocity(self.distance_r, self.prev_distance_r, self.timer_period)

        # wheel is backwards
        self.vel_l = -1*self.vel_l

        # TODO: should we reuse the linear velocity here
        x_dot = ((self.vel_r + self.vel_l) / 2) * math.cos(self.theta)
        self.x = self.x + x_dot * self.timer_period

        # TODO: should we reuse the linear velocity here
        y_dot = ((self.vel_r + self.vel_l) / 2) * math.sin(self.theta)
        self.y = self.y + y_dot * self.timer_period

        # TODO: should we reuse the angular velocity here
        theta_dot = (self.vel_r - self.vel_l) / 0.55 # measured from distance between wheels (m)
        self.theta = self.theta + theta_dot * self.timer_period

        tf.transform.translation.x = float(self.x)
        tf.transform.translation.y = float(self.y)
        tf.transform.translation.z = float(self.z)

        quat_z = math.sin(self.theta / 2.0)
        quat_w = math.cos(self.theta / 2.0)
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = quat_z
        tf.transform.rotation.w = quat_w

        self.tf_broadcaster.sendTransform(tf)

        msg = Odometry()
        msg.header = tf.header
        msg.child_frame_id = CHILD_FRAME

        # TODO: do we even want to publish the pose at all now?
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = tf.transform.rotation

        # linear velocity
        msg.twist.twist.linear.x = (self.vel_r + self.vel_l) / 2.0 # from https://en.wikipedia.org/wiki/Differential_wheeled_robot
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        # TODO: switch angular velocity to use IMU data (keep encoder data for linear)
        # angular velocity
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = (self.vel_r - self.vel_l) / 0.55 # from https://en.wikipedia.org/wiki/Differential_wheeled_robot

        # TODO: determine real values for this (account for IMU error for angular and encoder error for linear)
        # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        # 0.05 is Var(vx) (linear velocity) 0.1 is Var(wz) (angular velocity)
        # TODO: do we want to set `cov_vx_wz` (top right and bottom left of this matrix)?
        msg.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0,  1e6, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0, 1e6, 0.0, 0.0, 0.0,
            0.0,  0.0, 0.0, 1e6, 0.0, 0.0,
            0.0,  0.0, 0.0, 0.0, 1e6, 0.0,
            0.0,  0.0, 0.0, 0.0, 0.0, 0.1
        ]

        self.odom_publisher.publish(msg)

    def get_raw_angle(self, bus):
        data = bus.read_i2c_block_data(as5600, REG_RAW_ANGLE_H, 2)
        raw = ((data[0] << 8) | data[1]) & 0x0FFF
        return (raw * 360.0) / 4096.0
    
    def get_status(self, bus):
        status = bus.read_byte_data(as5600, REG_STATUS)
        return status
    
    def get_distance(self, angle, prev_angle, distance):
        delta_angle = angle - prev_angle
        if delta_angle > 180:
            delta_angle -= 360
        elif delta_angle < -180:
            delta_angle += 360
        distance += (delta_angle / 7.0) * (math.pi / 180.0) * 4 * 0.0254
        return distance, angle
    
    def get_velocity(self,dist,prev_dist,timer_period):
        vel = (dist - prev_dist)/timer_period
        prev_dist = dist
        return vel, prev_dist
    

def main(args=None):
    try:
        with rclpy.init(args=args):
            odom_publisher = OdometryPublisher()
            rclpy.spin(odom_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

