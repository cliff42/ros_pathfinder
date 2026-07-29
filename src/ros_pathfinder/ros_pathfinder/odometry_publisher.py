import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from smbus2 import SMBus

import math

HEADER_FRAME = 'raw_odom'  # uncorrected wheel odometry frame
CHILD_FRAME = 'base_link'  # robot frame
DEFAULT_WHEEL_TRACK_M = 0.24
DEFAULT_INITIAL_HEADING_RAD = math.pi

LEFT_ENCODER_SIGN = -1.0
RIGHT_ENCODER_SIGN = 1.0

as5600 = 0x36

REG_STATUS = 0x0B
REG_RAW_ANGLE_H = 0x0C  # read 2 bytes: 0x0C,0x0D


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')
        self.odom_publisher = self.create_publisher(Odometry, 'raw_odom', 10)
        self.wheel_velocity_publisher = self.create_publisher(
            Float64MultiArray,
            'wheel_velocities',
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer_period = 0.02
        self.initial_heading = float(
            self.declare_parameter(
                'initial_heading',
                DEFAULT_INITIAL_HEADING_RAD
            ).value
        )
        self.wheel_track_m = float(
            self.declare_parameter(
                'wheel_track_m',
                DEFAULT_WHEEL_TRACK_M,
            ).value
        )
        if self.wheel_track_m <= 0.0:
            raise ValueError('wheel_track_m must be greater than zero')
        self.use_imu_angular_velocity = bool(
            self.declare_parameter(
                'use_imu_angular_velocity',
                True,
            ).value
        )
        self.imu_topic = str(
            self.declare_parameter(
                'imu_topic',
                'imu/data_raw',
            ).value
        )
        self.imu_yaw_sign = float(
            self.declare_parameter('imu_yaw_sign', 1.0).value
        )
        self.imu_yaw_bias_rad_s = float(
            self.declare_parameter('imu_yaw_bias_rad_s', 0.0).value
        )
        self.imu_yaw_deadband_rad_s = abs(float(
            self.declare_parameter(
                'imu_yaw_deadband_rad_s',
                0.005,
            ).value
        ))
        self.imu_timeout_s = float(
            self.declare_parameter('imu_timeout_s', 0.1).value
        )
        if self.imu_timeout_s <= 0.0:
            raise ValueError('imu_timeout_s must be greater than zero')

        self.prev_angle_l = 0.0
        self.prev_angle_r = 0.0
        self.distance_l = 0.0
        self.distance_r = 0.0
        self.prev_distance_l = 0.0
        self.prev_distance_r = 0.0
        self.vel_l = 0.0
        self.vel_r = 0.0
        self.init_angle = True

        self.latest_imu_yaw_rate = None
        self.latest_imu_receive_time = None
        self.angular_velocity_source = None

        # The left encoder is on /dev/i2c-2 and the right encoder is on
        # /dev/i2c-1.
        self.left_encoder_bus = SMBus(2)
        self.right_encoder_bus = SMBus(1)

        if self.use_imu_angular_velocity:
            self.imu_subscriber = self.create_subscription(
                Imu,
                self.imu_topic,
                self.imu_callback,
                qos_profile_sensor_data,
            )

        self.timer = self.create_timer(
            self.timer_period,
            self.odom_callback,
        )

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = self.initial_heading
        self.get_logger().info(
            f'encoder signs: left={LEFT_ENCODER_SIGN}, '
            f'right={RIGHT_ENCODER_SIGN}, '
            f'wheel_track_m={self.wheel_track_m}, '
            f'initial_heading={self.initial_heading}, '
            f'use_imu_angular_velocity={self.use_imu_angular_velocity}, '
            f'imu_topic={self.imu_topic}, '
            f'imu_yaw_sign={self.imu_yaw_sign}, '
            f'imu_yaw_bias_rad_s={self.imu_yaw_bias_rad_s}, '
            f'imu_yaw_deadband_rad_s={self.imu_yaw_deadband_rad_s}, '
            f'imu_timeout_s={self.imu_timeout_s}'
        )

    def imu_callback(self, msg):
        yaw_rate = (
            self.imu_yaw_sign * float(msg.angular_velocity.z)
            - self.imu_yaw_bias_rad_s
        )
        if not math.isfinite(yaw_rate):
            self.get_logger().warning(
                'ignoring non-finite IMU angular velocity'
            )
            return

        if abs(yaw_rate) < self.imu_yaw_deadband_rad_s:
            yaw_rate = 0.0

        self.latest_imu_yaw_rate = yaw_rate
        self.latest_imu_receive_time = self.get_clock().now()

    def odom_callback(self):
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = HEADER_FRAME
        tf.child_frame_id = CHILD_FRAME

        angle_l = self.get_raw_angle(self.left_encoder_bus)
        angle_r = self.get_raw_angle(self.right_encoder_bus)

        if self.init_angle:
            self.prev_angle_l = angle_l
            self.prev_angle_r = angle_r
            self.init_angle = False

        self.distance_l, self.prev_angle_l = self.get_distance(
            angle_l,
            self.prev_angle_l,
            self.distance_l,
        )
        self.distance_r, self.prev_angle_r = self.get_distance(
            angle_r,
            self.prev_angle_r,
            self.distance_r,
        )

        self.vel_l, self.prev_distance_l = self.get_velocity(
            self.distance_l,
            self.prev_distance_l,
            self.timer_period,
        )
        self.vel_r, self.prev_distance_r = self.get_velocity(
            self.distance_r,
            self.prev_distance_r,
            self.timer_period,
        )

        self.vel_l *= LEFT_ENCODER_SIGN
        self.vel_r *= RIGHT_ENCODER_SIGN
        wheel_velocities = Float64MultiArray()
        wheel_velocities.data = [self.vel_l, self.vel_r]
        self.wheel_velocity_publisher.publish(wheel_velocities)

        linear_velocity = (self.vel_r + self.vel_l) / 2.0
        encoder_angular_velocity = (
            self.vel_r - self.vel_l
        ) / self.wheel_track_m
        angular_velocity = self.get_angular_velocity(
            encoder_angular_velocity
        )

        x_dot = linear_velocity * math.cos(self.theta)
        self.x = self.x + x_dot * self.timer_period

        y_dot = linear_velocity * math.sin(self.theta)
        self.y = self.y + y_dot * self.timer_period

        # Keep the pose and twist internally consistent by integrating the
        # same angular velocity that is published in the twist.
        self.theta = self.theta + angular_velocity * self.timer_period
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

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
        msg.twist.twist.linear.x = linear_velocity
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = angular_velocity

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

    def get_angular_velocity(self, encoder_angular_velocity):
        use_imu = False
        if (
            self.use_imu_angular_velocity
            and self.latest_imu_yaw_rate is not None
            and self.latest_imu_receive_time is not None
        ):
            age_s = (
                self.get_clock().now() - self.latest_imu_receive_time
            ).nanoseconds * 1e-9
            use_imu = 0.0 <= age_s <= self.imu_timeout_s

        source = 'imu' if use_imu else 'encoders'
        if source != self.angular_velocity_source:
            self.get_logger().info(
                f'angular velocity source: {source}'
            )
            self.angular_velocity_source = source

        if use_imu:
            return self.latest_imu_yaw_rate
        return encoder_angular_velocity

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

    def get_velocity(self, dist, prev_dist, timer_period):
        vel = (dist - prev_dist) / timer_period
        prev_dist = dist
        return vel, prev_dist

    def destroy_node(self):
        self.left_encoder_bus.close()
        self.right_encoder_bus.close()
        super().destroy_node()


def main(args=None):
    try:
        with rclpy.init(args=args):
            odom_publisher = OdometryPublisher()
            rclpy.spin(odom_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
