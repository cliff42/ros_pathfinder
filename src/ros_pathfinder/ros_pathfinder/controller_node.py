#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import threading


class MotorControlServer(Node):
    WHEELBASE     = 0.55        # distance between wheels
    WHEEL_RADIUS  = 4 * 0.0254
    MAX_WHEEL_VEL = 0.5         # TODO: measure based on hw
    MAX_MOTOR_CMD = 0.12
    MAX_CMD_STEP  = 0.01        # max motor command change per 20 Hz control tick
    KP            = 0.25        # motor command per m/s of wheel-speed error

    def __init__(self):
        super().__init__('controller_node')
        self.cb_group = ReentrantCallbackGroup()
        self.left_motor_publisher  = self.create_publisher(Float64, 'left_motor',  10)
        self.right_motor_publisher = self.create_publisher(Float64, 'right_motor', 10)
        self.data_lock = threading.Lock()

        self.create_subscription(Odometry, 'raw_odom', self._odom_cb, 10,
                                 callback_group=self.cb_group)

        self._cmd_vel = Twist()
        self._last_cmd_time = self.get_clock().now()
        self.CMD_VEL_TIMEOUT = 0.5  # seconds
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10,
                                 callback_group=self.cb_group)

        # Measured wheel velocities (m/s), updated by _odom_cb
        self._vel_l = 0.0
        self._vel_r = 0.0
        self._last_cmd_l = 0.0
        self._last_cmd_r = 0.0
        self._last_log_time = self.get_clock().now()

        # 20 Hz control loop
        self.create_timer(0.05, self._control_loop, callback_group=self.cb_group)

    def _cmd_vel_cb(self, msg: Twist):
        self._cmd_vel = msg
        self._last_cmd_time = self.get_clock().now()

    def _odom_cb(self, msg: Odometry):
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        with self.data_lock:
            self._vel_l = v - w * self.WHEELBASE / 2.0
            self._vel_r = v + w * self.WHEELBASE / 2.0

        # self.get_logger().info('left wheel speed: "%s" right wheel speed: "%s"' % (str(self._vel_l),str(self._vel_r)))

    def _control_loop(self):
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.CMD_VEL_TIMEOUT: # stop motors if no msg recieved for timeout seconds
            self.publish_to_motors(0.0, 0.0)
            return

        # 
        v = self._cmd_vel.linear.x
        w = self._cmd_vel.angular.z

        # desired linear speeds
        v_left  = v - w * self.WHEELBASE / 2.0
        v_right = v + w * self.WHEELBASE / 2.0

        with self.data_lock:
            err_l = v_left  - self._vel_l
            err_r = v_right - self._vel_r

        now = self.get_clock().now()
        if (now - self._last_log_time).nanoseconds * 1e-9 >= 1.0:
            self.get_logger().info(
                'v: "%s" w:"%s" l_vel a: "%s" ex: "%s" er: "%s" '
                'r_vel ac: "%s" ex: "%s" er: "%s"' %
                (str(v), str(w), str(self._vel_l), str(v_left), str(err_l),
                 str(self._vel_r), str(v_right), str(err_r))
            )
            self._last_log_time = now

        cmd_l = self.wheel_command(v_left, err_l)
        cmd_r = self.wheel_command(v_right, err_r)
        self.publish_to_motors(cmd_l, cmd_r)

    def wheel_command(self, desired_velocity, velocity_error):
        if abs(desired_velocity) < 1e-3:
            return 0.0

        feedforward = (desired_velocity / self.MAX_WHEEL_VEL) * self.MAX_MOTOR_CMD
        correction = self.KP * velocity_error
        command = feedforward + correction

        if desired_velocity > 0.0:
            command = max(0.0, command)
        else:
            command = min(0.0, command)

        return command



    def publish_to_motors(self, left_speed, right_speed):
        left_speed  = max(-self.MAX_MOTOR_CMD, min(self.MAX_MOTOR_CMD, left_speed))
        right_speed = max(-self.MAX_MOTOR_CMD, min(self.MAX_MOTOR_CMD, right_speed))
        left_speed = self.slew_limit(left_speed, self._last_cmd_l)
        right_speed = self.slew_limit(right_speed, self._last_cmd_r)
        self._last_cmd_l = left_speed
        self._last_cmd_r = right_speed
        left_msg = Float64();  left_msg.data  = left_speed
        right_msg = Float64(); right_msg.data = right_speed
        self.left_motor_publisher.publish(left_msg)
        self.right_motor_publisher.publish(right_msg)

    def slew_limit(self, target, previous):
        delta = target - previous
        if delta > self.MAX_CMD_STEP:
            return previous + self.MAX_CMD_STEP
        if delta < -self.MAX_CMD_STEP:
            return previous - self.MAX_CMD_STEP
        return target


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = MotorControlServer()
            executor = MultiThreadedExecutor()
            executor.add_node(node)
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == "__main__":
    main()
