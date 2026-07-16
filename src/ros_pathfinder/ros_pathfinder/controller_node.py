#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
import threading


class MotorControlServer(Node):
    WHEELBASE     = 0.55        # distance between wheels
    WHEEL_RADIUS  = 4 * 0.0254
    MAX_WHEEL_VEL = 0.5         # TODO: measure based on hw
    MAX_MOTOR_CMD = 0.2
    MIN_MOTOR_CMD = 0.08        # overcome motor static friction/stiction
    MIN_WHEEL_VEL = 0.01        # m/s; ignore tiny wheel requests
    MAX_CMD_STEP  = 0.01        # duty-cycle change per control tick
    KP            = 0.4         # motor command per m/s of wheel-speed error
    LOG_PERIOD    = 1.0         # seconds
    MOTOR_CMD_EPSILON = 0.002

    def __init__(self):
        super().__init__('controller_node')
        self.cb_group = ReentrantCallbackGroup()
        self.left_motor_publisher  = self.create_publisher(Float64, 'left_motor',  10)
        self.right_motor_publisher = self.create_publisher(Float64, 'right_motor', 10)
        self.data_lock = threading.Lock()
        self.use_odom_feedback = bool(
            self.declare_parameter('use_odom_feedback', False).value
        )

        self.create_subscription(Odometry, 'raw_odom', self._odom_cb, 10,
                                 callback_group=self.cb_group)

        self._cmd_vel = None
        self._last_cmd_time = None
        self.CMD_VEL_TIMEOUT = 0.5  # seconds
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10,
                                 callback_group=self.cb_group)

        # Measured wheel velocities (m/s), updated by _odom_cb
        self._vel_l = 0.0
        self._vel_r = 0.0
        self._last_cmd_l = 0.0
        self._last_cmd_r = 0.0
        self._last_published_l = None
        self._last_published_r = None
        self._motors_stopped = True
        self._last_log_time = self.get_clock().now()
        self.get_logger().info(
            f'controller mode: use_odom_feedback={self.use_odom_feedback}'
        )

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
        if self._cmd_vel is None or self._last_cmd_time is None:
            return

        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.CMD_VEL_TIMEOUT: # stop motors if no msg recieved for timeout seconds
            self.publish_stop_once()
            return

        # 
        v = self._cmd_vel.linear.x
        w = self._cmd_vel.angular.z
        if abs(v) < 1e-3 and abs(w) < 1e-3:
            self.publish_stop_once()
            return

        # desired linear speeds
        v_left  = v - w * self.WHEELBASE / 2.0
        v_right = v + w * self.WHEELBASE / 2.0

        if self.use_odom_feedback:
            with self.data_lock:
                err_l = v_left - self._vel_l
                err_r = v_right - self._vel_r
        else:
            err_l = 0.0
            err_r = 0.0

        cmd_l = self.wheel_command(v_left, err_l)
        cmd_r = self.wheel_command(v_right, err_r)
        actual_l, actual_r = self.publish_to_motors(cmd_l, cmd_r)

        if self.should_log():
            self.get_logger().info(
                f'cmd_vel=(v={v:.3f}, w={w:.3f}), '
                f'wheel_des=(l={v_left:.3f}, r={v_right:.3f}), '
                f'wheel_meas=(l={self._vel_l:.3f}, r={self._vel_r:.3f}), '
                f'wheel_err=(l={err_l:.3f}, r={err_r:.3f}), '
                f'motor_cmd=(l={actual_l:.3f}, r={actual_r:.3f})'
            )

    def wheel_command(self, desired_velocity, velocity_error):
        if abs(desired_velocity) < self.MIN_WHEEL_VEL:
            return 0.0

        feedforward = (desired_velocity / self.MAX_WHEEL_VEL) * self.MAX_MOTOR_CMD
        correction = self.KP * velocity_error
        command = feedforward + correction

        if desired_velocity > 0.0:
            command = max(0.0, command)
            if command > 0.0:
                command = max(self.MIN_MOTOR_CMD, command)
        else:
            command = min(0.0, command)
            if command < 0.0:
                command = min(-self.MIN_MOTOR_CMD, command)

        return self.clamp(command, -self.MAX_MOTOR_CMD, self.MAX_MOTOR_CMD)


    def publish_stop_once(self):
        if self._motors_stopped:
            return
        self.publish_to_motors(0.0, 0.0, immediate=True, force=True)
        self._motors_stopped = True

    def publish_to_motors(self, left_speed, right_speed, immediate=False, force=False):
        left_speed = self.clamp(left_speed, -self.MAX_MOTOR_CMD, self.MAX_MOTOR_CMD)
        right_speed = self.clamp(right_speed, -self.MAX_MOTOR_CMD, self.MAX_MOTOR_CMD)

        if immediate:
            self._last_cmd_l = left_speed
            self._last_cmd_r = right_speed
        else:
            left_speed = self.slew_limit(left_speed, self._last_cmd_l)
            right_speed = self.slew_limit(right_speed, self._last_cmd_r)
            self._last_cmd_l = left_speed
            self._last_cmd_r = right_speed

        if self.same_as_last_publish(left_speed, right_speed) and not force:
            return left_speed, right_speed

        left_msg = Float64();  left_msg.data  = left_speed
        right_msg = Float64(); right_msg.data = right_speed
        self.left_motor_publisher.publish(left_msg)
        self.right_motor_publisher.publish(right_msg)
        self._last_published_l = left_speed
        self._last_published_r = right_speed
        self._motors_stopped = abs(left_speed) < self.MOTOR_CMD_EPSILON and abs(right_speed) < self.MOTOR_CMD_EPSILON
        return left_speed, right_speed

    def slew_limit(self, target, current):
        if abs(target) < self.MOTOR_CMD_EPSILON:
            return 0.0
        if abs(current) < self.MOTOR_CMD_EPSILON:
            return math.copysign(min(abs(target), self.MIN_MOTOR_CMD), target)
        if target * current < 0.0:
            return 0.0

        delta = self.clamp(target - current, -self.MAX_CMD_STEP, self.MAX_CMD_STEP)
        return current + delta

    def same_as_last_publish(self, left_speed, right_speed):
        if self._last_published_l is None or self._last_published_r is None:
            return False
        return (
            abs(left_speed - self._last_published_l) < self.MOTOR_CMD_EPSILON
            and abs(right_speed - self._last_published_r) < self.MOTOR_CMD_EPSILON
        )

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def should_log(self):
        now = self.get_clock().now()
        elapsed = (now - self._last_log_time).nanoseconds * 1e-9
        if elapsed < self.LOG_PERIOD:
            return False
        self._last_log_time = now
        return True


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
