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
    KP            = 1.5         # TODO: measure based on hw

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

        self.get_logger().info('v: "%s" w:"%s" l_vel a: "%s" ex: "%s" er: "%s" r_vel ac: "%s" ex: "%s" er: "%s"' % (str(v),str(w),str(self._vel_l),str(v_left),str(err_l),str(self._vel_r),str(v_right),str(err_r)))

        cmd_l = v_left  / self.MAX_WHEEL_VEL + self.KP * err_l / self.MAX_WHEEL_VEL
        cmd_r = v_right / self.MAX_WHEEL_VEL + self.KP * err_r / self.MAX_WHEEL_VEL
        self.publish_to_motors(cmd_l, cmd_r)



    def publish_to_motors(self, left_speed, right_speed):
        left_speed  = max(-0.2, min(0.2, left_speed))
        right_speed = max(-0.2, min(0.2, right_speed))
        left_msg = Float64();  left_msg.data  = left_speed
        right_msg = Float64(); right_msg.data = right_speed
        self.left_motor_publisher.publish(left_msg)
        self.right_motor_publisher.publish(right_msg)


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
