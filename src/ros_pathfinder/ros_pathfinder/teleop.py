#!/usr/bin/env python3

import sys
import threading
from select import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import termios
import tty


MOVE_BINDINGS = {
    "w": ( 1.0,  1.0),
    "a": ( -1.0, 1.0),
    "s": (-1.0, -1.0),
    "d": (1.0,  -1.0),
    " ": ( 0.0,  0.0),
}

MOTOR_SPEED = 0.2
LINEAR_SPEED = 0.12
ANGULAR_SPEED = 0.6
PUBLISH_PERIOD_SEC = 0.05


class TeleopNode(Node):
    DIRECT_MOTOR = 'direct_motor'
    CMD_VEL = 'cmd_vel'

    def __init__(self, mode=DIRECT_MOTOR):
        super().__init__("teleop_node" if mode == self.DIRECT_MOTOR else "cmd_vel_teleop")
        self.mode = mode

        self.stop_event = threading.Event()
        self.term_settings = termios.tcgetattr(sys.stdin)

        self.left_pub = None
        self.right_pub = None
        self.cmd_vel_pub = None
        self.current_left = 0.0
        self.current_right = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0

        if self.mode == self.DIRECT_MOTOR:
            self.left_pub = self.create_publisher(Float64, "left_motor", 10)
            self.right_pub = self.create_publisher(Float64, "right_motor", 10)
            self.get_logger().warn(
                "Direct motor teleop publishes left_motor/right_motor. "
                "Do not run it at the same time as controller_node."
            )
        else:
            self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
            self.get_logger().info(
                "cmd_vel teleop started. Use this while the navigation stack is running."
            )

        self.thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.thread.start()

        self.get_logger().info("Keyboard teleop started (W/A/S/D, space to stop, Ctrl-C to quit).")

    def destroy_node(self):
        self.stop_event.set()
        self.publish_stop()
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_settings)
        except Exception:
            pass
        return super().destroy_node()

    def get_key(self, timeout_sec: float = 0.1) -> str:
        tty.setcbreak(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout_sec)
        key = sys.stdin.read(1) if rlist else ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_settings)
        return key

    def set_command(self, key: str) -> None:
        if self.mode == self.DIRECT_MOTOR:
            left, right = MOVE_BINDINGS[key]
            self.current_left = left * MOTOR_SPEED
            self.current_right = right * MOTOR_SPEED
            return

        if key == "w":
            self.current_linear = LINEAR_SPEED
            self.current_angular = 0.0
        elif key == "s":
            self.current_linear = -LINEAR_SPEED
            self.current_angular = 0.0
        elif key == "a":
            self.current_linear = 0.0
            self.current_angular = ANGULAR_SPEED
        elif key == "d":
            self.current_linear = 0.0
            self.current_angular = -ANGULAR_SPEED
        elif key == " ":
            self.current_linear = 0.0
            self.current_angular = 0.0

    def publish_to_motors(self, left: float, right: float) -> None:
        lm = Float64()
        rm = Float64()
        lm.data = float(left)
        rm.data = float(right)
        self.left_pub.publish(lm)
        self.right_pub.publish(rm)

    def publish_cmd_vel(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def publish_current(self) -> None:
        if self.mode == self.DIRECT_MOTOR:
            self.publish_to_motors(self.current_left, self.current_right)
        else:
            self.publish_cmd_vel(self.current_linear, self.current_angular)

    def publish_stop(self) -> None:
        self.current_left = 0.0
        self.current_right = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        try:
            self.publish_current()
        except Exception:
            pass

    def keyboard_loop(self):
        self.publish_stop()

        while rclpy.ok() and not self.stop_event.is_set():
            key = self.get_key(PUBLISH_PERIOD_SEC)
            if key in MOVE_BINDINGS:
                self.set_command(key)
                self.publish_current()
            elif self.mode == self.CMD_VEL:
                self.publish_current()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode(TeleopNode.DIRECT_MOTOR)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_cmd_vel(args=None):
    rclpy.init(args=args)
    node = TeleopNode(TeleopNode.CMD_VEL)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
