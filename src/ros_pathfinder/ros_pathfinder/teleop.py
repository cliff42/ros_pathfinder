#!/usr/bin/env python3

import sys
import threading
from select import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import termios
import tty


MOVE_BINDINGS = {
    "w": ( 1.0,  1.0),
    "a": ( 1.0, -1.0),
    "s": (-1.0, -1.0),
    "d": (-1.0,  1.0),
    " ": ( 0.0,  0.0),
}

MOTOR_SPEED = 0.2


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")

        self.left_pub = self.create_publisher(Float64, "left_motor", 10)
        self.right_pub = self.create_publisher(Float64, "right_motor", 10)

        self.stop_event = threading.Event()
        self.term_settings = termios.tcgetattr(sys.stdin)

        self.thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.thread.start()

        self.get_logger().info("Keyboard teleop started (W/A/S/D, space to stop, Ctrl-C to quit).")

    def destroy_node(self):
        self.stop_event.set()
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

    def publish_to_motors(self, left: float, right: float) -> None:
        lm = Float64()
        rm = Float64()
        lm.data = float(left)
        rm.data = float(right)
        self.left_pub.publish(lm)
        self.right_pub.publish(rm)

    def keyboard_loop(self):
        self.publish_to_motors(0.0, 0.0)

        while rclpy.ok() and not self.stop_event.is_set():
            key = self.get_key(0.1)
            if not key:
                continue

            if key in MOVE_BINDINGS:
                left, right = MOVE_BINDINGS[key]
                self.publish_to_motors(left * MOTOR_SPEED, right * MOTOR_SPEED)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
