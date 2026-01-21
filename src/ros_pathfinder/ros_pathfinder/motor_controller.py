import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Float64

class MotorController(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.left_motor_subscriber = self.create_subscription(
            Float64,
            'left_motor',
            self.listener_callback,
            10)
        self.right_motor_subscriber = self.create_subscription(
            Float64,
            'right_motor',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # TODO: send msg to hardware
        self.get_logger().info('msg: "%s"' % msg.data)



def main(args=None):
    try:
        with rclpy.init(args=args):
            motor_controller = MotorController()

            rclpy.spin(motor_controller)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()