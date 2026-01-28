import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from gpiozero import PhaseEnableMotor

from std_msgs.msg import Float64

left_motor = PhaseEnableMotor(20, 21)
right_motor = PhaseEnableMotor(23, 24)

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.left_motor_subscriber = self.create_subscription(
            Float64,
            'left_motor',
            self.listener_callback_left,
            10)
        self.right_motor_subscriber = self.create_subscription(
            Float64,
            'right_motor',
            self.listener_callback_right,
            10)

    def listener_callback_left(self, msg):
        speed = msg.data
        if speed < 0:
            left_motor.backward(abs(speed))
        else:
            left_motor.forward(abs(speed))
        self.get_logger().info('msg: "%s"' % msg.data)

    def listener_callback_right(self, msg):
        speed = msg.data
        if speed < 0:
            right_motor.backward(abs(speed))
        else:
            right_motor.forward(abs(speed))
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