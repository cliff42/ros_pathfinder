import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from gpiozero import PhaseEnableMotor

from std_msgs.msg import Float64

left_motor = PhaseEnableMotor(20, 21)
right_motor = PhaseEnableMotor(23, 24)

COMMAND_EPSILON = 0.005


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.last_left_speed = None
        self.last_right_speed = None

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
        self.last_left_speed = self.apply_motor_command(
            left_motor,
            'left',
            msg.data,
            self.last_left_speed,
        )

    def listener_callback_right(self, msg):
        self.last_right_speed = self.apply_motor_command(
            right_motor,
            'right',
            msg.data,
            self.last_right_speed,
        )

    def apply_motor_command(self, motor, label, speed, last_speed):
        speed = max(-1.0, min(1.0, float(speed)))
        if abs(speed) < COMMAND_EPSILON:
            speed = 0.0

        if last_speed is not None and abs(speed - last_speed) < COMMAND_EPSILON:
            return last_speed

        if speed > 0.0:
            motor.forward(speed)
        elif speed < 0.0:
            motor.backward(abs(speed))
        else:
            motor.stop()

        self.get_logger().info(f'{label} motor command: {speed:.3f}')
        return speed



def main(args=None):
    try:
        with rclpy.init(args=args):
            motor_controller = MotorController()

            rclpy.spin(motor_controller)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
