import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String, Float64MultiArray

class MinimalSubscriber(Node):
    distance_1 = 0
    delta_angle = 0
    prev_angle = 0
    count = 0

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'test_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        angle = int(msg.data[0])
        self.get_logger().info('angle (2): "%s"' % msg.data[1])
        if self.prev_angle > 270 and angle < 90:
            self.delta_angle = angle + 360 - self.prev_angle
        elif self.prev_angle < 90 and angle > 270:
            self.delta_angle = -1 * (angle + 360 - self.prev_angle)
        else:
            self.delta_angle = angle - self.prev_angle
        
        self.distance_1 += (self.delta_angle / 7) * (3.14 / 180) * 4 * 0.0254
        self.get_logger().info('distance (1): "%s"' % self.distance_1)
        self.prev_angle = angle


def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_subscriber = MinimalSubscriber()

            rclpy.spin(minimal_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()