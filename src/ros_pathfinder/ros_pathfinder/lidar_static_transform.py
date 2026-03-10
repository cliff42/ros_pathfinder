import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

import math
from builtin_interfaces.msg import Time

HEADER_FRAME = 'base_link' # TF from base_link frame (which is the moving robot frame)
CHILD_FRAME = 'laser' # TF to laser frame (this id is the default from the sllidar_ros2 submodule)

class LidarStaticTransform(Node):
    def __init__(self):
        super().__init__("lidar_static_transform")
        self.x = 0.32
        self.y = 0.0
        self.z = 0.065 
        self.yaw = 0.0

        quat = (0.0, 0.0, math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0))

        tf = TransformStamped()
        tf.header.stamp = Time(sec=0, nanosec=0) # time 0 keeps the TF static
        tf.header.frame_id = HEADER_FRAME
        tf.child_frame_id = CHILD_FRAME
        tf.transform.translation.x = float(self.x)
        tf.transform.translation.y = float(self.y)
        tf.transform.translation.z = float(self.z)
        tf.transform.rotation.x = quat[0]
        tf.transform.rotation.y = quat[1]
        tf.transform.rotation.z = quat[2]
        tf.transform.rotation.w = quat[3]

        # only need to send the TF once (this is static)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster.sendTransform(tf)

        self.get_logger().info('published lidar transform: %s' % tf)



def main(args=None):
    try:
        with rclpy.init(args=args):
            static_tf = LidarStaticTransform()
            rclpy.spin(static_tf)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()