import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, MapMetaData # https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import math

class OccupancyMapper(Node):
    def __init__(self):
        super().__init__('occupancy_mapper')
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 10)
        self.scan_subscriber = self.create_subscription(LaserScan,'scan', self.scan_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.resolution = 0.05 # m per cell
        self.width = 400 # num cells
        self.height = 400 # num cells

        self.origin_x = -(self.width * self.resolution) / 2.0
        self.origin_y = -(self.height * self.resolution) / 2.0

        # 0 for unoccupied, 1 for occupied, -1 for unknown
        self.grid = [-1] * (self.width * self.height) # init grid to be all unknown

    def scan_callback(self, msg: LaserScan):
        try:
            odom_to_laser_tf = self.tf_buffer.lookup_transform(
                'odom',                  
                msg.header.frame_id, # laser
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('could not look up odom->laser transform')
            return
        
        self.grid = [-1] * (self.width * self.height) # reset map
        laser_x = odom_to_laser_tf.transform.translation.x
        laser_y = odom_to_laser_tf.transform.translation.y
        qx = odom_to_laser_tf.transform.rotation.x
        qy = odom_to_laser_tf.transform.rotation.y
        qz = odom_to_laser_tf.transform.rotation.z
        qw = odom_to_laser_tf.transform.rotation.w
        laser_yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz * qz))

        angle = msg.angle_min

        for point in msg.ranges: 
            if math.isinf(point) or math.isnan(point):
                angle += msg.angle_increment
                continue

            if point > msg.range_max or point < msg.range_min:
                angle += msg.angle_increment
                continue

            scan_x = math.cos(angle + laser_yaw) * point + laser_x
            scan_y = math.sin(angle + laser_yaw) * point + laser_y

            scan_x = int((scan_x - self.origin_x) / self.resolution)
            scan_y = int((scan_y- self.origin_y) / self.resolution)

            start_x = int((laser_x - self.origin_x) / self.resolution)
            start_y = int((laser_y - self.origin_y) / self.resolution)

            self.bresenham(start_x, start_y, scan_x, scan_y)

            angle += msg.angle_increment

            

        self.publish_map()

    def bresenham(self, start_x, start_y, scan_x, scan_y):
        dx = abs(scan_x - start_x)
        dy = abs(scan_y - start_y)

        if dx > self.width / 2 or dy > self.height / 2:
            return

        x_inc = 1
        if scan_x < start_x:
            x_inc = -1
        
        y_inc = 1
        if scan_y < start_y:
            y_inc = -1

        x, y = start_x, start_y

        error = dx - dy

        while(True):
            if x == scan_x and y == scan_y:
                self.grid[scan_x + scan_y * self.width] = 100 # occupied
                break

            self.grid[x + y * self.width] = 0 # unoccupied   
            error2 = 2 * error # to avoid checking float

            # move in x dir
            if (error2 > -1 * dy):
                error -= dy
                x += x_inc
            
            # move in y dir
            if (error2 < dx):
                error += dx
                y += y_inc
            


    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid

        self.map_publisher.publish(msg)

def main(args=None):
    try:
        with rclpy.init(args=args):
            occupancy_mapper = OccupancyMapper()
            rclpy.spin(occupancy_mapper)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()