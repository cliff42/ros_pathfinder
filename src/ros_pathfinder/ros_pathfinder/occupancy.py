import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import math
import time


class OccupancyMapper(Node):
    def __init__(self):
        super().__init__('occupancy_mapper')
        self.map_frame = self.declare_parameter('map_frame', 'slam_odom').value
        self.slam_odom_topic = self.declare_parameter('slam_odom_topic', 'slam_odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.slam_odom_topic,
            self.odom_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        
        self.resolution = 0.05 # m per cell
        self.width = 400 # num cells
        self.height = 400 # num cells

        self.origin_x = -(self.width * self.resolution) / 2.0
        self.origin_y = -(self.height * self.resolution) / 2.0

        # log odds stores belief that cell is occupied
        self.log_odds = [0.0] * (self.width * self.height)
        self.log_odds_free = self.probability_to_log_odds(0.35)
        self.log_odds_occupied = self.probability_to_log_odds(0.70)
        self.log_odds_min = self.probability_to_log_odds(0.10)
        self.log_odds_max = self.probability_to_log_odds(0.95)
        self.free_threshold = self.probability_to_log_odds(0.40)
        self.occupied_threshold = self.probability_to_log_odds(0.65)

        self.grid = [-1] * (self.width * self.height)
        self.inflated_grid = [-1] * (self.width * self.height)
        self.inflation_radius_cells = 3

        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Publishing Map')
        self.publish_map()

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = self.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        
    def scan_callback(self, msg: LaserScan):
        start = time.time()

        if self.robot_x is None:
            self.get_logger().warn('waiting for slam_odom before mapping scans')
            return

        laser_pose = self.laser_pose_in_map(msg.header.frame_id)
        if laser_pose is None:
            return

        laser_x, laser_y, laser_yaw = laser_pose

        start_x, start_y = self.world_to_grid(laser_x, laser_y)
        if not self.in_bounds(start_x, start_y):
            self.get_logger().warn('laser origin is outside the occupancy grid')
            return

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

            # convert to grid coords
            scan_x, scan_y = self.world_to_grid(scan_x, scan_y)

            self.update_ray(start_x, start_y, scan_x, scan_y)

            angle += msg.angle_increment

        self.grid = self.log_odds_to_occupancy_grid()
        self.inflated_grid = self.inflate_occupied_cells(self.grid, self.inflation_radius_cells)
        end = time.time()
        duration = end - start
        self.get_logger().info('Duration :"%s" seconds' % duration)

    def update_ray(self, start_x, start_y, scan_x, scan_y):
        # bresenham's algorithm
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
            if not self.in_bounds(x, y):
                break

            if x == scan_x and y == scan_y:
                self.update_cell(x, y, self.log_odds_occupied)
                break

            self.update_cell(x, y, self.log_odds_free)
            error2 = 2 * error # to avoid checking float

            # move in x dir
            if (error2 > -1 * dy):
                error -= dy
                x += x_inc
            
            # move in y dir
            if (error2 < dx):
                error += dx
                y += y_inc

    def probability_to_log_odds(self, probability):
        return math.log(probability / (1.0 - probability))

    def log_odds_to_probability(self, log_odds):
        return 1.0 - (1.0 / (1.0 + math.exp(log_odds)))

    def world_to_grid(self, x, y):
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def grid_to_index(self, x, y):
        return x + y * self.width

    def in_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def update_cell(self, x, y, log_odds_update):
        if not self.in_bounds(x, y):
            return

        index = self.grid_to_index(x, y)
        updated = self.log_odds[index] + log_odds_update
        self.log_odds[index] = max(self.log_odds_min, min(self.log_odds_max, updated))

    def laser_pose_in_map(self, laser_frame):
        try:
            base_to_laser_tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                laser_frame,
                rclpy.time.Time()
            ) # from lidar_static_transform.py (offset of lidar to base link)
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('could not look up base link -> laser transform')
            return None

        laser_offset_x = base_to_laser_tf.transform.translation.x
        laser_offset_y = base_to_laser_tf.transform.translation.y
        q = base_to_laser_tf.transform.rotation
        laser_yaw_offset = self.yaw_from_quaternion(q.x, q.y, q.z, q.w)

        c = math.cos(self.robot_yaw)
        s = math.sin(self.robot_yaw)

        laser_x = self.robot_x + c * laser_offset_x - s * laser_offset_y
        laser_y = self.robot_y + s * laser_offset_x + c * laser_offset_y
        laser_yaw = self.robot_yaw + laser_yaw_offset

        return laser_x, laser_y, laser_yaw

    def yaw_from_quaternion(self, x, y, z, w):
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )

    # map log odds occupancy to 100 or 0 # TODO: eventually we can use the probabilities directly
    def log_odds_to_occupancy_grid(self):
        grid = []
        for cell_log_odds in self.log_odds:
            if cell_log_odds <= self.free_threshold:
                grid.append(0)
            elif cell_log_odds >= self.occupied_threshold:
                grid.append(100)
            else:
                grid.append(-1)
        return grid

    def inflate_occupied_cells(self, grid, radius):
        inflated_grid = list(grid)
        occupied_indices = [i for i, value in enumerate(grid) if value >= 65]

        for index in occupied_indices:
            center_x = index % self.width
            center_y = index // self.width

            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if dx * dx + dy * dy > radius * radius:
                        continue

                    x = center_x + dx
                    y = center_y + dy
                    if self.in_bounds(x, y):
                        inflated_grid[self.grid_to_index(x, y)] = 100

        return inflated_grid

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # msg.data = self.grid
        msg.data = self.inflated_grid

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
