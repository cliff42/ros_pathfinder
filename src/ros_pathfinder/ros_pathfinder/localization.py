import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import random
import math

class LandmarkIdentification(Node):
    def __init__(self):
        super().__init__('localization')
        self.line_publisher = self.create_publisher(MarkerArray, 'ransac', 10) #change
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

        angle_list = []
        range_dict = {}
        if len(msg.ranges) == 0:
            return
        for point in msg.ranges: 
            if math.isinf(point) or math.isnan(point):
                angle += msg.angle_increment
                continue

            if point > msg.range_max or point < msg.range_min:
                angle += msg.angle_increment
                continue

            scan_x = math.cos(angle + laser_yaw) * point + laser_x
            scan_y = math.sin(angle + laser_yaw) * point + laser_y

            range_dict[angle] = (scan_x,scan_y)
            angle_list.append(angle)
            angle += msg.angle_increment

        N = 100 #number of iterations for ransac (lines to check)
        S = 5 #number of samples to fit each line to
        D = 5 #degrees from initial sample point
        X = 0.1 #max distance (in meters) that points must be from line
        CONSENSUS = 100 #minimum number of points within X meters for a line to be considered adequate
        landmarks = self.ransac(angle_list,msg.angle_increment,range_dict,N,S,D,X,CONSENSUS)

        marker_array = MarkerArray()
        id = 0
        for landmark in landmarks:
            points_in_line = self.points_from_line(landmark[0], landmark[1])
            line_marker = Marker()
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.header.frame_id = 'odom'
            line_marker.type = Marker.LINE_STRIP
            line_marker.scale.x = 0.1
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0
            line_marker.id = id
            id += 1
            for point in points_in_line:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0.0
                line_marker.points.append(p)
            marker_array.markers.append(line_marker)


        self.line_publisher.publish(marker_array)


    def points_from_line(self, m, c):
        points = []
        for x in range(self.width):
            x_coord = -10 + x*self.resolution
            y_coord = m * x_coord + c
            points.append((x_coord, y_coord))
        return points
    
    def angle_diff(self, a, b):
        return min(abs(a - b), (2*math.pi - abs(a - b)))

    def ransac(self,angle_list,angle_increment,range_dict,N,S,D,X,CONSENSUS):
        landmarks = []
        for i in range(N):
            rand_angle = random.choice(angle_list)

            angle_subset = [a for a in angle_list if self.angle_diff(a, rand_angle) < math.radians(D)]

            if len(angle_subset) < S:
                self.get_logger().info('HERE: %s' % angle_subset)
                continue

            line_point_list = random.sample(angle_subset, S)
            point_list = [(range_dict[a][0],range_dict[a][1]) for a in line_point_list]

            m, c = self.least_squares(point_list, S)
            A = m
            B = -1
            C = c

            denom = math.sqrt(A*A + B*B)
            
            readings_on_line = []
            for a in angle_list:
                point_x = range_dict[a][0]
                point_y = range_dict[a][1]

                distance = abs(A * point_x + B * point_y + C) / denom
                if distance < X:
                    readings_on_line.append((point_x, point_y))

            if len(readings_on_line) > CONSENSUS:
                new_m, new_c = self.least_squares(readings_on_line, len(readings_on_line))
                landmarks.append((new_m, new_c))

        return landmarks

    def least_squares(self,list, S):
        sum_x = 0
        sum_x2 = 0
        sum_y = 0
        sum_xy = 0
        for a in list:
            scan_x = a[0]
            scan_y = a[1]
            sum_x += scan_x
            sum_x2 += scan_x * scan_x
            sum_y += scan_y
            sum_xy += scan_x * scan_y

        m = ((S * sum_xy) - (sum_x * sum_y))  / ((S * sum_x2) - (sum_x * sum_x))
        c = (sum_y - (m * sum_x)) / S

        return m, c










            


    def publish_map(self):
        msg = LandmarkIdentification()
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
            landmark_identification = LandmarkIdentification()
            rclpy.spin(landmark_identification)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()