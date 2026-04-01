import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import numpy as np

import random
import math

class LandmarkPoint:
    def __init__(self, x, y, count):
        self.x = x
        self.y = y
        self.count = count


class LandmarkIdentification(Node):
    def __init__(self):
        super().__init__('localization')
        self.line_publisher = self.create_publisher(MarkerArray, 'ransac', 10) #change
        self.landmark_publisher = self.create_publisher(MarkerArray,'landmarks',10)
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

        self.landmark_db = []
        self.good_landmark_min_count = 5
        self.min_distance_same_landmark = 0.01 # TODO: validate

        self.ekf_initialized = False
        self.prev_odom_x = None
        self.prev_odom_y = None
        self.prev_odom_theta = None
        # TODO: these are just the robot parts for now, need to add the landmarks later
        # EFK system state (called X in SLAM doc)
        self.system_state_X = np.zeros((3, 1), dtype=float)
        # EFK covariance matrix (called P in SLAM doc)
        self.covariance_matrix_P = np.diag([
            0.05 ** 2,
            0.05 ** 2,
            (5.0 * math.pi / 180.0) ** 2
        ])

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

        # TODO: update ekf to be based on odom pose not laser
        if not self.ekf_initialized:
            self.system_state_X[0, 0] = laser_x
            self.system_state_X[1, 0] = laser_y
            self.system_state_X[2, 0] = laser_yaw

            self.prev_odom_x = laser_x
            self.prev_odom_y = laser_y
            self.prev_odom_theta = laser_yaw
            self.ekf_initialized = True
        else:
            delta_x = laser_x - self.prev_odom_x
            delta_y = laser_y - self.prev_odom_y
            delta_theta = self.wrap_angle(laser_yaw - self.prev_odom_theta)

            self.ekf_predict(delta_x, delta_y, delta_theta)

            self.prev_odom_x = laser_x
            self.prev_odom_y = laser_y
            self.prev_odom_theta = laser_yaw

        self.get_logger().info(
            f"EKF pose (from X): x={self.system_state_X[0,0]:.3f}, y={self.system_state_X[1,0]:.3f}, theta={self.system_state_X[2,0]:.3f}"
        )
        self.get_logger().info(
            f"P diag: {self.covariance_matrix_P[0,0]:.6f}, {self.covariance_matrix_P[1,1]:.6f}, {self.covariance_matrix_P[2,2]:.6f}"
        )

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

            # scan_x = math.cos(angle + laser_yaw) * point + laser_x
            # scan_y = math.sin(angle + laser_yaw) * point + laser_y

            # TODO: confirm this works
            # TODO: after switching to odom pose for ekf we need to transform for laser here
            ekf_x = self.system_state_X[0, 0]
            ekf_y = self.system_state_X[1, 0]
            ekf_theta = self.system_state_X[2, 0]

            scan_x = math.cos(angle + ekf_theta) * point + ekf_x
            scan_y = math.sin(angle + ekf_theta) * point + ekf_y

            range_dict[angle] = (scan_x,scan_y)
            angle_list.append(angle)
            angle += msg.angle_increment

        N = 100 #number of iterations for ransac (lines to check)
        S = 5 #number of samples to fit each line to
        D = 5 #degrees from initial sample point
        # X = 0.005 #max distance (in meters) that points must be from line
        X = 0.05
        CONSENSUS = 50 #minimum number of points within X meters for a line to be considered adequate
        lines, landmarks = self.ransac(angle_list,msg.angle_increment,range_dict,N,S,D,X,CONSENSUS)
        self.get_logger().info('Number of lines: %s' % len(lines))
        self.get_logger().info('Number of landmarks: %s' % len(landmarks))
        

        marker_array = MarkerArray()
        id = 0
        for line in lines:
            points_in_line = self.points_from_line(line[0], line[1], line[2], line[3], line[4], line[5])
            line_marker = Marker()
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.header.frame_id = 'odom'
            line_marker.type = Marker.LINE_STRIP
            line_marker.scale.x = 0.005
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

        landmark_array = MarkerArray()
        id = 0
        for landmark in landmarks:
            landmark_marker = Marker()
            landmark_marker.header.stamp = self.get_clock().now().to_msg()
            landmark_marker.header.frame_id = 'odom'
            landmark_marker.type = Marker.SPHERE
            landmark_marker.scale.x = 0.1
            landmark_marker.scale.y = 0.1
            landmark_marker.scale.z = 0.1
            landmark_marker.color.r = 1.0
            landmark_marker.color.g = 0.0
            landmark_marker.color.b = 0.0
            landmark_marker.color.a = 1.0
            landmark_marker.id = id
            id += 1

            landmark_marker.pose.position.x = float(landmark[0])
            landmark_marker.pose.position.y = float(landmark[1])
            landmark_marker.pose.position.z = 0.0
            landmark_marker.pose.orientation.w = 1.0
            landmark_array.markers.append(landmark_marker)
        
        self.landmark_publisher.publish(landmark_array)

        # check landmarks against global db
        for landmark in landmarks:
            closest_landmark_idx, dist = self.get_closest_landmark_from_db(landmark)

            if closest_landmark_idx == -1:
                lp = LandmarkPoint(landmark[0], landmark[1], 1)
                self.landmark_db.append(lp)
                continue
            
            if dist <= self.min_distance_same_landmark:
               self.landmark_db[closest_landmark_idx].count += 1
            else:
                lp = LandmarkPoint(landmark[0], landmark[1], 1)
                self.landmark_db.append(lp)

    def get_closest_landmark_from_db(self, landmark):
        min_dist = float("inf")
        idx = -1
        for i, l in enumerate(self.landmark_db):
            dist = math.sqrt((landmark[0] - l.x)**2 + (landmark[1] - l.y)**2)
            if dist < min_dist:
                min_dist = dist
                idx = i
        return idx, min_dist
    

    # TODO: use this with EKF
    def get_closest_good_landmark(self, landmark):
        min_dist = float("inf")
        idx = -1
        for i, l in enumerate(self.landmark_db):
            if l.count < self.good_landmark_min_count:
                continue

            dist = math.sqrt((landmark[0] - l.x)**2 + (landmark[1] - l.y)**2)
            if dist < min_dist:
                min_dist = dist
                idx = i
        return idx, min_dist
    

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def ekf_predict(self, delta_x, delta_y, delta_theta):
        self.system_state_X[0, 0] += delta_x
        self.system_state_X[1, 0] += delta_y
        self.system_state_X[2, 0] = self.wrap_angle(self.Xr[2, 0] + delta_theta)

        jacobian_pred_A = np.array([
            [1.0, 0.0, -delta_y],
            [0.0, 1.0,  delta_x],
            [0.0, 0.0,  1.0],
        ], dtype=float)

        sx = 0.05 * abs(delta_x) + 0.005
        sy = 0.05 * abs(delta_y) + 0.005
        st = 0.10 * abs(delta_theta) + 0.01

        Q = np.diag([sx * sx, sy * sy, st * st])

        self.covariance_matrix_P = jacobian_pred_A @ self.covariance_matrix_P @ jacobian_pred_A.T + Q


    def points_from_line(self, m, c, min_x, max_x,min_y,max_y):
        #set bounds of line in y direction if line near vertical (|m|>1)
        #set bounds of line in x direction if line near horizontal (|m|<=1)
        if abs(m)>1:
            start_point = ((min_y-c)/m,min_y)
            end_point  = ((max_y-c)/m,max_y) 
        else:
            start_point = (min_x, m * min_x + c)
            end_point = (max_x, m * max_x + c)   
        return [start_point, end_point]
    
    def angle_diff(self, a, b):
        return min(abs(a - b), (2*math.pi - abs(a - b)))

    def ransac(self,angle_list,angle_increment,range_dict,N,S,D,X,CONSENSUS):
        landmark_lines = []
        landmarks = []
        for i in range(N):
            rand_angle = random.choice(angle_list)

            angle_subset = [a for a in angle_list if self.angle_diff(a, rand_angle) < math.radians(D)]

            if len(angle_subset) < S:
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
                    readings_on_line.append([point_x, point_y])

            if len(readings_on_line) > CONSENSUS:
                new_m, new_c = self.least_squares(readings_on_line, len(readings_on_line))
                xs = [p[0] for p in readings_on_line]
                ys = [p[1] for p in readings_on_line]
                landmark_lines.append((new_m, new_c, min(xs), max(xs), min(ys), max(ys)))


        # TODO: to check (is this orthog to static origin or odom origin) - i think we want this to be orthog to true static origin
        #find point on landmark line that we can track (point orthog. to origin)
        for line in landmark_lines:
            m,c = line[0],line[1]
            b = -1/m #orthogonal line has slope -1/m with y intercept 0 (passes through origin)
            landmark_x = c/(b-m)
            landmark_y = b *landmark_x
            landmarks.append([landmark_x,landmark_y])
        


        return landmark_lines, landmarks

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