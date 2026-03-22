import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class LandmarkIdentification(Node):
    def __init__(self):
        super().__init__('landmark_identification')

        self.line_publisher = self.create_publisher(MarkerArray, 'ransac', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # RANSAC parameters
        self.N = 100
        self.S = 2                    # 2 points define a line in ax+by+c=0 form
        self.D_FIT_DEG = 5.0
        self.D_VOTE_DEG = 10.0
        self.X = 0.05                 # max perpendicular distance in meters
        self.CONSENSUS = 15
        self.MIN_SEGMENT_LENGTH = 0.30

    def scan_callback(self, msg: LaserScan):
        try:
            odom_to_laser_tf = self.tf_buffer.lookup_transform(
                'odom',
                msg.header.frame_id,
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('could not look up odom->laser transform')
            return

        laser_x = odom_to_laser_tf.transform.translation.x
        laser_y = odom_to_laser_tf.transform.translation.y

        qx = odom_to_laser_tf.transform.rotation.x
        qy = odom_to_laser_tf.transform.rotation.y
        qz = odom_to_laser_tf.transform.rotation.z
        qw = odom_to_laser_tf.transform.rotation.w

        laser_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        scan_points = []
        angle = msg.angle_min

        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            if r > msg.range_max or r < msg.range_min:
                angle += msg.angle_increment
                continue

            x = math.cos(angle + laser_yaw) * r + laser_x
            y = math.sin(angle + laser_yaw) * r + laser_y

            scan_points.append({
                'angle': angle,
                'x': x,
                'y': y,
            })

            angle += msg.angle_increment

        if len(scan_points) < self.S:
            return

        segments = self.ransac(scan_points)

        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for marker_id, seg in enumerate(segments):
            p1, p2 = seg

            line_marker = Marker()
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.header.frame_id = 'odom'
            line_marker.ns = 'ransac_lines'
            line_marker.id = marker_id
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0

            pt1 = Point()
            pt1.x = p1[0]
            pt1.y = p1[1]
            pt1.z = 0.0

            pt2 = Point()
            pt2.x = p2[0]
            pt2.y = p2[1]
            pt2.z = 0.0

            line_marker.points.append(pt1)
            line_marker.points.append(pt2)

            marker_array.markers.append(line_marker)

        self.line_publisher.publish(marker_array)

    def angle_diff(self, a, b):
        d = abs(a - b)
        return min(d, 2.0 * math.pi - d)

    def ransac(self, scan_points):
        segments = []
        active_points = scan_points[:]

        fit_window = math.radians(self.D_FIT_DEG)
        vote_window = math.radians(self.D_VOTE_DEG)

        trials = 0
        while (
            trials < self.N
            and len(active_points) >= self.CONSENSUS
            and len(active_points) >= self.S
        ):
            trials += 1

            seed = random.choice(active_points)
            seed_angle = seed['angle']

            fit_subset = [
                p for p in active_points
                if self.angle_diff(p['angle'], seed_angle) < fit_window
            ]

            vote_subset = [
                p for p in active_points
                if self.angle_diff(p['angle'], seed_angle) < vote_window
            ]

            if len(fit_subset) < self.S or len(vote_subset) < self.CONSENSUS:
                continue

            sample_points = random.sample(fit_subset, self.S)
            p1 = (sample_points[0]['x'], sample_points[0]['y'])
            p2 = (sample_points[1]['x'], sample_points[1]['y'])

            line = self.line_from_two_points(p1, p2)
            if line is None:
                continue

            a, b, c = line

            inliers = []
            for p in vote_subset:
                distance = abs(a * p['x'] + b * p['y'] + c)  # normalized line
                if distance < self.X:
                    inliers.append(p)

            if len(inliers) < self.CONSENSUS:
                continue

            refined_line = self.fit_line_tls(inliers)
            if refined_line is None:
                continue

            segment = self.segment_from_inliers(refined_line, inliers)
            if segment is None:
                continue

            segments.append(segment)

            inlier_ids = {id(p) for p in inliers}
            active_points = [p for p in active_points if id(p) not in inlier_ids]

        return segments

    def line_from_two_points(self, p1, p2):
        """
        Return normalized (a, b, c) for line ax + by + c = 0 through p1 and p2.
        """
        x1, y1 = p1
        x2, y2 = p2

        dx = x2 - x1
        dy = y2 - y1

        if math.hypot(dx, dy) < 1e-8:
            return None

        # normal vector to direction (dx, dy) is (dy, -dx) or (-dy, dx)
        a = y1 - y2
        b = x2 - x1
        c = x1 * y2 - x2 * y1

        norm = math.sqrt(a * a + b * b)
        if norm < 1e-8:
            return None

        a /= norm
        b /= norm
        c /= norm

        return a, b, c

    def fit_line_tls(self, inliers):
        """
        Fit ax + by + c = 0 using total least squares / PCA.
        Returns normalized (a, b, c).
        """
        n = len(inliers)
        if n < 2:
            return None

        mean_x = sum(p['x'] for p in inliers) / n
        mean_y = sum(p['y'] for p in inliers) / n

        sxx = 0.0
        syy = 0.0
        sxy = 0.0

        for p in inliers:
            dx = p['x'] - mean_x
            dy = p['y'] - mean_y
            sxx += dx * dx
            syy += dy * dy
            sxy += dx * dy

        # Direction of best-fit line is principal eigenvector of covariance.
        # For 2x2 symmetric matrix, angle of principal axis:
        theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)

        # Direction along the line
        ux = math.cos(theta)
        uy = math.sin(theta)

        # Normal to the line
        a = -uy
        b = ux
        c = -(a * mean_x + b * mean_y)

        norm = math.sqrt(a * a + b * b)
        if norm < 1e-8:
            return None

        a /= norm
        b /= norm
        c /= norm

        return a, b, c

    def segment_from_inliers(self, line, inliers):
        """
        Build a finite segment by projecting inliers onto the fitted line and
        taking min/max projection.
        """
        a, b, c = line

        # Choose one point on the line
        # Since ax + by + c = 0 and (a,b) is normal, a point on the line is (-ac, -bc)
        x0 = -a * c
        y0 = -b * c

        # Direction vector along line is perpendicular to normal
        ux = b
        uy = -a

        projections = []
        for p in inliers:
            px = p['x'] - x0
            py = p['y'] - y0
            t = px * ux + py * uy
            projections.append(t)

        if not projections:
            return None

        t_min = min(projections)
        t_max = max(projections)

        p1 = (x0 + t_min * ux, y0 + t_min * uy)
        p2 = (x0 + t_max * ux, y0 + t_max * uy)

        seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if seg_len < self.MIN_SEGMENT_LENGTH:
            return None

        return p1, p2


def main(args=None):
    try:
        rclpy.init(args=args)
        landmark_identification = LandmarkIdentification()
        rclpy.spin(landmark_identification)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()