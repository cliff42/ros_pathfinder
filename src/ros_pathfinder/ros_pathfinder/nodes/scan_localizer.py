import math
from typing import Optional
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from ros_pathfinder.localization.scan_projection import scan_to_points
from ros_pathfinder.localization.icp_scan_matcher import ICPScanMatcher
from ros_pathfinder.localization.scan_localization import ScanLocalization, ScanLocalizationConfig
from ros_pathfinder.geometry.pose2d import Pose2d

import numpy as np

class ScanLocalizer(Node):

    SCAN_TOPIC = "scan"

    MAP_FRAME = "map"
    ODOM_FRAME = "odom"
    BASE_FRAME = "base_link"

    def __init__(self):
        super().__init__("scan_localizer")

        self._transform_buffer = Buffer()
        self._transform_listener = TransformListener(self._transform_buffer, self)
        
        # TODO: add config values for these
        icp_scan_matcher = ICPScanMatcher(
            max_iterations=30,
            max_correspondence_dist_m=0.25,
            min_match_count=30,
            translation_tolerance_m=0.0005,
            rotation_tolerance_rad=0.001,
        )
        config = ScanLocalizationConfig(
            min_translation_before_match_m=0.02,
            min_rotation_before_match_rad=0.02,
            max_accepted_rmse_m=0.08,
            min_inlier_ratio=0.35,
            max_translation_correction_m=0.20,
            max_rotation_correction_rad=0.20,
            keyframe_translation_threshold_m=0.15,
            keyframe_rotation_threshold_rad=0.15,
        )

        self._scan_localization = ScanLocalization(icp_scan_matcher, config)

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC,
            self.lidar_callback,
            qos_profile_sensor_data
        )

        self.transform_broadcaster = TransformBroadcaster(self)

    def lidar_callback(self, msg: LaserScan) -> None:
        scan_ts = Time.from_msg(msg.header.stamp)

        try:
            # get the odom transform closest to the scan time
            odom_to_base = self._transform_buffer.lookup_transform(self.ODOM_FRAME, self.BASE_FRAME, scan_ts)
            base_to_laser = self._transform_buffer.lookup_transform(self.BASE_FRAME, msg.header.frame_id, scan_ts)
        except Exception as e:
            self.get_logger().warning(f"cannot get buffered transforms in lidar_callback: {e}")
            return

        points_in_laser = scan_to_points(
            ranges=msg.ranges,
            angle_min_rad=msg.angle_min,
            angle_increment_rad=msg.angle_increment,
            range_min_m=msg.range_min,
            range_max_m=msg.range_max,
        )

        laser_pose_in_base = self._pose_from_transform(base_to_laser)

        points_in_base = laser_pose_in_base.transform_points(points_in_laser)

        odom_pose = self._pose_from_transform(odom_to_base)

        localization_update = self._scan_localization.update(
            current_points_base=points_in_base,
            current_odom_pose=odom_pose,
            timestamp_ns=self._timestamp_to_ns(msg.header.stamp)
        )

        # broadcast the map -> odom transform
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.MAP_FRAME
        transform.child_frame_id = self.ODOM_FRAME

        transform.transform.translation.x = localization_update.map_to_odom.x_m
        transform.transform.translation.y = localization_update.map_to_odom.y_m
        transform.transform.translation.z = 0.0

        half_yaw = localization_update.map_to_odom.yaw_rad / 2.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(half_yaw)
        transform.transform.rotation.w = math.cos(half_yaw)

        self.transform_broadcaster.sendTransform(transform)  

    def destroy_node(self):
        return super().destroy_node()

    def _pose_from_transform(self, transform: TransformStamped) -> Pose2d:
        translation = transform.transform.translation
        rot = transform.transform.rotation

        # only need rotation around the z axis
        yaw = math.atan2(
            2.0 * (rot.w * rot.z + rot.x * rot.y),
            1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z)
        )

        return Pose2d(x_m=translation.x, y_m=translation.y, yaw_rad=yaw)

    def _timestamp_to_ns(self, ts: Time) -> int:
        return(int(ts.sec) * 1000000000 + int(ts.nanosec))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = ScanLocalizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()