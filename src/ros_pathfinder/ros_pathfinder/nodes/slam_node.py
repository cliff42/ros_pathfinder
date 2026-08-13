import math
from typing import Optional
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid

from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from ros_pathfinder.localization.scan_projection import scan_to_observation
from ros_pathfinder.localization.icp_scan_matcher import ICPScanMatcher
from ros_pathfinder.localization.scan_localization import ScanLocalization, ScanLocalizationConfig
from ros_pathfinder.mapping.occupancy_grid import OccupancyGrid2d, OccupancyGridConfig
from ros_pathfinder.geometry.pose2d import Pose2d

class SlamNode(Node):

    SCAN_TOPIC = "scan"
    MAP_TOPIC = "map" # to publish the occupancy grid

    MAP_FRAME = "map"
    ODOM_FRAME = "odom"
    BASE_FRAME = "base_link"

    def __init__(self):
        super().__init__("slam_node")

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
        scan_localization_config = ScanLocalizationConfig(
            min_translation_before_match_m=0.02,
            min_rotation_before_match_rad=0.02,
            max_accepted_rmse_m=0.08,
            min_inlier_ratio=0.35,
            max_translation_correction_m=0.20,
            max_rotation_correction_rad=0.20,
            keyframe_translation_threshold_m=0.15,
            keyframe_rotation_threshold_rad=0.15,
            submap_max_keyframes=10,
            submap_grid_size_m=0.03
        )

        self._scan_localization = ScanLocalization(icp_scan_matcher=icp_scan_matcher, config=scan_localization_config)

        self._occupancy_grid_config = OccupancyGridConfig(
            resolution_m=0.05,
            width=400,
            height=400,
            origin_x_m=-10.0,
            origin_y_m=-10.0,
            hit_probability=0.7,
            miss_probability=0.35,
            min_probability=0.10,
            max_probability=0.95,
            free_probability_threshold=0.40,
            occupied_probability_threshold=0.65
        )
        self._occupancy_grid = OccupancyGrid2d(config=self._occupancy_grid_config)
        self._map_load_time = self.get_clock().now().to_msg()

        self.publish_rate_hz = 1.0 # TODO: put in config

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC,
            self.lidar_callback,
            qos_profile_sensor_data
        )

        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            self.MAP_TOPIC,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
        )

        self.transform_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(
            (1.0 / self.publish_rate_hz),
            self.map_timer_callback
        )

    def lidar_callback(self, msg: LaserScan) -> None:
        scan_ts = Time.from_msg(msg.header.stamp)

        try:
            # get the odom transform closest to the scan time
            odom_to_base = self._transform_buffer.lookup_transform(self.ODOM_FRAME, self.BASE_FRAME, scan_ts)
            base_to_laser = self._transform_buffer.lookup_transform(self.BASE_FRAME, msg.header.frame_id, scan_ts)
        except Exception as e:
            self.get_logger().warning(f"cannot get buffered transforms in lidar_callback: {e}")
            return

        laser_pose_in_base = self._pose_from_transform(base_to_laser)

        scan_observation = scan_to_observation(
            ranges=msg.ranges,
            angle_min_rad=msg.angle_min,
            angle_increment_rad=msg.angle_increment,
            range_min_m=msg.range_min,
            range_max_m=msg.range_max,
            laser_pose_in_base=laser_pose_in_base
        )

        odom_pose = self._pose_from_transform(odom_to_base)

        localization_update = self._scan_localization.update(
            current_scan=scan_observation,
            current_odom_pose=odom_pose,
            timestamp_ns=self._timestamp_to_ns(msg.header.stamp)
        )

        if localization_update.created_keyframe is not None:
            self._occupancy_grid.integrate_keyframe(
                keyframe=localization_update.created_keyframe, 
                map_to_base=localization_update.map_to_base
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

    def map_timer_callback(self) -> None:
        msg = self._occupancy_grid_msg()
        self.map_publisher.publish(msg)

    def destroy_node(self):
        return super().destroy_node()

    def _occupancy_grid_msg(self) -> OccupancyGrid:
        config = self._occupancy_grid_config

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.MAP_FRAME

        msg.info.map_load_time = self._map_load_time
        msg.info.resolution = config.resolution_m
        msg.info.width = config.width
        msg.info.height = config.height

        msg.info.origin.position.x = config.origin_x_m
        msg.info.origin.position.y = config.origin_y_m
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self._occupancy_grid.occupancy_values().reshape(-1).tolist()

        return msg

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
        node = SlamNode()
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