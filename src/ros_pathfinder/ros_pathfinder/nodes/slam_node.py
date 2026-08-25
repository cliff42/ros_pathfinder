import math
from collections import deque
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid

from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)

from ros_pathfinder.localization.icp_scan_matcher import ICPScanMatcher
from ros_pathfinder.localization.scan_localization import (
    LocalizationUpdate,
    ScanLocalization,
    ScanLocalizationConfig,
)
from ros_pathfinder.localization.scan_projection import scan_to_observation
from ros_pathfinder.mapping.occupancy_grid import (
    OccupancyGrid2d,
    OccupancyGridConfig,
)
from ros_pathfinder.geometry.footprint import FootprintBox2d
from ros_pathfinder.geometry.pose2d import Pose2d


class SlamNode(Node):

    SCAN_TOPIC = "scan"
    MAP_TOPIC = "map"  # Publishes the occupancy grid.

    MAP_FRAME = "map"
    ODOM_FRAME = "odom"
    BASE_FRAME = "base_link"

    def __init__(self):
        super().__init__("slam_node")

        self._scan_transform_timeout_s = float(
            self.declare_parameter(
                "scan_transform_timeout_s",
                0.25,
            ).value
        )
        if self._scan_transform_timeout_s <= 0.0:
            raise ValueError("scan_transform_timeout_s must be positive")
        self._diagnostic_logging_enabled = bool(
            self.declare_parameter(
                "diagnostic_logging_enabled",
                True,
            ).value
        )
        self._diagnostic_log_period_s = float(
            self.declare_parameter(
                "diagnostic_log_period_s",
                0.25,
            ).value
        )
        if self._diagnostic_log_period_s <= 0.0:
            raise ValueError("diagnostic_log_period_s must be positive")
        self._last_localization_diagnostic_ns = 0

        self._transform_buffer = Buffer()
        self._transform_listener = TransformListener(
            self._transform_buffer,
            self,
        )
        self._pending_scans = deque(maxlen=10)
        self._last_scan_transform_warning_ns = 0

        self._filter_footprint = FootprintBox2d(
            min_x_m=-0.15,
            max_x_m=0.50,
            min_y_m=-0.30,
            max_y_m=0.30,
        )

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

        self._scan_localization = ScanLocalization(
            icp_scan_matcher=icp_scan_matcher,
            config=scan_localization_config,
        )

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
        self._occupancy_grid = OccupancyGrid2d(
            config=self._occupancy_grid_config
        )
        self._map_load_time = self.get_clock().now().to_msg()

        self.publish_rate_hz = 1.0  # TODO: put in config

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
        self._scan_transform_timer = self.create_timer(
            0.01,
            self._process_pending_scan,
        )

    def lidar_callback(self, msg: LaserScan) -> None:
        if not msg.header.frame_id:
            return

        self._pending_scans.append(
            (msg, self.get_clock().now().nanoseconds)
        )
        self._process_pending_scan()

    def _process_pending_scan(self) -> None:
        if not self._pending_scans:
            return

        msg, received_time_ns = self._pending_scans[0]
        scan_ts = Time.from_msg(msg.header.stamp)
        odom_transform_ready = self._transform_buffer.can_transform(
            self.ODOM_FRAME,
            self.BASE_FRAME,
            scan_ts,
        )
        laser_transform_ready = self._transform_buffer.can_transform(
            self.BASE_FRAME,
            msg.header.frame_id,
            Time(),
        )

        if not (odom_transform_ready and laser_transform_ready):
            now_ns = self.get_clock().now().nanoseconds
            elapsed_s = (now_ns - received_time_ns) / 1e9
            if elapsed_s < self._scan_transform_timeout_s:
                return

            self._pending_scans.popleft()
            warning_elapsed_ns = (
                now_ns - self._last_scan_transform_warning_ns
            )
            if warning_elapsed_ns >= 1000000000:
                scan_stamp = (
                    f"{msg.header.stamp.sec}."
                    f"{msg.header.stamp.nanosec:09d}"
                )
                self.get_logger().warning(
                    f"dropping laser scan at {scan_stamp} after waiting "
                    f"{elapsed_s:.2f} s for transforms "
                    f"(odom_ready={odom_transform_ready}, "
                    f"laser_ready={laser_transform_ready})"
                )
                self._last_scan_transform_warning_ns = now_ns
            return

        try:
            odom_to_base = self._transform_buffer.lookup_transform(
                self.ODOM_FRAME,
                self.BASE_FRAME,
                scan_ts,
            )
            base_to_laser = self._transform_buffer.lookup_transform(
                self.BASE_FRAME,
                msg.header.frame_id,
                Time(),
            )
        except TransformException:
            return

        self._pending_scans.popleft()
        self._process_lidar_scan(msg, odom_to_base, base_to_laser)

    def _process_lidar_scan(
        self,
        msg: LaserScan,
        odom_to_base: TransformStamped,
        base_to_laser: TransformStamped,
    ) -> None:

        laser_pose_in_base = self._pose_from_transform(base_to_laser)

        scan_observation = scan_to_observation(
            ranges=msg.ranges,
            angle_min_rad=msg.angle_min,
            angle_increment_rad=msg.angle_increment,
            range_min_m=msg.range_min,
            range_max_m=msg.range_max,
            laser_pose_in_base=laser_pose_in_base,
            filter_footprint=self._filter_footprint
        )

        odom_pose = self._pose_from_transform(odom_to_base)

        localization_update = self._scan_localization.update(
            current_scan=scan_observation,
            current_odom_pose=odom_pose,
            timestamp_ns=self._timestamp_to_ns(msg.header.stamp)
        )
        self._log_localization_diagnostic(
            localization_update,
            odom_pose,
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

        transform.transform.translation.x = (
            localization_update.map_to_odom.x_m
        )
        transform.transform.translation.y = (
            localization_update.map_to_odom.y_m
        )
        transform.transform.translation.z = 0.0

        half_yaw = localization_update.map_to_odom.yaw_rad / 2.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(half_yaw)
        transform.transform.rotation.w = math.cos(half_yaw)

        self.transform_broadcaster.sendTransform(transform)

    def _log_localization_diagnostic(
        self,
        update: LocalizationUpdate,
        odom_pose: Pose2d,
    ) -> None:
        if not self._diagnostic_logging_enabled:
            return

        now_ns = self.get_clock().now().nanoseconds
        period_ns = int(self._diagnostic_log_period_s * 1e9)
        correction = update.icp_correction
        significant_correction = (
            correction is not None
            and (
                math.hypot(correction.x_m, correction.y_m) >= 0.03
                or abs(correction.yaw_rad) >= math.radians(2.0)
            )
        )
        if (
            not significant_correction
            and now_ns - self._last_localization_diagnostic_ns < period_ns
        ):
            return
        self._last_localization_diagnostic_ns = now_ns

        icp_result = update.icp_result
        if icp_result is None:
            icp_quality = "icp=none"
            icp_pose = "none"
        else:
            icp_quality = (
                f"matches={icp_result.match_count} "
                f"rmse={icp_result.rmse_m:.4f}m "
                f"iterations={icp_result.iterations} "
                f"converged={icp_result.converged}"
            )
            icp_pose = self._pose_diagnostic_text(icp_result.delta)

        self.get_logger().info(
            "slam_diag "
            f"status={update.status.value} "
            f"odom={self._pose_diagnostic_text(odom_pose)} "
            f"initial_guess="
            f"{self._pose_diagnostic_text(update.icp_initial_transform)} "
            f"icp_result={icp_pose} "
            f"icp_correction="
            f"{self._pose_diagnostic_text(update.icp_correction)} "
            f"chosen_delta={self._pose_diagnostic_text(update.chosen_delta)} "
            f"map_to_odom={self._pose_diagnostic_text(update.map_to_odom)} "
            f"map_to_base={self._pose_diagnostic_text(update.map_to_base)} "
            f"{icp_quality}"
        )

    @staticmethod
    def _pose_diagnostic_text(pose: Optional[Pose2d]) -> str:
        if pose is None:
            return "none"
        return (
            f"({pose.x_m:.3f},{pose.y_m:.3f},"
            f"{math.degrees(pose.yaw_rad):.1f}deg)"
        )

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

        msg.data = (
            self._occupancy_grid.occupancy_values().reshape(-1).tolist()
        )

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
        return int(ts.sec) * 1000000000 + int(ts.nanosec)


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
