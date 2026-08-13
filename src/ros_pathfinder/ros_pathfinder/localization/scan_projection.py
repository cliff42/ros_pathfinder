import numpy as np

from ros_pathfinder.geometry.scan2d import ScanObservation2d
from ros_pathfinder.geometry.pose2d import Pose2d

# scan msg to xy points
def scan_to_points(
        ranges, 
        angle_min_rad: float, 
        angle_increment_rad: float,
        range_min_m: float,
        range_max_m: float,
) -> np.ndarray:
    ranges_array = np.asarray(ranges, dtype=float)

    angles = angle_min_rad + np.arange(ranges_array.size) * angle_increment_rad

    # get valid ranges (bit mask)
    valid = (np.isfinite(ranges_array) & (ranges_array >= range_min_m) & (ranges_array <= range_max_m))

    valid_ranges = ranges_array[valid]
    valid_angles = angles[valid]

    # x is r * cos(theta), y is r * sin(theta)
    return np.column_stack((valid_ranges * np.cos(valid_angles), valid_ranges * np.sin(valid_angles)))

# scan msg to observation
def scan_to_observation(
        ranges, 
        angle_min_rad: float, 
        angle_increment_rad: float,
        range_min_m: float,
        range_max_m: float,
        laser_pose_in_base: Pose2d
) -> ScanObservation2d:
    ranges_array = np.asarray(ranges, dtype=float)

    angles = angle_min_rad + np.arange(ranges_array.size) * angle_increment_rad

    # if the ranges are finite we consider it a hit
    hits = (np.isfinite(ranges_array) & (ranges_array >= range_min_m) & (ranges_array < range_max_m))

    usable_angles = angles[hits]

    # rays with ostacles end at measurement
    endpoint_ranges = ranges_array[hits]

    endpoints_in_laser = np.column_stack((endpoint_ranges * np.cos(usable_angles), endpoint_ranges * np.sin(usable_angles)))

    endpoints_in_base = laser_pose_in_base.transform_points(endpoints_in_laser)

    sensor_origin_in_base = laser_pose_in_base.transform_points(np.zeros((1, 2), dtype=float))[0]

    return ScanObservation2d(
        sensor_origin_base=sensor_origin_in_base,
        ray_endpoints_base=endpoints_in_base,
        hit_mask=np.ones(endpoint_ranges.shape, dtype=bool)
    )