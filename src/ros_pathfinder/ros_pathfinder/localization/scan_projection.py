import numpy as np

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