def wheel_efforts_from_angular_velocities(left_rad_s: float, right_rad_s: float, max_wheel_rad_s: float, max_abs_effort: float) -> tuple[float, float]:
    scale = max(1.0, abs(left_rad_s) / max_wheel_rad_s, abs(right_rad_s) / max_wheel_rad_s)

    left_limited = left_rad_s / scale
    right_limited = right_rad_s / scale

    left_effort = (
        left_limited / max_wheel_rad_s
    ) * max_abs_effort

    right_effort = (
        right_limited / max_wheel_rad_s
    ) * max_abs_effort

    return left_effort, right_effort