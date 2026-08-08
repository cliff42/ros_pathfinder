def wheel_angular_velocities_from_twist(linear_vel_m_s: float, angular_vel_rad_s: float, wheel_rad_m: float, wheel_separation_m: float) -> tuple[float, float]:
    # vals in rad/s (flipped due to ROS coordinate system (positive angular.z is turning left))
    left_angular_vel_rad_s = (linear_vel_m_s - (angular_vel_rad_s * (wheel_separation_m / 2.0))) / wheel_rad_m
    right_angular_vel_rad_s = (linear_vel_m_s + (angular_vel_rad_s * (wheel_separation_m / 2.0))) / wheel_rad_m

    return left_angular_vel_rad_s, right_angular_vel_rad_s

def twist_from_wheel_angular_velocities(left_angular_vel_rad_s: float, right_angular_vel_rad_s: float, wheel_rad_m: float, wheel_separation_m: float) -> tuple[float, float]:
    linear_vel_m_s = wheel_rad_m * (left_angular_vel_rad_s + right_angular_vel_rad_s) / 2.0
    angular_rad_s = (wheel_rad_m * (right_angular_vel_rad_s - left_angular_vel_rad_s)) / wheel_separation_m

    return linear_vel_m_s, angular_rad_s