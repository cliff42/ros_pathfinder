import math
from dataclasses import dataclass
from typing import Optional

from ros_pathfinder.util.util import wrap_angle


@dataclass(frozen=True)
class OdometryState:
    timestamp_ns: int
    x_m: float
    y_m: float
    yaw_rad: float
    linear_vel_m_s: float
    angular_vel_rad_s: float


class WheelIMUOdometry:
    def __init__(
        self,
        wheel_radius_m: float,
        encoder_distance_scale: float,
    ) -> None:
        self._wheel_radius_m = wheel_radius_m
        self._encoder_distance_scale = encoder_distance_scale

        self._x_m = 0.0
        self._y_m = 0.0
        self._yaw_rad = 0.0
        self._angular_vel_rad_s = 0.0

        self._previous_imu_timestamp_ns: Optional[int] = None
        self._previous_yaw_rate_rad_s: Optional[float] = None

        self._previous_encoder_timestamp_ns: Optional[int] = None
        self._previous_left_position_rad: Optional[float] = None
        self._previous_right_position_rad: Optional[float] = None
        self._yaw_at_previous_encoder_sample = 0.0

    def add_imu_sample(
        self,
        yaw_rate_rad_s: float,
        timestamp_ns: int,
    ) -> None:
        if self._previous_imu_timestamp_ns is not None:
            elapsed_ns = timestamp_ns - self._previous_imu_timestamp_ns
            if elapsed_ns <= 0:
                raise ValueError("imu timestamps must increase")

            assert self._previous_yaw_rate_rad_s is not None
            elapsed_s = elapsed_ns * 1e-9
            average_rate = (
                self._previous_yaw_rate_rad_s + yaw_rate_rad_s
            ) / 2.0
            self._yaw_rad = self._wrap_angle(
                self._yaw_rad + average_rate * elapsed_s
            )

        self._angular_vel_rad_s = yaw_rate_rad_s
        self._previous_yaw_rate_rad_s = yaw_rate_rad_s
        self._previous_imu_timestamp_ns = timestamp_ns

    def add_encoder_sample(
        self,
        left_position_rad: float,
        right_position_rad: float,
        timestamp_ns: int,
    ) -> Optional[OdometryState]:
        if self._previous_encoder_timestamp_ns is None:
            self._previous_encoder_timestamp_ns = timestamp_ns
            self._previous_left_position_rad = left_position_rad
            self._previous_right_position_rad = right_position_rad
            self._yaw_at_previous_encoder_sample = self._yaw_rad
            return None

        elapsed_ns = timestamp_ns - self._previous_encoder_timestamp_ns
        if elapsed_ns <= 0:
            raise ValueError("encoder timestamps must increase")

        assert self._previous_left_position_rad is not None
        assert self._previous_right_position_rad is not None

        left_delta_rad = left_position_rad - self._previous_left_position_rad
        right_delta_rad = right_position_rad - self._previous_right_position_rad
        average_delta_rad = (left_delta_rad + right_delta_rad) / 2.0
        distance_m = (
            average_delta_rad
            * self._wheel_radius_m
            * self._encoder_distance_scale
        )

        yaw_delta = wrap_angle(self._yaw_rad - self._yaw_at_previous_encoder_sample)
        midpoint_yaw = wrap_angle(self._yaw_at_previous_encoder_sample + yaw_delta / 2.0)

        self._x_m += distance_m * math.cos(midpoint_yaw)
        self._y_m += distance_m * math.sin(midpoint_yaw)

        elapsed_s = elapsed_ns * 1e-9
        linear_velocity_m_s = distance_m / elapsed_s

        self._previous_encoder_timestamp_ns = timestamp_ns
        self._previous_left_position_rad = left_position_rad
        self._previous_right_position_rad = right_position_rad
        self._yaw_at_previous_encoder_sample = self._yaw_rad

        return OdometryState(
            timestamp_ns=timestamp_ns,
            x_m=self._x_m,
            y_m=self._y_m,
            yaw_rad=self._yaw_rad,
            linear_vel_m_s=linear_velocity_m_s,
            angular_vel_rad_s=self._angular_vel_rad_s
        )
