from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class WheelVelocityControlConfig:
    proportional_gain: float = 0.04
    integral_gain: float = 0.08
    feedforward_gain: float = 0.02
    integral_limit_rad: float = 2.0
    max_abs_effort: float = 0.40
    max_acceleration_rad_s2: float = 4.0
    stopped_velocity_tolerance_rad_s: float = 0.10

    def __post_init__(self) -> None:
        values = (
            self.proportional_gain,
            self.integral_gain,
            self.feedforward_gain,
            self.integral_limit_rad,
            self.max_abs_effort,
            self.max_acceleration_rad_s2,
            self.stopped_velocity_tolerance_rad_s,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("wheel-velocity configuration must be finite")
        if self.proportional_gain < 0.0:
            raise ValueError("proportional_gain must be non-negative")
        if self.integral_gain < 0.0:
            raise ValueError("integral_gain must be non-negative")
        if self.feedforward_gain < 0.0:
            raise ValueError("feedforward_gain must be non-negative")
        if self.integral_limit_rad < 0.0:
            raise ValueError("integral_limit_rad must be non-negative")
        if not 0.0 < self.max_abs_effort <= 1.0:
            raise ValueError("max_abs_effort must be in (0, 1]")
        if self.max_acceleration_rad_s2 <= 0.0:
            raise ValueError("max_acceleration_rad_s2 must be positive")
        if self.stopped_velocity_tolerance_rad_s < 0.0:
            raise ValueError(
                "stopped_velocity_tolerance_rad_s must be non-negative"
            )


class WheelVelocityController:
    def __init__(self, config: WheelVelocityControlConfig) -> None:
        self._config = config
        self._limited_target_rad_s = 0.0
        self._integral_error_rad = 0.0
        self._previous_requested_target_rad_s = 0.0

    @property
    def limited_target_rad_s(self) -> float:
        return self._limited_target_rad_s

    @property
    def integral_error_rad(self) -> float:
        return self._integral_error_rad

    def reset(self) -> None:
        self._limited_target_rad_s = 0.0
        self._integral_error_rad = 0.0
        self._previous_requested_target_rad_s = 0.0

    def update(
        self,
        requested_target_rad_s: float,
        measured_velocity_rad_s: float,
        elapsed_s: float,
    ) -> float:
        values = (
            requested_target_rad_s,
            measured_velocity_rad_s,
            elapsed_s,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("wheel-velocity update values must be finite")
        if elapsed_s <= 0.0:
            raise ValueError("elapsed_s must be positive")

        previous_request = self._previous_requested_target_rad_s
        request_reversed = (
            requested_target_rad_s * previous_request < 0.0
        )
        request_stopped = (
            requested_target_rad_s == 0.0
            and previous_request != 0.0
        )
        if request_reversed or request_stopped:
            self._integral_error_rad = 0.0

        self._previous_requested_target_rad_s = requested_target_rad_s
        self._limited_target_rad_s = self._limited_target(
            requested_target_rad_s,
            elapsed_s,
        )

        if (
            requested_target_rad_s == 0.0
            and abs(measured_velocity_rad_s)
            <= self._config.stopped_velocity_tolerance_rad_s
        ):
            self._limited_target_rad_s = 0.0
            self._integral_error_rad = 0.0
            return 0.0

        error_rad_s = (
            self._limited_target_rad_s - measured_velocity_rad_s
        )
        integral_limit = self._config.integral_limit_rad
        candidate_integral_error_rad = max(
            -integral_limit,
            min(
                integral_limit,
                self._integral_error_rad + error_rad_s * elapsed_s,
            ),
        )

        requested_effort = (
            self._config.feedforward_gain * self._limited_target_rad_s
            + self._config.proportional_gain * error_rad_s
            + self._config.integral_gain * candidate_integral_error_rad
        )
        limited_effort = max(
            -self._config.max_abs_effort,
            min(self._config.max_abs_effort, requested_effort),
        )
        saturation_would_increase = (
            requested_effort > self._config.max_abs_effort
            and error_rad_s > 0.0
        ) or (
            requested_effort < -self._config.max_abs_effort
            and error_rad_s < 0.0
        )
        if not saturation_would_increase:
            self._integral_error_rad = candidate_integral_error_rad
        return limited_effort

    def _limited_target(
        self,
        requested_target_rad_s: float,
        elapsed_s: float,
    ) -> float:
        if requested_target_rad_s == 0.0:
            return 0.0

        maximum_change = (
            self._config.max_acceleration_rad_s2 * elapsed_s
        )
        requested_change = (
            requested_target_rad_s - self._limited_target_rad_s
        )
        bounded_change = max(
            -maximum_change,
            min(maximum_change, requested_change),
        )
        return self._limited_target_rad_s + bounded_change
