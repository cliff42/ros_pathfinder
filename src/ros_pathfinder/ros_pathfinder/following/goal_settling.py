from dataclasses import dataclass
from math import isfinite
from typing import Optional


@dataclass(frozen=True)
class GoalSettlingConfig:
    linear_velocity_tolerance_m_s: float = 0.03
    angular_velocity_tolerance_rad_s: float = 0.15
    settle_time_s: float = 0.25

    def __post_init__(self) -> None:
        values = (
            self.linear_velocity_tolerance_m_s,
            self.angular_velocity_tolerance_rad_s,
            self.settle_time_s,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("goal-settling configuration must be finite")
        if self.linear_velocity_tolerance_m_s < 0.0:
            raise ValueError(
                "linear_velocity_tolerance_m_s must be non-negative"
            )
        if self.angular_velocity_tolerance_rad_s < 0.0:
            raise ValueError(
                "angular_velocity_tolerance_rad_s must be non-negative"
            )
        if self.settle_time_s < 0.0:
            raise ValueError("settle_time_s must be non-negative")


class GoalSettlingMonitor:
    def __init__(self, config: GoalSettlingConfig) -> None:
        self._config = config
        self._settled_since_ns: Optional[int] = None

    def reset(self) -> None:
        self._settled_since_ns = None

    def update(
        self,
        goal_pose_reached: bool,
        linear_velocity_m_s: float,
        angular_velocity_rad_s: float,
        timestamp_ns: int,
    ) -> bool:
        values = (linear_velocity_m_s, angular_velocity_rad_s)
        if not all(isfinite(value) for value in values):
            raise ValueError("goal-settling velocities must be finite")
        if timestamp_ns < 0:
            raise ValueError("timestamp_ns must be non-negative")

        motion_is_settled = (
            abs(linear_velocity_m_s)
            <= self._config.linear_velocity_tolerance_m_s
            and abs(angular_velocity_rad_s)
            <= self._config.angular_velocity_tolerance_rad_s
        )
        if not goal_pose_reached or not motion_is_settled:
            self.reset()
            return False

        if (
            self._settled_since_ns is None
            or timestamp_ns < self._settled_since_ns
        ):
            self._settled_since_ns = timestamp_ns

        elapsed_s = (timestamp_ns - self._settled_since_ns) * 1e-9
        return elapsed_s >= self._config.settle_time_s
