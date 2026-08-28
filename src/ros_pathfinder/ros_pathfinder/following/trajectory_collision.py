from dataclasses import dataclass
from math import cos, isfinite, sin
from typing import Optional

import numpy as np

from ros_pathfinder.geometry.footprint import FootprintBox2d


@dataclass
class TrajectoryCollisionConfig:
    footprint_min_x_m: float = -0.127
    footprint_max_x_m: float = 0.477
    footprint_min_y_m: float = -0.2655
    footprint_max_y_m: float = 0.2655
    collision_margin_m: float = 0.02
    prediction_horizon_s: float = 0.40
    prediction_step_s: float = 0.05
    linear_velocity_epsilon_m_s: float = 0.005
    angular_velocity_epsilon_rad_s: float = 0.01

    def __post_init__(self) -> None:
        values = (
            self.footprint_min_x_m,
            self.footprint_max_x_m,
            self.footprint_min_y_m,
            self.footprint_max_y_m,
            self.collision_margin_m,
            self.prediction_horizon_s,
            self.prediction_step_s,
            self.linear_velocity_epsilon_m_s,
            self.angular_velocity_epsilon_rad_s,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("collision-monitor configuration must be finite")
        if self.footprint_min_x_m >= self.footprint_max_x_m:
            raise ValueError("footprint x bounds are invalid")
        if self.footprint_min_y_m >= self.footprint_max_y_m:
            raise ValueError("footprint y bounds are invalid")
        if self.collision_margin_m < 0.0:
            raise ValueError("collision_margin_m must be non-negative")
        if self.prediction_horizon_s <= 0.0:
            raise ValueError("prediction_horizon_s must be positive")
        if self.prediction_step_s <= 0.0:
            raise ValueError("prediction_step_s must be positive")
        if self.linear_velocity_epsilon_m_s < 0.0:
            raise ValueError(
                "linear_velocity_epsilon_m_s must be non-negative"
            )
        if self.angular_velocity_epsilon_rad_s < 0.0:
            raise ValueError(
                "angular_velocity_epsilon_rad_s must be non-negative"
            )


@dataclass
class TrajectoryCollisionResult:
    collision_detected: bool
    time_to_collision_s: Optional[float] = None
    collision_point_base: Optional[tuple[float, float]] = None


class TrajectoryCollisionChecker:
    def __init__(self, config: TrajectoryCollisionConfig) -> None:
        self._config = config
        self._physical_footprint = FootprintBox2d(
            min_x_m=config.footprint_min_x_m,
            max_x_m=config.footprint_max_x_m,
            min_y_m=config.footprint_min_y_m,
            max_y_m=config.footprint_max_y_m,
        )
        self._collision_footprint = self._physical_footprint.expanded(
            config.collision_margin_m
        )

    def check(
        self,
        obstacle_points_base: np.ndarray,
        linear_velocity_m_s: float,
        angular_velocity_rad_s: float,
    ) -> TrajectoryCollisionResult:
        points = np.asarray(obstacle_points_base, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("obstacle_points_base must have shape (N, 2)")
        if not np.all(np.isfinite(points)):
            raise ValueError("obstacle_points_base must contain finite values")
        if not (
            isfinite(linear_velocity_m_s)
            and isfinite(angular_velocity_rad_s)
        ):
            raise ValueError("commanded velocities must be finite")

        if len(points) == 0 or self._command_is_stationary(
            linear_velocity_m_s,
            angular_velocity_rad_s,
        ):
            return TrajectoryCollisionResult(collision_detected=False)

        points = points[~self._points_inside_current_footprint(points)]
        if len(points) == 0:
            return TrajectoryCollisionResult(collision_detected=False)

        sample_times = np.arange(
            0.0,
            self._config.prediction_horizon_s
            + 0.5 * self._config.prediction_step_s,
            self._config.prediction_step_s,
        )
        for time_s in sample_times:
            x_m, y_m, yaw_rad = self._pose_at_time(
                linear_velocity_m_s,
                angular_velocity_rad_s,
                float(time_s),
            )
            collision_mask = self._points_inside_predicted_footprint(
                points,
                x_m,
                y_m,
                yaw_rad,
            )
            if np.any(collision_mask):
                point = points[int(np.flatnonzero(collision_mask)[0])]
                return TrajectoryCollisionResult(
                    collision_detected=True,
                    time_to_collision_s=float(time_s),
                    collision_point_base=(float(point[0]), float(point[1])),
                )

        return TrajectoryCollisionResult(collision_detected=False)

    def _command_is_stationary(
        self,
        linear_velocity_m_s: float,
        angular_velocity_rad_s: float,
    ) -> bool:
        return (
            abs(linear_velocity_m_s)
            <= self._config.linear_velocity_epsilon_m_s
            and abs(angular_velocity_rad_s)
            <= self._config.angular_velocity_epsilon_rad_s
        )

    def _points_inside_current_footprint(
        self,
        points: np.ndarray,
    ) -> np.ndarray:
        return self._physical_footprint.contains_points(points)

    def _points_inside_predicted_footprint(
        self,
        points: np.ndarray,
        robot_x_m: float,
        robot_y_m: float,
        robot_yaw_rad: float,
    ) -> np.ndarray:
        cos_yaw = cos(robot_yaw_rad)
        sin_yaw = sin(robot_yaw_rad)
        dx = points[:, 0] - robot_x_m
        dy = points[:, 1] - robot_y_m

        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        return self._collision_footprint.contains_points(
            np.column_stack((local_x, local_y))
        )

    @staticmethod
    def _pose_at_time(
        linear_velocity_m_s: float,
        angular_velocity_rad_s: float,
        time_s: float,
    ) -> tuple[float, float, float]:
        yaw_rad = angular_velocity_rad_s * time_s
        if abs(angular_velocity_rad_s) <= 1e-9:
            return linear_velocity_m_s * time_s, 0.0, yaw_rad

        radius_m = linear_velocity_m_s / angular_velocity_rad_s
        return (
            radius_m * sin(yaw_rad),
            radius_m * (1.0 - cos(yaw_rad)),
            yaw_rad,
        )
