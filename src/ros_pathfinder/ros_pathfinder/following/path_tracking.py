from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, sin

import numpy as np

from ros_pathfinder.util.util import wrap_angle


@dataclass
class PathTrackingConfig:
    desired_linear_velocity_m_s: float = 0.12
    lookahead_distance_m: float = 0.30
    goal_position_tolerance_m: float = 0.08
    goal_yaw_tolerance_rad: float = 0.12
    rotate_in_place_threshold_rad: float = 1.05
    angular_gain: float = 1.0
    max_angular_velocity_rad_s: float = 0.45
    minimum_linear_speed_ratio: float = 0.25
    angular_smoothing: float = 0.35
    angular_deadband_rad_s: float = 0.015

    def __post_init__(self) -> None:
        if self.desired_linear_velocity_m_s <= 0.0:
            raise ValueError("desired_linear_velocity_m_s must be positive")
        if self.lookahead_distance_m <= 0.0:
            raise ValueError("lookahead_distance_m must be positive")
        if self.goal_position_tolerance_m < 0.0:
            raise ValueError("goal_position_tolerance_m must be non-negative")
        if self.goal_yaw_tolerance_rad < 0.0:
            raise ValueError("goal_yaw_tolerance_rad must be non-negative")
        if self.rotate_in_place_threshold_rad <= 0.0:
            raise ValueError("rotate_in_place_threshold_rad must be positive")
        if self.angular_gain <= 0.0:
            raise ValueError("angular_gain must be positive")
        if self.max_angular_velocity_rad_s <= 0.0:
            raise ValueError("max_angular_velocity_rad_s must be positive")
        if not 0.0 <= self.minimum_linear_speed_ratio <= 1.0:
            raise ValueError("minimum_linear_speed_ratio must be in [0, 1]")
        if not 0.0 <= self.angular_smoothing < 1.0:
            raise ValueError("angular_smoothing must be in [0, 1)")
        if self.angular_deadband_rad_s < 0.0:
            raise ValueError("angular_deadband_rad_s must be non-negative")


@dataclass
class PathTrackingCommand:
    linear_velocity_m_s: float
    angular_velocity_rad_s: float
    target_index: int
    distance_to_goal_m: float
    goal_reached: bool


class PathTracker:
    def __init__(self, config: PathTrackingConfig) -> None:
        self._config = config

    def update(
        self,
        robot_pose: tuple[float, float, float],
        path_points: np.ndarray,
        final_yaw_rad: float,
        previous_target_index: int = 0,
        previous_angular_velocity_rad_s: float = 0.0,
    ) -> PathTrackingCommand:
        points = np.asarray(path_points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
            raise ValueError("path_points must have shape (N, 2) with N > 0")
        if not np.all(np.isfinite(points)):
            raise ValueError("path_points must contain only finite values")

        robot_x, robot_y, robot_yaw = (float(value) for value in robot_pose)
        values = (
            robot_x,
            robot_y,
            robot_yaw,
            final_yaw_rad,
            previous_angular_velocity_rad_s,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("pose, yaw, and previous velocity must be finite")

        goal_x, goal_y = points[-1]
        distance_to_goal = hypot(goal_x - robot_x, goal_y - robot_y)
        last_index = len(points) - 1

        if distance_to_goal <= self._config.goal_position_tolerance_m:
            yaw_error = wrap_angle(final_yaw_rad - robot_yaw)
            if abs(yaw_error) <= self._config.goal_yaw_tolerance_rad:
                return PathTrackingCommand(
                    linear_velocity_m_s=0.0,
                    angular_velocity_rad_s=0.0,
                    target_index=last_index,
                    distance_to_goal_m=distance_to_goal,
                    goal_reached=True,
                )

            angular_velocity = self._smoothed_angular_velocity(
                self._config.angular_gain * yaw_error,
                previous_angular_velocity_rad_s,
            )
            return PathTrackingCommand(
                linear_velocity_m_s=0.0,
                angular_velocity_rad_s=angular_velocity,
                target_index=last_index,
                distance_to_goal_m=distance_to_goal,
                goal_reached=False,
            )

        target_index, target_x, target_y = self._select_target(
            points,
            robot_x,
            robot_y,
            previous_target_index,
        )
        target_distance = hypot(target_x - robot_x, target_y - robot_y)
        target_heading = atan2(target_y - robot_y, target_x - robot_x)
        heading_error = wrap_angle(target_heading - robot_yaw)

        if abs(heading_error) >= self._config.rotate_in_place_threshold_rad:
            linear_velocity = 0.0
            raw_angular_velocity = self._config.angular_gain * heading_error
        else:
            heading_scale = max(
                self._config.minimum_linear_speed_ratio,
                cos(heading_error),
            )
            slowdown_distance = max(
                self._config.lookahead_distance_m,
                2.0 * self._config.goal_position_tolerance_m,
            )
            goal_scale = max(
                self._config.minimum_linear_speed_ratio,
                min(1.0, distance_to_goal / slowdown_distance),
            )
            linear_velocity = (
                self._config.desired_linear_velocity_m_s
                * heading_scale
                * goal_scale
            )
            curvature = 2.0 * sin(heading_error) / max(
                target_distance,
                self._config.goal_position_tolerance_m,
                1e-6,
            )
            raw_angular_velocity = (
                self._config.angular_gain * linear_velocity * curvature
            )

        angular_velocity = self._smoothed_angular_velocity(
            raw_angular_velocity,
            previous_angular_velocity_rad_s,
        )
        return PathTrackingCommand(
            linear_velocity_m_s=linear_velocity,
            angular_velocity_rad_s=angular_velocity,
            target_index=target_index,
            distance_to_goal_m=distance_to_goal,
            goal_reached=False,
        )

    def _select_target(
        self,
        points: np.ndarray,
        robot_x: float,
        robot_y: float,
        previous_target_index: int,
    ) -> tuple[int, float, float]:
        last_index = len(points) - 1
        if last_index == 0:
            return 0, float(points[0, 0]), float(points[0, 1])

        previous_target_index = min(
            max(int(previous_target_index), 0),
            last_index,
        )
        first_segment = max(previous_target_index - 1, 0)
        robot = np.array([robot_x, robot_y], dtype=float)

        nearest_segment = first_segment
        nearest_point = points[first_segment].copy()
        nearest_distance_squared = float("inf")

        for segment_index in range(first_segment, last_index):
            start = points[segment_index]
            segment = points[segment_index + 1] - start
            segment_length_squared = float(np.dot(segment, segment))
            if segment_length_squared <= 1e-12:
                projection = start
            else:
                fraction = float(
                    np.dot(robot - start, segment) / segment_length_squared
                )
                fraction = min(1.0, max(0.0, fraction))
                projection = start + fraction * segment

            offset = robot - projection
            distance_squared = float(np.dot(offset, offset))
            if distance_squared < nearest_distance_squared:
                nearest_distance_squared = distance_squared
                nearest_segment = segment_index
                nearest_point = projection

        remaining_distance = self._config.lookahead_distance_m
        current_point = nearest_point
        segment_index = nearest_segment

        while segment_index < last_index:
            segment_end = points[segment_index + 1]
            segment = segment_end - current_point
            segment_length = float(np.linalg.norm(segment))

            if segment_length >= remaining_distance and segment_length > 1e-12:
                target = (
                    current_point
                    + (remaining_distance / segment_length) * segment
                )
                return (
                    max(previous_target_index, segment_index + 1),
                    float(target[0]),
                    float(target[1]),
                )

            remaining_distance -= segment_length
            segment_index += 1
            current_point = points[segment_index]

        return last_index, float(points[-1, 0]), float(points[-1, 1])

    def _smoothed_angular_velocity(
        self,
        requested_velocity_rad_s: float,
        previous_velocity_rad_s: float,
    ) -> float:
        limit = self._config.max_angular_velocity_rad_s
        requested_velocity_rad_s = max(
            -limit,
            min(limit, requested_velocity_rad_s),
        )
        smoothing = self._config.angular_smoothing
        velocity = (
            smoothing * previous_velocity_rad_s
            + (1.0 - smoothing) * requested_velocity_rad_s
        )
        velocity = max(-limit, min(limit, velocity))
        if abs(velocity) < self._config.angular_deadband_rad_s:
            return 0.0
        return velocity
