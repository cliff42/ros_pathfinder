from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, sin

import numpy as np

from ros_pathfinder.util.util import wrap_angle


# Inspired by the Purdue SIGBots basic pure-pursuit controller.
@dataclass
class PathTrackingConfig:
    desired_linear_velocity_m_s: float = 0.18
    minimum_lookahead_distance_m: float = 0.10
    lookahead_time_s: float = 0.30
    maximum_lookahead_distance_m: float = 0.18
    goal_position_tolerance_m: float = 0.12
    goal_position_tolerance_buffer_m: float = 0.05
    goal_yaw_tolerance_rad: float = 0.25
    rotate_in_place_threshold_rad: float = 0.85
    angular_gain: float = 1.0
    max_angular_velocity_rad_s: float = 0.65
    minimum_linear_speed_ratio: float = 0.20
    angular_smoothing: float = 0.20
    angular_deadband_rad_s: float = 0.015

    def __post_init__(self) -> None:
        if self.desired_linear_velocity_m_s <= 0.0:
            raise ValueError("desired_linear_velocity_m_s must be positive")
        if self.minimum_lookahead_distance_m <= 0.0:
            raise ValueError(
                "minimum_lookahead_distance_m must be positive"
            )
        if self.lookahead_time_s < 0.0:
            raise ValueError("lookahead_time_s must be non-negative")
        if (
            self.maximum_lookahead_distance_m
            < self.minimum_lookahead_distance_m
        ):
            raise ValueError(
                "maximum_lookahead_distance_m must be greater than or "
                "equal to minimum_lookahead_distance_m"
            )
        if self.goal_position_tolerance_m < 0.0:
            raise ValueError("goal_position_tolerance_m must be non-negative")
        if self.goal_position_tolerance_buffer_m < 0.0:
            raise ValueError(
                "goal_position_tolerance_buffer_m must be non-negative"
            )
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


@dataclass(frozen=True)
class PathTrackingDiagnostics:
    control_mode: str
    lookahead_distance_m: float
    target_x_m: float
    target_y_m: float
    target_heading_rad: float
    heading_error_rad: float
    curvature_m_inv: float
    unconstrained_angular_velocity_rad_s: float
    angular_velocity_saturated: bool


@dataclass
class PathTrackingCommand:
    linear_velocity_m_s: float
    angular_velocity_rad_s: float
    target_index: int
    distance_to_goal_m: float
    goal_position_reached: bool
    goal_reached: bool
    diagnostics: PathTrackingDiagnostics


class PathTracker:
    def __init__(self, config: PathTrackingConfig) -> None:
        self._config = config

    def update(
        self,
        robot_pose: tuple[float, float, float],
        path_points: np.ndarray,
        final_yaw_rad: float,
        current_linear_velocity_m_s: float = 0.0,
        previous_target_index: int = 0,
        previous_angular_velocity_rad_s: float = 0.0,
        goal_position_reached: bool = False,
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
            current_linear_velocity_m_s,
            previous_angular_velocity_rad_s,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("pose, yaw, and previous velocity must be finite")

        goal_x, goal_y = points[-1]
        distance_to_goal = hypot(goal_x - robot_x, goal_y - robot_y)
        last_index = len(points) - 1
        lookahead_distance = self._lookahead_distance(
            current_linear_velocity_m_s
        )

        if (
            goal_position_reached
            and distance_to_goal
            > (
                self._config.goal_position_tolerance_m
                + self._config.goal_position_tolerance_buffer_m
            )
        ):
            goal_position_reached = False
        if distance_to_goal <= self._config.goal_position_tolerance_m:
            goal_position_reached = True

        if goal_position_reached:
            yaw_error = wrap_angle(final_yaw_rad - robot_yaw)
            if abs(yaw_error) <= self._config.goal_yaw_tolerance_rad:
                return PathTrackingCommand(
                    linear_velocity_m_s=0.0,
                    angular_velocity_rad_s=0.0,
                    target_index=last_index,
                    distance_to_goal_m=distance_to_goal,
                    goal_position_reached=True,
                    goal_reached=True,
                    diagnostics=PathTrackingDiagnostics(
                        control_mode="goal_reached",
                        lookahead_distance_m=lookahead_distance,
                        target_x_m=float(goal_x),
                        target_y_m=float(goal_y),
                        target_heading_rad=final_yaw_rad,
                        heading_error_rad=yaw_error,
                        curvature_m_inv=0.0,
                        unconstrained_angular_velocity_rad_s=0.0,
                        angular_velocity_saturated=False,
                    ),
                )

            requested_angular_velocity = (
                self._config.angular_gain * yaw_error
            )
            angular_velocity = self._smoothed_angular_velocity(
                requested_angular_velocity,
                previous_angular_velocity_rad_s,
            )
            return PathTrackingCommand(
                linear_velocity_m_s=0.0,
                angular_velocity_rad_s=angular_velocity,
                target_index=last_index,
                distance_to_goal_m=distance_to_goal,
                goal_position_reached=True,
                goal_reached=False,
                diagnostics=PathTrackingDiagnostics(
                    control_mode="goal_rotation",
                    lookahead_distance_m=lookahead_distance,
                    target_x_m=float(goal_x),
                    target_y_m=float(goal_y),
                    target_heading_rad=final_yaw_rad,
                    heading_error_rad=yaw_error,
                    curvature_m_inv=0.0,
                    unconstrained_angular_velocity_rad_s=(
                        requested_angular_velocity
                    ),
                    angular_velocity_saturated=(
                        abs(requested_angular_velocity)
                        > self._config.max_angular_velocity_rad_s
                    ),
                ),
            )

        target_index, target_x, target_y = self._select_target(
            points,
            robot_x,
            robot_y,
            previous_target_index,
            lookahead_distance,
        )
        target_distance = hypot(target_x - robot_x, target_y - robot_y)
        target_heading = atan2(target_y - robot_y, target_x - robot_x)
        heading_error = wrap_angle(target_heading - robot_yaw)

        angular_velocity_was_limited = False
        curvature = 0.0
        if abs(heading_error) >= self._config.rotate_in_place_threshold_rad:
            control_mode = "path_rotation"
            linear_velocity = 0.0
            raw_angular_velocity = self._config.angular_gain * heading_error
            unconstrained_angular_velocity = raw_angular_velocity
        else:
            control_mode = "pursuit"
            heading_scale = max(
                self._config.minimum_linear_speed_ratio,
                cos(heading_error),
            )
            slowdown_distance = max(
                lookahead_distance,
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
            requested_angular_velocity = (
                self._config.angular_gain * linear_velocity * curvature
            )
            unconstrained_angular_velocity = requested_angular_velocity

            angular_limit = self._config.max_angular_velocity_rad_s
            if abs(requested_angular_velocity) > angular_limit:
                angular_velocity_was_limited = True
                linear_velocity *= (
                    angular_limit / abs(requested_angular_velocity)
                )
                requested_angular_velocity = (
                    self._config.angular_gain
                    * linear_velocity
                    * curvature
                )

            raw_angular_velocity = requested_angular_velocity

        angular_velocity = self._smoothed_angular_velocity(
            raw_angular_velocity,
            previous_angular_velocity_rad_s,
        )
        if angular_velocity_was_limited:
            linear_velocity *= (
                abs(angular_velocity)
                / self._config.max_angular_velocity_rad_s
            )
        return PathTrackingCommand(
            linear_velocity_m_s=linear_velocity,
            angular_velocity_rad_s=angular_velocity,
            target_index=target_index,
            distance_to_goal_m=distance_to_goal,
            goal_position_reached=False,
            goal_reached=False,
            diagnostics=PathTrackingDiagnostics(
                control_mode=control_mode,
                lookahead_distance_m=lookahead_distance,
                target_x_m=target_x,
                target_y_m=target_y,
                target_heading_rad=target_heading,
                heading_error_rad=heading_error,
                curvature_m_inv=curvature,
                unconstrained_angular_velocity_rad_s=(
                    unconstrained_angular_velocity
                ),
                angular_velocity_saturated=(
                    abs(unconstrained_angular_velocity)
                    > self._config.max_angular_velocity_rad_s
                ),
            ),
        )

    def _select_target(
        self,
        points: np.ndarray,
        robot_x: float,
        robot_y: float,
        previous_target_index: int,
        lookahead_distance_m: float,
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

        first_projection_fraction = self._segment_projection_fraction(
            robot,
            points[first_segment],
            points[first_segment + 1],
        )
        for segment_index in range(first_segment, last_index):
            start = points[segment_index]
            end = points[segment_index + 1]
            intersection_fractions = self._circle_segment_intersections(
                circle_center=robot,
                circle_radius_m=lookahead_distance_m,
                segment_start=start,
                segment_end=end,
            )
            minimum_fraction = (
                first_projection_fraction
                if segment_index == first_segment
                else 0.0
            )
            forward_intersections = [
                fraction
                for fraction in intersection_fractions
                if fraction >= minimum_fraction - 1e-9
            ]
            if not forward_intersections:
                continue

            fraction = max(forward_intersections)
            target = start + fraction * (end - start)
            return (
                max(previous_target_index, segment_index + 1),
                float(target[0]),
                float(target[1]),
            )

        goal = points[-1]
        if float(np.linalg.norm(goal - robot)) <= lookahead_distance_m:
            return last_index, float(goal[0]), float(goal[1])

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

        return (
            max(previous_target_index, nearest_segment + 1),
            float(nearest_point[0]),
            float(nearest_point[1]),
        )

    @staticmethod
    def _segment_projection_fraction(
        point: np.ndarray,
        segment_start: np.ndarray,
        segment_end: np.ndarray,
    ) -> float:
        segment = segment_end - segment_start
        length_squared = float(np.dot(segment, segment))
        if length_squared <= 1e-12:
            return 0.0

        fraction = float(
            np.dot(point - segment_start, segment) / length_squared
        )
        return min(1.0, max(0.0, fraction))

    @staticmethod
    def _circle_segment_intersections(
        circle_center: np.ndarray,
        circle_radius_m: float,
        segment_start: np.ndarray,
        segment_end: np.ndarray,
    ) -> tuple[float, ...]:
        segment = segment_end - segment_start
        relative_start = segment_start - circle_center
        quadratic_a = float(np.dot(segment, segment))
        if quadratic_a <= 1e-12:
            return ()

        quadratic_b = 2.0 * float(np.dot(relative_start, segment))
        quadratic_c = float(
            np.dot(relative_start, relative_start)
            - circle_radius_m * circle_radius_m
        )
        discriminant = (
            quadratic_b * quadratic_b
            - 4.0 * quadratic_a * quadratic_c
        )
        if discriminant < -1e-12:
            return ()

        square_root = float(np.sqrt(max(0.0, discriminant)))
        denominator = 2.0 * quadratic_a
        roots = (
            (-quadratic_b - square_root) / denominator,
            (-quadratic_b + square_root) / denominator,
        )
        intersections = []
        for root in roots:
            if -1e-9 <= root <= 1.0 + 1e-9:
                bounded_root = min(1.0, max(0.0, root))
                if not intersections or not np.isclose(
                    bounded_root,
                    intersections[-1],
                ):
                    intersections.append(bounded_root)

        return tuple(intersections)

    def _lookahead_distance(
        self,
        current_linear_velocity_m_s: float,
    ) -> float:
        requested_distance = (
            self._config.minimum_lookahead_distance_m
            + self._config.lookahead_time_s
            * abs(current_linear_velocity_m_s)
        )
        return min(
            requested_distance,
            self._config.maximum_lookahead_distance_m,
        )

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
