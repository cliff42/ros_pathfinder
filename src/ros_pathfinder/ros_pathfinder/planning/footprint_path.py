import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from ros_pathfinder.geometry.footprint import FootprintBox2d
from ros_pathfinder.geometry.pose2d import Pose2d
from ros_pathfinder.planning.costmap import Costmap2d
from ros_pathfinder.planning.grid_geometry import GridGeometry2d
from ros_pathfinder.util.util import wrap_angle


@dataclass(frozen=True)
class FootprintPathValidity:
    is_valid: bool
    reason: str = ""
    collision_pose: Optional[Pose2d] = None


class FootprintPathChecker:
    def __init__(
        self,
        obstacle_costmap: Costmap2d,
        grid_geometry: GridGeometry2d,
        collision_footprint: FootprintBox2d,
    ) -> None:
        if (
            obstacle_costmap.width != grid_geometry.width
            or obstacle_costmap.height != grid_geometry.height
        ):
            raise ValueError(
                "grid geometry dimensions do not match the costmap"
            )

        self._costmap = obstacle_costmap
        self._geometry = grid_geometry
        self._footprint = collision_footprint
        self._position_step_m = grid_geometry.resolution_m / 2.0
        footprint_radius_m = collision_footprint.circumscribed_radius_m
        self._yaw_step_rad = min(
            math.radians(5.0),
            self._position_step_m / max(footprint_radius_m, 1e-9),
        )

    def check(
        self,
        start_pose_world: Pose2d,
        path_points_world: np.ndarray,
        final_yaw_rad: float,
    ) -> FootprintPathValidity:
        points = np.asarray(path_points_world, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
            raise ValueError(
                "path_points_world must have shape (N, 2), N > 0"
            )
        if not np.all(np.isfinite(points)):
            raise ValueError(
                "path_points_world must contain only finite values"
            )

        pose_values = (
            start_pose_world.x_m,
            start_pose_world.y_m,
            start_pose_world.yaw_rad,
            final_yaw_rad,
        )
        if not all(math.isfinite(value) for value in pose_values):
            raise ValueError("path poses and yaws must be finite")

        points = self._remove_duplicate_points(points)
        start_position = np.array(
            [start_pose_world.x_m, start_pose_world.y_m],
            dtype=float,
        )

        if (
            len(points) > 1
            and np.linalg.norm(points[0] - start_position)
            <= self._geometry.resolution_m
        ):
            points = points[1:]

        waypoints = np.vstack((start_position, points))
        waypoints = self._remove_duplicate_points(waypoints)

        if len(waypoints) == 1:
            return self._check_rotation(
                x_m=float(waypoints[0, 0]),
                y_m=float(waypoints[0, 1]),
                start_yaw_rad=start_pose_world.yaw_rad,
                end_yaw_rad=final_yaw_rad,
                description="at the goal",
            )

        segment_yaws = np.arctan2(
            np.diff(waypoints[:, 1]),
            np.diff(waypoints[:, 0]),
        )

        validity = self._check_rotation(
            x_m=float(waypoints[0, 0]),
            y_m=float(waypoints[0, 1]),
            start_yaw_rad=start_pose_world.yaw_rad,
            end_yaw_rad=float(segment_yaws[0]),
            description="while aligning with the path",
        )
        if not validity.is_valid:
            return validity

        for segment_index, segment_yaw in enumerate(segment_yaws):
            validity = self._check_translation(
                start=waypoints[segment_index],
                end=waypoints[segment_index + 1],
                yaw_rad=float(segment_yaw),
                segment_index=segment_index,
            )
            if not validity.is_valid:
                return validity

            if segment_index + 1 < len(segment_yaws):
                corner = waypoints[segment_index + 1]
                validity = self._check_rotation(
                    x_m=float(corner[0]),
                    y_m=float(corner[1]),
                    start_yaw_rad=float(segment_yaw),
                    end_yaw_rad=float(segment_yaws[segment_index + 1]),
                    description=(
                        f"while turning after segment {segment_index}"
                    ),
                )
                if not validity.is_valid:
                    return validity

        goal = waypoints[-1]
        return self._check_rotation(
            x_m=float(goal[0]),
            y_m=float(goal[1]),
            start_yaw_rad=float(segment_yaws[-1]),
            end_yaw_rad=final_yaw_rad,
            description="while turning to the goal orientation",
        )

    def pose_is_traversable(self, pose_world: Pose2d) -> bool:
        """Return whether the complete footprint is clear at one pose."""
        footprint_world = pose_world.transform_points(
            self._footprint.corners_base
        )
        footprint_grid_m = (
            self._geometry.origin_in_world.inverse().transform_points(
                footprint_world
            )
        )
        map_width_m = (
            self._geometry.width * self._geometry.resolution_m
        )
        map_height_m = (
            self._geometry.height * self._geometry.resolution_m
        )
        if (
            np.any(footprint_grid_m[:, 0] < 0.0)
            or np.any(footprint_grid_m[:, 0] >= map_width_m)
            or np.any(footprint_grid_m[:, 1] < 0.0)
            or np.any(footprint_grid_m[:, 1] >= map_height_m)
        ):
            return False

        resolution_m = self._geometry.resolution_m
        min_x = max(
            0,
            math.floor(np.min(footprint_grid_m[:, 0]) / resolution_m),
        )
        max_x = min(
            self._geometry.width - 1,
            math.floor(np.max(footprint_grid_m[:, 0]) / resolution_m),
        )
        min_y = max(
            0,
            math.floor(np.min(footprint_grid_m[:, 1]) / resolution_m),
        )
        max_y = min(
            self._geometry.height - 1,
            math.floor(np.max(footprint_grid_m[:, 1]) / resolution_m),
        )

        grid_x, grid_y = np.meshgrid(
            np.arange(min_x, max_x + 1),
            np.arange(min_y, max_y + 1),
        )
        cell_centers_grid = np.column_stack(
            (
                (grid_x.reshape(-1) + 0.5) * resolution_m,
                (grid_y.reshape(-1) + 0.5) * resolution_m,
            )
        )
        cell_centers_world = (
            self._geometry.origin_in_world.transform_points(
                cell_centers_grid
            )
        )
        cell_centers_base = pose_world.inverse().transform_points(
            cell_centers_world
        )

        relative_yaw = wrap_angle(
            self._geometry.origin_in_world.yaw_rad - pose_world.yaw_rad
        )
        cell_half_extent_m = 0.5 * resolution_m * (
            abs(math.cos(relative_yaw)) + abs(math.sin(relative_yaw))
        )
        raster_footprint = self._footprint.expanded(cell_half_extent_m)
        footprint_cells = raster_footprint.contains_points(
            cell_centers_base
        )

        for x, y in zip(
            grid_x.reshape(-1)[footprint_cells],
            grid_y.reshape(-1)[footprint_cells],
        ):
            if not self._costmap.is_traversable(int(x), int(y)):
                return False

        return True

    def _check_translation(
        self,
        start: np.ndarray,
        end: np.ndarray,
        yaw_rad: float,
        segment_index: int,
    ) -> FootprintPathValidity:
        distance_m = float(np.linalg.norm(end - start))
        sample_count = max(
            1,
            math.ceil(distance_m / self._position_step_m),
        )
        for sample_index in range(1, sample_count + 1):
            ratio = sample_index / sample_count
            point = start + ratio * (end - start)
            pose = Pose2d(
                x_m=float(point[0]),
                y_m=float(point[1]),
                yaw_rad=yaw_rad,
            )
            if not self.pose_is_traversable(pose):
                return FootprintPathValidity(
                    is_valid=False,
                    reason=(
                        "robot footprint intersects the map on path "
                        f"segment {segment_index}"
                    ),
                    collision_pose=pose,
                )

        return FootprintPathValidity(is_valid=True)

    def _check_rotation(
        self,
        x_m: float,
        y_m: float,
        start_yaw_rad: float,
        end_yaw_rad: float,
        description: str,
    ) -> FootprintPathValidity:
        yaw_delta = wrap_angle(end_yaw_rad - start_yaw_rad)
        sample_count = max(
            1,
            math.ceil(abs(yaw_delta) / self._yaw_step_rad),
        )
        for sample_index in range(sample_count + 1):
            ratio = sample_index / sample_count
            pose = Pose2d(
                x_m=x_m,
                y_m=y_m,
                yaw_rad=wrap_angle(start_yaw_rad + ratio * yaw_delta),
            )
            if not self.pose_is_traversable(pose):
                return FootprintPathValidity(
                    is_valid=False,
                    reason=f"robot footprint intersects the map {description}",
                    collision_pose=pose,
                )

        return FootprintPathValidity(is_valid=True)

    @staticmethod
    def _remove_duplicate_points(points: np.ndarray) -> np.ndarray:
        if len(points) <= 1:
            return points
        keep = np.ones(len(points), dtype=bool)
        keep[1:] = np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-9
        return points[keep]
