import math

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ros_pathfinder.planning.costmap import Costmap2d
from ros_pathfinder.planning.grid_geometry import GridGeometry2d
from ros_pathfinder.planning.path_simplification import (
    grid_line_is_traversable,
)


@dataclass
class PathValidityResult:
    is_valid: bool
    reason: str = ""
    blocked_waypoint_index: Optional[int] = None
    current_waypoint_index: int = 0


def validate_remaining_path(
    costmap: Costmap2d,
    grid_geometry: GridGeometry2d,
    robot_position_world: tuple[float, float],
    path_points_world: np.ndarray,
    previous_waypoint_index: int = 0,
) -> PathValidityResult:
    if (
        grid_geometry.width != costmap.width
        or grid_geometry.height != costmap.height
    ):
        raise ValueError("grid geometry dimensions do not match the costmap")

    points = np.asarray(path_points_world, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        raise ValueError("path_points_world must have shape (N, 2), N > 0")
    if not np.all(np.isfinite(points)):
        raise ValueError("path_points_world must contain finite values")

    robot_x, robot_y = (float(value) for value in robot_position_world)
    if not np.isfinite(robot_x) or not np.isfinite(robot_y):
        raise ValueError("robot_position_world must contain finite values")

    previous_index = min(
        max(int(previous_waypoint_index), 0),
        len(points) - 1,
    )
    waypoint_index = _estimate_current_waypoint_index(
        points=points,
        robot_position=np.array([robot_x, robot_y], dtype=float),
        previous_waypoint_index=previous_index,
    )

    robot_cell = grid_geometry.world_to_cell(robot_x, robot_y)
    if robot_cell is None:
        return PathValidityResult(
            is_valid=False,
            reason="robot is outside the costmap",
            current_waypoint_index=waypoint_index,
        )

    previous_cell = robot_cell

    for index in range(waypoint_index, len(points)):
        point_x, point_y = points[index]
        waypoint_cell = grid_geometry.world_to_cell(point_x, point_y)
        if waypoint_cell is None:
            return PathValidityResult(
                is_valid=False,
                reason="remaining path leaves the costmap",
                blocked_waypoint_index=index,
                current_waypoint_index=waypoint_index,
            )

        if not grid_line_is_traversable(
            costmap,
            previous_cell,
            waypoint_cell,
        ):
            return PathValidityResult(
                is_valid=False,
                reason=f"path is blocked before waypoint {index}",
                blocked_waypoint_index=index,
                current_waypoint_index=waypoint_index,
            )

        previous_cell = waypoint_cell

    return PathValidityResult(
        is_valid=True,
        current_waypoint_index=waypoint_index,
    )


def _estimate_current_waypoint_index(
    points: np.ndarray,
    robot_position: np.ndarray,
    previous_waypoint_index: int,
) -> int:
    """Return the next waypoint on the polyline, without moving backwards."""
    if len(points) == 1:
        return 0

    first_segment_index = max(previous_waypoint_index - 1, 0)
    best_segment_index = first_segment_index
    best_projection_ratio = 0.0
    best_distance_squared = math.inf

    for segment_index in range(first_segment_index, len(points) - 1):
        start = points[segment_index]
        segment = points[segment_index + 1] - start
        segment_length_squared = float(np.dot(segment, segment))
        if segment_length_squared <= 1e-12:
            projection_ratio = 0.0
            projection = start
        else:
            projection_ratio = float(
                np.clip(
                    np.dot(robot_position - start, segment)
                    / segment_length_squared,
                    0.0,
                    1.0,
                )
            )
            projection = start + projection_ratio * segment

        offset = robot_position - projection
        distance_squared = float(np.dot(offset, offset))
        if distance_squared <= best_distance_squared:
            best_segment_index = segment_index
            best_projection_ratio = projection_ratio
            best_distance_squared = distance_squared

    if best_projection_ratio <= 0.0 and best_distance_squared > 1e-12:
        estimated_index = best_segment_index
    else:
        estimated_index = best_segment_index + 1

    return max(previous_waypoint_index, estimated_index)
