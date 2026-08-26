import math
from typing import Optional

from ros_pathfinder.planning.base_planner import GridCell
from ros_pathfinder.planning.costmap import Costmap2d


def nearest_traversable_cell(
    costmap: Costmap2d,
    start: GridCell,
    maximum_distance_m: float,
) -> Optional[GridCell]:
    if not math.isfinite(maximum_distance_m) or maximum_distance_m < 0.0:
        raise ValueError(
            "maximum_distance_m must be finite and non-negative"
        )
    if not costmap.in_bounds(*start):
        raise ValueError(f"start cell {start} is outside the costmap")
    if costmap.is_traversable(*start):
        return start

    maximum_distance_cells = maximum_distance_m / costmap.resolution_m
    search_radius_cells = math.ceil(maximum_distance_cells)
    start_x, start_y = start
    candidates: list[tuple[float, int, int]] = []

    for y in range(
        max(0, start_y - search_radius_cells),
        min(costmap.height, start_y + search_radius_cells + 1),
    ):
        for x in range(
            max(0, start_x - search_radius_cells),
            min(costmap.width, start_x + search_radius_cells + 1),
        ):
            distance_squared = (
                (x - start_x) ** 2 + (y - start_y) ** 2
            )
            if distance_squared > maximum_distance_cells ** 2:
                continue
            if costmap.is_traversable(x, y):
                candidates.append((distance_squared, x, y))

    if not candidates:
        return None

    _, x, y = min(candidates)
    return x, y
