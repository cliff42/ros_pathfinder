import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from ros_pathfinder.geometry.pose2d import Pose2d
from ros_pathfinder.planning.base_planner import GridCell


@dataclass
class GridGeometry2d:
    width: int
    height: int
    resolution_m: float
    origin_in_world: Pose2d

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid width and height must be greater than zero")
        if self.resolution_m <= 0.0:
            raise ValueError("grid resolution must be greater than zero")

    def world_to_cell(self, x_m: float, y_m: float) -> Optional[GridCell]:
        point_world = np.array([[x_m, y_m]], dtype=float)
        point_grid = self.origin_in_world.inverse().transform_points(point_world)[0]

        grid_x = math.floor(point_grid[0] / self.resolution_m)
        grid_y = math.floor(point_grid[1] / self.resolution_m)

        if not self.in_bounds(grid_x, grid_y):
            return None

        return grid_x, grid_y

    def cell_center_in_world(self, cell: GridCell) -> tuple[float, float]:
        grid_x, grid_y = cell
        if not self.in_bounds(grid_x, grid_y):
            raise ValueError(f"grid cell is outside map bounds: {cell}")

        point_grid = np.array([[
            (grid_x + 0.5) * self.resolution_m,
            (grid_y + 0.5) * self.resolution_m,
        ]])
        point_world = self.origin_in_world.transform_points(point_grid)[0]

        return float(point_world[0]), float(point_world[1])

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height
