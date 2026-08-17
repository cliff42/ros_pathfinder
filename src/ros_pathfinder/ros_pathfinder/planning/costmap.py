import math

from dataclasses import dataclass

import numpy as np

UNKNOWN = -1
FREE = 0
OBSTACLE = 100

@dataclass
class CostmapConfig:
    robot_radius_m: float
    safety_margin_m: float
    occupied_threshold: int = 65
    allow_unknown: bool = False
    unknown_cost_multiplier: float = 3.0

class Costmap2d:
    def __init__(self, values: np.ndarray, resolution_m: float, config: CostmapConfig,x_m: float,y_m: float) -> None:
        self._values = values
        self._resolution_m = resolution_m
        self._config = config
        self.x_m = x_m
        self.y_m = y_m

    @classmethod
    def from_occupancy(
        cls,
        occupancy_values: np.ndarray,
        resolution_m: float,
        config: CostmapConfig,
    ) -> "Costmap2d":
        values = np.asarray(occupancy_values, dtype=np.int16).copy()

        if values.ndim != 2:
            raise ValueError("occupancy_values must have shape (height, width)")
        if resolution_m <= 0.0:
            raise ValueError("resolution_m must be greater than zero")
        if config.robot_radius_m < 0.0:
            raise ValueError("robot_radius_m cannot be negative")
        if config.safety_margin_m < 0.0:
            raise ValueError("safety_margin_m cannot be negative")
        if not 0 <= config.occupied_threshold <= 100:
            raise ValueError("occupied_threshold must be in [0, 100]")
        if config.unknown_cost_multiplier < 1.0:
            raise ValueError("unknown_cost_multiplier must be at least 1.0")

        unknown_mask = values < 0
        obstacle_mask = values >= config.occupied_threshold

        values[unknown_mask] = UNKNOWN
        values[~unknown_mask] = np.clip(
            values[~unknown_mask],
            FREE,
            OBSTACLE
        )

        clearance_radius_m = config.robot_radius_m + config.safety_margin_m

        collision_mask = cls._inflate_obstacles(
            obstacle_mask=obstacle_mask,
            resolution_m=resolution_m,
            radius_m=clearance_radius_m,
        )

        cls._mark_map_boundary(
            collision_mask=collision_mask,
            resolution_m=resolution_m,
            radius_m=clearance_radius_m,
        )

        # inflate the grid
        values[collision_mask] = OBSTACLE
        return cls(
            values=values,
            resolution_m=resolution_m,
            config=config
        )

    @property
    def values(self) -> np.ndarray:
        return self._values

    @property
    def width(self) -> int:
        return self._values.shape[1]

    @property
    def height(self) -> int:
        return self._values.shape[0]

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def is_traversable(self, x: int, y: int) -> bool:
        if not self.in_bounds(x, y):
            return False

        value = int(self._values[y, x])

        if value == UNKNOWN:
            return self._config.allow_unknown

        return value < OBSTACLE

    def traversal_multiplier(self, x: int, y: int) -> float:
        if not self.is_traversable(x, y):
            return math.inf

        value = int(self._values[y, x])

        if value == UNKNOWN:
            return self._config.unknown_cost_multiplier

        return 1.0

    @staticmethod
    def _inflate_obstacles(
        obstacle_mask: np.ndarray,
        resolution_m: float,
        radius_m: float,
    ) -> np.ndarray:
        inflated = obstacle_mask.copy()

        if radius_m <= 0.0 or not np.any(obstacle_mask):
            return inflated

        radius_cells = math.ceil(radius_m / resolution_m)
        obstacle_y, obstacle_x = np.nonzero(obstacle_mask)

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                distance_m = math.hypot(dx, dy) * resolution_m

                if distance_m > radius_m:
                    continue

                shifted_y = obstacle_y + dy
                shifted_x = obstacle_x + dx

                valid = (
                    (shifted_x >= 0)
                    & (shifted_x < obstacle_mask.shape[1])
                    & (shifted_y >= 0)
                    & (shifted_y < obstacle_mask.shape[0])
                )

                inflated[shifted_y[valid], shifted_x[valid]] = True

        return inflated

    @staticmethod
    def _mark_map_boundary(
        collision_mask: np.ndarray,
        resolution_m: float,
        radius_m: float,
    ) -> None:
        border_cells = math.ceil(radius_m / resolution_m)

        if border_cells <= 0:
            return None

        collision_mask[:border_cells, :] = True
        collision_mask[-border_cells:, :] = True
        collision_mask[:, :border_cells] = True
        collision_mask[:, -border_cells:] = True
