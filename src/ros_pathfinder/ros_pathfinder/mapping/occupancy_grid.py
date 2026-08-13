import math

from dataclasses import dataclass

import numpy as np

from ros_pathfinder.localization.keyframe import Keyframe
from ros_pathfinder.geometry.pose2d import Pose2d


@dataclass
class OccupancyGridConfig:
    resolution_m: float
    width: int
    height: int
    origin_x_m: float
    origin_y_m: float

    hit_probability: float
    miss_probability: float

    min_probability: float
    max_probability: float

    free_probability_threshold: float
    occupied_probability_threshold: float

class OccupancyGrid2d:
    def __init__(self, config: OccupancyGridConfig):
        self._config = config

        self._log_odds = np.zeros((config.height, config.width), dtype=float)
        self._observed = np.zeros((config.height, config.width), dtype=bool)

        self._hit_update = self._probability_to_log_odds(config.hit_probability)
        self._miss_update = self._probability_to_log_odds(config.miss_probability)
        self._min_log_odds = self._probability_to_log_odds(config.min_probability)
        self._max_log_odds = self._probability_to_log_odds(config.max_probability)
        self._free_threshold = self._probability_to_log_odds(config.free_probability_threshold)
        self._occupied_threshold = self._probability_to_log_odds(config.occupied_probability_threshold)

    def integrate_keyframe(self, keyframe: Keyframe, map_to_base: Pose2d) -> None:
        scan = keyframe.scan

        origin_map = map_to_base.transform_points(scan.sensor_origin_base.reshape(1, 2))[0]

        endpoints_map = map_to_base.transform_points(scan.ray_endpoints_base)

        for endpoint_map, is_hit in zip(endpoints_map, scan.hit_mask):
            self._integrate_ray(origin_map=origin_map, endpoint_map=endpoint_map, endpoint_is_occupied=bool(is_hit))

        pass

    def occupancy_values(self) -> np.ndarray:
        values = np.full(self._log_odds.shape, -1, dtype=np.int8)

        free = self._observed & (self._log_odds <= self._free_threshold)
        occupied = self._observed & (self._log_odds >= self._occupied_threshold)

        values[free] = 0
        values[occupied] = 100

        return values

    def _integrate_ray(self, origin_map: np.ndarray, endpoint_map: np.ndarray, endpoint_is_occupied: bool):
        start_x, start_y = self._world_to_grid(origin_map)
        end_x, end_y = self._world_to_grid(endpoint_map)

        if not self._in_bounds(start_x, start_y):
            return

        for x, y in self._bresenham_cells(start_x, start_y, end_x, end_y):
            if not self._in_bounds(x, y):
                break

            is_endpoint = x == end_x and y == end_y

            if is_endpoint and endpoint_is_occupied:
                # occupied point
                self._update_cell(x=x, y=y, log_odds_update=self._hit_update)
            else:
                # free points
                self._update_cell(x=x, y=y, log_odds_update=self._miss_update)


    # https://www.geeksforgeeks.org/dsa/bresenhams-line-generation-algorithm/
    def _bresenham_cells(self, start_x: int, start_y: int, end_x: int, end_y: int):
        x = start_x
        y = start_y

        dx = abs(end_x - start_x)
        dy = abs(end_y - start_y)

        step_x = 1 if start_x < end_x else -1
        step_y = 1 if start_y < end_y else -1

        error = dx - dy

        while True:
            yield x, y

            if x == end_x and y == end_y:
                break

            doubled_error = 2 * error

            if doubled_error > -dy:
                error -= dy
                x += step_x

            if doubled_error < dx:
                error += dx
                y += step_y

    def _update_cell(self, x: int, y: int, log_odds_update: float) -> None:
        if not self._in_bounds(x, y):
            return

        self._observed[y, x] = True

        updated = self._log_odds[y, x] + log_odds_update

        self._log_odds[y, x] = np.clip(updated, self._min_log_odds, self._max_log_odds)

    def _world_to_grid(self, point: np.ndarray) -> tuple[int, int]:
        x = math.floor((point[0] - self._config.origin_x_m) / self._config.resolution_m)
        y = math.floor((point[1] - self._config.origin_y_m) / self._config.resolution_m)

        return x, y

    def _in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self._config.width and 0 <= y < self._config.height

    def _probability_to_log_odds(self, probability: float) -> float:
        return math.log(probability / (1.0 - probability))
