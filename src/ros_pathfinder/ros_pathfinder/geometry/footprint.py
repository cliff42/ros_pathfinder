from dataclasses import dataclass

import numpy as np

@dataclass
class FootprintBox2d:
    min_x_m: float
    max_x_m: float
    min_y_m: float
    max_y_m: float

    def contains_points(self, points_base: np.ndarray):
        return (
            (points_base[:, 0] >= self.min_x_m)
            & (points_base[:, 0] <= self.max_x_m)
            & (points_base[:, 1] >= self.min_y_m)
            & (points_base[:, 1] <= self.max_y_m)
        )