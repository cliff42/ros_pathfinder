from dataclasses import dataclass

import numpy as np


@dataclass
class FootprintBox2d:
    min_x_m: float
    max_x_m: float
    min_y_m: float
    max_y_m: float

    def expanded(self, padding_m: float) -> "FootprintBox2d":
        if not np.isfinite(padding_m) or padding_m < 0.0:
            raise ValueError("padding_m must be finite and non-negative")

        return FootprintBox2d(
            min_x_m=self.min_x_m - padding_m,
            max_x_m=self.max_x_m + padding_m,
            min_y_m=self.min_y_m - padding_m,
            max_y_m=self.max_y_m + padding_m,
        )

    def contains_points(self, points_base: np.ndarray):
        return (
            (points_base[:, 0] >= self.min_x_m)
            & (points_base[:, 0] <= self.max_x_m)
            & (points_base[:, 1] >= self.min_y_m)
            & (points_base[:, 1] <= self.max_y_m)
        )
