import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FootprintBox2d:
    min_x_m: float
    max_x_m: float
    min_y_m: float
    max_y_m: float

    def __post_init__(self) -> None:
        bounds = (
            self.min_x_m,
            self.max_x_m,
            self.min_y_m,
            self.max_y_m,
        )
        if not all(math.isfinite(value) for value in bounds):
            raise ValueError("footprint bounds must be finite")
        if self.min_x_m >= self.max_x_m:
            raise ValueError("footprint x bounds are invalid")
        if self.min_y_m >= self.max_y_m:
            raise ValueError("footprint y bounds are invalid")

    @property
    def circumscribed_radius_m(self) -> float:
        return max(
            math.hypot(x_m, y_m)
            for x_m in (self.min_x_m, self.max_x_m)
            for y_m in (self.min_y_m, self.max_y_m)
        )

    @property
    def corners_base(self) -> np.ndarray:
        return np.array(
            [
                [self.min_x_m, self.min_y_m],
                [self.max_x_m, self.min_y_m],
                [self.max_x_m, self.max_y_m],
                [self.min_x_m, self.max_y_m],
            ],
            dtype=float,
        )

    def expanded(self, padding_m: float) -> "FootprintBox2d":
        if not np.isfinite(padding_m) or padding_m < 0.0:
            raise ValueError("padding_m must be finite and non-negative")

        return FootprintBox2d(
            min_x_m=self.min_x_m - padding_m,
            max_x_m=self.max_x_m + padding_m,
            min_y_m=self.min_y_m - padding_m,
            max_y_m=self.max_y_m + padding_m,
        )

    def contains_points(self, points_base: np.ndarray) -> np.ndarray:
        points = np.asarray(points_base, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("points_base must have shape (N, 2)")

        return (
            (points[:, 0] >= self.min_x_m)
            & (points[:, 0] <= self.max_x_m)
            & (points[:, 1] >= self.min_y_m)
            & (points[:, 1] <= self.max_y_m)
        )
