from dataclasses import dataclass

import numpy as np

@dataclass
class ScanObservation2d:
    # lidar origin in base_link
    sensor_origin_base: np.ndarray

    # ray endpoints in base_link
    ray_endpoints_base: np.ndarray

    # true for occupied, false otherwise
    hit_mask: np.ndarray

    @property
    def hit_points_base(self) -> np.ndarray:
        return self.ray_endpoints_base[self.hit_mask]