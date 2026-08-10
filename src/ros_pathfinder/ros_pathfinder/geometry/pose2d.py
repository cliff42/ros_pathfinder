import math
from dataclasses import dataclass

import numpy as np

from ros_pathfinder.util.util import wrap_angle

# helper fns for working with 2d poses
@dataclass
class Pose2d:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0

    def compose(self, child: "Pose2d") -> "Pose2d":
        cos = math.cos(self.yaw_rad)
        sin = math.sin(self.yaw_rad)

        return Pose2d(
            x_m=(self.x_m + cos * child.x_m - sin * child.y_m),
            y_m=(self.y_m + sin * child.x_m + cos * child.y_m),
            yaw_rad=wrap_angle(self.yaw_rad + child.yaw_rad)
        )
    
    def inverse(self) -> "Pose2d":
        cos = math.cos(self.yaw_rad)
        sin = math.sin(self.yaw_rad)

        return Pose2d(
            x_m=(-cos * self.x_m - sin * self.y_m),
            y_m=(sin * self.x_m - cos * self.y_m),
            yaw_rad=(-wrap_angle(self.yaw_rad))
        )

    def between(self, other: "Pose2d") -> "Pose2d":
        return self.inverse().compose(other)

    def transform_points(self, points: np.ndarray):
        cos = math.cos(self.yaw_rad)
        sin = math.sin(self.yaw_rad)

        rot = np.array([
            [cos, -sin],
            [sin, cos]
        ])

        translation = np.array([self.x_m, self.y_m])

        return (rot @ points.T).T + translation