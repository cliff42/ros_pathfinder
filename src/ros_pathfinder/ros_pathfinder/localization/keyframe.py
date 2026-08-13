from dataclasses import dataclass

import numpy as np

from ros_pathfinder.geometry.pose2d import Pose2d
from ros_pathfinder.geometry.scan2d import ScanObservation2d

@dataclass
class Keyframe:
    id: int
    timestamp_ns: int

    scan: ScanObservation2d
    odom_pose: Pose2d

    @property
    def points_base(self) -> np.ndarray:
        return self.scan.hit_points_base