from dataclasses import dataclass

import numpy as np

from ros_pathfinder.geometry.pose2d import Pose2d

@dataclass
class Keyframe:
    id: int
    timestamp_ns: int
    
    points_base: np.ndarray
    odom_pose: Pose2d