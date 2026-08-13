from dataclasses import dataclass

import numpy as np

from ros_pathfinder.geometry.pose2d import Pose2d
from ros_pathfinder.localization.keyframe import Keyframe

@dataclass
class SubmapKeyframe:
    keyframe: Keyframe

    # pose of keyframe's base_link in submap frame
    submap_to_base: Pose2d

class LocalSubmap:
    def __init__(self, origin_keyframe: Keyframe, max_keyframes: int, grid_size_m: float) -> None:
        pass