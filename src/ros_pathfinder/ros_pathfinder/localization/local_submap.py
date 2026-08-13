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
        self._max_keyframes = max_keyframes
        self._grid_size_m = grid_size_m

        self._entries = [
            SubmapKeyframe(keyframe=origin_keyframe, submap_to_base=Pose2d())
        ]

        self._points_submap = origin_keyframe.points_base.copy()

    def get_origin_keyframe(self) -> Keyframe:
        return self._entries[0].keyframe

    def get_last_keyframe_pose(self) -> Pose2d:
        return self._entries[-1].submap_to_base

    def get_points(self) -> np.ndarray:
        return self._points_submap

    def is_full(self) -> bool:
        return len(self._entries) >= self._max_keyframes

    def add_keyframe(self, keyframe: Keyframe, submap_to_base: Pose2d) -> None:
        points_submap = submap_to_base.transform_points(keyframe.points_base)

        self._entries.append(SubmapKeyframe(keyframe=keyframe, submap_to_base=submap_to_base))

        combined = np.vstack((self._points_submap, points_submap))

        self._points_submap = self._grid_downsample(combined)

    def _grid_downsample(self, points: np.ndarray) -> np.ndarray:
        cells = np.floor(points / self._grid_size_m).astype(np.int64)

        _, indices = np.unique(cells, axis=0, return_index=True)

        return points[np.sort(indices)]
