import math
from dataclasses import dataclass
from typing import Optional

from ros_pathfinder.ros_pathfinder.geometry.pose2d import Pose2d

import numpy as np

@dataclass
class ICPResult:
    delta: Pose2d
    match_count: int
    rmse_m: float
    iterations: int
    converged: bool

class ICPScanMatcher:
    def match(
        self,
        current_points_base: np.ndarray,
        previous_points_base: np.ndarray,
        initial_guess: Pose2d
    ) -> Optional[ICPResult]:
        pass