import math

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np

from ros_pathfinder.geometry.pose2d import Pose2d
from ros_pathfinder.localization.icp_scan_matcher import ICPResult, ICPScanMatcher

class LocalizationStatus(Enum):
    INITIALIZED = "initialized"
    STATIONARY = "stationary"
    ICP_ACCEPTED = "icp_accepted"
    NO_ICP_RESULT = "no_icp_result"
    NOT_CONVERGED = "not_converged"
    HIGH_RMSE = "high_rmse"
    LOW_INLIER_RATIO = "low_inlier_ratio"
    ODOM_DISAGREEMENT = "odom_disagreement"

@dataclass
class ScanLocalizationConfig:
    min_translation_before_match_m: float
    min_rotation_before_match_rad: float
    max_accepted_rmse_m: float
    min_inlier_ratio: float
    max_translation_correction_m: float
    max_rotation_correction_rad: float

@dataclass
class LocalizationUpdate:
    map_to_odom: Pose2d
    map_to_base: Pose2d
    chosen_delta: Pose2d
    status: LocalizationStatus
    icp_result: Optional[ICPResult]

class ScanLocalization:
    def __init__(self, icp_scan_matcher: ICPScanMatcher, config: ScanLocalizationConfig) -> None:
        self._icp_scan_matcher = icp_scan_matcher
        self._config = config

        self._prev_points_base: Optional[np.ndarray] = None
        self._prev_odom_pose: Optional[Pose2d] = None

        # corrected pose of base_link in map
        self._map_to_base: Optional[Pose2d] = None

    def update(self, current_points_base: np.ndarray, current_odom_pose: Pose2d) -> LocalizationUpdate:
        if self._prev_odom_pose is None:
            return self._initialize(current_points_base, current_odom_pose)

        odom_delta = self._prev_odom_pose.between(current_odom_pose)

        icp_result: Optional[ICPResult] = None

        if self._is_stationary(odom_delta):
            chosen_delta = odom_delta
            status = LocalizationStatus.STATIONARY
        else:
            icp_result = self._icp_scan_matcher.match(
                current_points_base=current_points_base,
                previous_points_base=self._prev_points_base,
                odom_initial_guess=odom_delta
            )

            status = self._evaluate_match(
                result=icp_result,
                odom_delta=odom_delta,
                current_point_count=current_points_base.shape[0]
            )

            if status is LocalizationStatus.ICP_ACCEPTED:
                chosen_delta = icp_result.delta
            else:
                chosen_delta = odom_delta

        # compose prev map_to_base with current motion
        self._map_to_base = self._map_to_base.compose(chosen_delta)

        map_to_odom = self._map_to_base.compose(current_odom_pose.inverse())

        self._store_reference_pair(current_points_base, current_odom_pose)

        return LocalizationUpdate(
            map_to_odom=map_to_odom,
            map_to_base=self._map_to_base,
            chosen_delta=chosen_delta,
            status=status,
            icp_result=icp_result
        )

    def _initialize(self, current_points_base: np.ndarray, current_odom_base: Pose2d) -> LocalizationUpdate:
        self._store_reference_pair(current_points_base, current_odom_base)

        self._map_to_base = current_odom_base

        return LocalizationUpdate(
            map_to_odom=Pose2d(),
            map_to_base=self._map_to_base,
            chosen_delta=Pose2d(),
            status=LocalizationStatus.INITIALIZED,
            icp_result=None
        )

    def _evaluate_match(self, result: Optional[ICPResult], odom_delta: Pose2d, current_point_count: int) -> LocalizationStatus:
        if result is None:
            return LocalizationStatus.NO_ICP_RESULT

        if not result.converged:
            return LocalizationStatus.NOT_CONVERGED

        if result.rmse_m > self._config.max_accepted_rmse_m:
            return LocalizationStatus.HIGH_RMSE

        inlier_ratio = result.match_count / current_point_count

        if inlier_ratio < self._config.min_inlier_ratio:
            return LocalizationStatus.LOW_INLIER_RATIO

        correction = odom_delta.between(result.delta)

        translation_correction_m = math.hypot(correction.x_m, correction.y_m)
        rot_correction_rad = abs(correction.yaw_rad)

        if (
            translation_correction_m > self._config.max_translation_correction_m
            or rot_correction_rad > self._config.max_rotation_correction_rad
        ):
            return LocalizationStatus.ODOM_DISAGREEMENT
        
        return LocalizationStatus.ICP_ACCEPTED

    def _is_stationary(self, odom_delta: Pose2d) -> bool:
        translation_m = math.hypot(odom_delta.x_m, odom_delta.y_m)
        rot_rad = abs(odom_delta.yaw_rad)

        return translation_m < self._config.min_translation_before_match_m and rot_rad < self._config.min_rotation_before_match_rad

    # store the pair together b/c they must reference the same timestamp
    def _store_reference_pair(self, points_base: np.ndarray, odom_pose: Pose2d) -> None:
        self._prev_points_base = points_base.copy()

        self._prev_odom_pose = Pose2d(x_m=odom_pose.x_m, y_m=odom_pose.y_m, yaw_rad=odom_pose.yaw_rad)