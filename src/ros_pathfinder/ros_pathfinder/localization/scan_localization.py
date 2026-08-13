import math

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np

from ros_pathfinder.geometry.pose2d import Pose2d
from ros_pathfinder.localization.icp_scan_matcher import ICPResult, ICPScanMatcher
from ros_pathfinder.localization.keyframe import Keyframe

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
    keyframe_translation_threshold_m: float
    keyframe_rotation_threshold_rad: float

@dataclass
class LocalizationUpdate:
    map_to_odom: Pose2d
    map_to_base: Pose2d
    chosen_delta: Pose2d
    status: LocalizationStatus
    icp_result: Optional[ICPResult]
    created_keyframe: Optional[Keyframe]

class ScanLocalization:
    def __init__(self, icp_scan_matcher: ICPScanMatcher, config: ScanLocalizationConfig) -> None:
        self._icp_scan_matcher = icp_scan_matcher
        self._config = config

        self._prev_odom_pose: Optional[Pose2d] = None

        # corrected pose of base_link in map
        self._map_to_base: Optional[Pose2d] = None

        # current localization correction
        self._map_to_odom: Optional[Pose2d] = None

        self._active_keyframe: Optional[Keyframe] = None
        self._active_keyframe_map_pose: Optional[Pose2d] = None
        self._next_keyframe_id = 0

    def update(self, current_points_base: np.ndarray, current_odom_pose: Pose2d, timestamp_ns: int) -> LocalizationUpdate:
        if self._active_keyframe is None:
            return self._initialize_keyframe(
                current_points_base,
                current_odom_pose,
                timestamp_ns
            )

        prev_map_to_base = self._map_to_base
        incremental_odom_delta = self._prev_odom_pose.between(current_odom_pose)
        icp_result: Optional[ICPResult] = None

        if self._is_stationary(incremental_odom_delta):
            status = LocalizationStatus.STATIONARY
            map_to_base = self._map_to_odom.compose(current_odom_pose)
        else:
            keyframe_odom_delta = self._active_keyframe.odom_pose.between(current_odom_pose)

            icp_result = self._icp_scan_matcher.match(
                current_points_base=current_points_base,
                previous_points_base=self._active_keyframe.points_base,
                initial_transform=keyframe_odom_delta
            )

            status = self._evaluate_match(
                result=icp_result,
                odom_delta=keyframe_odom_delta,
                current_point_count=current_points_base.shape[0]
            )

            # TODO: eventually instead of just accepting ICP or odom, we want to fuse both with their covariance matricies
            if status is LocalizationStatus.ICP_ACCEPTED:
                map_to_base = self._active_keyframe_map_pose.compose(icp_result.delta)
            else:
                map_to_base = self._map_to_odom.compose(current_odom_pose)

        chosen_delta = prev_map_to_base.between(map_to_base)
        
        self._map_to_base = map_to_base
        self._map_to_odom = map_to_base.compose(current_odom_pose.inverse())

        created_keyframe = None

        if icp_result is not None and self._should_create_keyframe(icp_result.delta, status):
            created_keyframe = self._create_keyframe(
                current_points_base,
                current_odom_pose,
                map_to_base,
                timestamp_ns
            )

        self._prev_odom_pose = current_odom_pose

        return LocalizationUpdate(
            map_to_odom=self._map_to_odom,
            map_to_base=self._map_to_base,
            chosen_delta=chosen_delta,
            status=status,
            icp_result=icp_result,
            created_keyframe=created_keyframe
        )

    def _initialize_keyframe(self, current_points_base: np.ndarray, current_odom_pose: Pose2d, timestamp_ns: int) -> LocalizationUpdate:
        # map and odom start off the same
        self._map_to_odom = Pose2d()

        self._map_to_base = Pose2d(x_m=current_odom_pose.x_m, y_m=current_odom_pose.y_m, yaw_rad=current_odom_pose.yaw_rad)

        self._prev_odom_pose = Pose2d(
            x_m=current_odom_pose.x_m,
            y_m=current_odom_pose.y_m,
            yaw_rad=current_odom_pose.yaw_rad
        )

        created_keyframe = self._create_keyframe(
            points_base=current_points_base, 
            odom_pose=current_odom_pose, 
            map_pose=self._map_to_base, 
            timestamp_ns=timestamp_ns
        )

        return LocalizationUpdate(
            map_to_odom=self._map_to_odom,
            map_to_base=self._map_to_base,
            chosen_delta=Pose2d(),
            status=LocalizationStatus.INITIALIZED,
            icp_result=None,
            created_keyframe=created_keyframe
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

    def _should_create_keyframe(self, relative_pose: Pose2d, status: LocalizationStatus) -> bool:
        if status is not LocalizationStatus.ICP_ACCEPTED:
            return False

        translation_m = math.hypot(relative_pose.x_m, relative_pose.y_m)
        rot_rad = abs(relative_pose.yaw_rad)

        return translation_m >= self._config.keyframe_translation_threshold_m or rot_rad >= self._config.keyframe_rotation_threshold_rad

    def _create_keyframe(self, points_base: np.ndarray, odom_pose: Pose2d, map_pose: Pose2d, timestamp_ns: int) -> Keyframe:
        keyframe = Keyframe(
            id=self._next_keyframe_id,
            timestamp_ns=timestamp_ns,
            points_base=points_base.copy(),
            odom_pose=Pose2d(x_m=odom_pose.x_m, y_m=odom_pose.y_m, yaw_rad=odom_pose.yaw_rad)
        )

        self._next_keyframe_id += 1

        self._active_keyframe = keyframe
        self._active_keyframe_map_pose = map_pose

        return keyframe