import math

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np

from ros_pathfinder.geometry.pose2d import Pose2d
from ros_pathfinder.geometry.scan2d import ScanObservation2d
from ros_pathfinder.localization.icp_scan_matcher import ICPResult, ICPScanMatcher
from ros_pathfinder.localization.keyframe import Keyframe
from ros_pathfinder.localization.local_submap import LocalSubmap

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
    submap_max_keyframes: int
    submap_grid_size_m: float

@dataclass
class LocalizationUpdate:
    map_to_odom: Pose2d
    map_to_base: Pose2d
    chosen_delta: Pose2d
    status: LocalizationStatus
    icp_result: Optional[ICPResult]
    created_keyframe: Optional[Keyframe]
    completed_submap: Optional[LocalSubmap]

class ScanLocalization:
    def __init__(self, icp_scan_matcher: ICPScanMatcher, config: ScanLocalizationConfig) -> None:
        self._icp_scan_matcher = icp_scan_matcher
        self._config = config

        # corrected pose of base_link in map
        self._map_to_base: Optional[Pose2d] = None

        # current localization correction
        self._map_to_odom: Optional[Pose2d] = None

        self._active_submap: Optional[LocalSubmap] = None
        self._map_to_submap: Optional[Pose2d] = None
        self._next_keyframe_id = 0

        self._last_match_odom_pose: Optional[Pose2d] = None

    def update(self, current_scan: ScanObservation2d, current_odom_pose: Pose2d, timestamp_ns: int) -> LocalizationUpdate:
        current_points_base = current_scan.hit_points_base

        if self._active_submap is None:
            return self._initialize(
                current_scan,
                current_odom_pose,
                timestamp_ns
            )

        prev_map_to_base = self._map_to_base
        odom_since_last_match = self._last_match_odom_pose.between(
            current_odom_pose
        )
        icp_result: Optional[ICPResult] = None
        created_keyframe: Optional[Keyframe] = None
        completed_submap: Optional[LocalSubmap] = None

        if self._is_stationary(odom_since_last_match):
            status = LocalizationStatus.STATIONARY
            map_to_base = self._map_to_odom.compose(current_odom_pose)
        else:
            submap_origin_odom_pose = self._active_submap.get_origin_keyframe().odom_pose
            submap_to_base_guess = submap_origin_odom_pose.between(current_odom_pose)

            self._last_match_odom_pose = Pose2d(
                x_m=current_odom_pose.x_m, 
                y_m=current_odom_pose.y_m,
                yaw_rad=current_odom_pose.yaw_rad
            )

            icp_result = self._icp_scan_matcher.match(
                current_points_base=current_points_base,
                previous_points_base=self._active_submap.get_points(),
                initial_transform=submap_to_base_guess
            )

            status = self._evaluate_match(
                result=icp_result,
                odom_delta=submap_to_base_guess,
                current_point_count=current_points_base.shape[0]
            )

            # TODO: eventually instead of just accepting ICP or odom, we want to fuse both with their covariance matricies
            if status is LocalizationStatus.ICP_ACCEPTED:
                map_to_base = self._map_to_submap.compose(icp_result.delta)
            else:
                map_to_base = self._map_to_odom.compose(current_odom_pose)

        chosen_delta = prev_map_to_base.between(map_to_base)
        
        self._map_to_base = map_to_base
        self._map_to_odom = map_to_base.compose(current_odom_pose.inverse())

        if status is LocalizationStatus.ICP_ACCEPTED:
            last_keyframe_to_current = self._active_submap.get_last_keyframe_pose().between(icp_result.delta)

            if self._should_create_keyframe(relative_pose=last_keyframe_to_current, status=status):
                created_keyframe = self._create_keyframe(
                    scan=current_scan,
                    odom_pose=current_odom_pose,
                    timestamp_ns=timestamp_ns
                )

                self._active_submap.add_keyframe(keyframe=created_keyframe, submap_to_base=icp_result.delta)

                if self._active_submap.is_full():
                    completed_submap = self._active_submap

                    # new submap shares origin with completed submap
                    self._active_submap = LocalSubmap(
                        origin_keyframe=created_keyframe,
                        max_keyframes=self._config.submap_max_keyframes,
                        grid_size_m=self._config.submap_grid_size_m
                    )

                    self._map_to_submap = map_to_base

        return LocalizationUpdate(
            map_to_odom=self._map_to_odom,
            map_to_base=self._map_to_base,
            chosen_delta=chosen_delta,
            status=status,
            icp_result=icp_result,
            created_keyframe=created_keyframe,
            completed_submap=completed_submap
        )

    def _initialize(self, current_scan: ScanObservation2d, current_odom_pose: Pose2d, timestamp_ns: int) -> LocalizationUpdate:
        # map and odom start off the same
        self._map_to_odom = Pose2d()
        self._map_to_base = Pose2d(x_m=current_odom_pose.x_m, y_m=current_odom_pose.y_m, yaw_rad=current_odom_pose.yaw_rad)

        created_keyframe = self._create_keyframe(
            scan=current_scan, 
            odom_pose=current_odom_pose,
            timestamp_ns=timestamp_ns
        )

        self._active_submap = LocalSubmap(
            origin_keyframe=created_keyframe,
            max_keyframes=self._config.submap_max_keyframes,
            grid_size_m=self._config.submap_grid_size_m
        )

        self._map_to_submap = self._map_to_base

        self._last_match_odom_pose = Pose2d(x_m=current_odom_pose.x_m, y_m=current_odom_pose.y_m, yaw_rad=current_odom_pose.yaw_rad)

        return LocalizationUpdate(
            map_to_odom=self._map_to_odom,
            map_to_base=self._map_to_base,
            chosen_delta=Pose2d(),
            status=LocalizationStatus.INITIALIZED,
            icp_result=None,
            created_keyframe=created_keyframe,
            completed_submap=None
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

    def _create_keyframe(self, scan: ScanObservation2d, odom_pose: Pose2d, timestamp_ns: int) -> Keyframe:
        keyframe = Keyframe(
            id=self._next_keyframe_id,
            timestamp_ns=timestamp_ns,
            scan=scan,
            odom_pose=Pose2d(x_m=odom_pose.x_m, y_m=odom_pose.y_m, yaw_rad=odom_pose.yaw_rad)
        )
        self._next_keyframe_id += 1
        return keyframe