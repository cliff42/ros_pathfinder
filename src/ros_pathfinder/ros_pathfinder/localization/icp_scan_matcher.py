import math
from dataclasses import dataclass
from typing import Optional

from ros_pathfinder.geometry.pose2d import Pose2d

from sklearn.neighbors import NearestNeighbors
import numpy as np

@dataclass
class ICPResult:
    delta: Pose2d
    match_count: int
    rmse_m: float
    iterations: int
    converged: bool

class ICPScanMatcher:
    def __init__(
        self,
        max_iterations: int = 30,
        max_correspondence_dist_m: float = 0.25,
        min_match_count: int = 30,
        translation_tolerance_m: float = 0.0005,
        rotation_tolerance_rad: float = 0.001,
    ) -> None:
        self._max_iterations = max_iterations
        self._max_correspondence_dist_m = max_correspondence_dist_m
        self._min_match_count = min_match_count
        self._translation_tolerance_m = translation_tolerance_m
        self._rotation_tolerance_rad = rotation_tolerance_rad

    # follows simple ICP algorithm: https://learnopencv.com/iterative-closest-point-icp-explained/
    def match(
        self,
        current_points_base: np.ndarray,
        previous_points_base: np.ndarray,
        odom_initial_guess: Pose2d
    ) -> Optional[ICPResult]:
        if (current_points_base.shape[0] < self._min_match_count or previous_points_base.shape[0] < self._min_match_count):
            return None

        nearest_neighbours = NearestNeighbors(n_neighbors=1)
        nearest_neighbours.fit(previous_points_base)

        estimate = Pose2d(x_m=odom_initial_guess.x_m, y_m=odom_initial_guess.y_m, yaw_rad=odom_initial_guess.yaw_rad)

        converged = False
        iterations = 0

        for _ in range(self._max_iterations):
            iterations += 1

            # transform the current scan into the previous scan's frame
            transformed_current = estimate.transform_points(current_points_base)

            distances, indices = nearest_neighbours.kneighbors(transformed_current)
            distances = distances[:, 0]
            indices = indices[:, 0]

            matches = distances <= self._max_correspondence_dist_m

            if np.count_nonzero(matches) < self._min_match_count:
                return None

            matched_current = transformed_current[matches]
            matched_previous = previous_points_base[indices[matches]]

            correction = self._estimate_rigid_transform(source_points=matched_current, target_points=matched_previous)

            estimate = correction.compose(estimate)

            translation_update = math.hypot(correction.x_m, correction.y_m)
            rot_update = abs(correction.yaw_rad)

            if translation_update <= self._translation_tolerance_m and rot_update <= self._rotation_tolerance_rad:
                converged = True
                break

        match_count, rmse_m = self._evaluate_result(
            current=current_points_base, 
            previous=previous_points_base, 
            estimate=estimate, 
            nearest_neighbours=nearest_neighbours
        )

        if match_count < self._min_match_count:
            return None

        return ICPResult(
            delta=estimate,
            match_count=match_count,
            rmse_m=rmse_m,
            iterations=iterations,
            converged=converged
        )

    def _estimate_rigid_transform(self, source_points: np.ndarray, target_points: np.ndarray) -> Pose2d:
        source_centroid = np.mean(source_points, axis=0)
        target_centroid = np.mean(target_points, axis=0)

        centered_source = source_points - source_centroid
        centered_target = target_points - target_centroid

        covariance = centered_source.T @ centered_target
        u, _, vt = np.linalg.svd(covariance)

        rot = vt.T @ u.T

        if np.linalg.det(rot) < 0.0:
            vt[-1, :] *= -1.0
            rot = vt.T @ u.T

        translation = target_centroid - rot @ source_centroid

        yaw = math.atan2(rot[1, 0], rot[0, 0])

        return Pose2d(x_m=float(translation[0]), y_m=float(translation[1]), yaw_rad=yaw)

    def _evaluate_result(self, current: np.ndarray, previous: np.ndarray, estimate: Pose2d, nearest_neighbours: NearestNeighbors) -> tuple[int, float]:
        transformed_current = estimate.transform_points(current)

        distances, _ = nearest_neighbours.kneighbors(transformed_current)
        distances = distances[:, 0]

        matched_distances = distances[distances <= self._max_correspondence_dist_m]

        if matched_distances.size == 0:
            return 0, math.inf

        rmse_m = math.sqrt(float(np.mean(np.square(matched_distances))))

        return int(matched_distances.size), rmse_m



