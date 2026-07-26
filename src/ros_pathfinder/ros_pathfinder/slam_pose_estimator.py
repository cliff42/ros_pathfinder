import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformBroadcaster,
    TransformListener,
)
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

import numpy as np
from sklearn.neighbors import KDTree

import math

HEADER_FRAME = 'slam_odom' # corrected odometry frame from encoder prediction + lidar correction
CHILD_FRAME = 'base_link' # robot frame
RAW_ODOM_FRAME = 'raw_odom'


class LidarOdometry(Node):

    def __init__(self):
        super().__init__('lidar_odometry')
        self.use_icp_correction = bool(
            self.declare_parameter('use_icp_correction', False).value
        )
        self.debug_icp = bool(
            self.declare_parameter('debug_icp', False).value
        )
        self.process_icp = self.use_icp_correction or self.debug_icp

        self.min_icp_translation = float(
            self.declare_parameter('min_icp_translation', 0.005).value
        )
        self.min_icp_rotation = float(
            self.declare_parameter('min_icp_rotation', 0.003).value
        )
        self.min_icp_matches = int(
            self.declare_parameter('min_icp_matches', 40).value
        )
        self.icp_match_distance = float(
            self.declare_parameter('icp_match_distance', 0.20).value
        )
        self.icp_trim_fraction = float(
            self.declare_parameter('icp_trim_fraction', 0.70).value
        )
        self.max_icp_rmse = float(
            self.declare_parameter('max_icp_rmse', 0.065).value
        )
        self.max_icp_translation_error = float(
            self.declare_parameter('max_icp_translation_error', 0.04).value
        )
        self.max_icp_rotation_error = float(
            self.declare_parameter('max_icp_rotation_error', 0.04).value
        )
        self.initial_std_xy = float(
            self.declare_parameter('initial_std_xy', 0.10).value
        )
        self.initial_std_yaw = float(
            self.declare_parameter('initial_std_yaw', 0.10).value
        )
        self.process_std_xy_per_sqrt_s = float(
            self.declare_parameter(
                'process_std_xy_per_sqrt_s',
                0.10,
            ).value
        )
        self.process_std_yaw_per_sqrt_s = float(
            self.declare_parameter(
                'process_std_yaw_per_sqrt_s',
                0.071,
            ).value
        )
        self.stationary_process_std_xy_per_sqrt_s = float(
            self.declare_parameter(
                'stationary_process_std_xy_per_sqrt_s',
                0.001,
            ).value
        )
        self.stationary_process_std_yaw_per_sqrt_s = float(
            self.declare_parameter(
                'stationary_process_std_yaw_per_sqrt_s',
                0.0003,
            ).value
        )
        self.icp_measurement_std_xy = float(
            self.declare_parameter('icp_measurement_std_xy', 0.05).value
        )
        self.icp_measurement_std_yaw = float(
            self.declare_parameter('icp_measurement_std_yaw', 0.04).value
        )
        self.max_ekf_nis = float(
            self.declare_parameter('max_ekf_nis', 11.34).value
        )

        # Kept temporarily so existing launch commands do not fail. EKF
        # correction strength is determined by P and R, not this old gain.
        self.declare_parameter('icp_correction_gain', 0.18)

        noise_parameters = {
            'initial_std_xy': self.initial_std_xy,
            'initial_std_yaw': self.initial_std_yaw,
            'process_std_xy_per_sqrt_s': self.process_std_xy_per_sqrt_s,
            'process_std_yaw_per_sqrt_s': self.process_std_yaw_per_sqrt_s,
            'stationary_process_std_xy_per_sqrt_s':
                self.stationary_process_std_xy_per_sqrt_s,
            'stationary_process_std_yaw_per_sqrt_s':
                self.stationary_process_std_yaw_per_sqrt_s,
            'icp_measurement_std_xy': self.icp_measurement_std_xy,
            'icp_measurement_std_yaw': self.icp_measurement_std_yaw,
        }
        invalid_noise_parameters = [
            name for name, value in noise_parameters.items() if value <= 0.0
        ]
        if invalid_noise_parameters:
            names = ', '.join(invalid_noise_parameters)
            raise ValueError(f'EKF standard deviations must be positive: {names}')
        if self.max_ekf_nis <= 0.0:
            raise ValueError('max_ekf_nis must be positive')

        self.slam_odom_publisher = self.create_publisher(Odometry, 'slam_odom', 10)
        self.scan_subscriber = None
        if self.process_icp:
            self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'raw_odom', self.odom_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.prev_points_tree = None
        self.prev_points = None

        # Planar EKF state mu = [x, y, yaw].
        self.mu = np.zeros(3)
        self.P = np.diag([
            self.initial_std_xy ** 2,
            self.initial_std_xy ** 2,
            self.initial_std_yaw ** 2,
        ])

        self.last_odom_time = None
        self.latest_odom_pose = None
        self.prev_scan_odom_pose = None
        self.prev_scan_mu_pose = None

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        self.corr_x = 0.0
        self.corr_y = 0.0
        self.corr_theta = 0.0

        # Last commanded/measured odom velocities after deadbanding.
        # Used by scan_callback to reject stationary ICP noise.
        self.last_v = 0.0
        self.last_w = 0.0
        self.latest_twist_covariance = [0.0] * 36
        self.logged_laser_transform = False
        self.warned_missing_laser_transform = False

        self.get_logger().info(
            f'slam pose estimator mode: use_icp_correction={self.use_icp_correction}, '
            f'debug_icp={self.debug_icp}'
        )
        self.get_logger().info(
            'icp_correction_gain is deprecated and ignored; EKF correction '
            'strength is determined by state and ICP covariance'
        )

    def odom_callback(self, msg: Odometry):
        now = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.last_odom_time is None:
            self.last_odom_time = now
            self.store_latest_odom_pose(msg)
            self.mu = self.latest_odom_pose.copy()
            self.update_correction_tf_state_from_mu()
            self.publish_correction_tf(msg.header.stamp)
            self.publish_slam_odom(msg.header.stamp)
            return
        dt = now - self.last_odom_time
        self.last_odom_time = now
        if dt <= 0.0:
            self.store_latest_odom_pose(msg)
            if not self.use_icp_correction:
                self.mu = self.latest_odom_pose.copy()
            self.update_correction_tf_state_from_mu()
            self.publish_correction_tf(msg.header.stamp)
            self.publish_slam_odom(msg.header.stamp)
            return

        v = msg.twist.twist.linear.x   # linear velocity  (m/s)
        w = msg.twist.twist.angular.z  # angular velocity (rad/s)

        # Treat tiny encoder/odom jitter as zero so it cannot integrate forever.
        V_DEADBAND = 0.005  # m/s
        W_DEADBAND = 0.005  # rad/s
        if abs(v) < V_DEADBAND:
            v = 0.0
        if abs(w) < W_DEADBAND:
            w = 0.0

        # Store deadbanded velocities for logging and stationary ICP rejection.
        self.last_v = v
        self.last_w = w

        self.store_latest_odom_pose(msg)
        self.ekf_predict(v, w, dt)
        if not self.use_icp_correction:
            # Preserve raw-odom passthrough mode while still tracking its
            # growing open-loop uncertainty in P.
            self.mu = self.latest_odom_pose.copy()
        self.update_correction_tf_state_from_mu()
        self.publish_correction_tf(msg.header.stamp)
        self.publish_slam_odom(msg.header.stamp)

    def ekf_predict(self, v, w, dt):
        theta = self.mu[2]

        self.mu[0] += v * math.cos(theta) * dt
        self.mu[1] += v * math.sin(theta) * dt
        self.mu[2] += w * dt
        self.mu[2] = math.atan2(math.sin(self.mu[2]), math.cos(self.mu[2]))

        F = np.array([
            [1.0, 0.0, -v * math.sin(theta) * dt],
            [0.0, 1.0,  v * math.cos(theta) * dt],
            [0.0, 0.0, 1.0],
        ])

        if v == 0.0 and w == 0.0:
            std_xy = self.stationary_process_std_xy_per_sqrt_s
            std_yaw = self.stationary_process_std_yaw_per_sqrt_s
        else:
            std_xy = self.process_std_xy_per_sqrt_s
            std_yaw = self.process_std_yaw_per_sqrt_s
        Q = np.diag([
            std_xy ** 2,
            std_xy ** 2,
            std_yaw ** 2,
        ]) * dt
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    def publish_correction_tf(self, stamp):
        corr_tf = TransformStamped()
        corr_tf.header.stamp = stamp
        corr_tf.header.frame_id = HEADER_FRAME
        corr_tf.child_frame_id = RAW_ODOM_FRAME
        corr_tf.transform.translation.x = self.corr_x
        corr_tf.transform.translation.y = self.corr_y
        corr_tf.transform.translation.z = 0.0
        corr_tf.transform.rotation.x = 0.0
        corr_tf.transform.rotation.y = 0.0
        corr_tf.transform.rotation.z = math.sin(self.corr_theta / 2.0)
        corr_tf.transform.rotation.w = math.cos(self.corr_theta / 2.0)
        self.tf_broadcaster.sendTransform(corr_tf)

    # EKF update/ correction step
    def scan_callback(self, msg: LaserScan):
        points = self.scan_to_base_points(msg)
        if points is None:
            return
        if points.shape[0] < 3:
            return

        if self.prev_points_tree is None:
            self.prev_points = points
            self.prev_points_tree = KDTree(points, leaf_size=10)
            if self.latest_odom_pose is not None:
                self.prev_scan_odom_pose = self.latest_odom_pose.copy()
            if self.mu is not None:
                self.prev_scan_mu_pose = self.mu.copy()
            return

        odom_delta = self.get_scan_to_scan_odom_delta()
        previous_mu_pose = None
        if self.prev_scan_mu_pose is not None:
            previous_mu_pose = self.prev_scan_mu_pose.copy()

        icp_result = self.run_icp(points, odom_delta)

        self.prev_points = points
        self.prev_points_tree = KDTree(points, leaf_size=10)
        if self.latest_odom_pose is not None:
            self.prev_scan_odom_pose = self.latest_odom_pose.copy()
        self.prev_scan_mu_pose = self.mu.copy()

        if icp_result is None:
            self.get_logger().info(
                f"ICP_REJECT reason=no_matches odom_delta="
                f"({odom_delta[0]:.5f}, {odom_delta[1]:.5f}, {odom_delta[2]:.5f})"
            )
            self.publish_slam_odom(msg.header.stamp)
            return

        dx, dy, dtheta, match_count, rmse, iterations = icp_result
        translation_error = math.hypot(dx - odom_delta[0], dy - odom_delta[1])
        rotation_error = self.wrap_angle(dtheta - odom_delta[2])

        self.get_logger().info(
            f"v={self.last_v:.5f}, w={self.last_w:.5f}, "
            f"odom_dx={odom_delta[0]:.5f}, odom_dy={odom_delta[1]:.5f}, "
            f"odom_dtheta={odom_delta[2]:.5f}, "
            f"icp_dx={dx:.5f}, icp_dy={dy:.5f}, icp_dtheta={dtheta:.5f}, "
            f"icp_matches={match_count}, icp_rmse={rmse:.5f}, "
            f"icp_iters={iterations}, trans_err={translation_error:.5f}, "
            f"rot_err={rotation_error:.5f}, "
            f"odom=({self.odom_x:.3f}, {self.odom_y:.3f}, {self.odom_theta:.3f}), "
            f"mu=({self.mu[0]:.3f}, {self.mu[1]:.3f}, {self.mu[2]:.3f})"
        )

        # If odom says the robot is stationary, do not let scan-to-scan ICP
        # accumulate LaserScan jitter as fake motion. We already updated
        # prev_points above, so the next scan still compares against the most
        # recent scan.
        STILL_V = 0.005  # m/s
        STILL_W = 0.005  # rad/s
        if abs(self.last_v) < STILL_V and abs(self.last_w) < STILL_W:
            self.get_logger().info(
                f"STATIONARY: rejecting ICP dx={dx:.5f}, "
                f"dy={dy:.5f}, dtheta={dtheta:.5f}"
            )
            self.publish_slam_odom(msg.header.stamp)
            return

        if self.debug_icp and not self.use_icp_correction:
            self.get_logger().info('ICP_DEBUG_ONLY: computed ICP but did not apply it')
            self.publish_slam_odom(msg.header.stamp)
            return

        if match_count < self.min_icp_matches:
            self.reject_icp(
                msg.header.stamp,
                'too_few_matches',
                dx,
                dy,
                dtheta,
                match_count,
                rmse,
            )
            return

        if rmse > self.max_icp_rmse:
            self.reject_icp(msg.header.stamp, 'rmse', dx, dy, dtheta, match_count, rmse)
            return

        if translation_error > self.max_icp_translation_error:
            self.reject_icp(
                msg.header.stamp,
                'translation_disagreement',
                dx,
                dy,
                dtheta,
                match_count,
                rmse,
            )
            return

        if abs(rotation_error) > self.max_icp_rotation_error:
            self.reject_icp(
                msg.header.stamp,
                'rotation_disagreement',
                dx,
                dy,
                dtheta,
                match_count,
                rmse,
            )
            return

        if (math.hypot(dx, dy) < self.min_icp_translation
                and abs(dtheta) < self.min_icp_rotation):
            self.reject_icp(msg.header.stamp, 'below_motion_threshold', dx, dy, dtheta, match_count, rmse)
            return

        if previous_mu_pose is None:
            self.reject_icp(msg.header.stamp, 'missing_previous_mu', dx, dy, dtheta, match_count, rmse)
            return

        measurement_pose = self.compose_pose(
            previous_mu_pose,
            np.array([dx, dy, dtheta]),
        )
        measurement_covariance = self.measurement_covariance_from_icp()
        accepted, innovation, kalman_gain, nis = self.ekf_update(
            measurement_pose,
            measurement_covariance,
        )

        if not accepted:
            self.get_logger().info(
                f"ICP_REJECT reason=ekf_nis nis={nis:.3f}, "
                f"max_nis={self.max_ekf_nis:.3f}, "
                f"innovation=({innovation[0]:.5f}, {innovation[1]:.5f}, "
                f"{innovation[2]:.5f})"
            )
            self.publish_slam_odom(msg.header.stamp)
            return

        self.get_logger().info(
            f"ICP_EKF_APPLY innovation=({innovation[0]:.5f}, "
            f"{innovation[1]:.5f}, {innovation[2]:.5f}), nis={nis:.3f}, "
            f"K_diag=({kalman_gain[0, 0]:.3f}, "
            f"{kalman_gain[1, 1]:.3f}, {kalman_gain[2, 2]:.3f}), "
            f"mu=({self.mu[0]:.3f}, {self.mu[1]:.3f}, {self.mu[2]:.3f})"
        )

        self.update_correction_tf_state_from_mu()
        self.prev_scan_mu_pose = self.mu.copy()
        self.publish_correction_tf(msg.header.stamp)
        self.publish_slam_odom(msg.header.stamp)

    def ekf_update(self, measurement_pose, measurement_covariance):
        H = np.eye(3)
        innovation = measurement_pose - H @ self.mu
        innovation[2] = self.wrap_angle(innovation[2])
        innovation_covariance = H @ self.P @ H.T + measurement_covariance

        try:
            solved_innovation = np.linalg.solve(
                innovation_covariance,
                innovation,
            )
            kalman_gain = np.linalg.solve(
                innovation_covariance.T,
                (self.P @ H.T).T,
            ).T
        except np.linalg.LinAlgError:
            return False, innovation, np.zeros((3, 3)), float('inf')

        nis = float(innovation.T @ solved_innovation)
        if not math.isfinite(nis) or nis > self.max_ekf_nis:
            return False, innovation, kalman_gain, nis

        self.mu = self.mu + kalman_gain @ innovation
        self.mu[2] = self.wrap_angle(self.mu[2])

        identity = np.eye(3)
        residual_factor = identity - kalman_gain @ H
        self.P = (
            residual_factor @ self.P @ residual_factor.T
            + kalman_gain @ measurement_covariance @ kalman_gain.T
        )
        self.P = 0.5 * (self.P + self.P.T)
        return True, innovation, kalman_gain, nis

    def measurement_covariance_from_icp(self):
        return np.diag([
            self.icp_measurement_std_xy ** 2,
            self.icp_measurement_std_xy ** 2,
            self.icp_measurement_std_yaw ** 2,
        ])

    def update_correction_tf_state_from_mu(self):
        if self.latest_odom_pose is None:
            return

        x, y, theta = self.mu
        theta_corr = self.wrap_angle(theta - self.odom_theta)
        c = math.cos(theta_corr)
        s = math.sin(theta_corr)
        self.corr_x = x - (c * self.odom_x - s * self.odom_y)
        self.corr_y = y - (s * self.odom_x + c * self.odom_y)
        self.corr_theta = theta_corr

    def run_icp(self, points, odom_delta):
        MAX_ITER = 10
        CONVERGENCE_T = 1e-4
        CONVERGENCE_R = 1e-5

        R_total, t_total = self.pose_to_matrix(odom_delta)
        source_pts = self.transform_points(points, R_total, t_total)
        match_count = 0
        rmse = float('inf')
        iterations = 0

        for iteration in range(1, MAX_ITER + 1):
            matched_source_pts, matched_target_pts, dists = self.get_matches(source_pts)
            if matched_source_pts is None:
                break

            match_count = int(dists.size)
            rmse = float(math.sqrt(np.mean(dists * dists)))

            source_centroid = self.get_centroid(matched_source_pts)
            target_centroid = self.get_centroid(matched_target_pts)

            source_centered = matched_source_pts - source_centroid
            target_centered = matched_target_pts - target_centroid

            M = np.dot(source_centered.T, target_centered)
            U, W, V_t = np.linalg.svd(M)

            if np.linalg.det(V_t.T @ U.T) < 0:
                V_t[-1, :] *= -1
            R = V_t.T @ U.T
            t = target_centroid - R @ source_centroid

            t_total = R @ t_total + t
            R_total = R @ R_total
            source_pts = self.transform_points(points, R_total, t_total)
            iterations = iteration

            dtheta_step = math.atan2(R[1, 0], R[0, 0])
            if np.linalg.norm(t) < CONVERGENCE_T and abs(dtheta_step) < CONVERGENCE_R:
                break

        if match_count == 0:
            return None

        dtheta = math.atan2(R_total[1, 0], R_total[0, 0])
        dx = float(t_total[0])
        dy = float(t_total[1])
        return dx, dy, dtheta, match_count, rmse, iterations

    def reject_icp(self, stamp, reason, dx, dy, dtheta, match_count, rmse):
        self.get_logger().info(
            f"ICP_REJECT reason={reason} dx={dx:.5f}, dy={dy:.5f}, "
            f"dtheta={dtheta:.5f}, matches={match_count}, rmse={rmse:.5f}"
        )
        self.publish_slam_odom(stamp)

    def publish_slam_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = HEADER_FRAME
        odom.child_frame_id = CHILD_FRAME

        odom.pose.pose.position.x = float(self.mu[0])
        odom.pose.pose.position.y = float(self.mu[1])
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.mu[2] / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.mu[2] / 2.0)
        odom.pose.covariance = self.pose_covariance_3x3_to_6x6(self.P)

        odom.twist.twist.linear.x = self.last_v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.last_w
        odom.twist.covariance = self.latest_twist_covariance.copy()

        self.slam_odom_publisher.publish(odom)

    def pose_covariance_3x3_to_6x6(self, covariance):
        pose_covariance = [0.0] * 36
        state_indexes = (0, 1, 5)
        for state_row, pose_row in enumerate(state_indexes):
            for state_col, pose_col in enumerate(state_indexes):
                pose_covariance[pose_row * 6 + pose_col] = float(
                    covariance[state_row, state_col]
                )

        pose_covariance[2 * 6 + 2] = 1e6
        pose_covariance[3 * 6 + 3] = 1e6
        pose_covariance[4 * 6 + 4] = 1e6
        return pose_covariance

    def store_latest_odom_pose(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.odom_theta = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        self.latest_odom_pose = np.array([self.odom_x, self.odom_y, self.odom_theta])
        self.latest_twist_covariance = list(msg.twist.covariance)

    def get_scan_to_scan_odom_delta(self):
        if self.prev_scan_odom_pose is None or self.latest_odom_pose is None:
            return np.zeros(3)
        return self.relative_pose(self.prev_scan_odom_pose, self.latest_odom_pose)

    # convert global diff into robot local frame
    def relative_pose(self, previous_pose, current_pose):
        dx = current_pose[0] - previous_pose[0]
        dy = current_pose[1] - previous_pose[1]
        theta = previous_pose[2]

        c = math.cos(theta)
        s = math.sin(theta)

        local_dx = c * dx + s * dy
        local_dy = -s * dx + c * dy
        local_dtheta = math.atan2(
            math.sin(current_pose[2] - previous_pose[2]),
            math.cos(current_pose[2] - previous_pose[2])
        )
        return np.array([local_dx, local_dy, local_dtheta])

    def compose_pose(self, pose, delta):
        c = math.cos(pose[2])
        s = math.sin(pose[2])
        return np.array([
            pose[0] + c * delta[0] - s * delta[1],
            pose[1] + s * delta[0] + c * delta[1],
            self.wrap_angle(pose[2] + delta[2]),
        ])

    def pose_to_matrix(self, pose):
        c = math.cos(pose[2])
        s = math.sin(pose[2])
        R = np.array([[c, -s], [s, c]]) # rot matrix
        t = np.array([pose[0], pose[1]]) # translation vector
        return R, t

    def transform_points(self, points, R, t):
        return (R @ points.T).T + t

    def scan_to_base_points(self, scan_msg):
        ranges = np.array(scan_msg.ranges, dtype=float)
        angles = (scan_msg.angle_min
                  + np.arange(len(ranges)) * scan_msg.angle_increment)
        valid = (np.isfinite(ranges)
                 & (ranges >= scan_msg.range_min)
                 & (ranges <= scan_msg.range_max))
        r = ranges[valid]
        a = angles[valid]
        if r.size == 0:
            return np.empty((0, 2), dtype=float)
        laser_points = np.column_stack((r * np.cos(a), r * np.sin(a)))
        laser_to_base = self.lookup_laser_to_base(scan_msg.header.frame_id)
        if laser_to_base is None:
            return None

        yaw, translation = laser_to_base
        R, _ = self.pose_to_matrix(np.array([0.0, 0.0, yaw]))
        return self.transform_points(laser_points, R, translation)

    def lookup_laser_to_base(self, laser_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                CHILD_FRAME,
                laser_frame,
                rclpy.time.Time(),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            if not self.warned_missing_laser_transform:
                self.get_logger().warn(
                    f'could not look up {CHILD_FRAME}->{laser_frame} transform; '
                    'skipping ICP until TF is available'
                )
                self.warned_missing_laser_transform = True
            return None

        translation = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
        ])
        q = tf.transform.rotation
        yaw = self.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        if not self.logged_laser_transform:
            self.get_logger().info(
                f'ICP scan frame transform: {laser_frame}->{CHILD_FRAME} '
                f'x={translation[0]:.3f}, y={translation[1]:.3f}, yaw={yaw:.3f}'
            )
            self.logged_laser_transform = True
        return yaw, translation

    def get_centroid(self, pts):
        return pts.mean(axis=0)

    def get_matches(self, source_pts):
        dists, idxs = self.prev_points_tree.query(source_pts, k=1)
        dists = dists[:, 0]
        idxs = idxs[:, 0].astype(int)
        mask = dists < self.icp_match_distance
        if mask.sum() < 3:
            return None, None, None

        matched_dists = dists[mask]
        matched_source_idxs = np.nonzero(mask)[0]
        keep_count = int(math.ceil(matched_dists.size * self.icp_trim_fraction))
        keep_count = max(3, min(matched_dists.size, keep_count))
        keep_order = np.argsort(matched_dists)[:keep_count]

        source_keep = matched_source_idxs[keep_order]
        target_keep = idxs[mask][keep_order]
        matched_src = source_pts[source_keep]
        matched_tgt = self.prev_points[target_keep]
        return matched_src, matched_tgt, matched_dists[keep_order]

    def yaw_from_quaternion(self, x, y, z, w):
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    try:
        with rclpy.init(args=args):
            odom_publisher = LidarOdometry()
            rclpy.spin(odom_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
