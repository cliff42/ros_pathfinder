#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${LOG_DIR:-"$ROOT_DIR/log/stack_$(date +%Y%m%d_%H%M%S)"}"

START_MOTORS=1
START_IMU=1
START_LIDAR=0
LIDAR_PACKAGE=""
LIDAR_LAUNCH_FILE=""
AUTO_PICK_GOAL=0
AUTO_GO_FORWARD_3M=0

usage() {
    cat <<'EOF'
Usage: ./start_pathfinder_stack.sh [options]

Starts the ros_pathfinder navigation stack:
  imu_node, odom_node, lidar_static_tf, slam_pose_estimator, occupancy, planner,
  path_follower, controller, motor_controller, and goal_picker.

Options:
  --with-lidar <package> <launch.py>  Start a LiDAR launch file first.
  --no-imu                           Use encoder angular velocity only.
  --no-motors                        Do not start controller or motor_controller.
  --pick-goal                        Publish one /pick_goal trigger after startup.
  --go-forward-3m                    Publish one /go_forward_3m trigger after startup.
  -h, --help                         Show this help.

Environment:
  ROBOT_WHEEL_TRACK_M=0.24           Wheel contact center-to-center distance.
  ROBOT_LENGTH_M=0.25                Robot footprint length.
  ROBOT_WIDTH_M=0.24                 Robot footprint width.
  LIDAR_X_M=<measured>                LiDAR X offset from base_link.
  LIDAR_Y_M=<measured>                LiDAR Y offset from base_link.
  LIDAR_YAW_RAD=<measured>            LiDAR yaw relative to base_link.
  ODOM_IMU_YAW_SIGN=1.0              Map IMU Z rate into base_link yaw direction.
  ODOM_IMU_YAW_BIAS_RAD_S=0.0        Bias subtracted from the mapped IMU rate.
  ODOM_IMU_YAW_DEADBAND_RAD_S=0.005  Zero small stationary IMU yaw rates.
  ODOM_IMU_TIMEOUT_S=0.1             Fall back to encoders after this IMU age.
  SLAM_DEBUG_ICP=true                 Compute/log ICP without applying correction.
  SLAM_USE_ICP_CORRECTION=true        Fuse gated ICP poses into the EKF.
  SLAM_ICP_MAX_RMSE=0.065             Override max accepted ICP RMSE.
  SLAM_ICP_MEASUREMENT_STD_XY=0.05    ICP x/y measurement standard deviation.
  SLAM_ICP_MEASUREMENT_STD_YAW=0.04   ICP yaw measurement standard deviation.
  SLAM_EKF_PROCESS_STD_XY=0.10        Moving x/y process std per sqrt(second).
  SLAM_EKF_PROCESS_STD_YAW=0.071      Moving yaw process std per sqrt(second).
  SLAM_EKF_MAX_NIS=11.34              Innovation gate (99% chi-square, 3 DoF).
  SLAM_ICP_CORRECTION_GAIN=0.18       Deprecated; accepted but ignored.
  OCCUPANCY_FOOTPRINT_PADDING=0.02    Padding used only to remove self returns.
  OCCUPANCY_INFLATION_MARGIN=0.10     Clearance beyond the robot half-diagonal.
  PLANNER_ALLOW_UNKNOWN=true          Permit paths through unknown map cells.
  PLANNER_UNKNOWN_COST_MULTIPLIER=3.0 Prefer observed free space over unknown.
  PLANNER_PATH_SPACING=0.10           Output spacing after line-of-sight cleanup.
  PLANNER_REPLAN_CONFIRMATIONS=2      Blocked maps required before replanning.
  PLANNER_REPLAN_COOLDOWN_S=1.5       Minimum active-path age before replanning.
  PATH_FOLLOWER_LOOKAHEAD_DIST=0.30   Override path follower lookahead.
  PATH_FOLLOWER_ANGULAR_GAIN=1.0      Override path follower angular gain.
  PATH_FOLLOWER_ANGULAR_SMOOTHING=0.35 Low-pass factor for steering commands.
  CONTROLLER_USE_ODOM_FEEDBACK=true   Apply bounded wheel-speed feedback.
  CONTROLLER_KP=0.60                  Wheel-speed feedback proportional gain.

Examples:
  ./start_pathfinder_stack.sh
  SLAM_DEBUG_ICP=true ./start_pathfinder_stack.sh --no-motors
  ./start_pathfinder_stack.sh --with-lidar sllidar_ros2 sllidar_a1_launch.py
  ./start_pathfinder_stack.sh --no-motors
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --with-lidar)
            START_LIDAR=1
            LIDAR_PACKAGE="${2:-}"
            LIDAR_LAUNCH_FILE="${3:-}"
            if [[ -z "$LIDAR_PACKAGE" || -z "$LIDAR_LAUNCH_FILE" ]]; then
                echo "error: --with-lidar requires <package> and <launch.py>" >&2
                exit 2
            fi
            shift 3
            ;;
        --no-imu)
            START_IMU=0
            shift
            ;;
        --no-motors)
            START_MOTORS=0
            shift
            ;;
        --pick-goal)
            AUTO_PICK_GOAL=1
            shift
            ;;
        --go-forward-3m)
            AUTO_GO_FORWARD_3M=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown option: $1" >&2
            usage
            exit 2
            ;;
    esac
done

source_ros() {
    if command -v ros2 >/dev/null 2>&1; then
        return
    fi

    if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
        source_setup_file "/opt/ros/$ROS_DISTRO/setup.bash"
    else
        for distro in jazzy humble iron rolling foxy; do
            if [[ -f "/opt/ros/$distro/setup.bash" ]]; then
                source_setup_file "/opt/ros/$distro/setup.bash"
                break
            fi
        done
    fi

    if ! command -v ros2 >/dev/null 2>&1; then
        echo "error: ros2 not found. Source your ROS environment first." >&2
        exit 1
    fi
}

source_setup_file() {
    local setup_file="$1"
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
}

source_envs() {
    source_ros

    if [[ -f "$ROOT_DIR/.venv/bin/activate" ]]; then
        source_setup_file "$ROOT_DIR/.venv/bin/activate"
    fi

    if [[ ! -f "$ROOT_DIR/install/setup.bash" ]]; then
        echo "error: $ROOT_DIR/install/setup.bash not found." >&2
        echo "Run: cd $ROOT_DIR && colcon build --symlink-install" >&2
        exit 1
    fi

    source_setup_file "$ROOT_DIR/install/setup.bash"
}

pids=()
labels=()

start_process() {
    local label="$1"
    shift

    echo "starting $label"
    "$@" >"$LOG_DIR/$label.log" 2>&1 &
    pids+=("$!")
    labels+=("$label")
}

cleanup() {
    trap - INT TERM EXIT
    echo
    echo "stopping stack..."
    for pid in "${pids[@]:-}"; do
        kill "$pid" >/dev/null 2>&1 || true
    done
    wait >/dev/null 2>&1 || true
}

monitor_processes() {
    while true; do
        for i in "${!pids[@]}"; do
            local pid="${pids[$i]}"
            local label="${labels[$i]}"
            if ! kill -0 "$pid" >/dev/null 2>&1; then
                if [[ "$label" == "pick_goal" || "$label" == "go_forward_3m" ]]; then
                    continue
                fi
                echo "process exited: $label"
                echo "log: $LOG_DIR/$label.log"
                cleanup
                exit 1
            fi
        done
        sleep 1
    done
}

source_envs
mkdir -p "$LOG_DIR"

trap cleanup INT TERM EXIT

echo "logs: $LOG_DIR"

if [[ "$START_LIDAR" -eq 1 ]]; then
    start_process lidar ros2 launch "$LIDAR_PACKAGE" "$LIDAR_LAUNCH_FILE"
fi

odom_cmd=(ros2 run ros_pathfinder odom_node)
odom_params=()
if [[ "$START_IMU" -eq 1 ]]; then
    start_process imu_node ros2 run ros_pathfinder imu_node
else
    odom_params+=(-p "use_imu_angular_velocity:=false")
    echo "imu disabled: odometry will use encoder angular velocity"
fi
if [[ -n "${ROBOT_WHEEL_TRACK_M:-}" ]]; then
    odom_params+=(-p "wheel_track_m:=$ROBOT_WHEEL_TRACK_M")
fi
if [[ -n "${ODOM_IMU_YAW_SIGN:-}" ]]; then
    odom_params+=(-p "imu_yaw_sign:=$ODOM_IMU_YAW_SIGN")
fi
if [[ -n "${ODOM_IMU_YAW_BIAS_RAD_S:-}" ]]; then
    odom_params+=(-p "imu_yaw_bias_rad_s:=$ODOM_IMU_YAW_BIAS_RAD_S")
fi
if [[ -n "${ODOM_IMU_YAW_DEADBAND_RAD_S:-}" ]]; then
    odom_params+=(-p "imu_yaw_deadband_rad_s:=$ODOM_IMU_YAW_DEADBAND_RAD_S")
fi
if [[ -n "${ODOM_IMU_TIMEOUT_S:-}" ]]; then
    odom_params+=(-p "imu_timeout_s:=$ODOM_IMU_TIMEOUT_S")
fi
if [[ "${#odom_params[@]}" -gt 0 ]]; then
    odom_cmd+=(--ros-args "${odom_params[@]}")
fi
start_process odom_node "${odom_cmd[@]}"
lidar_tf_cmd=(ros2 run ros_pathfinder lidar_static_tf)
lidar_tf_params=()
if [[ -n "${LIDAR_X_M:-}" ]]; then
    lidar_tf_params+=(-p "x:=$LIDAR_X_M")
fi
if [[ -n "${LIDAR_Y_M:-}" ]]; then
    lidar_tf_params+=(-p "y:=$LIDAR_Y_M")
fi
if [[ -n "${LIDAR_YAW_RAD:-}" ]]; then
    lidar_tf_params+=(-p "yaw:=$LIDAR_YAW_RAD")
fi
if [[ "${#lidar_tf_params[@]}" -gt 0 ]]; then
    lidar_tf_cmd+=(--ros-args "${lidar_tf_params[@]}")
fi
start_process lidar_static_tf "${lidar_tf_cmd[@]}"
slam_cmd=(ros2 run ros_pathfinder slam_pose_estimator)
slam_params=()
if [[ -n "${SLAM_USE_ICP_CORRECTION:-}" ]]; then
    slam_params+=(-p "use_icp_correction:=$SLAM_USE_ICP_CORRECTION")
fi
if [[ -n "${SLAM_DEBUG_ICP:-}" ]]; then
    slam_params+=(-p "debug_icp:=$SLAM_DEBUG_ICP")
fi
if [[ -n "${SLAM_ICP_CORRECTION_GAIN:-}" ]]; then
    slam_params+=(-p "icp_correction_gain:=$SLAM_ICP_CORRECTION_GAIN")
fi
if [[ -n "${SLAM_EKF_INITIAL_STD_XY:-}" ]]; then
    slam_params+=(-p "initial_std_xy:=$SLAM_EKF_INITIAL_STD_XY")
fi
if [[ -n "${SLAM_EKF_INITIAL_STD_YAW:-}" ]]; then
    slam_params+=(-p "initial_std_yaw:=$SLAM_EKF_INITIAL_STD_YAW")
fi
if [[ -n "${SLAM_EKF_PROCESS_STD_XY:-}" ]]; then
    slam_params+=(-p "process_std_xy_per_sqrt_s:=$SLAM_EKF_PROCESS_STD_XY")
fi
if [[ -n "${SLAM_EKF_PROCESS_STD_YAW:-}" ]]; then
    slam_params+=(-p "process_std_yaw_per_sqrt_s:=$SLAM_EKF_PROCESS_STD_YAW")
fi
if [[ -n "${SLAM_EKF_STATIONARY_PROCESS_STD_XY:-}" ]]; then
    slam_params+=(-p "stationary_process_std_xy_per_sqrt_s:=$SLAM_EKF_STATIONARY_PROCESS_STD_XY")
fi
if [[ -n "${SLAM_EKF_STATIONARY_PROCESS_STD_YAW:-}" ]]; then
    slam_params+=(-p "stationary_process_std_yaw_per_sqrt_s:=$SLAM_EKF_STATIONARY_PROCESS_STD_YAW")
fi
if [[ -n "${SLAM_ICP_MEASUREMENT_STD_XY:-}" ]]; then
    slam_params+=(-p "icp_measurement_std_xy:=$SLAM_ICP_MEASUREMENT_STD_XY")
fi
if [[ -n "${SLAM_ICP_MEASUREMENT_STD_YAW:-}" ]]; then
    slam_params+=(-p "icp_measurement_std_yaw:=$SLAM_ICP_MEASUREMENT_STD_YAW")
fi
if [[ -n "${SLAM_EKF_MAX_NIS:-}" ]]; then
    slam_params+=(-p "max_ekf_nis:=$SLAM_EKF_MAX_NIS")
fi
if [[ -n "${SLAM_ICP_MAX_RMSE:-}" ]]; then
    slam_params+=(-p "max_icp_rmse:=$SLAM_ICP_MAX_RMSE")
fi
if [[ -n "${SLAM_ICP_MATCH_DISTANCE:-}" ]]; then
    slam_params+=(-p "icp_match_distance:=$SLAM_ICP_MATCH_DISTANCE")
fi
if [[ -n "${SLAM_ICP_TRIM_FRACTION:-}" ]]; then
    slam_params+=(-p "icp_trim_fraction:=$SLAM_ICP_TRIM_FRACTION")
fi
if [[ -n "${SLAM_ICP_MAX_TRANSLATION_ERROR:-}" ]]; then
    slam_params+=(-p "max_icp_translation_error:=$SLAM_ICP_MAX_TRANSLATION_ERROR")
fi
if [[ -n "${SLAM_ICP_MAX_ROTATION_ERROR:-}" ]]; then
    slam_params+=(-p "max_icp_rotation_error:=$SLAM_ICP_MAX_ROTATION_ERROR")
fi
if [[ "${#slam_params[@]}" -gt 0 ]]; then
    slam_cmd+=(--ros-args "${slam_params[@]}")
fi
start_process slam_pose_estimator "${slam_cmd[@]}"
occupancy_cmd=(ros2 run ros_pathfinder occupancy)
occupancy_params=()
if [[ -n "${ROBOT_LENGTH_M:-}" ]]; then
    occupancy_params+=(-p "robot_length:=$ROBOT_LENGTH_M")
fi
if [[ -n "${ROBOT_WIDTH_M:-}" ]]; then
    occupancy_params+=(-p "robot_width:=$ROBOT_WIDTH_M")
fi
if [[ -n "${OCCUPANCY_FOOTPRINT_PADDING:-}" ]]; then
    occupancy_params+=(
        -p "footprint_padding:=$OCCUPANCY_FOOTPRINT_PADDING"
    )
fi
if [[ -n "${OCCUPANCY_INFLATION_MARGIN:-}" ]]; then
    occupancy_params+=(
        -p "inflation_margin:=$OCCUPANCY_INFLATION_MARGIN"
    )
fi
if [[ "${#occupancy_params[@]}" -gt 0 ]]; then
    occupancy_cmd+=(--ros-args "${occupancy_params[@]}")
fi
start_process occupancy "${occupancy_cmd[@]}"

planner_cmd=(ros2 run ros_pathfinder planner)
planner_params=()
if [[ -n "${PLANNER_ALLOW_UNKNOWN:-}" ]]; then
    planner_params+=(-p "allow_unknown:=$PLANNER_ALLOW_UNKNOWN")
fi
if [[ -n "${PLANNER_UNKNOWN_COST_MULTIPLIER:-}" ]]; then
    planner_params+=(
        -p "unknown_cost_multiplier:=$PLANNER_UNKNOWN_COST_MULTIPLIER"
    )
fi
if [[ -n "${PLANNER_PATH_SPACING:-}" ]]; then
    planner_params+=(-p "path_spacing:=$PLANNER_PATH_SPACING")
fi
if [[ -n "${PLANNER_REPLAN_CONFIRMATIONS:-}" ]]; then
    planner_params+=(
        -p "replan_confirmations:=$PLANNER_REPLAN_CONFIRMATIONS"
    )
fi
if [[ -n "${PLANNER_REPLAN_COOLDOWN_S:-}" ]]; then
    planner_params+=(
        -p "replan_cooldown_s:=$PLANNER_REPLAN_COOLDOWN_S"
    )
fi
if [[ "${#planner_params[@]}" -gt 0 ]]; then
    planner_cmd+=(--ros-args "${planner_params[@]}")
fi
start_process planner "${planner_cmd[@]}"

path_follower_cmd=(ros2 run ros_pathfinder path_follower)
path_follower_params=()
if [[ -n "${PATH_FOLLOWER_LINEAR_VEL:-}" ]]; then
    path_follower_params+=(-p "linear_vel:=$PATH_FOLLOWER_LINEAR_VEL")
fi
if [[ -n "${PATH_FOLLOWER_GOAL_TOL:-}" ]]; then
    path_follower_params+=(-p "goal_tol:=$PATH_FOLLOWER_GOAL_TOL")
fi
if [[ -n "${PATH_FOLLOWER_LOOKAHEAD_DIST:-}" ]]; then
    path_follower_params+=(-p "lookahead_dist:=$PATH_FOLLOWER_LOOKAHEAD_DIST")
fi
if [[ -n "${PATH_FOLLOWER_ANGULAR_GAIN:-}" ]]; then
    path_follower_params+=(-p "angular_gain:=$PATH_FOLLOWER_ANGULAR_GAIN")
fi
if [[ -n "${PATH_FOLLOWER_MAX_ANGULAR_VEL:-}" ]]; then
    path_follower_params+=(-p "max_angular_vel:=$PATH_FOLLOWER_MAX_ANGULAR_VEL")
fi
if [[ -n "${PATH_FOLLOWER_ANGULAR_SMOOTHING:-}" ]]; then
    path_follower_params+=(
        -p "angular_smoothing:=$PATH_FOLLOWER_ANGULAR_SMOOTHING"
    )
fi
if [[ -n "${PATH_FOLLOWER_ANGULAR_DEADBAND:-}" ]]; then
    path_follower_params+=(
        -p "angular_deadband:=$PATH_FOLLOWER_ANGULAR_DEADBAND"
    )
fi
if [[ "${#path_follower_params[@]}" -gt 0 ]]; then
    path_follower_cmd+=(--ros-args "${path_follower_params[@]}")
fi
start_process path_follower "${path_follower_cmd[@]}"
start_process goal_picker ros2 run ros_pathfinder goal_picker

if [[ "$START_MOTORS" -eq 1 ]]; then
    controller_cmd=(ros2 run ros_pathfinder controller)
    controller_params=()
    if [[ -n "${ROBOT_WHEEL_TRACK_M:-}" ]]; then
        controller_params+=(-p "wheel_track_m:=$ROBOT_WHEEL_TRACK_M")
    fi
    if [[ -n "${CONTROLLER_USE_ODOM_FEEDBACK:-}" ]]; then
        controller_params+=(
            -p "use_odom_feedback:=$CONTROLLER_USE_ODOM_FEEDBACK"
        )
    fi
    if [[ -n "${CONTROLLER_KP:-}" ]]; then
        controller_params+=(-p "kp:=$CONTROLLER_KP")
    fi
    if [[ -n "${CONTROLLER_VELOCITY_FILTER_ALPHA:-}" ]]; then
        controller_params+=(
            -p "velocity_filter_alpha:=$CONTROLLER_VELOCITY_FILTER_ALPHA"
        )
    fi
    if [[ -n "${CONTROLLER_MAX_FEEDBACK_CORRECTION:-}" ]]; then
        controller_params+=(
            -p "max_feedback_correction:=$CONTROLLER_MAX_FEEDBACK_CORRECTION"
        )
    fi
    if [[ -n "${CONTROLLER_WHEEL_VELOCITY_TIMEOUT_S:-}" ]]; then
        controller_params+=(
            -p "wheel_velocity_timeout_s:=$CONTROLLER_WHEEL_VELOCITY_TIMEOUT_S"
        )
    fi
    if [[ -n "${CONTROLLER_LINEAR_SIGN:-}" ]]; then
        controller_params+=(-p "linear_sign:=$CONTROLLER_LINEAR_SIGN")
    fi
    if [[ -n "${CONTROLLER_ANGULAR_SIGN:-}" ]]; then
        controller_params+=(-p "angular_sign:=$CONTROLLER_ANGULAR_SIGN")
    fi
    if [[ -n "${CONTROLLER_LEFT_MOTOR_SIGN:-}" ]]; then
        controller_params+=(-p "left_motor_sign:=$CONTROLLER_LEFT_MOTOR_SIGN")
    fi
    if [[ -n "${CONTROLLER_RIGHT_MOTOR_SIGN:-}" ]]; then
        controller_params+=(-p "right_motor_sign:=$CONTROLLER_RIGHT_MOTOR_SIGN")
    fi
    if [[ -n "${CONTROLLER_LEFT_MOTOR_SCALE:-}" ]]; then
        controller_params+=(-p "left_motor_scale:=$CONTROLLER_LEFT_MOTOR_SCALE")
    fi
    if [[ -n "${CONTROLLER_RIGHT_MOTOR_SCALE:-}" ]]; then
        controller_params+=(-p "right_motor_scale:=$CONTROLLER_RIGHT_MOTOR_SCALE")
    fi
    if [[ "${#controller_params[@]}" -gt 0 ]]; then
        controller_cmd+=(--ros-args "${controller_params[@]}")
    fi

    start_process controller "${controller_cmd[@]}"
    start_process motor_controller ros2 run ros_pathfinder motor_controller
else
    echo "motors disabled: controller and motor_controller were not started"
fi

if [[ "$AUTO_PICK_GOAL" -eq 1 ]]; then
    (
        {
            sleep 5
            echo "publishing /pick_goal"
            ros2 topic pub --once --wait-matching-subscriptions 1 \
                /pick_goal std_msgs/msg/Empty "{}" \
                || ros2 topic pub --once /pick_goal std_msgs/msg/Empty "{}"
        } >"$LOG_DIR/pick_goal.log" 2>&1 || true
    ) &
    pids+=("$!")
    labels+=("pick_goal")
fi

if [[ "$AUTO_GO_FORWARD_3M" -eq 1 ]]; then
    (
        {
            sleep 5
            echo "publishing /go_forward_3m"
            ros2 topic pub --once --wait-matching-subscriptions 1 \
                /go_forward_3m std_msgs/msg/Empty "{}" \
                || ros2 topic pub --once /go_forward_3m std_msgs/msg/Empty "{}"
        } >"$LOG_DIR/go_forward_3m.log" 2>&1 || true
    ) &
    pids+=("$!")
    labels+=("go_forward_3m")
fi

echo
echo "stack is running. Press Ctrl-C to stop."
echo "trigger a goal with:"
echo '  ros2 topic pub --once /pick_goal std_msgs/msg/Empty "{}"'
echo "or go forward 3m with:"
echo '  ros2 topic pub --once /go_forward_3m std_msgs/msg/Empty "{}"'
echo

monitor_processes
