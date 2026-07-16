#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${LOG_DIR:-"$ROOT_DIR/log/stack_$(date +%Y%m%d_%H%M%S)"}"

START_MOTORS=1
START_LIDAR=0
LIDAR_PACKAGE=""
LIDAR_LAUNCH_FILE=""
AUTO_PICK_GOAL=0
AUTO_GO_FORWARD_3M=0
ODOM_INITIAL_HEADING=""
LIDAR_YAW=""

usage() {
    cat <<'EOF'
Usage: ./start_pathfinder_stack.sh [options]

Starts the ros_pathfinder navigation stack:
  odom_node, lidar_static_tf, slam_pose_estimator, occupancy, planner,
  path_follower, controller, motor_controller, and goal_picker.

Options:
  --with-lidar <package> <launch.py>  Start a LiDAR launch file first.
  --no-motors                        Do not start controller or motor_controller.
  --pick-goal                        Publish one /pick_goal trigger after startup.
  --go-forward-3m                    Publish one /go_forward_3m trigger after startup.
  --initial-heading <radians>        Start base_link at this yaw in odom.
  --initial-heading-pi               Start base_link rotated 180 degrees.
  --laser-yaw <radians>              Set base_link->laser yaw.
  --laser-yaw-pi                     Rotate base_link->laser by 180 degrees.
  -h, --help                         Show this help.

Examples:
  ./start_pathfinder_stack.sh
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
        --initial-heading)
            ODOM_INITIAL_HEADING="${2:-}"
            if [[ -z "$ODOM_INITIAL_HEADING" ]]; then
                echo "error: --initial-heading requires <radians>" >&2
                exit 2
            fi
            shift 2
            ;;
        --initial-heading-pi)
            ODOM_INITIAL_HEADING="3.141592653589793"
            shift
            ;;
        --laser-yaw)
            LIDAR_YAW="${2:-}"
            if [[ -z "$LIDAR_YAW" ]]; then
                echo "error: --laser-yaw requires <radians>" >&2
                exit 2
            fi
            shift 2
            ;;
        --laser-yaw-pi)
            LIDAR_YAW="3.141592653589793"
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
if [[ -n "$ODOM_INITIAL_HEADING" ]]; then
    odom_cmd+=(--ros-args -p "initial_heading:=$ODOM_INITIAL_HEADING")
fi

lidar_static_cmd=(ros2 run ros_pathfinder lidar_static_tf)
if [[ -n "$LIDAR_YAW" ]]; then
    lidar_static_cmd+=(--ros-args -p "yaw:=$LIDAR_YAW")
fi

start_process odom_node "${odom_cmd[@]}"
start_process lidar_static_tf "${lidar_static_cmd[@]}"
start_process slam_pose_estimator ros2 run ros_pathfinder slam_pose_estimator
start_process occupancy ros2 run ros_pathfinder occupancy
start_process planner ros2 run ros_pathfinder planner
start_process path_follower ros2 run ros_pathfinder path_follower
start_process goal_picker ros2 run ros_pathfinder goal_picker

if [[ "$START_MOTORS" -eq 1 ]]; then
    start_process controller ros2 run ros_pathfinder controller
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
