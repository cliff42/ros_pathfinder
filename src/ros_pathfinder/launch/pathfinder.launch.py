from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_config = PathJoinSubstitution([
        FindPackageShare("ros_pathfinder"),
        "config",
        "robot.yaml"
    ])

    calibration_config = PathJoinSubstitution([
        FindPackageShare("ros_pathfinder"),
        "config",
        "calibration.yaml"
    ])

    control_config = PathJoinSubstitution([
        FindPackageShare("ros_pathfinder"),
        "config",
        "control.yaml"
    ])

    hardware_config = PathJoinSubstitution([
        FindPackageShare("ros_pathfinder"),
        "config",
        "hardware.yaml"
    ])

    robot_description_file = PathJoinSubstitution([
        FindPackageShare("ros_pathfinder"),
        "urdf",
        "pathfinder.urdf.xacro"
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
            }],
            output="both"
        ),
        Node(
            package="ros_pathfinder",
            executable="motor_driver",
            name="motor_driver",
            parameters=[
                hardware_config
            ],
            output="both"
        ),
        Node(
            package="ros_pathfinder",
            executable="velocity_controller",
            name="velocity_controller",
            parameters=[
                robot_config,
                control_config
            ],
            output="both"
        ),
        Node(
            package="ros_pathfinder",
            executable="wheel_state",
            name="wheel_state",
            parameters=[hardware_config],
            output="both"
        ),
        Node(
            package="ros_pathfinder",
            executable="imu",
            name="imu",
            parameters=[hardware_config, calibration_config],
            output="both"
        ),
        Node(
            package="ros_pathfinder",
            executable="local_odometry",
            name="local_odometry",
            parameters=[robot_config, calibration_config],
            output="both"
        ),
        Node(
            package="ros_pathfinder",
            executable="slam_node",
            name="slam_node",
            parameters=[],
            output="both"
        ),
    ])
