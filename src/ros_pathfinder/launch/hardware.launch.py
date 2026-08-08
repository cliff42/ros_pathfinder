from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_config = PathJoinSubstitution([
        FindPackageShare("ros_pathfinder"),
        "config",
        "hardware.yaml"
    ])

    return LaunchDescription([
        Node(
            package="ros_pathfinder",
            executable="motor_driver",
            name="motor_driver",
            parameters=[
                hardware_config
            ],
            output="both"
        ),
    ])