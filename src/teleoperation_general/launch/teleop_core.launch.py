"""Launch teleop core manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = LaunchConfiguration("config")
    start_keyboard = LaunchConfiguration("start_keyboard")
    start_observation = LaunchConfiguration("start_observation")

    default_config = PathJoinSubstitution([
        FindPackageShare("teleoperation_general"),
        "config",
        "teleop_core.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to teleop core YAML config.",
        ),
        DeclareLaunchArgument(
            "start_keyboard",
            default_value="false",
            description="Start the keyboard hardware input node.",
        ),
        DeclareLaunchArgument(
            "start_observation",
            default_value="true",
            description="Start the unified observation publisher node.",
        ),
        Node(
            package="teleoperation_general",
            executable="teleop_manager",
            name="teleop_manager",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="teleoperation_general",
            executable="keyboard_servo",
            name="keyboard_servo",
            output="screen",
            parameters=[config],
            condition=IfCondition(start_keyboard),
        ),
        Node(
            package="teleoperation_general",
            executable="observation_publisher",
            name="observation_publisher",
            output="screen",
            parameters=[config],
            condition=IfCondition(start_observation),
        ),
    ])
