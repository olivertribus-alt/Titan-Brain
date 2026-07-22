"""Launch the complete Titan Brain ROS 2 safety transport pipeline."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PACKAGE_NAME = "titan_brain_ros"


def generate_launch_description() -> LaunchDescription:
    """Start observation evaluation and authoritative velocity arbitration."""
    package_share = Path(get_package_share_directory(_PACKAGE_NAME))
    default_config = package_share / "config" / "titan_brain.yaml"
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config),
                description="Absolute path to the shared Titan Brain YAML profile.",
            ),
            Node(
                package=_PACKAGE_NAME,
                executable="safety_observation_node",
                name="safety_observation_node",
                parameters=[config_file],
                output="screen",
            ),
            Node(
                package=_PACKAGE_NAME,
                executable="velocity_arbiter_node",
                name="velocity_arbiter_node",
                parameters=[config_file],
                output="screen",
            ),
            Node(
                package=_PACKAGE_NAME,
                executable="command_path_observability_node",
                name="command_path_observability_node",
                parameters=[config_file],
                output="screen",
            ),
            Node(
                package=_PACKAGE_NAME,
                executable="actuator_feedback_monitor_node",
                name="actuator_feedback_monitor_node",
                parameters=[config_file],
                output="screen",
            ),
            Node(
                package=_PACKAGE_NAME,
                executable="safety_loop_supervisor_node",
                name="safety_loop_supervisor_node",
                parameters=[config_file],
                output="screen",
            ),
        ]
    )
