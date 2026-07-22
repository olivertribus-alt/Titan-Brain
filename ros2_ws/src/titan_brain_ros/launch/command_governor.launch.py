"""Launch the TB-EVAL-006B command governor node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PACKAGE_NAME = "titan_brain_ros"


def generate_launch_description() -> LaunchDescription:
    """Create a deterministic command-governor launch description."""
    package_share = Path(get_package_share_directory(_PACKAGE_NAME))
    default_config = package_share / "config" / "titan_brain.yaml"
    config_file = LaunchConfiguration("config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config),
                description="Titan Brain YAML profile containing governor parameters.",
            ),
            Node(
                package=_PACKAGE_NAME,
                executable="command_governor_node",
                name="command_governor_node",
                parameters=[config_file],
                output="screen",
            ),
        ]
    )
