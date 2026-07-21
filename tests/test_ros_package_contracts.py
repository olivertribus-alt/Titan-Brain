"""Static executable checks for the ROS 2 package and message skeletons."""

from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree

REPOSITORY_ROOT = Path(__file__).parents[1]
ROS_SOURCE = REPOSITORY_ROOT / "ros2_ws" / "src"


def _message_fields(path: Path) -> list[str]:
    return [
        line
        for raw_line in path.read_text(encoding="utf-8").splitlines()
        if (line := raw_line.strip()) and not line.startswith("#")
    ]


def _package_dependencies(path: Path) -> set[str]:
    root = ElementTree.parse(path).getroot()
    dependency_tags = {
        "build_depend",
        "buildtool_depend",
        "depend",
        "exec_depend",
        "test_depend",
    }
    return {
        element.text or ""
        for element in root
        if element.tag in dependency_tags
    }


def test_titan_brain_message_contracts_are_explicit() -> None:
    messages = ROS_SOURCE / "titan_brain_msgs" / "msg"

    assert _message_fields(messages / "SafetyObservation.msg") == [
        "std_msgs/Header header",
        "string map_id",
        "geometry_msgs/Pose2D pose",
        "float64 clearance_m",
        "float64 confidence",
        "string sensor_id",
    ]
    assert _message_fields(messages / "SafetyEvaluationStatus.msg") == [
        "std_msgs/Header header",
        "string schema_version",
        "string adapter_status",
        "string watchdog_status",
        "bool watchdog_healthy",
        "bool observation_accepted",
        "string decision_id",
        "string action",
        "string rule",
        "bool is_incident",
        "string detail",
    ]


def test_message_package_declares_rosidl_and_message_dependencies() -> None:
    package = ROS_SOURCE / "titan_brain_msgs"
    dependencies = _package_dependencies(package / "package.xml")
    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")

    assert {
        "ament_cmake",
        "geometry_msgs",
        "rosidl_default_generators",
        "rosidl_default_runtime",
        "std_msgs",
    } <= dependencies
    assert "rosidl_generate_interfaces(${PROJECT_NAME}" in cmake
    assert '"msg/SafetyObservation.msg"' in cmake
    assert '"msg/SafetyEvaluationStatus.msg"' in cmake


def test_node_package_declares_runtime_dependencies_and_entry_point() -> None:
    package = ROS_SOURCE / "titan_brain_ros"
    dependencies = _package_dependencies(package / "package.xml")
    setup = (package / "setup.py").read_text(encoding="utf-8")

    assert {"rclpy", "tf2_ros", "titan_brain_msgs"} <= dependencies
    assert (
        "titan_brain_ros.safety_observation_node:main"
        in setup
    )
    assert (package / "resource" / "titan_brain_ros").is_file()
    assert (
        package / "titan_brain_ros" / "safety_observation_node.py"
    ).is_file()


def test_ci_contains_a_real_jazzy_runtime_gate() -> None:
    workflow = (REPOSITORY_ROOT / ".github" / "workflows" / "ci.yml").read_text(
        encoding="utf-8"
    )

    assert "ros-tooling/setup-ros@v0.7" in workflow
    assert "required-ros-distributions: jazzy" in workflow
    assert "colcon --log-base ros2_ws/log build" in workflow
    assert "test_safety_observation_node.py" in workflow
    assert "python3 -m venv --system-site-packages" in workflow
    assert '"${TB_CI_VENV}/bin/python" -m pytest' in workflow
    assert "--break-system-packages" not in workflow
    assert (
        "source /opt/ros/jazzy/setup.bash\n"
        '          "${TB_CI_VENV}/bin/python" -c "import pydantic, rclpy"'
        in workflow
    )
