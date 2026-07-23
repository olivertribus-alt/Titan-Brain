"""Static executable checks for the ROS 2 package and message skeletons."""

from __future__ import annotations

import re
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


def _float_parameter(config_text: str, name: str) -> float:
    match = re.search(
        rf"^\s+{re.escape(name)}:\s+([0-9]+(?:\.[0-9]+)?)\s*$",
        config_text,
        flags=re.MULTILINE,
    )
    assert match is not None, f"Missing numeric ROS parameter {name!r}"
    return float(match.group(1))


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
    assert _message_fields(messages / "DirectionalSafetyObservation.msg") == [
        "std_msgs/Header header",
        "string map_id",
        "geometry_msgs/Pose2D pose",
        "float64 clearance_m",
        "float64 confidence",
        "string sensor_id",
        "float64 forward_clearance_m",
        "float64 reverse_clearance_m",
        "float64 left_clearance_m",
        "float64 right_clearance_m",
        "geometry_msgs/Twist velocity",
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
    assert _message_fields(messages / "SafetyStabilityStatus.msg") == [
        "uint8 STATE_OK=0",
        "uint8 STATE_WARNING=1",
        "uint8 STATE_E_STOP=2",
        "uint8 STATE_RECOVERY_HOLDING=3",
        "std_msgs/Header header",
        "string schema_version",
        "uint8 state",
        "string reason",
        "string instantaneous_action",
        "string effective_action",
        "bool recovery_active",
        "uint64 hold_elapsed_ns",
        "uint64 recovery_hold_time_ns",
        "bool has_release_threshold",
        "float64 release_threshold_m",
    ]
    assert _message_fields(messages / "EvaluatorObservabilityStatus.msg") == [
        "std_msgs/Header header",
        "string schema_version",
        "string policy_version",
        "string correlation_id",
        "string decision_id",
        "string outcome",
        "string latency_status",
        "bool timing_valid",
        "bool within_budget",
        "uint64 observation_timestamp_ns",
        "uint64 received_timestamp_ns",
        "uint64 decision_timestamp_ns",
        "uint64 published_timestamp_ns",
        "uint64 observation_to_receive_ns",
        "uint64 receive_to_decision_ns",
        "uint64 decision_to_publish_ns",
        "uint64 end_to_end_ns",
        "string[] exceeded_budgets",
        "string detail",
        "uint64 total_count",
        "uint64 normal_count",
        "uint64 warning_count",
        "uint64 e_stop_count",
        "uint64 rejected_count",
        "uint64 budget_exceeded_count",
        "uint64 invalid_timing_count",
    ]
    assert _message_fields(messages / "SafetyIntent.msg") == [
        "uint8 STATE_NORMAL=0",
        "uint8 STATE_WARNING=1",
        "uint8 STATE_E_STOP=2",
        "uint8 STATE_RECOVERY_HOLDING=3",
        "uint8 state",
        "builtin_interfaces/Time timestamp",
        "string correlation_id",
        "uint64 sequence_id",
    ]
    assert _message_fields(messages / "PermittedMotionEnvelope.msg") == [
        "std_msgs/Header header",
        "string policy_version",
        "string correlation_id",
        "uint64 sequence_id",
        "float64 min_linear_x_mps",
        "float64 max_linear_x_mps",
        "float64 min_linear_y_mps",
        "float64 max_linear_y_mps",
        "float64 min_angular_z_radps",
        "float64 max_angular_z_radps",
    ]
    assert _message_fields(messages / "ArbitrationStatus.msg") == [
        "uint8 MODE_PASS_THROUGH=0",
        "uint8 MODE_CLAMPED=1",
        "uint8 MODE_FORCED_ZERO=2",
        "uint8 SOURCE_NONE=0",
        "uint8 SOURCE_AUTONOMY=1",
        "uint8 SOURCE_TELEOPERATION=2",
        "uint8 FAULT_OK=0",
        "uint8 FAULT_E_STOP_ACTIVE=1",
        "uint8 FAULT_HARDWARE_FAULT=2",
        "uint8 FAULT_LATCHED_SAFETY_FAULT=3",
        "std_msgs/Header header",
        "uint8 mode",
        "string reason",
        "string policy_version",
        "string correlation_id",
        "bool is_safe",
        "uint64 command_sequence_id",
        "uint64 safety_intent_sequence_id",
        "string motion_envelope_correlation_id",
        "uint64 motion_envelope_sequence_id",
        "uint64 motion_envelope_timestamp_ns",
        "string arbitration_latency_status",
        "bool arbitration_timing_valid",
        "bool arbitration_within_budget",
        "uint64 intent_received_timestamp_ns",
        "uint64 command_published_timestamp_ns",
        "uint64 arbitration_latency_ns",
        "uint64 arbitration_latency_budget_ns",
        "float64 max_abs_linear_x",
        "float64 max_abs_linear_y",
        "float64 max_abs_angular_z",
        "float64 warning_max_abs_linear_x",
        "float64 warning_max_abs_linear_y",
        "float64 warning_max_abs_angular_z",
        "geometry_msgs/Twist commanded_twist",
        "string active_source",
        "uint8 system_fault_state",
        "string rejection_reason",
    ]
    assert _message_fields(messages / "SystemFaultStatus.msg") == [
        "uint8 FAULT_OK=0",
        "uint8 FAULT_E_STOP_ACTIVE=1",
        "uint8 FAULT_HARDWARE_FAULT=2",
        "uint8 FAULT_LATCHED_SAFETY_FAULT=3",
        "std_msgs/Header header",
        "uint8 fault_state",
    ]
    assert _message_fields(messages / "CommandPathObservabilityStatus.msg") == [
        "std_msgs/Header header",
        "string schema_version",
        "string policy_version",
        "string correlation_id",
        "string decision_id",
        "string outcome",
        "string arbitration_reason",
        "uint8 arbitration_mode",
        "uint64 command_sequence_id",
        "uint64 safety_intent_sequence_id",
        "string latency_status",
        "bool timing_valid",
        "bool within_budget",
        "uint64 observation_timestamp_ns",
        "uint64 evaluator_published_timestamp_ns",
        "uint64 intent_received_timestamp_ns",
        "uint64 command_published_timestamp_ns",
        "uint64 evaluator_end_to_end_ns",
        "uint64 arbitration_latency_ns",
        "uint64 observation_to_command_ns",
        "uint64 arbitration_latency_budget_ns",
        "uint64 observation_to_command_budget_ns",
        "string[] exceeded_budgets",
        "string detail",
    ]
    assert _message_fields(messages / "ActuatorFeedback.msg") == [
        "uint8 STATE_STOPPED=0",
        "uint8 STATE_MOVING=1",
        "uint8 STATE_INVALID_DATA=2",
        "uint8 STATE_STALE_DATA=3",
        "std_msgs/Header header",
        "string schema_version",
        "string correlation_id",
        "uint64 sequence_id",
        "float64 measured_linear_x",
        "float64 measured_linear_y",
        "float64 measured_angular_z",
        "uint8 state",
        "bool is_stopped",
        "bool is_fresh",
        "bool is_valid",
    ]
    assert _message_fields(messages / "StopAcknowledgement.msg") == [
        "uint8 STATE_IDLE=0",
        "uint8 STATE_STOP_PENDING=1",
        "uint8 STATE_STOP_ACKNOWLEDGED=2",
        "uint8 STATE_HARDWARE_FAULT_LATCH=3",
        "uint8 PRIORITY_NORMAL=0",
        "uint8 PRIORITY_CRITICAL=255",
        "std_msgs/Header header",
        "string schema_version",
        "uint8 state",
        "string reason",
        "string correlation_id",
        "uint64 request_sequence_id",
        "uint64 feedback_sequence_id",
        "bool is_stopped",
        "bool latched_fault",
        "bool critical",
        "uint8 priority",
        "uint64 stop_elapsed_ns",
        "uint64 evaluated_timestamp_ns",
    ]
    assert _message_fields(messages / "SafetyHeartbeat.msg") == [
        "std_msgs/Header header",
        "string sender_id",
        "uint64 sequence_number",
        "uint32 status_code",
    ]
    assert _message_fields(messages / "SafetyRelayStatus.msg") == [
        "std_msgs/Header header",
        "uint8 COMMANDED_CLOSED=1",
        "uint8 COMMANDED_OPEN=2",
        "uint8 commanded_state",
        "uint8 FEEDBACK_UNKNOWN=0",
        "uint8 FEEDBACK_CLOSED=1",
        "uint8 FEEDBACK_OPEN=2",
        "uint8 feedback_state",
        "bool is_latched",
    ]
    assert _message_fields(messages / "SafetySupervisorStatus.msg") == [
        "std_msgs/Header header",
        "uint8 STATE_INITIALIZING=0",
        "uint8 STATE_OK=1",
        "uint8 STATE_TRIPPED=2",
        "uint8 STATE_HARDWARE_FAULT_LATCH=3",
        "uint8 supervisor_state",
        "string[] active_faults",
        "bool relay_closed_request",
    ]


def test_message_package_declares_rosidl_and_message_dependencies() -> None:
    package = ROS_SOURCE / "titan_brain_msgs"
    dependencies = _package_dependencies(package / "package.xml")
    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")

    assert {
        "ament_cmake",
        "builtin_interfaces",
        "geometry_msgs",
        "rosidl_default_generators",
        "rosidl_default_runtime",
        "std_msgs",
    } <= dependencies
    assert "rosidl_generate_interfaces(${PROJECT_NAME}" in cmake
    assert '"msg/SafetyObservation.msg"' in cmake
    assert '"msg/DirectionalSafetyObservation.msg"' in cmake
    assert '"msg/SafetyEvaluationStatus.msg"' in cmake
    assert '"msg/SafetyStabilityStatus.msg"' in cmake
    assert '"msg/EvaluatorObservabilityStatus.msg"' in cmake
    assert '"msg/SafetyIntent.msg"' in cmake
    assert '"msg/PermittedMotionEnvelope.msg"' in cmake
    assert '"msg/ArbitrationStatus.msg"' in cmake
    assert '"msg/CommandPathObservabilityStatus.msg"' in cmake
    assert '"msg/ActuatorFeedback.msg"' in cmake
    assert '"msg/StopAcknowledgement.msg"' in cmake
    assert '"msg/SafetyHeartbeat.msg"' in cmake
    assert '"msg/SafetyRelayStatus.msg"' in cmake
    assert '"msg/SafetySupervisorStatus.msg"' in cmake
    assert '"msg/SystemFaultStatus.msg"' in cmake


def test_node_package_declares_runtime_dependencies_and_entry_point() -> None:
    package = ROS_SOURCE / "titan_brain_ros"
    dependencies = _package_dependencies(package / "package.xml")
    manifest = (package / "package.xml").read_text(encoding="utf-8")
    setup = (package / "setup.py").read_text(encoding="utf-8")

    assert {
        "ament_index_python",
        "launch",
        "launch_ros",
        "launch_testing",
        "launch_testing_ros",
        "rclpy",
        "tf2_ros",
        "titan_brain_msgs",
    } <= dependencies
    assert "ament_python" not in dependencies
    assert "<build_type>ament_python</build_type>" in manifest
    assert (
        "titan_brain_ros.safety_observation_node:main"
        in setup
    )
    assert "titan_brain_ros.velocity_arbiter_node:main" in setup
    assert "titan_brain_ros.command_path_observability_node:main" in setup
    assert "titan_brain_ros.actuator_feedback_monitor_node:main" in setup
    assert "titan_brain_ros.safety_loop_supervisor_node:main" in setup
    assert "titan_brain_ros.command_governor_node:main" in setup
    assert "titan_brain_ros.safety_velocity_arbiter_node:main" in setup
    assert (package / "resource" / "titan_brain_ros").is_file()
    assert (
        package / "titan_brain_ros" / "safety_observation_node.py"
    ).is_file()
    assert (
        package / "titan_brain_ros" / "velocity_arbiter_node.py"
    ).is_file()
    assert (
        package / "titan_brain_ros" / "command_path_observability_node.py"
    ).is_file()
    assert (
        package / "titan_brain_ros" / "actuator_feedback_monitor_node.py"
    ).is_file()
    assert (
        package / "titan_brain_ros" / "safety_loop_supervisor_node.py"
    ).is_file()
    assert (
        package / "titan_brain_ros" / "command_governor_node.py"
    ).is_file()
    assert (
        package / "titan_brain_ros" / "safety_velocity_arbiter_node.py"
    ).is_file()
    shared_config = package / "config" / "titan_brain.yaml"
    launch_file = package / "launch" / "titan_brain.launch.py"
    e2e_test = package / "test" / "test_e2e_transport.py"
    assert shared_config.is_file()
    assert launch_file.is_file()
    command_launch_file = package / "launch" / "command_governor.launch.py"
    assert command_launch_file.is_file()
    safety_launch_file = package / "launch" / "safety_control_plane.launch.py"
    assert safety_launch_file.is_file()
    replay_test = package / "test" / "test_integration_rosbag_007b.py"
    assert replay_test.is_file()
    assert e2e_test.is_file()
    launch_text = launch_file.read_text(encoding="utf-8")
    assert 'executable="actuator_feedback_monitor_node"' in launch_text
    assert 'executable="safety_loop_supervisor_node"' in launch_text
    assert 'executable="command_governor_node"' in launch_text
    command_launch_text = command_launch_file.read_text(encoding="utf-8")
    assert 'executable="command_governor_node"' in command_launch_text
    safety_launch_text = safety_launch_file.read_text(encoding="utf-8")
    assert 'executable="safety_velocity_arbiter_node"' in safety_launch_text
    assert 'executable="velocity_arbiter_node"' not in safety_launch_text
    assert 'executable="command_governor_node"' not in safety_launch_text
    replay_text = replay_test.read_text(encoding="utf-8")
    assert "@pytest.mark.launch_test" in replay_text
    assert '"/teleop/cmd_vel"' in replay_text
    assert '"/autonomy/cmd_vel"' in replay_text
    assert '"/safety/system_fault_status"' in replay_text
    assert '"/safety/permitted_motion_envelope"' in replay_text
    assert '"/cmd_vel"' in replay_text
    config_text = shared_config.read_text(encoding="utf-8")
    for required_parameter in (
        "target_frame",
        "watchdog_timeout_sec",
        "policy_version",
        "output_frame_id",
        "command_stale_threshold_sec",
        "safety_stale_threshold_sec",
        "motion_envelope_stale_threshold_sec",
        "timer_period_sec",
        "max_abs_linear_x",
        "max_abs_linear_y",
        "max_abs_angular_z",
        "arbitration_latency_budget_sec",
        "warning_max_abs_linear_x",
        "warning_max_abs_linear_y",
        "warning_max_abs_angular_z",
        "stop_budget_sec",
        "feedback_stale_threshold_sec",
        "epsilon_stop_linear",
        "epsilon_stop_angular",
        "dynamic_braking_enabled",
        "safety_policy_version",
        "clearance_threshold_m",
        "confidence_threshold",
        "braking_policy_version",
        "reaction_time_ns",
        "assured_deceleration_mps2",
        "clearance_margin_m",
        "motion_envelope_frame_id",
        "stability_enabled",
        "stability_policy_version",
        "clearance_hysteresis_m",
        "recovery_hold_time_s",
        "observability_policy_version",
        "receive_to_decision_budget_s",
        "decision_to_publish_budget_s",
        "end_to_end_budget_s",
        "command_path_policy_version",
        "observation_to_command_budget_sec",
        "max_correlation_entries",
        "max_pending_per_correlation",
        "heartbeat_timeout_sec",
        "initialization_timeout_sec",
        "relay_budget_sec",
        "reset_authorization_token",
        "cmd_timeout_sec",
        "safety_timeout_sec",
        "stale_command_emergency_stop",
        "max_linear_velocity_mps",
        "max_angular_velocity_radps",
        "max_linear_acceleration_mps2",
        "max_linear_deceleration_mps2",
        "max_angular_acceleration_radps2",
        "max_angular_deceleration_radps2",
        "max_linear_jerk_mps3",
        "max_angular_jerk_radps3",
        "envelope_timeout_sec",
        "fault_timeout_sec",
        "arbitration_latency_budget_sec",
        "policy_version",
    ):
        assert f"{required_parameter}:" in config_text
    max_observation_age_sec = _float_parameter(
        config_text,
        "max_observation_age_sec",
    )
    watchdog_timeout_sec = _float_parameter(
        config_text,
        "watchdog_timeout_sec",
    )
    assert max_observation_age_sec == 0.20
    assert watchdog_timeout_sec == 0.20
    assert watchdog_timeout_sec >= max_observation_age_sec
    assert "dynamic_braking_enabled: true" in config_text
    assert "reaction_time_ns: 250000000" in config_text
    assert "stability_enabled: true" in config_text
    assert _float_parameter(config_text, "clearance_hysteresis_m") == 0.10
    assert _float_parameter(config_text, "recovery_hold_time_s") == 0.20
    assert _float_parameter(config_text, "receive_to_decision_budget_s") == 0.05
    assert _float_parameter(config_text, "decision_to_publish_budget_s") == 0.02
    assert _float_parameter(config_text, "end_to_end_budget_s") == 0.07
    assert _float_parameter(config_text, "warning_max_abs_linear_x") == 0.3
    assert _float_parameter(config_text, "warning_max_abs_linear_y") == 0.1
    assert _float_parameter(config_text, "warning_max_abs_angular_z") == 0.5
    assert _float_parameter(config_text, "arbitration_latency_budget_sec") == 0.03
    assert _float_parameter(config_text, "observation_to_command_budget_sec") == 0.10
    assert '"config/titan_brain.yaml"' in setup
    assert '"launch/titan_brain.launch.py"' in setup
    assert '"launch/safety_control_plane.launch.py"' in setup
    launch_text = launch_file.read_text(encoding="utf-8")
    e2e_text = e2e_test.read_text(encoding="utf-8")
    assert 'executable="safety_observation_node"' in launch_text
    assert 'executable="velocity_arbiter_node"' in launch_text
    assert 'executable="command_path_observability_node"' in launch_text
    assert 'parameters=[config_file]' in launch_text
    assert "@pytest.mark.launch_test" in e2e_text
    assert '"/safety/observation"' in e2e_text
    assert '"/safety/directional_observation"' in e2e_text
    assert '"/safety/stability_status"' in e2e_text
    assert '"/safety/evaluator_observability"' in e2e_text
    assert '"/safety/command_path_observability"' in e2e_text
    assert '"/safety/intent"' in e2e_text
    assert '"/safety/permitted_motion_envelope"' in e2e_text
    assert '"/cmd_vel_raw"' in e2e_text
    assert '"/cmd_vel"' in e2e_text


def test_ci_uses_the_reproducible_jazzy_container_gate() -> None:
    workflow = (REPOSITORY_ROOT / ".github" / "workflows" / "ci.yml").read_text(
        encoding="utf-8"
    )

    assert "pip install --constraint requirements/constraints.txt" in workflow
    assert "docker build --tag titan-brain-dev:ci ." in workflow
    assert (
        "docker run --rm titan-brain-dev:ci scripts/quality-gate.sh all"
        in workflow
    )
    assert "ros-tooling/setup-ros" not in workflow
    assert "--break-system-packages" not in workflow
