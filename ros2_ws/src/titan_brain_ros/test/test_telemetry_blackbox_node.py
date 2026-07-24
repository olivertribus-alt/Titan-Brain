"""ROS 2 Jazzy tests for the TB-EVAL-009B blackbox adapter."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from std_srvs.srv import Trigger
from titan_brain_msgs.msg import (
    ArbitrationStatus,
    EnvelopeDiagnostics,
    SafetyLifecycleStatus,
    SystemFaultStatus,
)
from titan_brain_ros.telemetry_blackbox_node import (
    ARBITRATION_STATUS_TOPIC,
    AUTONOMY_COMMAND_TOPIC,
    ENVELOPE_DIAGNOSTICS_TOPIC,
    MANUAL_TRIGGER_SERVICE,
    OUTPUT_COMMAND_TOPIC,
    SAFETY_LIFECYCLE_TOPIC,
    SYSTEM_FAULT_TOPIC,
    TELEOP_COMMAND_TOPIC,
    TelemetryBlackboxNode,
    command_qos_profile,
    safety_qos_profile,
)

from core.telemetry_blackbox import BlackboxState, SnapshotTrigger


def _parameters(
    output_directory: Path,
    *,
    post_trigger_frames: int = 2,
) -> list[Parameter]:
    return [
        Parameter("policy_version", value="test-009b-policy"),
        Parameter("timer_period_sec", value=0.02),
        Parameter("capacity_frames", value=5),
        Parameter("post_trigger_frames", value=post_trigger_frames),
        Parameter(
            "snapshot_output_directory",
            value=str(output_directory),
        ),
    ]


def _stamp(message: object, timestamp_ns: int) -> None:
    header = message.header  # type: ignore[attr-defined]
    header.stamp.sec = timestamp_ns // 1_000_000_000
    header.stamp.nanosec = timestamp_ns % 1_000_000_000


def _command(timestamp_ns: int, value: float) -> TwistStamped:
    message = TwistStamped()
    _stamp(message, timestamp_ns)
    message.twist.linear.x = value
    message.twist.angular.z = -value
    return message


def _arbitration(timestamp_ns: int) -> ArbitrationStatus:
    message = ArbitrationStatus()
    _stamp(message, timestamp_ns)
    message.mode = ArbitrationStatus.MODE_CLAMPED
    message.reason = "MOTION_ENVELOPE_CLAMPED"
    message.active_source = "teleoperation"
    message.system_fault_state = ArbitrationStatus.FAULT_OK
    message.correlation_id = "arbitration-1"
    return message


def _envelope(timestamp_ns: int) -> EnvelopeDiagnostics:
    message = EnvelopeDiagnostics()
    _stamp(message, timestamp_ns)
    message.state = EnvelopeDiagnostics.STATE_LIMITED
    message.reason = "CLEARANCE_LIMITED"
    message.scan_valid = True
    message.distance_forward_m = 0.8
    message.distance_lateral_m = 1.2
    message.max_linear_velocity_mps = 0.4
    message.max_angular_velocity_radps = 0.5
    return message


def _lifecycle(timestamp_ns: int, state: int) -> SafetyLifecycleStatus:
    message = SafetyLifecycleStatus()
    _stamp(message, timestamp_ns)
    message.state = state
    message.reason = (
        "system_fault"
        if state == SafetyLifecycleStatus.STATE_EMERGENCY_STOP
        else "normal"
    )
    message.is_faulted = state == SafetyLifecycleStatus.STATE_EMERGENCY_STOP
    message.recovery_active = state == SafetyLifecycleStatus.STATE_RECOVERY
    message.max_linear_velocity_mps = (
        0.0 if state == SafetyLifecycleStatus.STATE_EMERGENCY_STOP else 1.0
    )
    message.max_angular_velocity_radps = message.max_linear_velocity_mps
    return message


def _fault(timestamp_ns: int, state: int) -> SystemFaultStatus:
    message = SystemFaultStatus()
    _stamp(message, timestamp_ns)
    message.fault_state = state
    return message


def _destroy(node: TelemetryBlackboxNode) -> None:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_qos_contracts_are_bounded_reliable_and_volatile() -> None:
    command_qos = command_qos_profile()
    safety_qos = safety_qos_profile()

    assert command_qos.history == HistoryPolicy.KEEP_LAST
    assert command_qos.depth == 1
    assert command_qos.reliability == ReliabilityPolicy.RELIABLE
    assert command_qos.durability == DurabilityPolicy.VOLATILE
    assert safety_qos.history == HistoryPolicy.KEEP_LAST
    assert safety_qos.depth == 10
    assert safety_qos.reliability == ReliabilityPolicy.RELIABLE
    assert safety_qos.durability == DurabilityPolicy.VOLATILE


def test_startup_records_partial_evidence_and_creates_all_endpoints(
    tmp_path: Path,
) -> None:
    rclpy.init()
    node = TelemetryBlackboxNode(
        parameter_overrides=_parameters(tmp_path),
    )
    try:
        assert node.blackbox.frame_count == 1
        frame = node.blackbox.rolling_frames()[0]
        assert frame.authoritative_command is None
        assert frame.lifecycle is None
        for topic in (
            TELEOP_COMMAND_TOPIC,
            AUTONOMY_COMMAND_TOPIC,
            OUTPUT_COMMAND_TOPIC,
            ARBITRATION_STATUS_TOPIC,
            ENVELOPE_DIAGNOSTICS_TOPIC,
            SAFETY_LIFECYCLE_TOPIC,
            SYSTEM_FAULT_TOPIC,
        ):
            assert node.count_subscribers(topic) == 1
        services = dict(node.get_service_names_and_types())
        assert MANUAL_TRIGGER_SERVICE in services
    finally:
        _destroy(node)


def test_one_tick_correlates_the_latest_complete_control_plane(
    tmp_path: Path,
) -> None:
    rclpy.init()
    node = TelemetryBlackboxNode(
        parameter_overrides=_parameters(tmp_path),
    )
    try:
        now_ns = node._now_ns() + 1_000_000
        node._on_teleop_command(_command(now_ns, 0.5))
        node._on_autonomy_command(_command(now_ns, 0.3))
        node._on_output_command(_command(now_ns, 0.2))
        node._on_arbitration_status(_arbitration(now_ns))
        node._on_envelope_diagnostics(_envelope(now_ns))
        node._on_lifecycle_status(
            _lifecycle(now_ns, SafetyLifecycleStatus.STATE_NORMAL)
        )
        node._now_ns = lambda: now_ns
        node._on_timer()

        frame = node.blackbox.rolling_frames()[-1]
        assert frame.teleoperation_command is not None
        assert frame.autonomy_command is not None
        assert frame.authoritative_command is not None
        assert frame.authoritative_command.linear_x_mps == 0.2
        assert frame.arbitration is not None
        assert frame.arbitration.active_source == "teleoperation"
        assert frame.envelope is not None
        assert frame.envelope.distance_forward_m == 0.8
        assert frame.lifecycle is not None
        assert frame.lifecycle.state == SafetyLifecycleStatus.STATE_NORMAL
    finally:
        _destroy(node)


def test_emergency_transition_freezes_exact_post_window_and_exports(
    tmp_path: Path,
) -> None:
    rclpy.init()
    node = TelemetryBlackboxNode(
        parameter_overrides=_parameters(tmp_path),
    )
    try:
        baseline_ns = node._now_ns() + 1_000_000
        node._on_lifecycle_status(
            _lifecycle(
                baseline_ns,
                SafetyLifecycleStatus.STATE_NORMAL,
            )
        )
        node._now_ns = lambda: baseline_ns
        node._on_timer()
        emergency_ns = baseline_ns + 20_000_000
        node._on_lifecycle_status(
            _lifecycle(
                emergency_ns,
                SafetyLifecycleStatus.STATE_EMERGENCY_STOP,
            )
        )
        node._now_ns = lambda: emergency_ns
        node._on_timer()

        assert node.blackbox.state is BlackboxState.CAPTURING_POST_TRIGGER
        assert node.blackbox.remaining_post_trigger_frames == 2
        for offset in (40_000_000, 60_000_000):
            node._now_ns = lambda offset=offset: baseline_ns + offset
            node._on_timer()

        snapshot = node.blackbox.last_snapshot
        assert snapshot is not None
        assert snapshot.trigger is SnapshotTrigger.EMERGENCY_STOP
        assert snapshot.trigger_reason == "system_fault"
        assert len(snapshot.frames) <= 7
        assert node.last_export_path is not None
        payload = json.loads(node.last_export_path.read_text(encoding="utf-8"))
        assert payload["snapshot_id"] == snapshot.snapshot_id
        assert payload["trigger"] == "emergency_stop"
    finally:
        _destroy(node)


def test_hard_fault_has_trigger_priority_over_same_tick_emergency(
    tmp_path: Path,
) -> None:
    rclpy.init()
    node = TelemetryBlackboxNode(
        parameter_overrides=_parameters(tmp_path, post_trigger_frames=0),
    )
    try:
        now_ns = node._now_ns() + 1_000_000
        node._on_lifecycle_status(
            _lifecycle(now_ns, SafetyLifecycleStatus.STATE_NORMAL)
        )
        node._on_lifecycle_status(
            _lifecycle(
                now_ns,
                SafetyLifecycleStatus.STATE_EMERGENCY_STOP,
            )
        )
        node._on_fault_status(_fault(now_ns, SystemFaultStatus.FAULT_HARDWARE_FAULT))
        node._now_ns = lambda: now_ns
        node._on_timer()

        snapshot = node.blackbox.last_snapshot
        assert snapshot is not None
        assert snapshot.trigger is SnapshotTrigger.HARD_FAULT
        assert (
            snapshot.trigger_reason
            == f"system fault state {SystemFaultStatus.FAULT_HARDWARE_FAULT}"
        )
        assert node.last_export_path is not None
    finally:
        _destroy(node)


def test_initial_emergency_state_does_not_create_false_transition(
    tmp_path: Path,
) -> None:
    rclpy.init()
    node = TelemetryBlackboxNode(
        parameter_overrides=_parameters(tmp_path, post_trigger_frames=0),
    )
    try:
        now_ns = node._now_ns() + 1_000_000
        node._on_lifecycle_status(
            _lifecycle(
                now_ns,
                SafetyLifecycleStatus.STATE_EMERGENCY_STOP,
            )
        )
        node._now_ns = lambda: now_ns
        node._on_timer()

        assert node.blackbox.last_snapshot is None
        assert node.last_export_path is None
    finally:
        _destroy(node)


def test_manual_service_trigger_exports_synchronous_snapshot(
    tmp_path: Path,
) -> None:
    rclpy.init()
    node = TelemetryBlackboxNode(
        parameter_overrides=_parameters(tmp_path, post_trigger_frames=0),
    )
    try:
        response = node._on_manual_trigger(
            Trigger.Request(),
            Trigger.Response(),
        )

        assert response.success is True
        assert response.message == "blackbox snapshot capture started"
        assert node.blackbox.last_snapshot is not None
        assert node.blackbox.last_snapshot.trigger is SnapshotTrigger.MANUAL
        assert node.last_export_path is not None
    finally:
        _destroy(node)


def test_manual_trigger_cannot_replace_pending_automatic_incident(
    tmp_path: Path,
) -> None:
    rclpy.init()
    node = TelemetryBlackboxNode(
        parameter_overrides=_parameters(tmp_path, post_trigger_frames=0),
    )
    try:
        now_ns = node._now_ns() + 1_000_000
        node._on_fault_status(_fault(now_ns, SystemFaultStatus.FAULT_HARDWARE_FAULT))

        response = node._on_manual_trigger(
            Trigger.Request(),
            Trigger.Response(),
        )
        node._now_ns = lambda: now_ns
        node._on_timer()

        snapshot = node.blackbox.last_snapshot
        assert response.success is False
        assert response.message == "automatic incident trigger pending"
        assert snapshot is not None
        assert snapshot.trigger is SnapshotTrigger.HARD_FAULT
    finally:
        _destroy(node)


def test_clock_regression_is_clamped_and_captured(
    tmp_path: Path,
) -> None:
    rclpy.init()
    node = TelemetryBlackboxNode(
        parameter_overrides=_parameters(tmp_path, post_trigger_frames=0),
    )
    try:
        baseline_ns = node._now_ns() + 1_000_000
        node._now_ns = lambda: baseline_ns
        node._on_timer()
        node._now_ns = lambda: baseline_ns - 1
        node._on_timer()

        snapshot = node.blackbox.last_snapshot
        assert snapshot is not None
        assert snapshot.trigger is SnapshotTrigger.EMERGENCY_STOP
        assert snapshot.trigger_reason == "blackbox clock regression"
        assert snapshot.frames[-1].recorded_at_ns == baseline_ns
    finally:
        _destroy(node)


def test_relative_snapshot_directory_is_rejected() -> None:
    rclpy.init()
    try:
        try:
            TelemetryBlackboxNode(
                parameter_overrides=[
                    Parameter(
                        "snapshot_output_directory",
                        value="relative/path",
                    )
                ],
            )
        except ValueError as error:
            assert "must be absolute" in str(error)
        else:
            raise AssertionError("relative output directory was accepted")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
