"""ROS 2 Jazzy tests for the TB-EVAL-009A lifecycle adapter."""

from __future__ import annotations

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from titan_brain_msgs.msg import (
    EnvelopeDiagnostics,
    SafetyLifecycleStatus,
    SystemFaultStatus,
)
from titan_brain_ros.safety_recovery_manager_node import (
    ENVELOPE_DIAGNOSTICS_TOPIC,
    SAFETY_LIFECYCLE_TOPIC,
    SYSTEM_FAULT_TOPIC,
    SafetyRecoveryManagerNode,
    safety_qos_profile,
)


def _parameters() -> list[Parameter]:
    return [
        Parameter("policy_version", value="test-009a-policy"),
        Parameter("output_frame_id", value="base_link"),
        Parameter("timer_period_sec", value=0.02),
        Parameter("diagnostics_timeout_sec", value=0.20),
        Parameter("fault_timeout_sec", value=0.20),
        Parameter("stop_margin_m", value=0.30),
        Parameter("warning_distance_m", value=1.00),
        Parameter("distance_hysteresis_m", value=0.10),
        Parameter("recovery_dwell_time_sec", value=0.05),
        Parameter("degraded_linear_speed_limit_mps", value=0.50),
        Parameter("degraded_angular_speed_limit_radps", value=0.50),
        Parameter("recovery_linear_speed_limit_mps", value=0.20),
        Parameter("recovery_angular_speed_limit_radps", value=0.40),
    ]


def _stamp(
    message: EnvelopeDiagnostics | SystemFaultStatus,
    timestamp_ns: int,
) -> None:
    message.header.stamp.sec = timestamp_ns // 1_000_000_000
    message.header.stamp.nanosec = timestamp_ns % 1_000_000_000


def _fault(timestamp_ns: int, state: int) -> SystemFaultStatus:
    message = SystemFaultStatus()
    _stamp(message, timestamp_ns)
    message.fault_state = state
    return message


def _diagnostics(
    timestamp_ns: int,
    *,
    distance_m: float = 5.0,
    state: int = EnvelopeDiagnostics.STATE_NOMINAL,
    reason: str = "NOMINAL_AUTHORITY",
    scan_valid: bool = True,
    fault_status_valid: bool = True,
    max_linear_velocity_mps: float = 1.0,
    max_angular_velocity_radps: float = 1.0,
) -> EnvelopeDiagnostics:
    message = EnvelopeDiagnostics()
    _stamp(message, timestamp_ns)
    message.policy_version = "test-envelope"
    message.correlation_id = f"envelope-{timestamp_ns}"
    message.sequence_id = timestamp_ns
    message.state = state
    message.limiting_zone = EnvelopeDiagnostics.ZONE_NONE
    message.reason = reason
    message.scan_valid = scan_valid
    message.fault_status_valid = fault_status_valid
    message.scan_age_sec = 0.0
    message.distance_forward_m = distance_m
    message.distance_lateral_m = distance_m
    message.max_linear_velocity_mps = max_linear_velocity_mps
    message.max_angular_velocity_radps = max_angular_velocity_radps
    return message


def _destroy(node: SafetyRecoveryManagerNode) -> None:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _update(
    node: SafetyRecoveryManagerNode,
    timestamp_ns: int,
    diagnostics: EnvelopeDiagnostics,
    *,
    fault_state: int = SystemFaultStatus.FAULT_OK,
) -> SafetyLifecycleStatus:
    node._now_ns = lambda: timestamp_ns
    node._on_fault_status(_fault(timestamp_ns, fault_state))
    node._on_diagnostics(diagnostics)
    node._on_timer()
    assert node.last_status is not None
    return node.last_status


def test_qos_contract_is_bounded_reliable_and_volatile() -> None:
    qos = safety_qos_profile()
    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_startup_is_exact_zero_and_topics_have_single_endpoints() -> None:
    rclpy.init()
    node = SafetyRecoveryManagerNode(parameter_overrides=_parameters())
    try:
        status = node.last_status
        assert status is not None
        assert status.state == SafetyLifecycleStatus.STATE_EMERGENCY_STOP
        assert status.max_linear_velocity_mps == 0.0
        assert status.max_angular_velocity_radps == 0.0
        assert node.count_publishers(SAFETY_LIFECYCLE_TOPIC) == 1
        assert node.count_subscribers(ENVELOPE_DIAGNOSTICS_TOPIC) == 1
        assert node.count_subscribers(SYSTEM_FAULT_TOPIC) == 1
    finally:
        _destroy(node)


def test_valid_stream_requires_dwell_then_publishes_normal_authority() -> None:
    rclpy.init()
    node = SafetyRecoveryManagerNode(parameter_overrides=_parameters())
    try:
        baseline_ns = node._now_ns() + 1_000_000
        started = _update(
            node,
            baseline_ns,
            _diagnostics(baseline_ns),
        )
        released_at_ns = baseline_ns + 50_000_000
        released = _update(
            node,
            released_at_ns,
            _diagnostics(released_at_ns),
        )

        assert started.state == SafetyLifecycleStatus.STATE_RECOVERY
        assert started.recovery_active is True
        assert started.max_linear_velocity_mps == 0.20
        assert released.state == SafetyLifecycleStatus.STATE_NORMAL
        assert released.recovery_active is False
        assert released.max_linear_velocity_mps == 1.0
        assert released.correlation_id == f"envelope-{released_at_ns}"
    finally:
        _destroy(node)


def test_degraded_hysteresis_requires_strict_release_distance() -> None:
    rclpy.init()
    node = SafetyRecoveryManagerNode(parameter_overrides=_parameters())
    try:
        baseline_ns = node._now_ns() + 1_000_000
        _update(node, baseline_ns, _diagnostics(baseline_ns))
        normal_ns = baseline_ns + 50_000_000
        _update(node, normal_ns, _diagnostics(normal_ns))
        degraded_ns = normal_ns + 1
        degraded = _update(
            node,
            degraded_ns,
            _diagnostics(
                degraded_ns,
                distance_m=0.80,
                state=EnvelopeDiagnostics.STATE_LIMITED,
                reason="CLEARANCE_LIMITED",
            ),
        )
        hysteresis_ns = degraded_ns + 1
        hysteresis = _update(
            node,
            hysteresis_ns,
            _diagnostics(hysteresis_ns, distance_m=1.10),
        )
        released_ns = hysteresis_ns + 1
        released = _update(
            node,
            released_ns,
            _diagnostics(released_ns, distance_m=1.100001),
        )

        assert degraded.state == SafetyLifecycleStatus.STATE_DEGRADED
        assert degraded.max_linear_velocity_mps == 0.50
        assert hysteresis.state == SafetyLifecycleStatus.STATE_DEGRADED
        assert released.state == SafetyLifecycleStatus.STATE_NORMAL
    finally:
        _destroy(node)


def test_hard_fault_has_immediate_exact_zero_precedence() -> None:
    rclpy.init()
    node = SafetyRecoveryManagerNode(parameter_overrides=_parameters())
    try:
        now_ns = node._now_ns() + 1_000_000
        status = _update(
            node,
            now_ns,
            _diagnostics(now_ns),
            fault_state=SystemFaultStatus.FAULT_HARDWARE_FAULT,
        )

        assert status.state == SafetyLifecycleStatus.STATE_EMERGENCY_STOP
        assert status.reason == "system_fault"
        assert status.is_faulted is True
        assert status.max_linear_velocity_mps == 0.0
        assert status.max_angular_velocity_radps == 0.0
    finally:
        _destroy(node)


def test_diagnostics_timestamp_regression_latches_time_fault() -> None:
    rclpy.init()
    node = SafetyRecoveryManagerNode(parameter_overrides=_parameters())
    try:
        baseline_ns = node._now_ns() + 2_000_000
        _update(node, baseline_ns, _diagnostics(baseline_ns))
        regressed_ns = baseline_ns + 1
        status = _update(
            node,
            regressed_ns,
            _diagnostics(baseline_ns - 1),
        )

        assert status.state == SafetyLifecycleStatus.STATE_EMERGENCY_STOP
        assert status.time_valid is False
        assert status.max_linear_velocity_mps == 0.0
    finally:
        _destroy(node)
