"""ROS 2 Jazzy runtime tests for VelocityArbiterNode."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from titan_brain_msgs.msg import (
    ArbitrationStatus,
    PermittedMotionEnvelope,
    SafetyIntent,
)
from titan_brain_ros.velocity_arbiter_node import (
    VelocityArbiterNode,
    command_qos_profile,
    status_qos_profile,
)

from core.arbitrator import ArbitrationMode, ArbitrationReason

_NANOSECONDS_PER_SECOND = 1_000_000_000


def _parameters() -> list[Parameter]:
    return [
        Parameter("policy_version", value="TB-EVAL-004C-0.1.0"),
        Parameter("output_frame_id", value="base_link"),
        Parameter("command_stale_threshold_sec", value=0.1),
        Parameter("safety_stale_threshold_sec", value=0.25),
        Parameter("motion_envelope_stale_threshold_sec", value=0.05),
        Parameter("timer_period_sec", value=0.02),
        Parameter("arbitration_latency_budget_sec", value=0.03),
        Parameter("max_abs_linear_x", value=0.8),
        Parameter("max_abs_linear_y", value=0.2),
        Parameter("max_abs_angular_z", value=1.5),
        Parameter("warning_max_abs_linear_x", value=0.3),
        Parameter("warning_max_abs_linear_y", value=0.1),
        Parameter("warning_max_abs_angular_z", value=0.5),
    ]


def _node() -> VelocityArbiterNode:
    node = VelocityArbiterNode(parameter_overrides=_parameters())
    node._on_motion_envelope(_envelope(node))
    return node


def _set_intent_stamp(message: SafetyIntent, timestamp_ns: int) -> None:
    message.timestamp.sec = timestamp_ns // _NANOSECONDS_PER_SECOND
    message.timestamp.nanosec = timestamp_ns % _NANOSECONDS_PER_SECOND


def _intent(
    node: VelocityArbiterNode,
    *,
    state: int = SafetyIntent.STATE_NORMAL,
    sequence_id: int = 1,
    correlation_id: str = "decision-001",
    timestamp_ns: int | None = None,
) -> SafetyIntent:
    message = SafetyIntent()
    message.state = state
    _set_intent_stamp(
        message,
        node.get_clock().now().nanoseconds
        if timestamp_ns is None
        else timestamp_ns,
    )
    message.correlation_id = correlation_id
    message.sequence_id = sequence_id
    return message


def _envelope(
    node: VelocityArbiterNode,
    *,
    min_linear_x_mps: float = -0.8,
    max_linear_x_mps: float = 0.8,
    min_linear_y_mps: float = -0.2,
    max_linear_y_mps: float = 0.2,
    min_angular_z_radps: float = 0.0,
    max_angular_z_radps: float = 0.0,
    frame_id: str = "base_link",
    sequence_id: int = 1,
    correlation_id: str = "decision-001",
    timestamp_ns: int | None = None,
) -> PermittedMotionEnvelope:
    message = PermittedMotionEnvelope()
    envelope_timestamp_ns = (
        node.get_clock().now().nanoseconds
        if timestamp_ns is None
        else timestamp_ns
    )
    message.header.stamp.sec = envelope_timestamp_ns // _NANOSECONDS_PER_SECOND
    message.header.stamp.nanosec = (
        envelope_timestamp_ns % _NANOSECONDS_PER_SECOND
    )
    message.header.frame_id = frame_id
    message.policy_version = "TB-EVAL-005C-ENVELOPE-0.1.0"
    message.correlation_id = correlation_id
    message.sequence_id = sequence_id
    message.min_linear_x_mps = min_linear_x_mps
    message.max_linear_x_mps = max_linear_x_mps
    message.min_linear_y_mps = min_linear_y_mps
    message.max_linear_y_mps = max_linear_y_mps
    message.min_angular_z_radps = min_angular_z_radps
    message.max_angular_z_radps = max_angular_z_radps
    return message


def _desired_twist(
    *,
    linear_x: float = 0.4,
    linear_y: float = 0.1,
    angular_z: float = 0.0,
) -> Twist:
    message = Twist()
    message.linear.x = linear_x
    message.linear.y = linear_y
    message.angular.z = angular_z
    return message


def test_qos_contracts_are_reliable_volatile_and_bounded() -> None:
    command_qos = command_qos_profile()
    status_qos = status_qos_profile()

    assert command_qos.history == HistoryPolicy.KEEP_LAST
    assert command_qos.depth == 1
    assert command_qos.reliability == ReliabilityPolicy.RELIABLE
    assert command_qos.durability == DurabilityPolicy.VOLATILE
    assert status_qos.history == HistoryPolicy.KEEP_LAST
    assert status_qos.depth == 10
    assert status_qos.reliability == ReliabilityPolicy.RELIABLE
    assert status_qos.durability == DurabilityPolicy.VOLATILE


def test_startup_is_forced_zero_and_node_owns_control_plane_topics() -> None:
    rclpy.init()
    node = _node()
    try:
        result = node.last_result
        status = node.last_status

        assert result is not None
        assert result.mode is ArbitrationMode.FORCED_ZERO
        assert result.reason is ArbitrationReason.SAFETY_INTENT_MISSING
        assert result.command.linear_x == 0.0
        assert status is not None
        assert status.mode == ArbitrationStatus.MODE_FORCED_ZERO
        assert status.is_safe is False
        assert status.arbitration_timing_valid is False
        assert status.arbitration_latency_status == "invalid_timing"
        assert status.arbitration_latency_ns == 0
        assert status.arbitration_latency_budget_ns == 30_000_000
        assert status.commanded_twist.linear.x == 0.0
        assert node.count_publishers("/cmd_vel") == 1
        assert node.count_publishers("/safety/arbitration_status") == 1
        assert node.count_subscribers("/cmd_vel_raw") == 1
        assert node.count_subscribers("/safety/intent") == 1
        assert node.count_subscribers("/safety/permitted_motion_envelope") == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_fresh_normal_intent_and_newer_command_are_passed_through() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_safety_intent(_intent(node))
        node._on_desired_velocity(_desired_twist())
        node._on_timer()

        result = node.last_result
        status = node.last_status
        assert result is not None
        assert result.mode is ArbitrationMode.PASS_THROUGH
        assert result.command.linear_x == 0.4
        assert result.command.linear_y == 0.1
        assert result.command.angular_z == 0.0
        assert result.correlation_id == "decision-001"
        assert status is not None
        assert status.mode == ArbitrationStatus.MODE_PASS_THROUGH
        assert status.reason == "proceed"
        assert status.policy_version == "TB-EVAL-004C-0.1.0"
        assert status.correlation_id == "decision-001"
        assert status.is_safe is True
        assert status.command_sequence_id > 0
        assert status.safety_intent_sequence_id == 1
        assert status.motion_envelope_sequence_id == 1
        assert status.motion_envelope_correlation_id == "decision-001"
        assert status.motion_envelope_timestamp_ns > 0
        assert status.arbitration_timing_valid is True
        assert status.arbitration_within_budget is True
        assert status.arbitration_latency_status == "within_budget"
        assert (
            status.command_published_timestamp_ns
            - status.intent_received_timestamp_ns
            == status.arbitration_latency_ns
        )
        assert status.max_abs_linear_x == 0.8
        assert status.warning_max_abs_linear_x == 0.3
        assert status.header.frame_id == "base_link"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_normal_release_requires_command_received_after_the_intent() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_desired_velocity(_desired_twist())
        node._on_safety_intent(_intent(node))
        node._on_timer()

        blocked = node.last_result
        assert blocked is not None
        assert blocked.reason is ArbitrationReason.RECOVERY_COMMAND_REQUIRED
        assert blocked.command.linear_x == 0.0

        node._on_desired_velocity(_desired_twist())
        node._on_timer()
        released = node.last_result
        assert released is not None
        assert released.reason is ArbitrationReason.PROCEED
        assert released.command.linear_x == 0.4
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_warning_uses_symmetric_limits_and_non_acceleration_baseline() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_safety_intent(_intent(node))
        node._on_desired_velocity(
            _desired_twist(linear_x=0.4, linear_y=0.1, angular_z=0.0)
        )
        node._on_timer()

        node._on_safety_intent(
            _intent(
                node,
                state=SafetyIntent.STATE_WARNING,
                sequence_id=2,
                correlation_id="decision-warning",
            )
        )
        node._on_motion_envelope(
            _envelope(
                node,
                sequence_id=2,
                correlation_id="decision-warning",
            )
        )
        node._on_desired_velocity(
            _desired_twist(linear_x=-2.0, linear_y=-1.0, angular_z=0.0)
        )
        node._on_timer()

        result = node.last_result
        status = node.last_status
        assert result is not None
        assert result.reason is ArbitrationReason.WARNING_SHAPED
        assert result.command.linear_x == -0.3
        assert result.command.linear_y == -0.1
        assert result.command.angular_z == 0.0
        assert status is not None
        assert status.mode == ArbitrationStatus.MODE_CLAMPED
        assert status.correlation_id == "decision-warning"
        assert status.warning_max_abs_angular_z == 0.5
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_asymmetric_envelope_clamps_translation_and_blocks_rotation() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_motion_envelope(
            _envelope(
                node,
                min_linear_x_mps=-0.2,
                max_linear_x_mps=0.4,
                min_linear_y_mps=-0.3,
                max_linear_y_mps=0.1,
                sequence_id=2,
                correlation_id="decision-envelope",
            )
        )
        node._on_safety_intent(
            _intent(
                node,
                sequence_id=2,
                correlation_id="decision-envelope",
            )
        )
        node._on_desired_velocity(
            _desired_twist(linear_x=2.0, linear_y=-2.0, angular_z=1.0)
        )
        node._on_timer()

        result = node.last_result
        assert result is not None
        assert result.reason is ArbitrationReason.MOTION_ENVELOPE_CLAMPED
        assert result.mode is ArbitrationMode.CLAMPED
        assert result.command.linear_x == 0.4
        assert result.command.linear_y == -0.3
        assert result.command.angular_z == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_missing_and_invalid_envelopes_fail_closed() -> None:
    rclpy.init()
    missing_node = _node()
    invalid_node = _node()
    try:
        missing_node._motion_envelope = None
        missing_node._on_safety_intent(_intent(missing_node))
        missing_node._on_desired_velocity(_desired_twist())
        missing_node._on_timer()
        assert missing_node.last_result is not None
        assert (
            missing_node.last_result.reason
            is ArbitrationReason.MOTION_ENVELOPE_MISSING
        )

        invalid_node._on_motion_envelope(
            _envelope(invalid_node, max_angular_z_radps=0.1)
        )
        invalid_node._on_safety_intent(_intent(invalid_node))
        invalid_node._on_desired_velocity(_desired_twist())
        invalid_node._on_timer()
        assert invalid_node.last_result is not None
        assert (
            invalid_node.last_result.reason
            is ArbitrationReason.MOTION_ENVELOPE_INVALID
        )
        assert invalid_node.last_result.command.linear_x == 0.0
    finally:
        missing_node.destroy_node()
        invalid_node.destroy_node()
        rclpy.shutdown()


def test_envelope_timeout_clock_regression_and_identity_mismatch_fail_closed(
) -> None:
    rclpy.init()
    stale_node = _node()
    future_node = _node()
    correlation_node = _node()
    sequence_node = _node()
    nodes = (stale_node, future_node, correlation_node, sequence_node)
    try:
        stale_node._on_motion_envelope(
            _envelope(
                stale_node,
                sequence_id=2,
                timestamp_ns=(
                    stale_node.get_clock().now().nanoseconds - 50_000_000
                ),
            )
        )
        stale_node._on_safety_intent(_intent(stale_node, sequence_id=2))
        stale_node._on_desired_velocity(_desired_twist())
        stale_node._on_timer()
        assert stale_node.last_result is not None
        assert (
            stale_node.last_result.reason
            is ArbitrationReason.MOTION_ENVELOPE_TIMEOUT
        )

        future_node._on_motion_envelope(
            _envelope(
                future_node,
                sequence_id=2,
                timestamp_ns=(
                    future_node.get_clock().now().nanoseconds + 1_000_000_000
                ),
            )
        )
        future_node._on_safety_intent(_intent(future_node, sequence_id=2))
        future_node._on_desired_velocity(_desired_twist())
        future_node._on_timer()
        assert future_node.last_result is not None
        assert (
            future_node.last_result.reason
            is ArbitrationReason.MOTION_ENVELOPE_CLOCK_REGRESSION
        )

        correlation_node._on_motion_envelope(
            _envelope(
                correlation_node,
                sequence_id=2,
                correlation_id="wrong-decision",
            )
        )
        correlation_node._on_safety_intent(
            _intent(correlation_node, sequence_id=2)
        )
        correlation_node._on_desired_velocity(_desired_twist())
        correlation_node._on_timer()
        assert correlation_node.last_result is not None
        assert (
            correlation_node.last_result.reason
            is ArbitrationReason.MOTION_ENVELOPE_INTENT_MISMATCH
        )

        sequence_node._on_motion_envelope(
            _envelope(sequence_node, sequence_id=2)
        )
        sequence_node._on_safety_intent(_intent(sequence_node, sequence_id=3))
        sequence_node._on_desired_velocity(_desired_twist())
        sequence_node._on_timer()
        assert sequence_node.last_result is not None
        assert (
            sequence_node.last_result.reason
            is ArbitrationReason.MOTION_ENVELOPE_INTENT_MISMATCH
        )
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def test_command_must_arrive_after_the_matched_envelope() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_safety_intent(_intent(node))
        node._on_desired_velocity(_desired_twist())
        node._on_motion_envelope(
            _envelope(
                node,
                sequence_id=2,
                correlation_id="decision-002",
            )
        )
        node._on_safety_intent(
            _intent(
                node,
                sequence_id=2,
                correlation_id="decision-002",
            )
        )
        node._on_timer()
        assert node.last_result is not None
        assert (
            node.last_result.reason
            is ArbitrationReason.MOTION_ENVELOPE_COMMAND_REQUIRED
        )
        assert node.last_result.command.linear_x == 0.0

        node._on_desired_velocity(_desired_twist(linear_x=2.0))
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.reason is ArbitrationReason.MOTION_ENVELOPE_CLAMPED
        assert node.last_result.command.linear_x == 0.8
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_envelope_replay_does_not_refresh_and_mutation_fails_closed() -> None:
    rclpy.init()
    node = _node()
    try:
        original = _envelope(
            node,
            sequence_id=7,
            correlation_id="decision-007",
        )
        node._on_motion_envelope(original)
        node._on_safety_intent(
            _intent(node, sequence_id=7, correlation_id="decision-007")
        )
        node._on_desired_velocity(_desired_twist())
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.reason is ArbitrationReason.PROCEED

        accepted_ingress = node._ingress_sequence_id
        node._on_motion_envelope(original)
        assert node._ingress_sequence_id == accepted_ingress
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.reason is ArbitrationReason.PROCEED

        node._on_motion_envelope(
            _envelope(
                node,
                sequence_id=7,
                correlation_id="decision-mutated",
            )
        )
        node._on_desired_velocity(_desired_twist())
        node._on_timer()
        assert node.last_result is not None
        assert (
            node.last_result.reason
            is ArbitrationReason.MOTION_ENVELOPE_INVALID
        )
        assert node.last_status is not None
        assert (
            node.last_status.motion_envelope_correlation_id
            == "decision-mutated"
        )

        node._on_motion_envelope(
            _envelope(
                node,
                sequence_id=6,
                correlation_id="decision-replayed",
            )
        )
        node._on_timer()
        assert node.last_result is not None
        assert (
            node.last_result.reason
            is ArbitrationReason.MOTION_ENVELOPE_INVALID
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stop_states_latch_and_require_explicit_normal_then_new_command() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_safety_intent(
            _intent(node, state=SafetyIntent.STATE_E_STOP)
        )
        node._on_desired_velocity(_desired_twist())
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.reason is ArbitrationReason.E_STOP_ACTIVE
        assert node.last_status is not None
        assert node.last_status.correlation_id == "decision-001"

        node._on_safety_intent(
            _intent(
                node,
                state=SafetyIntent.STATE_RECOVERY_HOLDING,
                sequence_id=2,
                correlation_id="decision-holding",
            )
        )
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.reason is ArbitrationReason.RECOVERY_HOLDING

        node._on_safety_intent(
            _intent(
                node,
                sequence_id=3,
                correlation_id="decision-release",
            )
        )
        node._on_motion_envelope(
            _envelope(
                node,
                sequence_id=3,
                correlation_id="decision-release",
            )
        )
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.reason is ArbitrationReason.RECOVERY_COMMAND_REQUIRED

        node._on_desired_velocity(_desired_twist())
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.reason is ArbitrationReason.PROCEED
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_independent_safety_and_command_timeouts_force_zero() -> None:
    rclpy.init()
    safety_timeout_node = _node()
    command_timeout_node = _node()
    try:
        stale_safety_ns = (
            safety_timeout_node.get_clock().now().nanoseconds - 300_000_000
        )
        safety_timeout_node._on_safety_intent(
            _intent(safety_timeout_node, timestamp_ns=stale_safety_ns)
        )
        safety_timeout_node._on_desired_velocity(_desired_twist())
        safety_timeout_node._on_timer()
        assert safety_timeout_node.last_result is not None
        assert (
            safety_timeout_node.last_result.reason
            is ArbitrationReason.SAFETY_INTENT_TIMEOUT
        )

        command_timeout_node._on_safety_intent(_intent(command_timeout_node))
        command_timeout_node._desired_velocity = {
            "linear_x": 0.4,
            "linear_y": 0.0,
            "angular_z": 0.0,
            "timestamp_ns": (
                command_timeout_node.get_clock().now().nanoseconds
                - 200_000_000
            ),
            "frame_id": "base_link",
            "sequence_id": 2,
        }
        command_timeout_node._on_timer()
        assert command_timeout_node.last_result is not None
        assert (
            command_timeout_node.last_result.reason
            is ArbitrationReason.COMMAND_TIMEOUT
        )
    finally:
        safety_timeout_node.destroy_node()
        command_timeout_node.destroy_node()
        rclpy.shutdown()


def test_source_sequence_replay_and_payload_mutation_fail_closed() -> None:
    rclpy.init()
    node = _node()
    try:
        original = _intent(node, sequence_id=7, correlation_id="decision-007")
        node._on_motion_envelope(
            _envelope(
                node,
                sequence_id=7,
                correlation_id="decision-007",
            )
        )
        node._on_safety_intent(original)
        node._on_desired_velocity(_desired_twist())
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.reason is ArbitrationReason.PROCEED

        node._on_safety_intent(original)
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.reason is ArbitrationReason.PROCEED

        mutated = _intent(
            node,
            state=SafetyIntent.STATE_E_STOP,
            sequence_id=7,
            correlation_id="decision-mutated",
        )
        node._on_safety_intent(mutated)
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.reason is ArbitrationReason.SAFETY_INTENT_INVALID
        assert node.last_status is not None
        assert node.last_status.correlation_id == "decision-mutated"

        node._on_safety_intent(
            _intent(node, sequence_id=6, correlation_id="decision-replayed")
        )
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.reason is ArbitrationReason.SAFETY_INTENT_INVALID
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_invalid_intent_and_non_finite_twist_fail_closed() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_safety_intent(
            _intent(node, state=99, sequence_id=0, correlation_id="bad")
        )
        node._on_desired_velocity(_desired_twist())
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.reason is ArbitrationReason.SAFETY_INTENT_INVALID

        node._on_safety_intent(
            _intent(node, sequence_id=1, correlation_id="decision-valid")
        )
        node._on_desired_velocity(_desired_twist(linear_x=float("nan")))
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.reason is ArbitrationReason.COMMAND_INVALID
        assert node.last_result.command.linear_x == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()
