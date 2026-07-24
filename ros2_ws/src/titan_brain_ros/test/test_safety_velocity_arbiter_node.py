"""ROS 2 Jazzy tests for the TB-EVAL-007B single-point velocity arbiter."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from titan_brain_msgs.msg import (
    PermittedMotionEnvelope,
    SafetyLifecycleStatus,
    SystemFaultStatus,
)
from titan_brain_ros.safety_velocity_arbiter_node import (
    ARBITRATION_STATUS_TOPIC,
    AUTONOMY_COMMAND_TOPIC,
    MOTION_ENVELOPE_TOPIC,
    OUTPUT_COMMAND_TOPIC,
    SAFETY_LIFECYCLE_TOPIC,
    SYSTEM_FAULT_TOPIC,
    TELEOP_COMMAND_TOPIC,
    SafetyVelocityArbiterNode,
    command_qos_profile,
    safety_qos_profile,
)


def _parameters() -> list[Parameter]:
    return [
        Parameter("output_frame_id", value="base_link"),
        Parameter("command_timeout_sec", value=0.10),
        Parameter("envelope_timeout_sec", value=0.05),
        Parameter("fault_timeout_sec", value=0.20),
        Parameter("arbitration_latency_budget_sec", value=0.03),
        Parameter("timer_period_sec", value=0.02),
        Parameter("policy_version", value="test-policy"),
        Parameter("max_linear_velocity_mps", value=2.0),
        Parameter("max_angular_velocity_radps", value=2.0),
        Parameter("max_linear_acceleration_mps2", value=100.0),
        Parameter("max_linear_deceleration_mps2", value=100.0),
        Parameter("max_angular_acceleration_radps2", value=100.0),
        Parameter("max_angular_deceleration_radps2", value=100.0),
        Parameter("max_linear_jerk_mps3", value=1000.0),
        Parameter("max_angular_jerk_radps3", value=1000.0),
    ]


def _lifecycle_parameters() -> list[Parameter]:
    return [
        *_parameters(),
        Parameter("lifecycle_gate_enabled", value=True),
        Parameter("lifecycle_timeout_sec", value=0.05),
    ]


def _new_node() -> SafetyVelocityArbiterNode:
    return SafetyVelocityArbiterNode(parameter_overrides=_parameters())


def _stamp(
    message: (
        TwistStamped
        | PermittedMotionEnvelope
        | SafetyLifecycleStatus
        | SystemFaultStatus
    ),
    timestamp_ns: int,
) -> None:
    message.header.stamp.sec = timestamp_ns // 1_000_000_000
    message.header.stamp.nanosec = timestamp_ns % 1_000_000_000


def _twist(
    timestamp_ns: int,
    *,
    linear_x: float = 0.0,
    angular_z: float = 0.0,
    frame_id: str = "base_link",
) -> TwistStamped:
    message = TwistStamped()
    message.header.frame_id = frame_id
    _stamp(message, timestamp_ns)
    message.twist.linear.x = linear_x
    message.twist.angular.z = angular_z
    return message


def _envelope(
    timestamp_ns: int,
    *,
    max_linear_x: float = 1.0,
    max_angular_z: float = 0.0,
    frame_id: str = "base_link",
) -> PermittedMotionEnvelope:
    message = PermittedMotionEnvelope()
    message.header.frame_id = frame_id
    _stamp(message, timestamp_ns)
    message.policy_version = "test-envelope-policy"
    message.correlation_id = "observation-1"
    message.sequence_id = 1
    message.min_linear_x_mps = -max_linear_x
    message.max_linear_x_mps = max_linear_x
    message.min_linear_y_mps = 0.0
    message.max_linear_y_mps = 0.0
    message.min_angular_z_radps = -max_angular_z
    message.max_angular_z_radps = max_angular_z
    return message


def _fault(state: int, timestamp_ns: int) -> SystemFaultStatus:
    message = SystemFaultStatus()
    _stamp(message, timestamp_ns)
    message.fault_state = state
    return message


def _lifecycle(
    timestamp_ns: int,
    *,
    state: int,
    max_linear: float,
    max_angular: float,
) -> SafetyLifecycleStatus:
    message = SafetyLifecycleStatus()
    message.header.frame_id = "base_link"
    _stamp(message, timestamp_ns)
    message.schema_version = "0.1"
    message.policy_version = "test-lifecycle-policy"
    message.correlation_id = "lifecycle-1"
    message.sequence_id = 1
    message.state = state
    message.reason = "test"
    message.fault_status_valid = True
    message.sensor_valid = True
    message.sensor_fresh = True
    message.time_valid = True
    message.recovery_active = state == SafetyLifecycleStatus.STATE_RECOVERY
    message.max_linear_velocity_mps = max_linear
    message.max_angular_velocity_radps = max_angular
    return message


def _set_test_time(node: SafetyVelocityArbiterNode) -> int:
    timestamp_ns = node.governor.last_timestamp_ns + 1_000_000_000
    node._now_ns = lambda: timestamp_ns
    return timestamp_ns


def _destroy(node: SafetyVelocityArbiterNode) -> None:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_qos_profiles_are_bounded_reliable_and_volatile() -> None:
    command = command_qos_profile()
    safety = safety_qos_profile()
    assert command.history == HistoryPolicy.KEEP_LAST
    assert command.depth == 1
    assert command.reliability == ReliabilityPolicy.RELIABLE
    assert command.durability == DurabilityPolicy.VOLATILE
    assert safety.history == HistoryPolicy.KEEP_LAST
    assert safety.depth == 10
    assert safety.reliability == ReliabilityPolicy.RELIABLE
    assert safety.durability == DurabilityPolicy.VOLATILE


def test_startup_is_fail_closed_and_owns_single_output() -> None:
    rclpy.init()
    node = _new_node()
    try:
        assert node.last_result is not None
        assert node.last_result.emergency_override is True
        assert node.last_result.linear_velocity_mps == 0.0
        assert node.count_publishers(OUTPUT_COMMAND_TOPIC) == 1
        assert node.count_publishers(ARBITRATION_STATUS_TOPIC) == 1
        assert node.count_subscribers(TELEOP_COMMAND_TOPIC) == 1
        assert node.count_subscribers(AUTONOMY_COMMAND_TOPIC) == 1
        assert node.count_subscribers(SYSTEM_FAULT_TOPIC) == 1
        assert node.count_subscribers(MOTION_ENVELOPE_TOPIC) == 1
        assert node.count_subscribers(SAFETY_LIFECYCLE_TOPIC) == 1
    finally:
        _destroy(node)


def test_enabled_lifecycle_gate_fails_closed_when_status_is_missing() -> None:
    rclpy.init()
    node = SafetyVelocityArbiterNode(parameter_overrides=_lifecycle_parameters())
    try:
        now_ns = _set_test_time(node)
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns))
        node._on_motion_envelope(_envelope(now_ns, max_linear_x=1.0))
        node._on_teleop_command(_twist(now_ns, linear_x=1.0))
        node._on_timer()

        assert node.last_result is not None
        assert node.last_result.linear_velocity_mps == 0.0
        assert node.last_status is not None
        assert node.last_status.rejection_reason == "SAFETY_LIFECYCLE_MISSING"
    finally:
        _destroy(node)


def test_unhealthy_normal_lifecycle_status_is_rejected() -> None:
    rclpy.init()
    node = SafetyVelocityArbiterNode(parameter_overrides=_lifecycle_parameters())
    try:
        now_ns = _set_test_time(node)
        lifecycle = _lifecycle(
            now_ns,
            state=SafetyLifecycleStatus.STATE_NORMAL,
            max_linear=1.0,
            max_angular=1.0,
        )
        lifecycle.sensor_fresh = False
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns))
        node._on_motion_envelope(_envelope(now_ns, max_linear_x=1.0))
        node._on_lifecycle_status(lifecycle)
        node._on_teleop_command(_twist(now_ns, linear_x=1.0))
        node._on_timer()

        assert node.last_result is not None
        assert node.last_result.linear_velocity_mps == 0.0
        assert node.last_status is not None
        assert node.last_status.rejection_reason == "SAFETY_LIFECYCLE_INVALID"
    finally:
        _destroy(node)


def test_recovery_lifecycle_cap_is_enforced_on_final_command() -> None:
    rclpy.init()
    node = SafetyVelocityArbiterNode(parameter_overrides=_lifecycle_parameters())
    try:
        now_ns = _set_test_time(node)
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns))
        node._on_motion_envelope(
            _envelope(
                now_ns,
                max_linear_x=1.0,
                max_angular_z=1.0,
            )
        )
        node._on_lifecycle_status(
            _lifecycle(
                now_ns,
                state=SafetyLifecycleStatus.STATE_RECOVERY,
                max_linear=0.20,
                max_angular=0.40,
            )
        )
        node._on_teleop_command(_twist(now_ns, linear_x=1.0, angular_z=1.0))
        node._on_timer()

        assert node.last_result is not None
        assert node.last_result.linear_velocity_mps == 0.20
        assert node.last_result.angular_velocity_radps == 0.40
        assert node.last_status is not None
        assert node.last_status.mode == node.last_status.MODE_CLAMPED
        assert node.last_status.reason == "SAFETY_LIFECYCLE_RECOVERY_CLAMPED"
    finally:
        _destroy(node)


def test_lifecycle_emergency_stop_bypasses_governor_to_exact_zero() -> None:
    rclpy.init()
    node = SafetyVelocityArbiterNode(parameter_overrides=_lifecycle_parameters())
    try:
        now_ns = _set_test_time(node)
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns))
        node._on_motion_envelope(_envelope(now_ns, max_linear_x=1.0))
        node._on_lifecycle_status(
            _lifecycle(
                now_ns,
                state=SafetyLifecycleStatus.STATE_NORMAL,
                max_linear=1.0,
                max_angular=1.0,
            )
        )
        node._on_teleop_command(_twist(now_ns, linear_x=1.0))
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.linear_velocity_mps == 1.0

        stopped_ns = now_ns + 1
        node._now_ns = lambda: stopped_ns
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, stopped_ns))
        node._on_motion_envelope(_envelope(stopped_ns, max_linear_x=1.0))
        node._on_lifecycle_status(
            _lifecycle(
                stopped_ns,
                state=SafetyLifecycleStatus.STATE_EMERGENCY_STOP,
                max_linear=0.0,
                max_angular=0.0,
            )
        )
        node._on_teleop_command(_twist(stopped_ns, linear_x=1.0))
        node._on_timer()

        assert node.last_result is not None
        assert node.last_result.emergency_override is True
        assert node.last_result.linear_velocity_mps == 0.0
        assert node.last_result.angular_velocity_radps == 0.0
        assert node.last_status is not None
        assert node.last_status.reason == "SAFETY_LIFECYCLE_EMERGENCY_STOP"
    finally:
        _destroy(node)


def test_teleoperation_wins_and_is_envelope_clamped() -> None:
    rclpy.init()
    node = _new_node()
    try:
        now_ns = _set_test_time(node)
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns))
        node._on_motion_envelope(_envelope(now_ns, max_linear_x=0.5))
        node._on_autonomy_command(_twist(now_ns, linear_x=0.2))
        node._on_teleop_command(_twist(now_ns, linear_x=1.5))
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.is_safe is True
        assert node.last_result.linear_velocity_mps == 0.5
        assert node.last_status is not None
        assert node.last_status.active_source == "teleoperation"
        assert node.last_status.mode == node.last_status.MODE_CLAMPED
        assert node.last_status.arbitration_timing_valid is True
        assert node.last_status.arbitration_within_budget is True
        assert node.last_status.arbitration_latency_status == "within_budget"
        assert node.last_status.command_sequence_id >= 2
    finally:
        _destroy(node)


def test_autonomy_is_selected_when_teleop_is_stale() -> None:
    rclpy.init()
    node = _new_node()
    try:
        now_ns = _set_test_time(node)
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns))
        node._on_motion_envelope(_envelope(now_ns))
        node._on_teleop_command(_twist(now_ns - 101_000_000, linear_x=1.0))
        node._on_autonomy_command(_twist(now_ns, linear_x=0.25))
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.linear_velocity_mps == 0.25
        assert node.last_status is not None
        assert node.last_status.active_source == "autonomy"
    finally:
        _destroy(node)


def test_fault_status_bypasses_all_commands() -> None:
    rclpy.init()
    node = _new_node()
    try:
        now_ns = _set_test_time(node)
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_E_STOP_ACTIVE, now_ns))
        node._on_motion_envelope(_envelope(now_ns))
        node._on_teleop_command(_twist(now_ns, linear_x=1.0))
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.emergency_override is True
        assert node.last_result.linear_velocity_mps == 0.0
        assert node.last_status is not None
        assert (
            node.last_status.system_fault_state == SystemFaultStatus.FAULT_E_STOP_ACTIVE
        )
        assert node.last_status.rejection_reason == "SYSTEM_FAULT_E_STOP_ACTIVE"
    finally:
        _destroy(node)


def test_missing_or_invalid_envelope_is_fail_closed() -> None:
    rclpy.init()
    node = _new_node()
    try:
        now_ns = _set_test_time(node)
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns))
        node._on_teleop_command(_twist(now_ns, linear_x=0.5))
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.emergency_override is True
        assert node.last_status is not None
        assert node.last_status.rejection_reason == "MOTION_ENVELOPE_MISSING"

        invalid = _envelope(now_ns)
        invalid.min_angular_z_radps = 0.1
        node._on_motion_envelope(invalid)
        node._on_timer()
        assert node.last_status is not None
        assert node.last_status.rejection_reason == "MOTION_ENVELOPE_INVALID"
    finally:
        _destroy(node)


def test_stop_only_envelope_bypasses_governor_and_forces_zero() -> None:
    rclpy.init()
    node = _new_node()
    try:
        now_ns = _set_test_time(node)
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns))
        node._on_motion_envelope(_envelope(now_ns))
        node._on_teleop_command(_twist(now_ns, linear_x=0.5))
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.linear_velocity_mps == 0.5

        node._on_motion_envelope(_envelope(now_ns, max_linear_x=0.0))
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.emergency_override is True
        assert node.last_result.linear_velocity_mps == 0.0
        assert node.last_result.angular_velocity_radps == 0.0
        assert node.last_status is not None
        assert node.last_status.mode == node.last_status.MODE_FORCED_ZERO
        assert node.last_status.rejection_reason == "MOTION_ENVELOPE_STOP_ONLY"
    finally:
        _destroy(node)


def test_future_command_timestamp_is_rejected_fail_closed() -> None:
    rclpy.init()
    node = _new_node()
    try:
        now_ns = _set_test_time(node)
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns))
        node._on_motion_envelope(_envelope(now_ns))
        node._on_teleop_command(_twist(now_ns + 1, linear_x=0.5))
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.emergency_override is True
        assert node.last_status is not None
        assert node.last_status.rejection_reason == "FUTURE_TIMESTAMP"
    finally:
        _destroy(node)


def test_unsupported_lateral_or_vertical_motion_is_rejected() -> None:
    rclpy.init()
    node = _new_node()
    try:
        now_ns = _set_test_time(node)
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns))
        node._on_motion_envelope(_envelope(now_ns))
        message = _twist(now_ns, linear_x=0.25)
        message.twist.linear.y = 0.01
        node._on_teleop_command(message)
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.emergency_override is True
        assert node.last_status is not None
        assert node.last_status.rejection_reason == "INVALID_COMMAND_FRAME"
    finally:
        _destroy(node)


def test_fault_status_timestamp_is_checked_fail_closed() -> None:
    rclpy.init()
    node = _new_node()
    try:
        now_ns = _set_test_time(node)
        node._on_motion_envelope(_envelope(now_ns))
        node._on_teleop_command(_twist(now_ns, linear_x=0.25))

        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns + 1))
        node._on_timer()
        assert node.last_status is not None
        assert (
            node.last_status.rejection_reason == "SYSTEM_FAULT_STATUS_FUTURE_TIMESTAMP"
        )

        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns - 201_000_000))
        node._on_timer()
        assert node.last_status is not None
        assert node.last_status.rejection_reason == "SYSTEM_FAULT_STATUS_TIMEOUT"

        node._on_fault_status(_fault(255, now_ns))
        node._on_timer()
        assert node.last_status is not None
        assert node.last_status.rejection_reason == "INVALID_SYSTEM_FAULT_STATE"
    finally:
        _destroy(node)


def test_nonzero_lateral_envelope_authority_is_rejected() -> None:
    rclpy.init()
    node = _new_node()
    try:
        now_ns = _set_test_time(node)
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns))
        invalid = _envelope(now_ns)
        invalid.max_linear_y_mps = 0.1
        node._on_motion_envelope(invalid)
        node._on_teleop_command(_twist(now_ns, linear_x=0.25))
        node._on_timer()
        assert node.last_status is not None
        assert node.last_status.rejection_reason == "MOTION_ENVELOPE_INVALID"
        assert node.last_result is not None
        assert node.last_result.emergency_override is True
    finally:
        _destroy(node)


def test_symmetric_angular_envelope_authority_is_clamped() -> None:
    rclpy.init()
    node = _new_node()
    try:
        now_ns = _set_test_time(node)
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns))
        node._on_motion_envelope(
            _envelope(now_ns, max_linear_x=1.0, max_angular_z=0.25)
        )
        node._on_teleop_command(_twist(now_ns, linear_x=0.1, angular_z=1.0))
        node._on_timer()
        assert node.last_status is not None
        assert node.last_status.mode == node.last_status.MODE_CLAMPED
        assert node.last_result is not None
        assert node.last_result.angular_velocity_radps == 0.25
    finally:
        _destroy(node)


def test_asymmetric_angular_envelope_authority_is_rejected() -> None:
    rclpy.init()
    node = _new_node()
    try:
        now_ns = _set_test_time(node)
        node._on_fault_status(_fault(SystemFaultStatus.FAULT_OK, now_ns))
        invalid = _envelope(now_ns, max_angular_z=0.25)
        invalid.min_angular_z_radps = -0.10
        node._on_motion_envelope(invalid)
        node._on_teleop_command(_twist(now_ns, angular_z=0.1))
        node._on_timer()
        assert node.last_status is not None
        assert node.last_status.rejection_reason == "MOTION_ENVELOPE_INVALID"
    finally:
        _destroy(node)
