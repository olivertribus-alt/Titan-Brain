"""ROS 2 tests for the TB-SAFE-001C supervisor adapter."""

from __future__ import annotations

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from titan_brain_msgs.msg import SafetyHeartbeat, SafetyRelayStatus
from titan_brain_ros.safety_loop_supervisor_node import (
    RELAY_COMMAND_TOPIC,
    RELAY_STATUS_TOPIC,
    SUPERVISOR_STATUS_TOPIC,
    SafetyLoopSupervisorNode,
    control_qos_profile,
    heartbeat_qos_profile,
    status_qos_profile,
)


def _parameters() -> list[Parameter]:
    return [
        Parameter("policy_version", value="TB-SAFE-001C-0.1.0"),
        Parameter("output_frame_id", value="base_link"),
        Parameter("heartbeat_timeout_sec", value=0.20),
        Parameter("initialization_timeout_sec", value=0.20),
        Parameter("relay_budget_sec", value=0.05),
        Parameter("timer_period_sec", value=0.01),
        Parameter("reset_authorization_token", value="TB-SAFE-RESET-001B"),
    ]


def _new_node() -> SafetyLoopSupervisorNode:
    return SafetyLoopSupervisorNode(parameter_overrides=_parameters())


def _heartbeat(sender_id: str, sequence_number: int = 1) -> SafetyHeartbeat:
    message = SafetyHeartbeat()
    message.sender_id = sender_id
    message.sequence_number = sequence_number
    message.status_code = 0
    return message


def _relay(feedback_state: int, *, is_latched: bool = False) -> SafetyRelayStatus:
    message = SafetyRelayStatus()
    message.header.frame_id = "base_link"
    message.commanded_state = SafetyRelayStatus.COMMANDED_OPEN
    message.feedback_state = feedback_state
    message.is_latched = is_latched
    return message


def _destroy(node: SafetyLoopSupervisorNode) -> None:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _healthy_start(node: SafetyLoopSupervisorNode) -> None:
    node._on_control_heartbeat(_heartbeat("control_arbiter"))
    node._on_actuator_heartbeat(_heartbeat("actuator_monitor"))
    node._on_odometry_heartbeat(_heartbeat("odometry"))
    node._on_relay_status(_relay(SafetyRelayStatus.FEEDBACK_CLOSED))


def test_qos_profiles_are_bounded_and_explicit() -> None:
    heartbeat = heartbeat_qos_profile()
    control = control_qos_profile()
    status = status_qos_profile()

    assert heartbeat.history == HistoryPolicy.KEEP_LAST
    assert heartbeat.depth == 5
    assert heartbeat.reliability == ReliabilityPolicy.BEST_EFFORT
    assert heartbeat.durability == DurabilityPolicy.VOLATILE
    for profile in (control, status):
        assert profile.history == HistoryPolicy.KEEP_LAST
        assert profile.depth == 10
        assert profile.reliability == ReliabilityPolicy.RELIABLE
        assert profile.durability == DurabilityPolicy.VOLATILE


def test_startup_is_initializing_and_requests_open_relay() -> None:
    rclpy.init()
    node = _new_node()
    try:
        assert node.last_result is not None
        assert node.last_result.state.value == "initializing"
        assert node.last_relay_command is not None
        assert node.last_relay_command.commanded_state == (
            SafetyRelayStatus.COMMANDED_OPEN
        )
        assert node.count_publishers(SUPERVISOR_STATUS_TOPIC) == 1
        assert node.count_publishers(RELAY_COMMAND_TOPIC) == 1
        assert node.count_subscribers(RELAY_STATUS_TOPIC) == 1
        assert node.count_subscribers("/safety/heartbeat/control_arbiter") == 1
        assert node.count_subscribers("/safety/heartbeat/actuator_monitor") == 1
        assert node.count_subscribers("/safety/heartbeat/odometry") == 1
    finally:
        _destroy(node)


def test_healthy_matrix_closes_relay_after_physical_feedback() -> None:
    rclpy.init()
    node = _new_node()
    try:
        _healthy_start(node)
        assert node.last_result is not None
        assert node.last_result.state.value == "ok"
        assert node.last_status is not None
        assert node.last_status.supervisor_state == (node.last_status.STATE_OK)
        assert node.last_status.relay_closed_request is True
        assert node.last_relay_command is not None
        assert node.last_relay_command.commanded_state == (
            SafetyRelayStatus.COMMANDED_CLOSED
        )
        assert node.last_result.relay_transition_pending is False
        assert node.last_status.active_faults == []
    finally:
        _destroy(node)


def test_unhealthy_status_code_trips_and_opens_relay() -> None:
    rclpy.init()
    node = _new_node()
    try:
        message = _heartbeat("control_arbiter")
        message.status_code = 7
        node._on_control_heartbeat(message)
        assert node.last_result is not None
        assert node.last_result.state.value == "tripped"
        assert node.last_relay_command is not None
        assert node.last_relay_command.commanded_state == (
            SafetyRelayStatus.COMMANDED_OPEN
        )
        assert node.last_status is not None
        assert "heartbeat_error" in node.last_status.active_faults
    finally:
        _destroy(node)


def test_replayed_sequence_is_rejected_fail_closed() -> None:
    rclpy.init()
    node = _new_node()
    try:
        node._on_control_heartbeat(_heartbeat("control_arbiter", 4))
        node._on_control_heartbeat(_heartbeat("control_arbiter", 4))
        assert node.last_result is not None
        assert node.last_result.state.value == "tripped"
        assert node.last_result.reason.value == "heartbeat_error"
    finally:
        _destroy(node)


def test_unknown_relay_feedback_latches_hardware_fault() -> None:
    rclpy.init()
    node = _new_node()
    try:
        node._on_relay_status(_relay(SafetyRelayStatus.FEEDBACK_UNKNOWN))
        assert node.last_result is not None
        assert node.last_result.state.value == "hardware_fault_latch"
        assert node.last_relay_command is not None
        assert node.last_relay_command.commanded_state == (
            SafetyRelayStatus.COMMANDED_OPEN
        )
        assert node.last_relay_command.is_latched is True
        assert node.last_status is not None
        assert node.last_status.supervisor_state == (
            node.last_status.STATE_HARDWARE_FAULT_LATCH
        )
    finally:
        _destroy(node)


def test_driver_latch_is_propagated_without_automatic_release() -> None:
    rclpy.init()
    node = _new_node()
    try:
        node._on_relay_status(_relay(SafetyRelayStatus.FEEDBACK_OPEN, is_latched=True))
        node._on_control_heartbeat(_heartbeat("control_arbiter"))
        assert node.last_result is not None
        assert node.last_result.state.value == "hardware_fault_latch"
        assert node.last_relay_command is not None
        assert node.last_relay_command.commanded_state == (
            SafetyRelayStatus.COMMANDED_OPEN
        )
    finally:
        _destroy(node)


def test_timer_trips_when_a_healthy_channel_times_out() -> None:
    rclpy.init()
    node = _new_node()
    try:
        _healthy_start(node)
        assert node.last_result is not None
        baseline = node.last_result.evaluated_at_ns

        def future_now() -> int:
            return baseline + 201_000_000

        node._now_ns = future_now
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.state.value == "tripped"
        assert node.last_result.reason.value == "heartbeat_timeout"
        assert node.last_status is not None
        assert node.last_status.relay_closed_request is False
    finally:
        _destroy(node)
