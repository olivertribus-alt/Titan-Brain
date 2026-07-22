"""ROS 2 tests for the TB-ACT-001C actuator feedback monitor adapter."""

from __future__ import annotations

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from titan_brain_msgs.msg import ActuatorFeedback, ArbitrationStatus
from titan_brain_ros.actuator_feedback_monitor_node import (
    ActuatorFeedbackMonitorNode,
    control_qos_profile,
    feedback_qos_profile,
    status_qos_profile,
)

_NANOSECONDS_PER_SECOND = 1_000_000_000


def _parameters() -> list[Parameter]:
    return [
        Parameter("policy_version", value="TB-ACT-001C-0.1.0"),
        Parameter("output_frame_id", value="base_link"),
        Parameter("stop_budget_sec", value=0.1),
        Parameter("feedback_stale_threshold_sec", value=0.05),
        Parameter("epsilon_stop_linear", value=0.01),
        Parameter("epsilon_stop_angular", value=0.02),
        Parameter("timer_period_sec", value=0.01),
    ]


def _node() -> ActuatorFeedbackMonitorNode:
    return ActuatorFeedbackMonitorNode(parameter_overrides=_parameters())


def _set_stamp(message: object, timestamp_ns: int) -> None:
    message.header.stamp.sec = timestamp_ns // _NANOSECONDS_PER_SECOND
    message.header.stamp.nanosec = timestamp_ns % _NANOSECONDS_PER_SECOND


def _stop_status(node: ActuatorFeedbackMonitorNode) -> ArbitrationStatus:
    status = ArbitrationStatus()
    status.mode = ArbitrationStatus.MODE_FORCED_ZERO
    status.correlation_id = "stop-ros-001"
    status.command_sequence_id = 10
    _set_stamp(status, node.get_clock().now().nanoseconds)
    return status


def _stopped_feedback(
    node: ActuatorFeedbackMonitorNode,
    *,
    sequence_id: int = 1,
) -> ActuatorFeedback:
    feedback = ActuatorFeedback()
    _set_stamp(feedback, node.get_clock().now().nanoseconds)
    feedback.correlation_id = "stop-ros-001"
    feedback.sequence_id = sequence_id
    feedback.measured_linear_x = 0.0
    feedback.measured_linear_y = 0.0
    feedback.measured_angular_z = 0.0
    feedback.state = ActuatorFeedback.STATE_STOPPED
    feedback.is_stopped = True
    feedback.is_fresh = True
    feedback.is_valid = True
    return feedback


def _moving_feedback(
    node: ActuatorFeedbackMonitorNode,
    *,
    sequence_id: int = 2,
    correlation_id: str = "stop-ros-001",
) -> ActuatorFeedback:
    feedback = _stopped_feedback(node, sequence_id=sequence_id)
    feedback.correlation_id = correlation_id
    feedback.measured_linear_x = 0.25
    feedback.state = ActuatorFeedback.STATE_MOVING
    feedback.is_stopped = False
    return feedback


def test_qos_profiles_are_bounded_and_fail_closed() -> None:
    feedback = feedback_qos_profile()
    control = control_qos_profile()
    status = status_qos_profile()

    assert feedback.history == HistoryPolicy.KEEP_LAST
    assert feedback.depth == 5
    assert feedback.reliability == ReliabilityPolicy.BEST_EFFORT
    assert feedback.durability == DurabilityPolicy.VOLATILE
    for profile in (control, status):
        assert profile.history == HistoryPolicy.KEEP_LAST
        assert profile.depth == 10
        assert profile.reliability == ReliabilityPolicy.RELIABLE
        assert profile.durability == DurabilityPolicy.VOLATILE


def test_startup_publishes_non_latched_idle_status_and_expected_topics() -> None:
    rclpy.init()
    node = _node()
    try:
        assert node.last_result is not None
        assert node.last_result.state.value == "idle"
        assert node.last_result.is_latched is False
        assert node.last_status is not None
        assert node.last_status.latched_fault is False
        assert node.count_publishers("/actuator/stop_acknowledgement") == 1
        assert node.count_publishers("/actuator/status") == 1
        assert node.count_subscribers("/actuator/feedback") == 1
        assert node.count_subscribers("/safety/arbitration_status") == 1
        assert node.count_subscribers("/cmd_vel") == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_fresh_stopped_feedback_publishes_acknowledgement() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_arbitration_status(_stop_status(node))
        node._on_feedback(_stopped_feedback(node))

        assert node.last_result is not None
        assert node.last_result.state.value == "stop_acknowledged"
        assert node.last_result.acknowledgement is not None
        assert node.last_status is not None
        assert node.last_status.state == (
            node.last_status.STATE_STOP_ACKNOWLEDGED
        )
        assert node.last_status.is_stopped is True
        assert node.last_status.latched_fault is False
        assert node.last_status.feedback_sequence_id == 1
        assert node.last_acknowledgement is not None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_invalid_feedback_publishes_critical_latch() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_arbitration_status(_stop_status(node))
        feedback = _stopped_feedback(node)
        feedback.state = ActuatorFeedback.STATE_INVALID_DATA
        feedback.is_valid = False
        node._on_feedback(feedback)

        assert node.last_result is not None
        assert node.last_result.is_latched is True
        assert node.last_status is not None
        assert node.last_status.latched_fault is True
        assert node.last_status.critical is True
        assert node.last_status.priority == (
            node.last_status.PRIORITY_CRITICAL
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_spurious_movement_after_ack_publishes_critical_latch() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_arbitration_status(_stop_status(node))
        node._on_feedback(_stopped_feedback(node, sequence_id=1))
        node._on_feedback(_moving_feedback(node, sequence_id=2))

        assert node.last_result is not None
        assert node.last_result.reason.value == "spurious_movement_after_ack"
        assert node.last_result.is_latched is True
        assert node.last_status is not None
        assert node.last_status.critical is True
        assert node.last_status.priority == node.last_status.PRIORITY_CRITICAL
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_frozen_feedback_after_stop_request_latches_stale_fault() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_arbitration_status(_stop_status(node))
        stale = _stopped_feedback(node)
        stale.header.stamp.sec = 0
        stale.header.stamp.nanosec = 1
        node._on_feedback(stale)

        assert node.last_result is not None
        assert node.last_result.reason.value == "stale_feedback"
        assert node.last_result.is_latched is True
        assert node.last_status is not None
        assert node.last_status.latched_fault is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_vel_fallback_uses_last_arbitration_correlation() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_arbitration_status(_stop_status(node))
        zero_command = type("TwistLike", (), {})()
        zero_command.linear = type("Vector", (), {"x": 0.0, "y": 0.0})()
        zero_command.angular = type("Vector", (), {"z": 0.0})()
        node._on_cmd_vel(zero_command)

        assert node.last_result is not None
        assert node.last_result.correlation_id == "stop-ros-001"
    finally:
        node.destroy_node()
        rclpy.shutdown()
