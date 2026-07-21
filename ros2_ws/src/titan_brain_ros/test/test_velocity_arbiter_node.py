"""ROS 2 Jazzy runtime tests for VelocityArbiterNode."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from titan_brain_msgs.msg import ArbitrationStatus, SafetyEvaluationStatus
from titan_brain_ros.velocity_arbiter_node import (
    VelocityArbiterNode,
    command_qos_profile,
    status_qos_profile,
)

from core.arbitrator import ArbitrationMode, ArbitrationReason

_NANOSECONDS_PER_SECOND = 1_000_000_000


def _parameters() -> list[Parameter]:
    return [
        Parameter("policy_version", value="TB-VEL-ARB-0.1.0"),
        Parameter("output_frame_id", value="base_link"),
        Parameter("command_stale_threshold_sec", value=0.1),
        Parameter("safety_stale_threshold_sec", value=0.25),
        Parameter("timer_period_sec", value=0.02),
        Parameter("max_abs_linear_x", value=0.8),
        Parameter("max_abs_linear_y", value=0.2),
        Parameter("max_abs_angular_z", value=1.5),
    ]


def _node() -> VelocityArbiterNode:
    return VelocityArbiterNode(parameter_overrides=_parameters())


def _set_stamp(message: SafetyEvaluationStatus, timestamp_ns: int) -> None:
    message.header.stamp.sec = timestamp_ns // _NANOSECONDS_PER_SECOND
    message.header.stamp.nanosec = timestamp_ns % _NANOSECONDS_PER_SECOND


def _accepted_status(
    node: VelocityArbiterNode,
    *,
    action: str = "proceed",
    timestamp_ns: int | None = None,
) -> SafetyEvaluationStatus:
    message = SafetyEvaluationStatus()
    _set_stamp(
        message,
        node.get_clock().now().nanoseconds
        if timestamp_ns is None
        else timestamp_ns,
    )
    message.header.frame_id = "map"
    message.schema_version = "0.1"
    message.adapter_status = "accepted"
    message.watchdog_status = "healthy"
    message.watchdog_healthy = True
    message.observation_accepted = True
    message.action = action
    return message


def _heartbeat(
    node: VelocityArbiterNode,
    *,
    watchdog_status: str = "healthy",
    watchdog_healthy: bool = True,
) -> SafetyEvaluationStatus:
    message = SafetyEvaluationStatus()
    _set_stamp(message, node.get_clock().now().nanoseconds)
    message.schema_version = "0.1"
    message.adapter_status = "watchdog"
    message.watchdog_status = watchdog_status
    message.watchdog_healthy = watchdog_healthy
    message.observation_accepted = False
    return message


def _desired_twist(
    *,
    linear_x: float = 0.4,
    linear_y: float = 0.1,
    angular_z: float = 0.5,
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


def test_startup_is_forced_zero_and_node_owns_expected_topics() -> None:
    rclpy.init()
    node = _node()
    try:
        result = node.last_result
        status = node.last_status

        assert result is not None
        assert result.mode is ArbitrationMode.FORCED_ZERO
        assert result.reason is ArbitrationReason.SAFETY_STATE_MISSING
        assert result.command.linear_x == 0.0
        assert status is not None
        assert status.mode == ArbitrationStatus.MODE_FORCED_ZERO
        assert status.commanded_twist.linear.x == 0.0
        assert node.count_publishers("/cmd_vel") == 1
        assert node.count_publishers("/safety/arbitration_status") == 1
        assert node.count_subscribers("/cmd_vel_nav") == 1
        assert node.count_subscribers("/safety/evaluation_status") == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_fresh_proceed_status_and_command_are_passed_through() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_safety_status(_accepted_status(node))
        node._on_desired_velocity(_desired_twist())

        node._on_timer()

        result = node.last_result
        status = node.last_status
        assert result is not None
        assert result.mode is ArbitrationMode.PASS_THROUGH
        assert result.command.linear_x == 0.4
        assert result.command.linear_y == 0.1
        assert result.command.angular_z == 0.5
        assert status is not None
        assert status.mode == ArbitrationStatus.MODE_PASS_THROUGH
        assert status.reason == "proceed"
        assert status.commanded_twist.linear.x == 0.4
        assert status.header.frame_id == "base_link"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_clamp_and_stop_actions_preserve_core_policy_results() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_desired_velocity(
            _desired_twist(linear_x=2.0, linear_y=-1.0, angular_z=3.0)
        )
        node._on_safety_status(_accepted_status(node, action="clamp"))
        node._on_timer()

        clamped = node.last_result
        assert clamped is not None
        assert clamped.mode is ArbitrationMode.CLAMPED
        assert clamped.command.linear_x == 0.8
        assert clamped.command.linear_y == -0.2
        assert clamped.command.angular_z == 1.5
        assert node.last_status is not None
        assert node.last_status.mode == ArbitrationStatus.MODE_CLAMPED

        node._on_safety_status(
            _accepted_status(node, action="protective_stop")
        )
        node._on_timer()
        protective = node.last_result
        assert protective is not None
        assert protective.reason is ArbitrationReason.PROTECTIVE_STOP
        assert protective.command.linear_x == 0.0

        node._on_safety_status(
            _accepted_status(node, action="emergency_stop")
        )
        node._on_timer()
        emergency = node.last_result
        assert emergency is not None
        assert emergency.reason is ArbitrationReason.EMERGENCY_STOP
        assert emergency.command.linear_x == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_watchdog_heartbeat_does_not_refresh_evaluation_timestamp() -> None:
    rclpy.init()
    node = _node()
    try:
        stale_timestamp = node.get_clock().now().nanoseconds - 300_000_000
        node._on_safety_status(
            _accepted_status(node, timestamp_ns=stale_timestamp)
        )
        node._on_safety_status(_heartbeat(node))
        node._on_desired_velocity(_desired_twist())

        node._on_timer()

        result = node.last_result
        assert result is not None
        assert result.mode is ArbitrationMode.FORCED_ZERO
        assert result.reason is ArbitrationReason.SAFETY_STATE_STALE
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_unhealthy_watchdog_forces_zero_with_stable_reason() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_safety_status(_accepted_status(node))
        node._on_safety_status(
            _heartbeat(
                node,
                watchdog_status="timed_out",
                watchdog_healthy=False,
            )
        )
        node._on_desired_velocity(_desired_twist())

        node._on_timer()

        result = node.last_result
        assert result is not None
        assert result.mode is ArbitrationMode.FORCED_ZERO
        assert result.reason is ArbitrationReason.WATCHDOG_TIMED_OUT
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_transport_failure_stays_latched_until_new_accepted_evaluation() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_safety_status(_accepted_status(node))
        failure = SafetyEvaluationStatus()
        _set_stamp(failure, node.get_clock().now().nanoseconds)
        failure.schema_version = "0.1"
        failure.adapter_status = "tf_unavailable"
        failure.watchdog_status = "healthy"
        failure.watchdog_healthy = True
        failure.observation_accepted = False
        node._on_safety_status(failure)
        node._on_safety_status(_heartbeat(node))
        node._on_desired_velocity(_desired_twist())

        node._on_timer()
        failed_result = node.last_result

        assert failed_result is not None
        assert failed_result.reason is ArbitrationReason.SAFETY_STATE_INVALID

        node._on_safety_status(_accepted_status(node))
        node._on_timer()
        recovered_result = node.last_result

        assert recovered_result is not None
        assert recovered_result.mode is ArbitrationMode.PASS_THROUGH
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_unknown_safety_status_schema_fails_closed() -> None:
    rclpy.init()
    node = _node()
    try:
        unknown = _accepted_status(node)
        unknown.schema_version = "99.0"
        node._on_safety_status(unknown)
        node._on_desired_velocity(_desired_twist())

        node._on_timer()

        result = node.last_result
        assert result is not None
        assert result.mode is ArbitrationMode.FORCED_ZERO
        assert result.reason is ArbitrationReason.SAFETY_STATE_INVALID
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_non_finite_twist_fails_closed_in_core_arbiter() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_safety_status(_accepted_status(node))
        node._on_desired_velocity(_desired_twist(linear_x=float("nan")))

        node._on_timer()

        result = node.last_result
        assert result is not None
        assert result.mode is ArbitrationMode.FORCED_ZERO
        assert result.reason is ArbitrationReason.COMMAND_INVALID
        assert result.command.linear_x == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()
