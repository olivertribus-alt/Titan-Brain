"""ROS 2 Jazzy tests for the TB-EVAL-006B command governor adapter."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from titan_brain_msgs.msg import SafetySupervisorStatus
from titan_brain_ros.command_governor_node import (
    GOVERNED_COMMAND_TOPIC,
    RAW_COMMAND_TOPIC,
    SAFETY_STATUS_TOPIC,
    CommandGovernorNode,
    command_qos_profile,
    safety_qos_profile,
)


def _parameters() -> list[Parameter]:
    return [
        Parameter("timer_period_sec", value=0.02),
        Parameter("cmd_timeout_sec", value=0.20),
        Parameter("safety_timeout_sec", value=0.25),
        Parameter("stale_command_emergency_stop", value=True),
        Parameter("max_linear_velocity_mps", value=2.0),
        Parameter("max_angular_velocity_radps", value=2.0),
        Parameter("max_linear_acceleration_mps2", value=2.0),
        Parameter("max_linear_deceleration_mps2", value=4.0),
        Parameter("max_angular_acceleration_radps2", value=2.0),
        Parameter("max_angular_deceleration_radps2", value=4.0),
        Parameter("max_linear_jerk_mps3", value=100.0),
        Parameter("max_angular_jerk_radps3", value=100.0),
    ]


def _new_node() -> CommandGovernorNode:
    return CommandGovernorNode(parameter_overrides=_parameters())


def _safe_status() -> SafetySupervisorStatus:
    message = SafetySupervisorStatus()
    message.supervisor_state = SafetySupervisorStatus.STATE_OK
    message.relay_closed_request = True
    message.active_faults = []
    return message


def _trip_status() -> SafetySupervisorStatus:
    message = SafetySupervisorStatus()
    message.supervisor_state = SafetySupervisorStatus.STATE_TRIPPED
    message.relay_closed_request = False
    message.active_faults = ["heartbeat_timeout"]
    return message


def _twist(linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
    message = Twist()
    message.linear.x = linear_x
    message.angular.z = angular_z
    return message


def _destroy(node: CommandGovernorNode) -> None:
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


def test_startup_is_fail_closed_and_topics_are_owned() -> None:
    rclpy.init()
    node = _new_node()
    try:
        assert node.last_result is not None
        assert node.last_result.linear_velocity_mps == 0.0
        assert node.last_result.emergency_override is True
        assert node.count_publishers(GOVERNED_COMMAND_TOPIC) == 1
        assert node.count_subscribers(RAW_COMMAND_TOPIC) == 1
        assert node.count_subscribers(SAFETY_STATUS_TOPIC) == 1
    finally:
        _destroy(node)


def test_safe_status_and_fresh_command_are_governed() -> None:
    rclpy.init()
    node = _new_node()
    try:
        node._on_safety_status(_safe_status())
        node._on_raw_command(_twist(linear_x=1.5, angular_z=-1.0))
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.is_safe is True
        assert node.last_result.emergency_override is False
        assert node.last_result.linear_velocity_mps > 0.0
        assert node.last_result.angular_velocity_radps < 0.0
    finally:
        _destroy(node)


def test_trip_status_immediately_bypasses_governor() -> None:
    rclpy.init()
    node = _new_node()
    try:
        node._on_safety_status(_safe_status())
        node._on_raw_command(_twist(linear_x=1.0))
        node._on_timer()
        node._on_safety_status(_trip_status())
        assert node.last_result is not None
        assert node.last_result.emergency_override is True
        assert node.last_result.linear_velocity_mps == 0.0
        assert node.last_result.angular_velocity_radps == 0.0
    finally:
        _destroy(node)


def test_missing_or_stale_command_is_fail_closed() -> None:
    rclpy.init()
    node = _new_node()
    try:
        node._on_safety_status(_safe_status())
        assert node.last_result is not None
        assert node.last_result.emergency_override is True
        baseline = node.last_command_received_ns
        assert baseline is None
    finally:
        _destroy(node)


def test_configured_stale_command_path_can_use_governed_deceleration() -> None:
    rclpy.init()
    parameters = [
        parameter
        for parameter in _parameters()
        if parameter.name != "stale_command_emergency_stop"
    ]
    parameters.append(Parameter("stale_command_emergency_stop", value=False))
    node = CommandGovernorNode(parameter_overrides=parameters)
    try:
        node._on_safety_status(_safe_status())
        node._on_raw_command(_twist(linear_x=1.0))
        first = node.last_command_received_ns
        assert first is not None

        node._now_ns = lambda: first + 201_000_000
        node._on_timer()
        assert node.last_result is not None
        assert node.last_result.emergency_override is False
        assert node.last_result.linear_velocity_mps == 0.0
    finally:
        _destroy(node)


def test_nonzero_lateral_command_is_rejected() -> None:
    rclpy.init()
    node = _new_node()
    try:
        node._on_safety_status(_safe_status())
        message = _twist(linear_x=1.0)
        message.linear.y = 0.1
        node._on_raw_command(message)
        assert node.last_result is not None
        assert node.last_result.emergency_override is True
        assert node.last_result.linear_velocity_mps == 0.0
    finally:
        _destroy(node)
