"""ROS 2 Jazzy tests for CommandPathObservabilityNode."""

from __future__ import annotations

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from titan_brain_msgs.msg import ArbitrationStatus, EvaluatorObservabilityStatus
from titan_brain_ros.command_path_observability_node import (
    CommandPathObservabilityNode,
    status_qos_profile,
)

_SECOND_NS = 1_000_000_000
_OBSERVATION_NS = _SECOND_NS
_RECEIVED_NS = _OBSERVATION_NS + 10_000_000
_DECISION_NS = _OBSERVATION_NS + 30_000_000
_EVALUATOR_PUBLISHED_NS = _OBSERVATION_NS + 40_000_000
_INTENT_RECEIVED_NS = _OBSERVATION_NS + 45_000_000
_COMMAND_PUBLISHED_NS = _OBSERVATION_NS + 60_000_000


def _parameters() -> list[Parameter]:
    return [
        Parameter(
            "command_path_policy_version",
            value="TB-EVAL-004D-0.1.0",
        ),
        Parameter("arbitration_latency_budget_sec", value=0.03),
        Parameter("observation_to_command_budget_sec", value=0.10),
        Parameter("max_correlation_entries", value=16),
        Parameter("max_pending_per_correlation", value=4),
    ]


def _node() -> CommandPathObservabilityNode:
    return CommandPathObservabilityNode(parameter_overrides=_parameters())


def _evaluator_status(
    *,
    correlation_id: str = "eval_001",
    timing_valid: bool = True,
) -> EvaluatorObservabilityStatus:
    message = EvaluatorObservabilityStatus()
    message.schema_version = "0.1"
    message.policy_version = "TB-OBS-004-0.1.0"
    message.correlation_id = correlation_id
    message.decision_id = "decision_001"
    message.outcome = "normal"
    message.latency_status = "within_budget" if timing_valid else "invalid_timestamp"
    message.timing_valid = timing_valid
    message.within_budget = timing_valid
    if timing_valid:
        message.header.stamp.sec = _EVALUATOR_PUBLISHED_NS // _SECOND_NS
        message.header.stamp.nanosec = _EVALUATOR_PUBLISHED_NS % _SECOND_NS
        message.observation_timestamp_ns = _OBSERVATION_NS
        message.received_timestamp_ns = _RECEIVED_NS
        message.decision_timestamp_ns = _DECISION_NS
        message.published_timestamp_ns = _EVALUATOR_PUBLISHED_NS
        message.observation_to_receive_ns = 10_000_000
        message.receive_to_decision_ns = 20_000_000
        message.decision_to_publish_ns = 10_000_000
        message.end_to_end_ns = 40_000_000
        message.detail = "none"
    else:
        message.detail = "invalid evaluator timestamp"
    return message


def _arbitration_status(
    *,
    correlation_id: str = "eval_001",
    reason: str = "proceed",
    mode: int = ArbitrationStatus.MODE_PASS_THROUGH,
) -> ArbitrationStatus:
    message = ArbitrationStatus()
    message.header.stamp.sec = _COMMAND_PUBLISHED_NS // _SECOND_NS
    message.header.stamp.nanosec = _COMMAND_PUBLISHED_NS % _SECOND_NS
    message.correlation_id = correlation_id
    message.reason = reason
    message.mode = mode
    message.command_sequence_id = 2
    message.safety_intent_sequence_id = 1
    message.arbitration_latency_status = "within_budget"
    message.arbitration_timing_valid = True
    message.arbitration_within_budget = True
    message.intent_received_timestamp_ns = _INTENT_RECEIVED_NS
    message.command_published_timestamp_ns = _COMMAND_PUBLISHED_NS
    message.arbitration_latency_ns = 15_000_000
    message.arbitration_latency_budget_ns = 30_000_000
    return message


def test_qos_contract_is_reliable_volatile_and_bounded() -> None:
    qos = status_qos_profile()

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_node_correlates_exact_timing_and_owns_only_observability_topics() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_evaluator_status(_evaluator_status())
        node._on_arbitration_status(_arbitration_status())

        report = node.last_report
        status = node.last_status
        assert report is not None
        assert report.observation_to_command_ns == 60_000_000
        assert report.arbitration_latency_ns == 15_000_000
        assert status is not None
        assert status.correlation_id == "eval_001"
        assert status.decision_id == "decision_001"
        assert status.timing_valid is True
        assert status.within_budget is True
        assert status.latency_status == "within_budget"
        assert status.observation_to_command_ns == 60_000_000
        assert status.arbitration_latency_budget_ns == 30_000_000
        assert status.observation_to_command_budget_ns == 100_000_000
        assert node.count_publishers("/safety/command_path_observability") == 1
        assert node.count_publishers("/cmd_vel") == 0
        assert node.count_subscribers("/safety/evaluator_observability") == 1
        assert node.count_subscribers("/safety/arbitration_status") == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_out_of_order_estop_retains_correlation_and_forced_zero_reason() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_arbitration_status(
            _arbitration_status(
                reason="e_stop_active",
                mode=ArbitrationStatus.MODE_FORCED_ZERO,
            )
        )
        assert node.last_report is None

        node._on_evaluator_status(_evaluator_status())

        assert node.last_report is not None
        assert node.last_report.correlation_id == "eval_001"
        assert node.last_report.arbitration_reason == "e_stop_active"
        assert node.last_report.arbitration_mode == ArbitrationStatus.MODE_FORCED_ZERO
        assert node.last_status is not None
        assert node.last_status.correlation_id == "eval_001"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_invalid_evaluator_timing_is_published_as_explicit_invalid_path() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_evaluator_status(_evaluator_status(timing_valid=False))
        node._on_arbitration_status(_arbitration_status())

        assert node.last_status is not None
        assert node.last_status.timing_valid is False
        assert node.last_status.within_budget is False
        assert node.last_status.latency_status == "invalid_timing"
        assert node.last_status.observation_to_command_ns == 0
        assert node.last_status.arbitration_latency_ns == 15_000_000
        assert node.last_status.detail == "Invalid timing in: evaluator."
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_empty_or_inconsistent_arbitration_status_is_never_correlated() -> None:
    rclpy.init()
    node = _node()
    try:
        node._on_evaluator_status(_evaluator_status())
        node._on_arbitration_status(_arbitration_status(correlation_id=""))
        assert node.last_report is None

        inconsistent = _arbitration_status()
        inconsistent.arbitration_latency_ns = 1
        node._on_arbitration_status(inconsistent)
        assert node.last_report is None

        inconsistent_evaluator = _evaluator_status()
        inconsistent_evaluator.end_to_end_ns = 1
        node._on_evaluator_status(inconsistent_evaluator)
        assert node.last_report is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
