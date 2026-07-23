"""ROS 2 Jazzy runtime tests for SafetyObservationNode."""

from __future__ import annotations

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from titan_brain_msgs.msg import (
    DirectionalSafetyObservation,
    SafetyIntent,
    SafetyObservation,
)
from titan_brain_ros.safety_observation_node import (
    SafetyObservationNode,
    sensor_data_qos_profile,
    status_qos_profile,
)


def _dynamic_parameters() -> list[Parameter]:
    return [
        Parameter("dynamic_braking_enabled", value=True),
        Parameter("safety_policy_version", value="TB-SAFE-005C-0.1.0"),
        Parameter("clearance_threshold_m", value=0.5),
        Parameter("confidence_threshold", value=0.7),
        Parameter("braking_policy_version", value="TB-BRAKE-005C-0.1.0"),
        Parameter("reaction_time_ns", value=250_000_000),
        Parameter("assured_deceleration_mps2", value=2.0),
        Parameter("clearance_margin_m", value=0.1),
        Parameter("motion_envelope_frame_id", value="base_link"),
    ]


def _directional_observation(
    node: SafetyObservationNode,
) -> DirectionalSafetyObservation:
    message = DirectionalSafetyObservation()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = "map"
    message.map_id = "warehouse_zone_c"
    message.pose.x = 1.0
    message.pose.y = 2.0
    message.pose.theta = 0.25
    message.clearance_m = 0.6
    message.confidence = 0.95
    message.sensor_id = "directional_lidar"
    message.forward_clearance_m = 0.6
    message.reverse_clearance_m = 0.2875
    message.left_clearance_m = 0.178125
    message.right_clearance_m = 0.428125
    message.velocity.linear.x = 0.1
    return message


def test_qos_contracts() -> None:
    sensor_qos = sensor_data_qos_profile()
    status_qos = status_qos_profile()

    assert sensor_qos.history == HistoryPolicy.KEEP_LAST
    assert sensor_qos.depth == 5
    assert sensor_qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert sensor_qos.durability == DurabilityPolicy.VOLATILE
    assert status_qos.history == HistoryPolicy.KEEP_LAST
    assert status_qos.depth == 10
    assert status_qos.reliability == ReliabilityPolicy.RELIABLE
    assert status_qos.durability == DurabilityPolicy.VOLATILE


def test_node_accepts_normalized_observation_in_target_frame() -> None:
    rclpy.init()
    node = SafetyObservationNode()
    try:
        message = SafetyObservation()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.map_id = "warehouse_zone_c"
        message.pose.x = 1.0
        message.pose.y = 2.0
        message.pose.theta = 0.25
        message.clearance_m = 1.2
        message.confidence = 0.95
        message.sensor_id = "front_lidar"

        node._on_observation(message)

        assert node.adapter.last_valid_received_at_ns is not None
        assert node.count_publishers("/safety/evaluation_status") == 1
        assert node.count_publishers("/safety/stability_status") == 1
        assert node.count_publishers("/safety/evaluator_observability") == 1
        assert node.count_publishers("/safety/intent") == 1
        assert node.count_publishers("/safety/permitted_motion_envelope") == 1
        assert node.observability.counters.total == 1
        assert node.observability.counters.normal == 1
        assert node.last_safety_intent is not None
        assert node.last_safety_intent.state == SafetyIntent.STATE_NORMAL
        assert node.last_safety_intent.sequence_id == 1
        assert node.last_safety_intent.correlation_id.startswith("eval_")
        assert node.count_subscribers("/safety/observation") == 1
        assert node.count_subscribers("/safety/directional_observation") == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_directional_observation_publishes_correlated_motion_envelope() -> None:
    rclpy.init()
    node = SafetyObservationNode(parameter_overrides=_dynamic_parameters())
    try:
        node._on_directional_observation(_directional_observation(node))

        intent = node.last_safety_intent
        envelope = node.last_motion_envelope
        assert intent is not None
        assert envelope is not None
        assert envelope.header.frame_id == "base_link"
        assert envelope.policy_version == "TB-BRAKE-005C-0.1.0"
        assert envelope.correlation_id == intent.correlation_id
        assert envelope.sequence_id == intent.sequence_id
        assert envelope.min_linear_x_mps == -0.5
        assert envelope.max_linear_x_mps == 1.0
        assert envelope.min_linear_y_mps == -0.75
        assert envelope.max_linear_y_mps == 0.25
        assert envelope.min_angular_z_radps == 0.0
        assert envelope.max_angular_z_radps == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_motion_envelope_publisher_can_be_delegated() -> None:
    rclpy.init()
    node = SafetyObservationNode(
        parameter_overrides=[
            Parameter("publish_motion_envelope", value=False),
        ]
    )
    try:
        assert node.count_publishers("/safety/permitted_motion_envelope") == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_missing_directional_evidence_publishes_zero_envelope() -> None:
    rclpy.init()
    node = SafetyObservationNode(parameter_overrides=_dynamic_parameters())
    try:
        message = SafetyObservation()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.map_id = "warehouse_zone_c"
        message.pose.x = 1.0
        message.pose.y = 2.0
        message.pose.theta = 0.25
        message.clearance_m = 1.2
        message.confidence = 0.95
        message.sensor_id = "legacy_lidar"

        node._on_observation(message)

        envelope = node.last_motion_envelope
        assert envelope is not None
        assert envelope.min_linear_x_mps == 0.0
        assert envelope.max_linear_x_mps == 0.0
        assert envelope.min_linear_y_mps == 0.0
        assert envelope.max_linear_y_mps == 0.0
        assert envelope.min_angular_z_radps == 0.0
        assert envelope.max_angular_z_radps == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()
