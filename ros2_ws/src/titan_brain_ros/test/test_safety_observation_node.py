"""ROS 2 Jazzy runtime tests for SafetyObservationNode."""

from __future__ import annotations

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from titan_brain_msgs.msg import SafetyIntent, SafetyObservation
from titan_brain_ros.safety_observation_node import (
    SafetyObservationNode,
    sensor_data_qos_profile,
    status_qos_profile,
)


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
