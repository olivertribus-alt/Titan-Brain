"""ROS 2 Jazzy tests for the TB-EVAL-008B dynamic envelope adapter."""

from __future__ import annotations

import math

import pytest
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from titan_brain_msgs.msg import SystemFaultStatus
from titan_brain_ros.dynamic_envelope_node import (
    DynamicEnvelopeNode,
    extract_scan_distances,
    safety_qos_profile,
    sensor_qos_profile,
)
from titan_brain_ros.safety_velocity_arbiter_node import (
    SafetyVelocityArbiterNode,
)


def _parameters() -> list[Parameter]:
    return [
        Parameter("output_frame_id", value="base_link"),
        Parameter("policy_version", value="test-008b-policy"),
        Parameter("timer_period_sec", value=0.02),
        Parameter("sensor_timeout_sec", value=0.20),
        Parameter("fault_timeout_sec", value=0.20),
        Parameter("front_sector_deg", value=90.0),
        Parameter("max_scan_samples", value=512),
        Parameter("reaction_time_ns", value=100_000_000),
        Parameter("assured_deceleration_mps2", value=1.5),
        Parameter("clearance_margin_m", value=0.30),
        Parameter("nominal_linear_velocity_mps", value=1.0),
        Parameter("nominal_angular_velocity_radps", value=1.0),
        Parameter("angular_swept_radius_m", value=0.45),
        Parameter("confidence_threshold", value=0.5),
    ]


def _arbiter_parameters() -> list[Parameter]:
    return [
        Parameter("output_frame_id", value="base_link"),
        Parameter("command_timeout_sec", value=0.10),
        Parameter("envelope_timeout_sec", value=0.05),
        Parameter("fault_timeout_sec", value=0.20),
        Parameter("arbitration_latency_budget_sec", value=0.03),
        Parameter("timer_period_sec", value=0.02),
        Parameter("policy_version", value="test-arbiter-policy"),
        Parameter("max_linear_velocity_mps", value=2.0),
        Parameter("max_angular_velocity_radps", value=2.0),
        Parameter("max_linear_acceleration_mps2", value=100.0),
        Parameter("max_linear_deceleration_mps2", value=100.0),
        Parameter("max_angular_acceleration_radps2", value=100.0),
        Parameter("max_angular_deceleration_radps2", value=100.0),
        Parameter("max_linear_jerk_mps3", value=1000.0),
        Parameter("max_angular_jerk_radps3", value=1000.0),
    ]


def _stamp(
    message: LaserScan | SystemFaultStatus | TwistStamped,
    timestamp_ns: int,
) -> None:
    message.header.stamp.sec = timestamp_ns // 1_000_000_000
    message.header.stamp.nanosec = timestamp_ns % 1_000_000_000


def _scan(
    timestamp_ns: int,
    *,
    forward_m: float = 5.0,
    lateral_m: float = 5.0,
    samples: int = 360,
) -> LaserScan:
    message = LaserScan()
    message.header.frame_id = "laser"
    _stamp(message, timestamp_ns)
    message.angle_min = -math.pi
    message.angle_max = math.pi
    message.angle_increment = (2.0 * math.pi) / samples
    message.range_min = 0.10
    message.range_max = 10.0
    message.ranges = [lateral_m] * samples
    middle = samples // 2
    quarter = samples // 8
    for index in range(middle - quarter, middle + quarter + 1):
        message.ranges[index] = forward_m
    return message


def _fault(timestamp_ns: int, state: int) -> SystemFaultStatus:
    message = SystemFaultStatus()
    _stamp(message, timestamp_ns)
    message.fault_state = state
    return message


def _command(timestamp_ns: int) -> TwistStamped:
    message = TwistStamped()
    message.header.frame_id = "base_link"
    _stamp(message, timestamp_ns)
    message.twist.linear.x = 1.0
    message.twist.angular.z = 1.0
    return message


def _destroy(*nodes: DynamicEnvelopeNode | SafetyVelocityArbiterNode) -> None:
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_qos_contracts() -> None:
    sensor = sensor_qos_profile()
    safety = safety_qos_profile()

    assert sensor.history == HistoryPolicy.KEEP_LAST
    assert sensor.depth == 1
    assert sensor.reliability == ReliabilityPolicy.BEST_EFFORT
    assert sensor.durability == DurabilityPolicy.VOLATILE
    assert safety.history == HistoryPolicy.KEEP_LAST
    assert safety.depth == 10
    assert safety.reliability == ReliabilityPolicy.RELIABLE
    assert safety.durability == DurabilityPolicy.VOLATILE


def test_bounded_sector_extraction() -> None:
    distances = extract_scan_distances(
        _scan(1, forward_m=0.8, lateral_m=0.4),
        front_half_angle_rad=math.radians(45.0),
        max_scan_samples=512,
    )

    assert distances.forward_m == pytest.approx(0.8)
    assert distances.lateral_m == pytest.approx(0.4)


def test_scan_sample_bound_fails_closed_before_iteration() -> None:
    with pytest.raises(ValueError, match="SCAN_SAMPLE_LIMIT_EXCEEDED"):
        extract_scan_distances(
            _scan(1, samples=513),
            front_half_angle_rad=math.radians(45.0),
            max_scan_samples=512,
        )


@pytest.mark.parametrize("invalid", [float("nan"), -math.inf, 0.0])
def test_invalid_ranges_are_rejected(invalid: float) -> None:
    scan = _scan(1)
    scan.ranges[180] = invalid
    with pytest.raises(ValueError, match="SCAN_RANGE_INVALID"):
        extract_scan_distances(
            scan,
            front_half_angle_rad=math.radians(45.0),
            max_scan_samples=512,
        )


def test_positive_infinity_is_a_valid_no_return() -> None:
    scan = _scan(1, forward_m=math.inf, lateral_m=math.inf)
    distances = extract_scan_distances(
        scan,
        front_half_angle_rad=math.radians(45.0),
        max_scan_samples=512,
    )
    assert distances.forward_m == scan.range_max
    assert distances.lateral_m == scan.range_max


def test_startup_is_fail_closed_and_has_single_envelope_publisher() -> None:
    rclpy.init()
    node = DynamicEnvelopeNode(parameter_overrides=_parameters())
    try:
        assert node.last_envelope is not None
        assert node.last_envelope.max_linear_x_mps == 0.0
        assert node.last_envelope.max_angular_z_radps == 0.0
        assert node.last_diagnostics is not None
        assert (
            node.last_diagnostics.reason == "SYSTEM_FAULT_STATUS_MISSING"
        )
        assert node.count_publishers("/safety/permitted_motion_envelope") == 1
        assert node.count_publishers("/safety/envelope_diagnostics") == 1
        assert node.count_subscribers("/scan") == 1
    finally:
        _destroy(node)


def test_fresh_scan_publishes_correlated_dynamic_authority() -> None:
    rclpy.init()
    node = DynamicEnvelopeNode(parameter_overrides=_parameters())
    try:
        now_ns = node._now_ns() + 1_000_000
        node._now_ns = lambda: now_ns
        node._on_fault_status(_fault(now_ns, SystemFaultStatus.FAULT_OK))
        node._on_scan(_scan(now_ns, forward_m=0.8, lateral_m=0.6))
        node._on_timer()

        envelope = node.last_envelope
        diagnostics = node.last_diagnostics
        assert envelope is not None
        assert diagnostics is not None
        assert 0.0 < envelope.max_linear_x_mps < 1.0
        assert 0.0 < envelope.max_angular_z_radps <= 1.0
        assert envelope.min_angular_z_radps == -envelope.max_angular_z_radps
        assert envelope.correlation_id == diagnostics.correlation_id
        assert envelope.sequence_id == diagnostics.sequence_id
        assert diagnostics.scan_valid is True
        assert diagnostics.fault_status_valid is True
    finally:
        _destroy(node)


def test_close_obstacle_produces_zero_envelope() -> None:
    rclpy.init()
    node = DynamicEnvelopeNode(parameter_overrides=_parameters())
    try:
        now_ns = node._now_ns() + 1_000_000
        node._now_ns = lambda: now_ns
        node._on_fault_status(_fault(now_ns, SystemFaultStatus.FAULT_OK))
        node._on_scan(_scan(now_ns, forward_m=0.30, lateral_m=0.30))
        node._on_timer()
        assert node.last_envelope is not None
        assert node.last_envelope.max_linear_x_mps == 0.0
        assert node.last_envelope.max_angular_z_radps == 0.0
        assert node.last_diagnostics is not None
        assert (
            node.last_diagnostics.state
            == node.last_diagnostics.STATE_PROTECTIVE_STOP
        )
    finally:
        _destroy(node)


@pytest.mark.parametrize(
    ("scan_offset_ns", "reason"),
    [
        (-200_000_001, "SCAN_TIMEOUT"),
        (1, "SCAN_FUTURE_TIMESTAMP"),
    ],
)
def test_scan_freshness_guard(
    scan_offset_ns: int,
    reason: str,
) -> None:
    rclpy.init()
    node = DynamicEnvelopeNode(parameter_overrides=_parameters())
    try:
        now_ns = node._now_ns() + 1_000_000
        node._now_ns = lambda: now_ns
        node._on_fault_status(_fault(now_ns, SystemFaultStatus.FAULT_OK))
        node._on_scan(_scan(now_ns + scan_offset_ns))
        node._on_timer()
        assert node.last_envelope is not None
        assert node.last_envelope.max_linear_x_mps == 0.0
        assert node.last_diagnostics is not None
        assert node.last_diagnostics.reason == reason
    finally:
        _destroy(node)


@pytest.mark.parametrize(
    ("state", "reason"),
    [
        (
            SystemFaultStatus.FAULT_E_STOP_ACTIVE,
            "SYSTEM_FAULT_E_STOP_ACTIVE",
        ),
        (
            SystemFaultStatus.FAULT_HARDWARE_FAULT,
            "SYSTEM_FAULT_HARDWARE_FAULT",
        ),
        (
            SystemFaultStatus.FAULT_LATCHED_SAFETY_FAULT,
            "SYSTEM_FAULT_LATCHED_SAFETY_FAULT",
        ),
        (255, "INVALID_SYSTEM_FAULT_STATE"),
    ],
)
def test_fault_override_is_immediate(state: int, reason: str) -> None:
    rclpy.init()
    node = DynamicEnvelopeNode(parameter_overrides=_parameters())
    try:
        now_ns = node._now_ns() + 1_000_000
        node._now_ns = lambda: now_ns
        node._on_fault_status(_fault(now_ns, state))
        node._on_scan(_scan(now_ns))
        node._on_timer()
        assert node.last_envelope is not None
        assert node.last_envelope.max_linear_x_mps == 0.0
        assert node.last_diagnostics is not None
        assert node.last_diagnostics.reason == reason
    finally:
        _destroy(node)


def test_clock_regression_latches_fail_closed() -> None:
    rclpy.init()
    node = DynamicEnvelopeNode(parameter_overrides=_parameters())
    try:
        baseline_ns = node._now_ns() + 1_000_000
        node._now_ns = lambda: baseline_ns
        node._on_fault_status(
            _fault(baseline_ns, SystemFaultStatus.FAULT_OK)
        )
        node._on_scan(_scan(baseline_ns))
        node._on_timer()
        assert node.last_envelope is not None
        assert node.last_envelope.max_linear_x_mps > 0.0

        node._now_ns = lambda: baseline_ns - 1
        node._on_timer()
        assert node.last_diagnostics is not None
        assert node.last_diagnostics.reason == "CLOCK_REGRESSION_LATCHED"

        node._now_ns = lambda: baseline_ns + 1
        node._on_timer()
        assert node.last_envelope is not None
        assert node.last_envelope.max_linear_x_mps == 0.0
        assert node.last_diagnostics is not None
        assert node.last_diagnostics.reason == "CLOCK_REGRESSION_LATCHED"
    finally:
        _destroy(node)


def test_dynamic_envelope_clamps_safety_velocity_arbiter() -> None:
    rclpy.init()
    envelope_node = DynamicEnvelopeNode(parameter_overrides=_parameters())
    arbiter = SafetyVelocityArbiterNode(
        parameter_overrides=_arbiter_parameters()
    )
    try:
        now_ns = max(
            envelope_node._now_ns(),
            arbiter.governor.last_timestamp_ns,
        ) + 1_000_000_000
        envelope_node._now_ns = lambda: now_ns
        arbiter._now_ns = lambda: now_ns
        fault = _fault(now_ns, SystemFaultStatus.FAULT_OK)
        envelope_node._on_fault_status(fault)
        envelope_node._on_scan(
            _scan(now_ns, forward_m=0.8, lateral_m=0.35)
        )
        envelope_node._on_timer()
        envelope = envelope_node.last_envelope
        assert envelope is not None

        arbiter._on_fault_status(fault)
        arbiter._on_motion_envelope(envelope)
        arbiter._on_teleop_command(_command(now_ns))
        arbiter._on_timer()

        assert arbiter.last_result is not None
        assert arbiter.last_result.linear_velocity_mps == pytest.approx(
            envelope.max_linear_x_mps
        )
        assert arbiter.last_result.angular_velocity_radps == pytest.approx(
            envelope.max_angular_z_radps
        )
        assert arbiter.last_status is not None
        assert arbiter.last_status.mode == arbiter.last_status.MODE_CLAMPED
    finally:
        _destroy(envelope_node, arbiter)
