"""ROS 2 adapter for TB-EVAL-008 dynamic motion-envelope evaluation."""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from titan_brain_msgs.msg import (
    EnvelopeDiagnostics,
    PermittedMotionEnvelope,
    SystemFaultStatus,
)

from core.command_governor import NANOSECONDS_PER_SECOND
from core.dynamic_envelope_evaluator import (
    DynamicEnvelopeEvaluator,
    EnvelopeConfig,
    EnvelopeResult,
    EnvelopeSource,
    EnvelopeState,
    LimitingZone,
    SensorFrame,
)

SCAN_TOPIC = "/scan"
SYSTEM_FAULT_TOPIC = "/safety/system_fault_status"
MOTION_ENVELOPE_TOPIC = "/safety/permitted_motion_envelope"
ENVELOPE_DIAGNOSTICS_TOPIC = "/safety/envelope_diagnostics"


@dataclass(frozen=True, slots=True)
class ScanDistances:
    """Constant-size evidence extracted from a bounded LaserScan."""

    forward_m: float
    lateral_m: float


def sensor_qos_profile() -> QoSProfile:
    """Return the bounded best-effort ROS sensor-data contract."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def safety_qos_profile() -> QoSProfile:
    """Return the reliable safety control-plane contract."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _finite_float_parameter(
    node: Node,
    name: str,
    default: float,
    *,
    allow_zero: bool,
) -> float:
    value = node.declare_parameter(name, default).value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"ROS parameter {name!r} must be numeric")
    checked = float(value)
    lower_bound_ok = checked >= 0.0 if allow_zero else checked > 0.0
    if not math.isfinite(checked) or not lower_bound_ok:
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(
            f"ROS parameter {name!r} must be finite and {qualifier}"
        )
    return checked


def _positive_int_parameter(node: Node, name: str, default: int) -> int:
    value = node.declare_parameter(name, default).value
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"ROS parameter {name!r} must be a positive integer")
    return value


def _text_parameter(node: Node, name: str, default: str) -> str:
    value = node.declare_parameter(name, default).value
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"ROS parameter {name!r} must be non-blank text")
    return value.strip()


def _seconds_to_ns(seconds: float, *, name: str) -> int:
    value = round(seconds * NANOSECONDS_PER_SECOND)
    if value <= 0:
        raise ValueError(f"ROS parameter {name!r} must round to at least 1 ns")
    return value


def _stamp_ns(message: LaserScan | SystemFaultStatus) -> int:
    return (
        int(message.header.stamp.sec) * NANOSECONDS_PER_SECOND
        + int(message.header.stamp.nanosec)
    )


def _normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def extract_scan_distances(
    scan: LaserScan,
    *,
    front_half_angle_rad: float,
    max_scan_samples: int,
) -> ScanDistances:
    """Validate and reduce at most ``max_scan_samples`` beams."""
    if not str(scan.header.frame_id).strip():
        raise ValueError("SCAN_FRAME_MISSING")
    scalars = (
        float(scan.angle_min),
        float(scan.angle_increment),
        float(scan.range_min),
        float(scan.range_max),
    )
    if any(not math.isfinite(value) for value in scalars):
        raise ValueError("SCAN_METADATA_INVALID")
    if scan.angle_increment == 0.0:
        raise ValueError("SCAN_ANGLE_INCREMENT_INVALID")
    if scan.range_min < 0.0 or scan.range_max <= scan.range_min:
        raise ValueError("SCAN_RANGE_BOUNDS_INVALID")
    sample_count = len(scan.ranges)
    if sample_count == 0:
        raise ValueError("SCAN_EMPTY")
    if sample_count > max_scan_samples:
        raise ValueError("SCAN_SAMPLE_LIMIT_EXCEEDED")

    forward = float(scan.range_max)
    lateral = float(scan.range_max)
    forward_seen = False
    lateral_seen = False
    finite_return_seen = False
    angle = float(scan.angle_min)
    for raw_range in scan.ranges:
        measured = float(raw_range)
        if math.isnan(measured) or measured == -math.inf:
            raise ValueError("SCAN_RANGE_INVALID")
        if measured == math.inf:
            measured = float(scan.range_max)
        elif measured < scan.range_min or measured > scan.range_max:
            raise ValueError("SCAN_RANGE_INVALID")
        else:
            finite_return_seen = True

        normalized_angle = _normalize_angle(angle)
        if abs(normalized_angle) <= front_half_angle_rad:
            forward = min(forward, measured)
            forward_seen = True
        else:
            lateral = min(lateral, measured)
            lateral_seen = True
        angle += float(scan.angle_increment)

    if not forward_seen or not lateral_seen:
        raise ValueError("SCAN_SECTOR_COVERAGE_INCOMPLETE")
    if not finite_return_seen:
        raise ValueError("SCAN_NO_FINITE_RETURNS")
    return ScanDistances(forward_m=forward, lateral_m=lateral)


class DynamicEnvelopeNode(Node):
    """Publish one fresh, bounded envelope from scan and fault evidence."""

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
    ) -> None:
        super().__init__(
            "dynamic_envelope_node",
            parameter_overrides=parameter_overrides,
        )
        self._output_frame_id = _text_parameter(
            self,
            "output_frame_id",
            "base_link",
        )
        policy_version = _text_parameter(
            self,
            "policy_version",
            "TB-EVAL-008B-0.1.0",
        )
        timer_period_sec = _finite_float_parameter(
            self,
            "timer_period_sec",
            0.02,
            allow_zero=False,
        )
        sensor_timeout_sec = _finite_float_parameter(
            self,
            "sensor_timeout_sec",
            0.20,
            allow_zero=False,
        )
        fault_timeout_sec = _finite_float_parameter(
            self,
            "fault_timeout_sec",
            0.20,
            allow_zero=False,
        )
        front_sector_deg = _finite_float_parameter(
            self,
            "front_sector_deg",
            90.0,
            allow_zero=False,
        )
        if front_sector_deg >= 360.0:
            raise ValueError("front_sector_deg must be less than 360 degrees")
        self._front_half_angle_rad = math.radians(front_sector_deg / 2.0)
        self._max_scan_samples = _positive_int_parameter(
            self,
            "max_scan_samples",
            4096,
        )
        self._sensor_timeout_ns = _seconds_to_ns(
            sensor_timeout_sec,
            name="sensor_timeout_sec",
        )
        self._fault_timeout_ns = _seconds_to_ns(
            fault_timeout_sec,
            name="fault_timeout_sec",
        )
        self._evaluator = DynamicEnvelopeEvaluator(
            EnvelopeConfig(
                policy_version=policy_version,
                reaction_time_ns=_positive_int_parameter(
                    self,
                    "reaction_time_ns",
                    100_000_000,
                ),
                assured_deceleration_mps2=_finite_float_parameter(
                    self,
                    "assured_deceleration_mps2",
                    1.5,
                    allow_zero=False,
                ),
                clearance_margin_m=_finite_float_parameter(
                    self,
                    "clearance_margin_m",
                    0.30,
                    allow_zero=True,
                ),
                nominal_linear_velocity_mps=_finite_float_parameter(
                    self,
                    "nominal_linear_velocity_mps",
                    1.0,
                    allow_zero=False,
                ),
                nominal_angular_velocity_radps=_finite_float_parameter(
                    self,
                    "nominal_angular_velocity_radps",
                    1.0,
                    allow_zero=False,
                ),
                angular_swept_radius_m=_finite_float_parameter(
                    self,
                    "angular_swept_radius_m",
                    0.45,
                    allow_zero=False,
                ),
                confidence_threshold=_finite_float_parameter(
                    self,
                    "confidence_threshold",
                    0.5,
                    allow_zero=True,
                ),
                max_sensor_age_s=sensor_timeout_sec,
            )
        )
        if self._evaluator.config.confidence_threshold > 1.0:
            raise ValueError("confidence_threshold must not exceed 1.0")

        self._latest_scan: LaserScan | None = None
        self._latest_fault: SystemFaultStatus | None = None
        self._last_scan_timestamp_ns: int | None = None
        self._last_now_ns: int | None = None
        self._timing_fault_latched = False
        self._sequence_id = 0
        self._last_envelope: PermittedMotionEnvelope | None = None
        self._last_diagnostics: EnvelopeDiagnostics | None = None

        sensor_qos = sensor_qos_profile()
        safety_qos = safety_qos_profile()
        self._envelope_publisher = self.create_publisher(
            PermittedMotionEnvelope,
            MOTION_ENVELOPE_TOPIC,
            safety_qos,
        )
        self._diagnostics_publisher = self.create_publisher(
            EnvelopeDiagnostics,
            ENVELOPE_DIAGNOSTICS_TOPIC,
            safety_qos,
        )
        self._scan_subscription = self.create_subscription(
            LaserScan,
            SCAN_TOPIC,
            self._on_scan,
            sensor_qos,
        )
        self._fault_subscription = self.create_subscription(
            SystemFaultStatus,
            SYSTEM_FAULT_TOPIC,
            self._on_fault_status,
            safety_qos,
        )
        self._timer = self.create_timer(timer_period_sec, self._on_timer)
        self._on_timer()

    @property
    def evaluator(self) -> DynamicEnvelopeEvaluator:
        """Expose the dependency-free evaluator for diagnostics and tests."""
        return self._evaluator

    @property
    def last_envelope(self) -> PermittedMotionEnvelope | None:
        """Return the latest published envelope."""
        return self._last_envelope

    @property
    def last_diagnostics(self) -> EnvelopeDiagnostics | None:
        """Return the latest published diagnostics."""
        return self._last_diagnostics

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _on_scan(self, message: LaserScan) -> None:
        timestamp_ns = _stamp_ns(message)
        if (
            self._last_scan_timestamp_ns is not None
            and timestamp_ns <= self._last_scan_timestamp_ns
        ):
            self._timing_fault_latched = True
        if (
            self._last_scan_timestamp_ns is None
            or timestamp_ns > self._last_scan_timestamp_ns
        ):
            self._last_scan_timestamp_ns = timestamp_ns
        self._latest_scan = message

    def _on_fault_status(self, message: SystemFaultStatus) -> None:
        self._latest_fault = message

    def _fault_rejection(self, now_ns: int) -> str | None:
        message = self._latest_fault
        if message is None:
            return "SYSTEM_FAULT_STATUS_MISSING"
        timestamp_ns = _stamp_ns(message)
        if timestamp_ns > now_ns:
            return "SYSTEM_FAULT_STATUS_FUTURE_TIMESTAMP"
        if now_ns - timestamp_ns > self._fault_timeout_ns:
            return "SYSTEM_FAULT_STATUS_TIMEOUT"
        return {
            SystemFaultStatus.FAULT_OK: None,
            SystemFaultStatus.FAULT_E_STOP_ACTIVE: "SYSTEM_FAULT_E_STOP_ACTIVE",
            SystemFaultStatus.FAULT_HARDWARE_FAULT: (
                "SYSTEM_FAULT_HARDWARE_FAULT"
            ),
            SystemFaultStatus.FAULT_LATCHED_SAFETY_FAULT: (
                "SYSTEM_FAULT_LATCHED_SAFETY_FAULT"
            ),
        }.get(int(message.fault_state), "INVALID_SYSTEM_FAULT_STATE")

    def _sensor_frame(
        self,
        now_ns: int,
    ) -> tuple[SensorFrame | None, str | None, float]:
        scan = self._latest_scan
        if scan is None:
            return None, "SCAN_MISSING", -1.0
        timestamp_ns = _stamp_ns(scan)
        if timestamp_ns > now_ns:
            return None, "SCAN_FUTURE_TIMESTAMP", -1.0
        age_ns = now_ns - timestamp_ns
        age_s = age_ns / NANOSECONDS_PER_SECOND
        if age_ns > self._sensor_timeout_ns:
            return None, "SCAN_TIMEOUT", age_s
        try:
            distances = extract_scan_distances(
                scan,
                front_half_angle_rad=self._front_half_angle_rad,
                max_scan_samples=self._max_scan_samples,
            )
            frame = SensorFrame(
                distance_forward_m=distances.forward_m,
                distance_lateral_m=distances.lateral_m,
                confidence=1.0,
                source=EnvelopeSource.LIDAR,
                age_s=age_s,
            )
        except (TypeError, ValueError) as error:
            return None, str(error), age_s
        return frame, None, age_s

    def _on_timer(self) -> None:
        now_ns = self._now_ns()
        if self._last_now_ns is not None and now_ns < self._last_now_ns:
            self._timing_fault_latched = True
        self._last_now_ns = now_ns

        frame: SensorFrame | None = None
        scan_reason: str | None = None
        scan_age_s = -1.0
        fault_rejection = self._fault_rejection(now_ns)
        if self._timing_fault_latched:
            result = self._evaluator.fail_closed(
                "CLOCK_REGRESSION_LATCHED",
                limiting_zone=LimitingZone.TIMING,
            )
        elif fault_rejection is not None:
            result = self._evaluator.fail_closed(
                fault_rejection,
                limiting_zone=LimitingZone.SYSTEM_FAULT,
            )
        else:
            frame, scan_reason, scan_age_s = self._sensor_frame(now_ns)
            result = (
                self._evaluator.evaluate(frame)
                if scan_reason is None
                else self._evaluator.fail_closed(scan_reason)
            )
        self._publish(now_ns, result, frame, scan_age_s, fault_rejection)

    def _publish(
        self,
        now_ns: int,
        result: EnvelopeResult,
        frame: SensorFrame | None,
        scan_age_s: float,
        fault_rejection: str | None,
    ) -> None:
        self._sequence_id += 1
        correlation_id = f"dynamic-envelope-{self._sequence_id}"

        envelope = PermittedMotionEnvelope()
        envelope.header.stamp.sec = now_ns // NANOSECONDS_PER_SECOND
        envelope.header.stamp.nanosec = now_ns % NANOSECONDS_PER_SECOND
        envelope.header.frame_id = self._output_frame_id
        envelope.policy_version = result.policy_version
        envelope.correlation_id = correlation_id
        envelope.sequence_id = self._sequence_id
        envelope.min_linear_x_mps = 0.0
        envelope.max_linear_x_mps = result.max_linear_velocity_mps
        envelope.min_linear_y_mps = 0.0
        envelope.max_linear_y_mps = 0.0
        envelope.min_angular_z_radps = -result.max_angular_velocity_radps
        envelope.max_angular_z_radps = result.max_angular_velocity_radps
        self._envelope_publisher.publish(envelope)

        diagnostics = EnvelopeDiagnostics()
        diagnostics.header = envelope.header
        diagnostics.policy_version = result.policy_version
        diagnostics.correlation_id = correlation_id
        diagnostics.sequence_id = self._sequence_id
        diagnostics.state = {
            EnvelopeState.FAIL_CLOSED: EnvelopeDiagnostics.STATE_FAIL_CLOSED,
            EnvelopeState.PROTECTIVE_STOP: (
                EnvelopeDiagnostics.STATE_PROTECTIVE_STOP
            ),
            EnvelopeState.LIMITED: EnvelopeDiagnostics.STATE_LIMITED,
            EnvelopeState.NOMINAL: EnvelopeDiagnostics.STATE_NOMINAL,
        }[result.state]
        diagnostics.limiting_zone = {
            LimitingZone.NONE: EnvelopeDiagnostics.ZONE_NONE,
            LimitingZone.FORWARD: EnvelopeDiagnostics.ZONE_FORWARD,
            LimitingZone.LATERAL: EnvelopeDiagnostics.ZONE_LATERAL,
            LimitingZone.SENSOR: EnvelopeDiagnostics.ZONE_SENSOR,
            LimitingZone.SYSTEM_FAULT: EnvelopeDiagnostics.ZONE_SYSTEM_FAULT,
            LimitingZone.TIMING: EnvelopeDiagnostics.ZONE_TIMING,
        }[result.limiting_zone]
        diagnostics.reason = result.reason
        diagnostics.scan_valid = frame is not None
        diagnostics.fault_status_valid = fault_rejection is None
        diagnostics.scan_age_sec = scan_age_s
        diagnostics.distance_forward_m = (
            result.distance_forward_m
            if result.distance_forward_m is not None
            else -1.0
        )
        diagnostics.distance_lateral_m = (
            result.distance_lateral_m
            if result.distance_lateral_m is not None
            else -1.0
        )
        diagnostics.linear_stopping_distance_m = (
            result.linear_stopping_distance_m
        )
        diagnostics.angular_stopping_distance_m = (
            result.angular_stopping_distance_m
        )
        diagnostics.max_linear_velocity_mps = (
            result.max_linear_velocity_mps
        )
        diagnostics.max_angular_velocity_radps = (
            result.max_angular_velocity_radps
        )
        self._diagnostics_publisher.publish(diagnostics)
        self._last_envelope = envelope
        self._last_diagnostics = diagnostics


def _shutdown_rclpy() -> None:
    if not rclpy.ok():
        return
    try:
        rclpy.shutdown()
    except Exception:
        return


def main(args: list[str] | None = None) -> None:
    """Run the dynamic envelope adapter until shutdown."""
    rclpy.init(args=args)
    node: DynamicEnvelopeNode | None = None
    try:
        node = DynamicEnvelopeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        _shutdown_rclpy()


if __name__ == "__main__":
    main()


__all__ = [
    "ENVELOPE_DIAGNOSTICS_TOPIC",
    "MOTION_ENVELOPE_TOPIC",
    "SCAN_TOPIC",
    "SYSTEM_FAULT_TOPIC",
    "DynamicEnvelopeNode",
    "ScanDistances",
    "extract_scan_distances",
    "main",
    "safety_qos_profile",
    "sensor_qos_profile",
]
