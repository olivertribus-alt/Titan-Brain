"""ROS 2 adapter for the TB-EVAL-009A safety lifecycle manager."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from titan_brain_msgs.msg import (
    EnvelopeDiagnostics,
    SafetyLifecycleStatus,
    SystemFaultStatus,
)

from core.command_governor import NANOSECONDS_PER_SECOND
from core.safety_recovery_manager import (
    SafetyLifecycleEvidence,
    SafetyLifecycleState,
    SafetyLifecycleTransition,
    SafetyRecoveryConfig,
    SafetyRecoveryManager,
)

ENVELOPE_DIAGNOSTICS_TOPIC = "/safety/envelope_diagnostics"
SYSTEM_FAULT_TOPIC = "/safety/system_fault_status"
SAFETY_LIFECYCLE_TOPIC = "/safety/lifecycle_status"


def safety_qos_profile() -> QoSProfile:
    """Return the reliable bounded safety control-plane contract."""
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
        raise ValueError(f"ROS parameter {name!r} must be finite and {qualifier}")
    return checked


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


def _stamp_ns(message: EnvelopeDiagnostics | SystemFaultStatus) -> int:
    return int(message.header.stamp.sec) * NANOSECONDS_PER_SECOND + int(
        message.header.stamp.nanosec
    )


class SafetyRecoveryManagerNode(Node):
    """Publish fail-closed lifecycle state from envelope and fault evidence."""

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
    ) -> None:
        super().__init__(
            "safety_recovery_manager_node",
            parameter_overrides=parameter_overrides,
        )
        self._output_frame_id = _text_parameter(
            self,
            "output_frame_id",
            "base_link",
        )
        timer_period_sec = _finite_float_parameter(
            self,
            "timer_period_sec",
            0.02,
            allow_zero=False,
        )
        diagnostics_timeout_sec = _finite_float_parameter(
            self,
            "diagnostics_timeout_sec",
            0.20,
            allow_zero=False,
        )
        fault_timeout_sec = _finite_float_parameter(
            self,
            "fault_timeout_sec",
            0.20,
            allow_zero=False,
        )
        self._diagnostics_timeout_ns = _seconds_to_ns(
            diagnostics_timeout_sec,
            name="diagnostics_timeout_sec",
        )
        self._fault_timeout_ns = _seconds_to_ns(
            fault_timeout_sec,
            name="fault_timeout_sec",
        )
        self._manager = SafetyRecoveryManager(
            SafetyRecoveryConfig(
                policy_version=_text_parameter(
                    self,
                    "policy_version",
                    "TB-EVAL-009A-0.1.0",
                ),
                stop_margin_m=_finite_float_parameter(
                    self,
                    "stop_margin_m",
                    0.30,
                    allow_zero=True,
                ),
                warning_distance_m=_finite_float_parameter(
                    self,
                    "warning_distance_m",
                    1.00,
                    allow_zero=False,
                ),
                distance_hysteresis_m=_finite_float_parameter(
                    self,
                    "distance_hysteresis_m",
                    0.10,
                    allow_zero=True,
                ),
                recovery_dwell_time_ns=_seconds_to_ns(
                    _finite_float_parameter(
                        self,
                        "recovery_dwell_time_sec",
                        1.00,
                        allow_zero=False,
                    ),
                    name="recovery_dwell_time_sec",
                ),
                degraded_linear_speed_limit_mps=_finite_float_parameter(
                    self,
                    "degraded_linear_speed_limit_mps",
                    0.50,
                    allow_zero=False,
                ),
                degraded_angular_speed_limit_radps=_finite_float_parameter(
                    self,
                    "degraded_angular_speed_limit_radps",
                    0.50,
                    allow_zero=False,
                ),
                recovery_linear_speed_limit_mps=_finite_float_parameter(
                    self,
                    "recovery_linear_speed_limit_mps",
                    0.20,
                    allow_zero=False,
                ),
                recovery_angular_speed_limit_radps=_finite_float_parameter(
                    self,
                    "recovery_angular_speed_limit_radps",
                    0.50,
                    allow_zero=False,
                ),
            )
        )

        self._latest_diagnostics: EnvelopeDiagnostics | None = None
        self._latest_fault: SystemFaultStatus | None = None
        self._last_diagnostics_timestamp_ns: int | None = None
        self._last_fault_timestamp_ns: int | None = None
        self._last_now_ns: int | None = None
        self._timing_fault_latched = False
        self._sequence_id = 0
        self._last_status: SafetyLifecycleStatus | None = None

        qos = safety_qos_profile()
        self._status_publisher = self.create_publisher(
            SafetyLifecycleStatus,
            SAFETY_LIFECYCLE_TOPIC,
            qos,
        )
        self._diagnostics_subscription = self.create_subscription(
            EnvelopeDiagnostics,
            ENVELOPE_DIAGNOSTICS_TOPIC,
            self._on_diagnostics,
            qos,
        )
        self._fault_subscription = self.create_subscription(
            SystemFaultStatus,
            SYSTEM_FAULT_TOPIC,
            self._on_fault_status,
            qos,
        )
        self._timer = self.create_timer(timer_period_sec, self._on_timer)
        self._on_timer()

    @property
    def manager(self) -> SafetyRecoveryManager:
        """Expose the dependency-free manager for diagnostics and tests."""
        return self._manager

    @property
    def last_status(self) -> SafetyLifecycleStatus | None:
        """Return the latest lifecycle status emitted by the node."""
        return self._last_status

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _on_diagnostics(self, message: EnvelopeDiagnostics) -> None:
        timestamp_ns = _stamp_ns(message)
        if (
            self._last_diagnostics_timestamp_ns is not None
            and timestamp_ns <= self._last_diagnostics_timestamp_ns
        ):
            self._timing_fault_latched = True
        if (
            self._last_diagnostics_timestamp_ns is None
            or timestamp_ns > self._last_diagnostics_timestamp_ns
        ):
            self._last_diagnostics_timestamp_ns = timestamp_ns
        self._latest_diagnostics = message

    def _on_fault_status(self, message: SystemFaultStatus) -> None:
        timestamp_ns = _stamp_ns(message)
        if (
            self._last_fault_timestamp_ns is not None
            and timestamp_ns < self._last_fault_timestamp_ns
        ):
            self._timing_fault_latched = True
        if (
            self._last_fault_timestamp_ns is None
            or timestamp_ns > self._last_fault_timestamp_ns
        ):
            self._last_fault_timestamp_ns = timestamp_ns
        self._latest_fault = message

    def _fault_health(self, now_ns: int) -> tuple[bool, bool]:
        message = self._latest_fault
        if message is None:
            return False, False
        timestamp_ns = _stamp_ns(message)
        if timestamp_ns > now_ns or now_ns - timestamp_ns > self._fault_timeout_ns:
            return False, False
        fault_state = int(message.fault_state)
        valid_states = {
            SystemFaultStatus.FAULT_OK,
            SystemFaultStatus.FAULT_E_STOP_ACTIVE,
            SystemFaultStatus.FAULT_HARDWARE_FAULT,
            SystemFaultStatus.FAULT_LATCHED_SAFETY_FAULT,
        }
        if fault_state not in valid_states:
            return False, False
        return True, fault_state != SystemFaultStatus.FAULT_OK

    def _evidence(self, now_ns: int) -> SafetyLifecycleEvidence:
        fault_status_valid, is_faulted = self._fault_health(now_ns)
        diagnostics = self._latest_diagnostics
        if diagnostics is None:
            return SafetyLifecycleEvidence(
                fault_status_valid=fault_status_valid,
                is_faulted=is_faulted,
                sensor_valid=False,
                sensor_fresh=False,
                time_valid=not self._timing_fault_latched,
                max_linear_velocity_mps=0.0,
                max_angular_velocity_radps=0.0,
            )

        timestamp_ns = _stamp_ns(diagnostics)
        timestamp_valid = timestamp_ns <= now_ns
        message_fresh = (
            timestamp_valid and now_ns - timestamp_ns <= self._diagnostics_timeout_ns
        )
        scan_age_sec = float(diagnostics.scan_age_sec)
        scan_age_valid = (
            math.isfinite(scan_age_sec)
            and scan_age_sec >= 0.0
            and scan_age_sec <= self._diagnostics_timeout_ns / NANOSECONDS_PER_SECOND
        )
        distances = (
            float(diagnostics.distance_forward_m),
            float(diagnostics.distance_lateral_m),
        )
        velocities = (
            float(diagnostics.max_linear_velocity_mps),
            float(diagnostics.max_angular_velocity_radps),
        )
        numeric_valid = all(
            math.isfinite(value) and value >= 0.0 for value in distances + velocities
        )
        fail_closed = int(diagnostics.state) == EnvelopeDiagnostics.STATE_FAIL_CLOSED
        timing_reason = (
            int(diagnostics.limiting_zone) == EnvelopeDiagnostics.ZONE_TIMING
            or "CLOCK" in str(diagnostics.reason).upper()
            or "TIME_REGRESSION" in str(diagnostics.reason).upper()
        )
        time_valid = (
            timestamp_valid and not timing_reason and not self._timing_fault_latched
        )
        sensor_valid = (
            bool(diagnostics.scan_valid) and numeric_valid and not fail_closed
        )
        sensor_fresh = sensor_valid and message_fresh and scan_age_valid
        distance_min_m = min(distances) if sensor_valid else None
        return SafetyLifecycleEvidence(
            fault_status_valid=(
                fault_status_valid and bool(diagnostics.fault_status_valid)
            ),
            is_faulted=is_faulted,
            sensor_valid=sensor_valid,
            sensor_fresh=sensor_fresh,
            time_valid=time_valid,
            noncritical_warning=(
                int(diagnostics.state) == EnvelopeDiagnostics.STATE_LIMITED
            ),
            distance_min_m=distance_min_m,
            max_linear_velocity_mps=(velocities[0] if numeric_valid else 0.0),
            max_angular_velocity_radps=(velocities[1] if numeric_valid else 0.0),
        )

    def _on_timer(self) -> None:
        now_ns = self._now_ns()
        if self._last_now_ns is not None and now_ns < self._last_now_ns:
            self._timing_fault_latched = True
        self._last_now_ns = now_ns
        evidence = self._evidence(now_ns)
        transition = self._manager.update(evidence, now_ns=now_ns)
        self._publish(now_ns, evidence, transition)

    def _publish(
        self,
        now_ns: int,
        evidence: SafetyLifecycleEvidence,
        transition: SafetyLifecycleTransition,
    ) -> None:
        self._sequence_id += 1
        diagnostics = self._latest_diagnostics
        correlation_id = (
            str(diagnostics.correlation_id)
            if diagnostics is not None and str(diagnostics.correlation_id).strip()
            else f"safety-lifecycle-{self._sequence_id}"
        )
        config = self._manager.config

        status = SafetyLifecycleStatus()
        status.header.stamp.sec = now_ns // NANOSECONDS_PER_SECOND
        status.header.stamp.nanosec = now_ns % NANOSECONDS_PER_SECOND
        status.header.frame_id = self._output_frame_id
        status.schema_version = transition.schema_version
        status.policy_version = transition.policy_version
        status.correlation_id = correlation_id
        status.sequence_id = self._sequence_id
        status.state = {
            SafetyLifecycleState.NORMAL: SafetyLifecycleStatus.STATE_NORMAL,
            SafetyLifecycleState.DEGRADED: (SafetyLifecycleStatus.STATE_DEGRADED),
            SafetyLifecycleState.RECOVERY: SafetyLifecycleStatus.STATE_RECOVERY,
            SafetyLifecycleState.EMERGENCY_STOP: (
                SafetyLifecycleStatus.STATE_EMERGENCY_STOP
            ),
        }[transition.state]
        status.reason = transition.reason.value
        status.fault_status_valid = evidence.fault_status_valid
        status.is_faulted = evidence.is_faulted
        status.sensor_valid = evidence.sensor_valid
        status.sensor_fresh = evidence.sensor_fresh
        status.time_valid = evidence.time_valid
        status.noncritical_warning = evidence.noncritical_warning
        status.recovery_active = transition.state is SafetyLifecycleState.RECOVERY
        status.distance_min_m = (
            transition.distance_min_m if transition.distance_min_m is not None else -1.0
        )
        status.stop_margin_m = config.stop_margin_m
        status.warning_distance_m = config.warning_distance_m
        status.normal_release_distance_m = config.normal_release_distance_m
        status.recovery_elapsed_ns = transition.recovery_elapsed_ns
        status.recovery_dwell_time_ns = config.recovery_dwell_time_ns
        status.max_linear_velocity_mps = transition.max_linear_velocity_mps
        status.max_angular_velocity_radps = transition.max_angular_velocity_radps
        self._status_publisher.publish(status)
        self._last_status = status


def _shutdown_rclpy() -> None:
    if not rclpy.ok():
        return
    try:
        rclpy.shutdown()
    except Exception:
        return


def main(args: list[str] | None = None) -> None:
    """Run the safety recovery manager until shutdown."""
    rclpy.init(args=args)
    node: SafetyRecoveryManagerNode | None = None
    try:
        node = SafetyRecoveryManagerNode()
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
    "SAFETY_LIFECYCLE_TOPIC",
    "SYSTEM_FAULT_TOPIC",
    "SafetyRecoveryManagerNode",
    "main",
    "safety_qos_profile",
]
