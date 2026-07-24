"""ROS 2 adapter for the TB-EVAL-009B diagnostic blackbox."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Trigger
from titan_brain_msgs.msg import (
    ArbitrationStatus,
    EnvelopeDiagnostics,
    SafetyLifecycleStatus,
    SystemFaultStatus,
)

from core.command_governor import NANOSECONDS_PER_SECOND
from core.telemetry_blackbox import (
    ArbitrationTelemetry,
    BlackboxState,
    CommandTelemetry,
    EnvelopeTelemetry,
    LifecycleTelemetry,
    SnapshotTrigger,
    TelemetryBlackbox,
    TelemetryBlackboxConfig,
    TelemetryBlackboxFrame,
)

TELEOP_COMMAND_TOPIC = "/teleop/cmd_vel"
AUTONOMY_COMMAND_TOPIC = "/autonomy/cmd_vel"
OUTPUT_COMMAND_TOPIC = "/cmd_vel"
ARBITRATION_STATUS_TOPIC = "/safety/arbitration_status"
ENVELOPE_DIAGNOSTICS_TOPIC = "/safety/envelope_diagnostics"
SAFETY_LIFECYCLE_TOPIC = "/safety/lifecycle_status"
SYSTEM_FAULT_TOPIC = "/safety/system_fault_status"
MANUAL_TRIGGER_SERVICE = "/safety/telemetry_blackbox/trigger"


def command_qos_profile() -> QoSProfile:
    """Return a low-depth reliable command telemetry profile."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def safety_qos_profile() -> QoSProfile:
    """Return a bounded reliable safety telemetry profile."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _finite_positive_parameter(node: Node, name: str, default: float) -> float:
    value = node.declare_parameter(name, default).value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"ROS parameter {name!r} must be numeric")
    checked = float(value)
    if not math.isfinite(checked) or checked <= 0.0:
        raise ValueError(f"ROS parameter {name!r} must be finite and positive")
    return checked


def _integer_parameter(
    node: Node,
    name: str,
    default: int,
    *,
    allow_zero: bool,
) -> int:
    value = node.declare_parameter(name, default).value
    if isinstance(value, bool) or not isinstance(value, int):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"ROS parameter {name!r} must be a {qualifier} integer")
    lower_bound_ok = value >= 0 if allow_zero else value > 0
    if not lower_bound_ok:
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"ROS parameter {name!r} must be a {qualifier} integer")
    return value


def _text_parameter(node: Node, name: str, default: str) -> str:
    value = node.declare_parameter(name, default).value
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"ROS parameter {name!r} must be non-blank text")
    return value.strip()


def _output_directory_parameter(node: Node) -> Path:
    value = _text_parameter(
        node,
        "snapshot_output_directory",
        "/tmp/titan_brain_blackbox",
    )
    path = Path(value)
    if not path.is_absolute():
        raise ValueError("snapshot_output_directory must be absolute")
    return path


def _stamp_ns(
    message: (
        ArbitrationStatus
        | EnvelopeDiagnostics
        | SafetyLifecycleStatus
        | SystemFaultStatus
        | TwistStamped
    ),
) -> int:
    return int(message.header.stamp.sec) * NANOSECONDS_PER_SECOND + int(
        message.header.stamp.nanosec
    )


def _command_telemetry(message: TwistStamped | None) -> CommandTelemetry | None:
    if message is None:
        return None
    try:
        return CommandTelemetry(
            source_timestamp_ns=_stamp_ns(message),
            linear_x_mps=float(message.twist.linear.x),
            angular_z_radps=float(message.twist.angular.z),
        )
    except (TypeError, ValueError):
        return None


def _arbitration_telemetry(
    message: ArbitrationStatus | None,
) -> ArbitrationTelemetry | None:
    if message is None:
        return None
    try:
        return ArbitrationTelemetry(
            source_timestamp_ns=_stamp_ns(message),
            mode=int(message.mode),
            reason=str(message.reason).strip() or "unknown",
            active_source=str(message.active_source).strip() or "unknown",
            system_fault_state=int(message.system_fault_state),
            correlation_id=str(message.correlation_id),
        )
    except (TypeError, ValueError):
        return None


def _optional_distance(value: object) -> float | None:
    try:
        checked = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return checked if math.isfinite(checked) and checked >= 0.0 else None


def _envelope_telemetry(
    message: EnvelopeDiagnostics | None,
) -> EnvelopeTelemetry | None:
    if message is None:
        return None
    try:
        return EnvelopeTelemetry(
            source_timestamp_ns=_stamp_ns(message),
            state=int(message.state),
            reason=str(message.reason).strip() or "unknown",
            scan_valid=bool(message.scan_valid),
            distance_forward_m=_optional_distance(message.distance_forward_m),
            distance_lateral_m=_optional_distance(message.distance_lateral_m),
            max_linear_velocity_mps=float(message.max_linear_velocity_mps),
            max_angular_velocity_radps=float(message.max_angular_velocity_radps),
        )
    except (TypeError, ValueError):
        return None


def _lifecycle_telemetry(
    message: SafetyLifecycleStatus | None,
) -> LifecycleTelemetry | None:
    if message is None:
        return None
    try:
        return LifecycleTelemetry(
            source_timestamp_ns=_stamp_ns(message),
            state=int(message.state),
            reason=str(message.reason).strip() or "unknown",
            is_faulted=bool(message.is_faulted),
            recovery_active=bool(message.recovery_active),
            max_linear_velocity_mps=float(message.max_linear_velocity_mps),
            max_angular_velocity_radps=float(message.max_angular_velocity_radps),
        )
    except (TypeError, ValueError):
        return None


class TelemetryBlackboxNode(Node):
    """Sample the safety control plane and freeze bounded incident windows."""

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
    ) -> None:
        super().__init__(
            "telemetry_blackbox_node",
            parameter_overrides=parameter_overrides,
        )
        timer_period_sec = _finite_positive_parameter(
            self,
            "timer_period_sec",
            0.02,
        )
        self._blackbox = TelemetryBlackbox(
            TelemetryBlackboxConfig(
                policy_version=_text_parameter(
                    self,
                    "policy_version",
                    "TB-EVAL-009B-0.1.0",
                ),
                capacity_frames=_integer_parameter(
                    self,
                    "capacity_frames",
                    500,
                    allow_zero=False,
                ),
                post_trigger_frames=_integer_parameter(
                    self,
                    "post_trigger_frames",
                    50,
                    allow_zero=True,
                ),
            )
        )
        self._snapshot_output_directory = _output_directory_parameter(self)
        self._sequence_id = 0
        self._last_recorded_at_ns: int | None = None
        self._last_exported_snapshot_id = 0
        self._last_export_path: Path | None = None
        self._pending_trigger: tuple[SnapshotTrigger, str] | None = None
        self._last_lifecycle_state: int | None = None
        self._last_fault_state: int | None = None

        self._latest_teleop: TwistStamped | None = None
        self._latest_autonomy: TwistStamped | None = None
        self._latest_output: TwistStamped | None = None
        self._latest_arbitration: ArbitrationStatus | None = None
        self._latest_envelope: EnvelopeDiagnostics | None = None
        self._latest_lifecycle: SafetyLifecycleStatus | None = None

        command_qos = command_qos_profile()
        safety_qos = safety_qos_profile()
        self._teleop_subscription = self.create_subscription(
            TwistStamped,
            TELEOP_COMMAND_TOPIC,
            self._on_teleop_command,
            command_qos,
        )
        self._autonomy_subscription = self.create_subscription(
            TwistStamped,
            AUTONOMY_COMMAND_TOPIC,
            self._on_autonomy_command,
            command_qos,
        )
        self._output_subscription = self.create_subscription(
            TwistStamped,
            OUTPUT_COMMAND_TOPIC,
            self._on_output_command,
            command_qos,
        )
        self._arbitration_subscription = self.create_subscription(
            ArbitrationStatus,
            ARBITRATION_STATUS_TOPIC,
            self._on_arbitration_status,
            safety_qos,
        )
        self._envelope_subscription = self.create_subscription(
            EnvelopeDiagnostics,
            ENVELOPE_DIAGNOSTICS_TOPIC,
            self._on_envelope_diagnostics,
            safety_qos,
        )
        self._lifecycle_subscription = self.create_subscription(
            SafetyLifecycleStatus,
            SAFETY_LIFECYCLE_TOPIC,
            self._on_lifecycle_status,
            safety_qos,
        )
        self._fault_subscription = self.create_subscription(
            SystemFaultStatus,
            SYSTEM_FAULT_TOPIC,
            self._on_fault_status,
            safety_qos,
        )
        self._manual_trigger_service = self.create_service(
            Trigger,
            MANUAL_TRIGGER_SERVICE,
            self._on_manual_trigger,
        )
        self._timer = self.create_timer(timer_period_sec, self._on_timer)
        self._on_timer()

    @property
    def blackbox(self) -> TelemetryBlackbox:
        """Expose the dependency-free recorder for diagnostics and tests."""
        return self._blackbox

    @property
    def last_export_path(self) -> Path | None:
        """Return the most recent successfully exported snapshot path."""
        return self._last_export_path

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _on_teleop_command(self, message: TwistStamped) -> None:
        self._latest_teleop = message

    def _on_autonomy_command(self, message: TwistStamped) -> None:
        self._latest_autonomy = message

    def _on_output_command(self, message: TwistStamped) -> None:
        self._latest_output = message

    def _on_arbitration_status(self, message: ArbitrationStatus) -> None:
        self._latest_arbitration = message

    def _on_envelope_diagnostics(self, message: EnvelopeDiagnostics) -> None:
        self._latest_envelope = message

    def _queue_trigger(self, trigger: SnapshotTrigger, reason: str) -> None:
        if self._blackbox.state is BlackboxState.CAPTURING_POST_TRIGGER:
            return
        if self._pending_trigger is None or trigger is SnapshotTrigger.HARD_FAULT:
            self._pending_trigger = (trigger, reason)

    def _on_lifecycle_status(self, message: SafetyLifecycleStatus) -> None:
        state = int(message.state)
        if (
            self._last_lifecycle_state is not None
            and self._last_lifecycle_state != SafetyLifecycleStatus.STATE_EMERGENCY_STOP
            and state == SafetyLifecycleStatus.STATE_EMERGENCY_STOP
        ):
            self._queue_trigger(
                SnapshotTrigger.EMERGENCY_STOP,
                str(message.reason).strip() or "lifecycle emergency stop",
            )
        self._last_lifecycle_state = state
        self._latest_lifecycle = message

    def _on_fault_status(self, message: SystemFaultStatus) -> None:
        state = int(message.fault_state)
        hard_states = {
            SystemFaultStatus.FAULT_HARDWARE_FAULT,
            SystemFaultStatus.FAULT_LATCHED_SAFETY_FAULT,
        }
        if state in hard_states and self._last_fault_state not in hard_states:
            self._queue_trigger(
                SnapshotTrigger.HARD_FAULT,
                f"system fault state {state}",
            )
        self._last_fault_state = state

    def _record_tick(self, now_ns: int) -> None:
        effective_now_ns = (
            now_ns
            if self._last_recorded_at_ns is None
            else max(now_ns, self._last_recorded_at_ns)
        )
        if self._last_recorded_at_ns is not None and now_ns < self._last_recorded_at_ns:
            self._queue_trigger(
                SnapshotTrigger.EMERGENCY_STOP,
                "blackbox clock regression",
            )
        self._sequence_id += 1
        self._blackbox.record(
            TelemetryBlackboxFrame(
                sequence_id=self._sequence_id,
                recorded_at_ns=effective_now_ns,
                teleoperation_command=_command_telemetry(self._latest_teleop),
                autonomy_command=_command_telemetry(self._latest_autonomy),
                authoritative_command=_command_telemetry(self._latest_output),
                arbitration=_arbitration_telemetry(self._latest_arbitration),
                envelope=_envelope_telemetry(self._latest_envelope),
                lifecycle=_lifecycle_telemetry(self._latest_lifecycle),
            )
        )
        self._last_recorded_at_ns = effective_now_ns

    def _on_timer(self) -> None:
        self._record_tick(self._now_ns())
        if self._pending_trigger is not None:
            trigger, reason = self._pending_trigger
            self._pending_trigger = None
            self._blackbox.trigger(trigger, reason)
        self._export_if_ready()

    def _on_manual_trigger(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._pending_trigger is not None:
            response.success = False
            response.message = "automatic incident trigger pending"
            return response
        if self._blackbox.state is BlackboxState.CAPTURING_POST_TRIGGER:
            response.success = False
            response.message = "blackbox post-trigger capture already active"
            return response
        self._record_tick(self._now_ns())
        accepted = self._blackbox.trigger(
            SnapshotTrigger.MANUAL,
            "manual ROS service request",
        )
        self._export_if_ready()
        response.success = accepted
        response.message = (
            "blackbox snapshot capture started"
            if accepted
            else "blackbox has no frame available"
        )
        return response

    def _export_if_ready(self) -> None:
        snapshot = self._blackbox.last_snapshot
        if snapshot is None or snapshot.snapshot_id <= self._last_exported_snapshot_id:
            return
        filename = (
            f"blackbox-{snapshot.snapshot_id:06d}-{snapshot.trigger_timestamp_ns}.json"
        )
        destination = self._snapshot_output_directory / filename
        temporary = destination.with_suffix(".json.tmp")
        try:
            self._snapshot_output_directory.mkdir(
                parents=True,
                exist_ok=True,
            )
            temporary.write_text(
                self._blackbox.snapshot_json(indent=2) + "\n",
                encoding="utf-8",
            )
            temporary.replace(destination)
        except OSError as error:
            self.get_logger().error(
                f"Failed to export telemetry blackbox snapshot: {error}"
            )
            return
        self._last_exported_snapshot_id = snapshot.snapshot_id
        self._last_export_path = destination
        self.get_logger().error(f"Telemetry blackbox snapshot exported: {destination}")


def _shutdown_rclpy() -> None:
    if not rclpy.ok():
        return
    try:
        rclpy.shutdown()
    except Exception:
        return


def main(args: list[str] | None = None) -> None:
    """Run the telemetry blackbox until shutdown."""
    rclpy.init(args=args)
    node: TelemetryBlackboxNode | None = None
    try:
        node = TelemetryBlackboxNode()
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
    "ARBITRATION_STATUS_TOPIC",
    "AUTONOMY_COMMAND_TOPIC",
    "ENVELOPE_DIAGNOSTICS_TOPIC",
    "MANUAL_TRIGGER_SERVICE",
    "OUTPUT_COMMAND_TOPIC",
    "SAFETY_LIFECYCLE_TOPIC",
    "SYSTEM_FAULT_TOPIC",
    "TELEOP_COMMAND_TOPIC",
    "TelemetryBlackboxNode",
    "command_qos_profile",
    "main",
    "safety_qos_profile",
]
