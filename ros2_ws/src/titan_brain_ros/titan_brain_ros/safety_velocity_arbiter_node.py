"""ROS 2 adapter for the TB-EVAL-007B priority selector.

This node is a single-point output authority when launched in place of the
legacy velocity arbiter.  It selects teleoperation over autonomy, applies the
existing permitted-motion policy, and sends the result through the
dependency-free TB-EVAL-006 governor before publishing ``/cmd_vel``.
"""

from __future__ import annotations

import math

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
from titan_brain_msgs.msg import (
    ArbitrationStatus,
    PermittedMotionEnvelope,
    SystemFaultStatus,
)

from core.command_governor import (
    NANOSECONDS_PER_SECOND,
    CommandGovernor,
    GovernorCommand,
    GovernorConfig,
    GovernorResult,
)
from core.priority_selector import (
    CommandSourcePriority,
    PrioritySelectorCore,
    RawCommandFrame,
    SelectionResult,
    SystemFaultState,
)

TELEOP_COMMAND_TOPIC = "/teleop/cmd_vel"
AUTONOMY_COMMAND_TOPIC = "/autonomy/cmd_vel"
SYSTEM_FAULT_TOPIC = "/safety/system_fault_status"
MOTION_ENVELOPE_TOPIC = "/safety/permitted_motion_envelope"
OUTPUT_COMMAND_TOPIC = "/cmd_vel"
ARBITRATION_STATUS_TOPIC = "/safety/arbitration_status"


def command_qos_profile() -> QoSProfile:
    """Return a bounded reliable profile for command candidates."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def safety_qos_profile() -> QoSProfile:
    """Return a reliable profile for safety state and envelope policy."""
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


def _required_text_parameter(node: Node, name: str, default: str) -> str:
    value = node.declare_parameter(name, default).value
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"ROS parameter {name!r} must be non-blank text")
    return value.strip()


def _seconds_to_ns(value: float, *, name: str) -> int:
    result = round(value * NANOSECONDS_PER_SECOND)
    if result <= 0:
        raise ValueError(f"ROS parameter {name!r} must round to at least 1 ns")
    return result


def _stamp_ns(
    message: TwistStamped | PermittedMotionEnvelope | SystemFaultStatus,
) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)


def _fault_state_from_message(value: int) -> SystemFaultState | None:
    return {
        SystemFaultStatus.FAULT_OK: SystemFaultState.OK,
        SystemFaultStatus.FAULT_E_STOP_ACTIVE: SystemFaultState.E_STOP_ACTIVE,
        SystemFaultStatus.FAULT_HARDWARE_FAULT: SystemFaultState.HARDWARE_FAULT,
        SystemFaultStatus.FAULT_LATCHED_SAFETY_FAULT: (
            SystemFaultState.LATCHED_SAFETY_FAULT
        ),
    }.get(value)


def _source_name(priority: CommandSourcePriority) -> str:
    return {
        CommandSourcePriority.NONE: "none",
        CommandSourcePriority.AUTONOMY: "autonomy",
        CommandSourcePriority.TELEOPERATION: "teleoperation",
    }[priority]


class SafetyVelocityArbiterNode(Node):
    """Select, envelope-limit, govern, and publish the final command."""

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
    ) -> None:
        super().__init__(
            "safety_velocity_arbiter_node",
            parameter_overrides=parameter_overrides,
        )
        self._output_frame_id = _required_text_parameter(
            self,
            "output_frame_id",
            "base_link",
        )
        self._command_timeout_ns = _seconds_to_ns(
            _finite_positive_parameter(self, "command_timeout_sec", 0.10),
            name="command_timeout_sec",
        )
        self._envelope_timeout_ns = _seconds_to_ns(
            _finite_positive_parameter(self, "envelope_timeout_sec", 0.05),
            name="envelope_timeout_sec",
        )
        self._fault_timeout_ns = _seconds_to_ns(
            _finite_positive_parameter(self, "fault_timeout_sec", 0.20),
            name="fault_timeout_sec",
        )
        self._arbitration_latency_budget_ns = _seconds_to_ns(
            _finite_positive_parameter(
                self,
                "arbitration_latency_budget_sec",
                0.03,
            ),
            name="arbitration_latency_budget_sec",
        )
        timer_period_sec = _finite_positive_parameter(
            self,
            "timer_period_sec",
            0.02,
        )
        self._policy_version = _required_text_parameter(
            self,
            "policy_version",
            "TB-EVAL-007B-0.1.0",
        )
        governor_config = GovernorConfig(
            max_linear_velocity_mps=_finite_positive_parameter(
                self, "max_linear_velocity_mps", 1.0
            ),
            max_angular_velocity_radps=_finite_positive_parameter(
                self, "max_angular_velocity_radps", 1.0
            ),
            max_linear_acceleration_mps2=_finite_positive_parameter(
                self, "max_linear_acceleration_mps2", 1.0
            ),
            max_linear_deceleration_mps2=_finite_positive_parameter(
                self, "max_linear_deceleration_mps2", 2.0
            ),
            max_angular_acceleration_radps2=_finite_positive_parameter(
                self, "max_angular_acceleration_radps2", 1.0
            ),
            max_angular_deceleration_radps2=_finite_positive_parameter(
                self, "max_angular_deceleration_radps2", 2.0
            ),
            max_linear_jerk_mps3=_finite_positive_parameter(
                self, "max_linear_jerk_mps3", 5.0
            ),
            max_angular_jerk_radps3=_finite_positive_parameter(
                self, "max_angular_jerk_radps3", 5.0
            ),
        )

        now_ns = self._now_ns()
        self._selector = PrioritySelectorCore(self._command_timeout_ns)
        self._governor = CommandGovernor(
            governor_config,
            initial_timestamp_ns=max(0, now_ns - 1),
        )
        self._teleop_frame: object | None = None
        self._autonomy_frame: object | None = None
        self._fault_state: SystemFaultState | None = None
        self._fault_timestamp_ns: int | None = None
        self._envelope: PermittedMotionEnvelope | None = None
        self._last_selection: SelectionResult | None = None
        self._last_result: GovernorResult | None = None
        self._last_status: ArbitrationStatus | None = None
        self._command_sequence_id = 0

        self._command_publisher = self.create_publisher(
            TwistStamped,
            OUTPUT_COMMAND_TOPIC,
            command_qos_profile(),
        )
        self._status_publisher = self.create_publisher(
            ArbitrationStatus,
            ARBITRATION_STATUS_TOPIC,
            safety_qos_profile(),
        )
        self._teleop_subscription = self.create_subscription(
            TwistStamped,
            TELEOP_COMMAND_TOPIC,
            self._on_teleop_command,
            command_qos_profile(),
        )
        self._autonomy_subscription = self.create_subscription(
            TwistStamped,
            AUTONOMY_COMMAND_TOPIC,
            self._on_autonomy_command,
            command_qos_profile(),
        )
        self._fault_subscription = self.create_subscription(
            SystemFaultStatus,
            SYSTEM_FAULT_TOPIC,
            self._on_fault_status,
            safety_qos_profile(),
        )
        self._envelope_subscription = self.create_subscription(
            PermittedMotionEnvelope,
            MOTION_ENVELOPE_TOPIC,
            self._on_motion_envelope,
            safety_qos_profile(),
        )
        self._timer = self.create_timer(timer_period_sec, self._on_timer)

        self._on_timer()

    @property
    def selector(self) -> PrioritySelectorCore:
        """Expose the dependency-free selector for diagnostics and tests."""
        return self._selector

    @property
    def governor(self) -> CommandGovernor:
        """Expose the downstream kinematic governor for diagnostics/tests."""
        return self._governor

    @property
    def last_result(self) -> GovernorResult | None:
        """Return the latest governed command result."""
        return self._last_result

    @property
    def last_status(self) -> ArbitrationStatus | None:
        """Return the latest published arbitration status."""
        return self._last_status

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _frame_from_message(
        self,
        message: TwistStamped,
        priority: CommandSourcePriority,
        source_id: str,
    ) -> object:
        if str(message.header.frame_id).strip() != self._output_frame_id:
            return object()
        unsupported_axes = (
            message.twist.linear.y,
            message.twist.linear.z,
            message.twist.angular.x,
            message.twist.angular.y,
        )
        if any(
            not math.isfinite(float(value)) or float(value) != 0.0
            for value in unsupported_axes
        ):
            return object()
        return RawCommandFrame(
            linear_x=float(message.twist.linear.x),
            angular_z=float(message.twist.angular.z),
            priority=priority,
            timestamp_ns=_stamp_ns(message),
            source_id=source_id,
        )

    def _on_teleop_command(self, message: TwistStamped) -> None:
        self._teleop_frame = self._frame_from_message(
            message,
            CommandSourcePriority.TELEOPERATION,
            "teleop",
        )

    def _on_autonomy_command(self, message: TwistStamped) -> None:
        self._autonomy_frame = self._frame_from_message(
            message,
            CommandSourcePriority.AUTONOMY,
            "autonomy",
        )

    def _on_fault_status(self, message: SystemFaultStatus) -> None:
        self._fault_state = _fault_state_from_message(int(message.fault_state))
        self._fault_timestamp_ns = _stamp_ns(message)

    def _on_motion_envelope(self, message: PermittedMotionEnvelope) -> None:
        self._envelope = message

    def _current_fault_state(
        self,
        now_ns: int,
    ) -> tuple[object, SystemFaultState, str | None]:
        """Return selector input, auditable state, and freshness rejection."""
        if self._fault_timestamp_ns is None:
            return (
                SystemFaultState.LATCHED_SAFETY_FAULT,
                SystemFaultState.LATCHED_SAFETY_FAULT,
                "SYSTEM_FAULT_STATUS_MISSING",
            )
        if self._fault_state is None:
            return (
                object(),
                SystemFaultState.LATCHED_SAFETY_FAULT,
                "INVALID_SYSTEM_FAULT_STATE",
            )
        if self._fault_timestamp_ns > now_ns:
            return (
                SystemFaultState.LATCHED_SAFETY_FAULT,
                SystemFaultState.LATCHED_SAFETY_FAULT,
                "SYSTEM_FAULT_STATUS_FUTURE_TIMESTAMP",
            )
        if now_ns - self._fault_timestamp_ns > self._fault_timeout_ns:
            return (
                SystemFaultState.LATCHED_SAFETY_FAULT,
                SystemFaultState.LATCHED_SAFETY_FAULT,
                "SYSTEM_FAULT_STATUS_TIMEOUT",
            )
        return self._fault_state, self._fault_state, None

    def _clamp_with_envelope(
        self,
        frame: RawCommandFrame,
        now_ns: int,
    ) -> tuple[float, float, str | None, bool]:
        envelope = self._envelope
        if envelope is None:
            return 0.0, 0.0, "MOTION_ENVELOPE_MISSING", False
        timestamp_ns = _stamp_ns(envelope)
        if str(envelope.header.frame_id).strip() != self._output_frame_id:
            return 0.0, 0.0, "MOTION_ENVELOPE_FRAME_MISMATCH", False
        if not str(envelope.policy_version).strip() or not str(
            envelope.correlation_id
        ).strip():
            return 0.0, 0.0, "MOTION_ENVELOPE_INVALID", False
        if timestamp_ns > now_ns:
            return 0.0, 0.0, "MOTION_ENVELOPE_FUTURE_TIMESTAMP", False
        if now_ns - timestamp_ns > self._envelope_timeout_ns:
            return 0.0, 0.0, "MOTION_ENVELOPE_TIMEOUT", False
        limits = (
            float(envelope.min_linear_x_mps),
            float(envelope.max_linear_x_mps),
            float(envelope.min_linear_y_mps),
            float(envelope.max_linear_y_mps),
            float(envelope.min_angular_z_radps),
            float(envelope.max_angular_z_radps),
        )
        if any(not math.isfinite(limit) for limit in limits):
            return 0.0, 0.0, "MOTION_ENVELOPE_INVALID", False
        (
            min_linear,
            max_linear,
            min_lateral,
            max_lateral,
            min_angular,
            max_angular,
        ) = limits
        if (
            min_linear > max_linear
            or min_lateral != 0.0
            or max_lateral != 0.0
            or min_angular > max_angular
            or min_angular != 0.0
            or max_angular != 0.0
        ):
            return 0.0, 0.0, "MOTION_ENVELOPE_INVALID", False
        clamped_linear = max(min_linear, min(max_linear, frame.linear_x))
        clamped_angular = max(min_angular, min(max_angular, frame.angular_z))
        changed = (
            clamped_linear != frame.linear_x
            or clamped_angular != frame.angular_z
        )
        return (
            clamped_linear,
            clamped_angular,
            "MOTION_ENVELOPE_CLAMPED" if changed else None,
            changed,
        )

    def _governor_timestamp(self, now_ns: int) -> int:
        return max(now_ns, self._governor.last_timestamp_ns + 1)

    def _publish_result(
        self,
        result: GovernorResult,
        *,
        selection: SelectionResult,
        fault_state: SystemFaultState,
        rejection_reason: str | None,
        mode: int,
        reason: str,
    ) -> None:
        published_at_ns = self._now_ns()
        output = TwistStamped()
        output.header.frame_id = self._output_frame_id
        output.header.stamp.sec = published_at_ns // NANOSECONDS_PER_SECOND
        output.header.stamp.nanosec = published_at_ns % NANOSECONDS_PER_SECOND
        output.twist.linear.x = result.linear_velocity_mps
        output.twist.linear.y = 0.0
        output.twist.angular.z = result.angular_velocity_radps
        self._command_publisher.publish(output)

        status = ArbitrationStatus()
        status.header = output.header
        status.mode = mode
        status.reason = reason
        status.policy_version = self._policy_version
        self._command_sequence_id += 1
        envelope = self._envelope
        status.correlation_id = (
            str(envelope.correlation_id).strip()
            if envelope is not None and str(envelope.correlation_id).strip()
            else (
                selection.selected_frame.source_id
                if selection.selected_frame is not None
                else ""
            )
        )
        status.command_sequence_id = self._command_sequence_id
        status.motion_envelope_correlation_id = (
            str(envelope.correlation_id).strip() if envelope is not None else ""
        )
        status.motion_envelope_sequence_id = (
            int(envelope.sequence_id) if envelope is not None else 0
        )
        status.motion_envelope_timestamp_ns = (
            _stamp_ns(envelope) if envelope is not None else 0
        )
        status.max_abs_linear_x = self._governor.config.max_linear_velocity_mps
        status.max_abs_linear_y = 0.0
        status.max_abs_angular_z = (
            self._governor.config.max_angular_velocity_radps
        )
        status.warning_max_abs_linear_x = (
            float(envelope.max_linear_x_mps) if envelope is not None else 0.0
        )
        status.warning_max_abs_linear_y = (
            float(envelope.max_linear_y_mps) if envelope is not None else 0.0
        )
        status.warning_max_abs_angular_z = (
            float(envelope.max_angular_z_radps) if envelope is not None else 0.0
        )
        status.command_published_timestamp_ns = published_at_ns
        selected_frame = selection.selected_frame
        timing_valid = (
            selected_frame is not None
            and selected_frame.timestamp_ns <= published_at_ns
        )
        latency_ns = (
            published_at_ns - selected_frame.timestamp_ns
            if timing_valid and selected_frame is not None
            else 0
        )
        within_budget = (
            timing_valid and latency_ns <= self._arbitration_latency_budget_ns
        )
        status.intent_received_timestamp_ns = (
            selected_frame.timestamp_ns if selected_frame is not None else 0
        )
        status.arbitration_latency_status = (
            "within_budget"
            if within_budget
            else "over_budget"
            if timing_valid
            else "invalid_timing"
        )
        status.arbitration_timing_valid = timing_valid
        status.arbitration_within_budget = within_budget
        status.arbitration_latency_ns = latency_ns
        status.arbitration_latency_budget_ns = self._arbitration_latency_budget_ns
        status.is_safe = mode != ArbitrationStatus.MODE_FORCED_ZERO
        status.commanded_twist = output.twist
        status.active_source = _source_name(selection.active_priority)
        status.system_fault_state = int(fault_state)
        status.rejection_reason = rejection_reason or ""
        self._status_publisher.publish(status)
        self._last_result = result
        self._last_status = status

    def _publish_emergency(
        self,
        *,
        now_ns: int,
        selection: SelectionResult,
        fault_state: SystemFaultState,
        reason: str,
    ) -> None:
        result = self._governor.step(
            GovernorCommand(emergency_stop=True),
            timestamp_ns=self._governor_timestamp(now_ns),
        )
        self._publish_result(
            result,
            selection=selection,
            fault_state=fault_state,
            rejection_reason=reason,
            mode=ArbitrationStatus.MODE_FORCED_ZERO,
            reason=reason,
        )

    def _on_timer(self) -> None:
        now_ns = self._now_ns()
        fault_input, fault_state, fault_rejection = self._current_fault_state(now_ns)
        selection = self._selector.select_source(
            fault_input,  # type: ignore[arg-type]
            self._teleop_frame,  # type: ignore[arg-type]
            self._autonomy_frame,  # type: ignore[arg-type]
            now_ns,
        )
        self._last_selection = selection
        if selection.selected_frame is None:
            self._publish_emergency(
                now_ns=now_ns,
                selection=selection,
                fault_state=fault_state,
                reason=(
                    fault_rejection
                    or selection.rejection_reason
                    or "NO_VALID_COMMAND_SOURCE"
                ),
            )
            return
        linear, angular, envelope_reason, envelope_changed = (
            self._clamp_with_envelope(selection.selected_frame, now_ns)
        )
        if envelope_reason is not None and not envelope_changed:
            self._publish_emergency(
                now_ns=now_ns,
                selection=selection,
                fault_state=fault_state,
                reason=envelope_reason,
            )
            return
        result = self._governor.step(
            GovernorCommand(
                linear_velocity_mps=linear,
                angular_velocity_radps=angular,
                correlation_id=selection.selected_frame.source_id,
            ),
            timestamp_ns=self._governor_timestamp(now_ns),
        )
        self._publish_result(
            result,
            selection=selection,
            fault_state=fault_state,
            rejection_reason=envelope_reason,
            mode=(
                ArbitrationStatus.MODE_CLAMPED
                if envelope_changed
                else ArbitrationStatus.MODE_PASS_THROUGH
            ),
            reason=(envelope_reason or result.reason.value),
        )


SafetyVelocityArbiterRosNode = SafetyVelocityArbiterNode


def _shutdown_rclpy() -> None:
    if not rclpy.ok():
        return
    try:
        rclpy.shutdown()
    except Exception:
        return


def main(args: list[str] | None = None) -> None:
    """Run the single-point safety velocity arbiter."""
    rclpy.init(args=args)
    node: SafetyVelocityArbiterNode | None = None
    try:
        node = SafetyVelocityArbiterNode()
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
    "MOTION_ENVELOPE_TOPIC",
    "OUTPUT_COMMAND_TOPIC",
    "SYSTEM_FAULT_TOPIC",
    "SafetyVelocityArbiterNode",
    "SafetyVelocityArbiterRosNode",
    "TELEOP_COMMAND_TOPIC",
    "command_qos_profile",
    "main",
    "safety_qos_profile",
]
