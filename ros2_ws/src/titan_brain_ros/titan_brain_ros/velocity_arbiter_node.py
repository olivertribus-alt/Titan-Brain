"""ROS 2 Jazzy transport wrapper for authoritative velocity arbitration."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from titan_brain_msgs.msg import ArbitrationStatus as ArbitrationStatusMsg
from titan_brain_msgs.msg import (
    PermittedMotionEnvelope as PermittedMotionEnvelopeMsg,
)
from titan_brain_msgs.msg import SafetyIntent as SafetyIntentMsg

from core.arbitrator import (
    ArbitrationMode,
    ArbitrationResult,
    DynamicSafetyCommandArbiter,
    EnvelopeInput,
    IntentInput,
    SafetyIntentState,
    VelocityArbiterConfig,
    VelocityInput,
)
from core.command_observability import measure_arbitration_latency

_NANOSECONDS_PER_SECOND = 1_000_000_000


def command_qos_profile() -> QoSProfile:
    """Return the explicit low-depth reliable velocity-command QoS contract."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def status_qos_profile() -> QoSProfile:
    """Return the reliable, non-latched safety control/status QoS contract."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _required_text_parameter(node: Node, name: str) -> str:
    value = node.declare_parameter(name, Parameter.Type.STRING).value
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"ROS parameter {name!r} must be a non-blank string")
    return value


def _required_finite_parameter(
    node: Node,
    name: str,
    *,
    allow_zero: bool,
) -> float:
    value = node.declare_parameter(name, Parameter.Type.DOUBLE).value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"ROS parameter {name!r} must be numeric")
    checked = float(value)
    minimum_is_valid = checked >= 0.0 if allow_zero else checked > 0.0
    if not math.isfinite(checked) or not minimum_is_valid:
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(
            f"ROS parameter {name!r} must be finite and {qualifier}"
        )
    return checked


def _seconds_to_ns(value: float, *, name: str) -> int:
    nanoseconds = round(value * _NANOSECONDS_PER_SECOND)
    if nanoseconds <= 0:
        raise ValueError(f"ROS parameter {name!r} must round to at least 1 ns")
    return nanoseconds


def _intent_timestamp_ns(message: SafetyIntentMsg) -> int:
    return (
        int(message.timestamp.sec) * _NANOSECONDS_PER_SECOND
        + int(message.timestamp.nanosec)
    )


def _envelope_timestamp_ns(message: PermittedMotionEnvelopeMsg) -> int:
    return (
        int(message.header.stamp.sec) * _NANOSECONDS_PER_SECOND
        + int(message.header.stamp.nanosec)
    )


def _twist_from_result(result: ArbitrationResult) -> Twist:
    message = Twist()
    message.linear.x = result.command.linear_x
    message.linear.y = result.command.linear_y
    message.angular.z = result.command.angular_z
    return message


def _status_mode(result: ArbitrationResult) -> int:
    return {
        ArbitrationMode.PASS_THROUGH: ArbitrationStatusMsg.MODE_PASS_THROUGH,
        ArbitrationMode.CLAMPED: ArbitrationStatusMsg.MODE_CLAMPED,
        ArbitrationMode.FORCED_ZERO: ArbitrationStatusMsg.MODE_FORCED_ZERO,
    }[result.mode]


def _intent_state(value: int) -> SafetyIntentState | None:
    return {
        SafetyIntentMsg.STATE_NORMAL: SafetyIntentState.NORMAL,
        SafetyIntentMsg.STATE_WARNING: SafetyIntentState.WARNING,
        SafetyIntentMsg.STATE_E_STOP: SafetyIntentState.E_STOP,
        SafetyIntentMsg.STATE_RECOVERY_HOLDING: (
            SafetyIntentState.RECOVERY_HOLDING
        ),
    }.get(value)


class VelocityArbiterNode(Node):
    """Apply the pure stateful arbiter and own the final actuator topic."""

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
    ) -> None:
        super().__init__(
            "velocity_arbiter_node",
            parameter_overrides=parameter_overrides,
        )

        policy_version = _required_text_parameter(self, "policy_version")
        output_frame_id = _required_text_parameter(self, "output_frame_id")
        command_stale_sec = _required_finite_parameter(
            self,
            "command_stale_threshold_sec",
            allow_zero=False,
        )
        safety_stale_sec = _required_finite_parameter(
            self,
            "safety_stale_threshold_sec",
            allow_zero=False,
        )
        timer_period_sec = _required_finite_parameter(
            self,
            "timer_period_sec",
            allow_zero=False,
        )
        arbitration_latency_budget_sec = _required_finite_parameter(
            self,
            "arbitration_latency_budget_sec",
            allow_zero=False,
        )
        config = VelocityArbiterConfig(
            policy_version=policy_version,
            output_frame_id=output_frame_id,
            command_stale_threshold_ns=_seconds_to_ns(
                command_stale_sec,
                name="command_stale_threshold_sec",
            ),
            safety_stale_threshold_ns=_seconds_to_ns(
                safety_stale_sec,
                name="safety_stale_threshold_sec",
            ),
            max_abs_linear_x=_required_finite_parameter(
                self,
                "max_abs_linear_x",
                allow_zero=True,
            ),
            max_abs_linear_y=_required_finite_parameter(
                self,
                "max_abs_linear_y",
                allow_zero=True,
            ),
            max_abs_angular_z=_required_finite_parameter(
                self,
                "max_abs_angular_z",
                allow_zero=True,
            ),
            warning_max_abs_linear_x=_required_finite_parameter(
                self,
                "warning_max_abs_linear_x",
                allow_zero=True,
            ),
            warning_max_abs_linear_y=_required_finite_parameter(
                self,
                "warning_max_abs_linear_y",
                allow_zero=True,
            ),
            warning_max_abs_angular_z=_required_finite_parameter(
                self,
                "warning_max_abs_angular_z",
                allow_zero=True,
            ),
        )

        self._arbiter = DynamicSafetyCommandArbiter(config)
        self._arbitration_latency_budget_ns = _seconds_to_ns(
            arbitration_latency_budget_sec,
            name="arbitration_latency_budget_sec",
        )
        self._desired_velocity: VelocityInput = None
        self._safety_intent: IntentInput = None
        self._motion_envelope: EnvelopeInput = None
        self._ingress_sequence_id = 0
        self._command_sequence_id = 0
        self._source_intent_sequence_id = 0
        self._last_source_intent_sequence_id: int | None = None
        self._last_source_intent_payload: tuple[int, int, str] | None = None
        self._audit_correlation_id = ""
        self._safety_intent_received_ns: int | None = None
        self._last_result: ArbitrationResult | None = None
        self._last_status: ArbitrationStatusMsg | None = None

        self._command_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            command_qos_profile(),
        )
        self._status_publisher = self.create_publisher(
            ArbitrationStatusMsg,
            "/safety/arbitration_status",
            status_qos_profile(),
        )
        self._command_subscription = self.create_subscription(
            Twist,
            "/cmd_vel_raw",
            self._on_desired_velocity,
            command_qos_profile(),
        )
        self._safety_subscription = self.create_subscription(
            SafetyIntentMsg,
            "/safety/intent",
            self._on_safety_intent,
            status_qos_profile(),
        )
        self._motion_envelope_subscription = self.create_subscription(
            PermittedMotionEnvelopeMsg,
            "/safety/permitted_motion_envelope",
            self._on_motion_envelope,
            status_qos_profile(),
        )
        self._arbitration_timer = self.create_timer(
            timer_period_sec,
            self._on_timer,
        )

        # Do not wait for DDS discovery or the first timer tick to stop motion.
        self._on_timer()
        self.get_logger().info(
            "VelocityArbiterNode initialized as /cmd_vel authority "
            f"(frame={output_frame_id!r}, policy={policy_version!r})"
        )

    @property
    def arbiter(self) -> DynamicSafetyCommandArbiter:
        """Expose the active stateful policy for runtime diagnostics/tests."""
        return self._arbiter

    @property
    def last_result(self) -> ArbitrationResult | None:
        """Return the most recent authoritative arbitration result."""
        return self._last_result

    @property
    def last_status(self) -> ArbitrationStatusMsg | None:
        """Return the diagnostic message paired with the latest command."""
        return self._last_status

    def _next_ingress_sequence_id(self) -> int:
        self._ingress_sequence_id += 1
        return self._ingress_sequence_id

    def _on_desired_velocity(self, message: Twist) -> None:
        received_at_ns = self.get_clock().now().nanoseconds
        sequence_id = self._next_ingress_sequence_id()
        self._command_sequence_id = sequence_id
        self._desired_velocity = {
            "linear_x": float(message.linear.x),
            "linear_y": float(message.linear.y),
            "angular_z": float(message.angular.z),
            "timestamp_ns": received_at_ns,
            "frame_id": self._arbiter.config.output_frame_id,
            "sequence_id": sequence_id,
        }

    def _on_safety_intent(self, message: SafetyIntentMsg) -> None:
        received_at_ns = self.get_clock().now().nanoseconds
        source_sequence_id = int(message.sequence_id)
        timestamp_ns = _intent_timestamp_ns(message)
        correlation_id = str(message.correlation_id)
        payload = (int(message.state), timestamp_ns, correlation_id)
        self._source_intent_sequence_id = source_sequence_id
        self._audit_correlation_id = correlation_id

        if source_sequence_id <= 0:
            self._safety_intent_received_ns = received_at_ns
            self._safety_intent = {"invalid_source_sequence_id": source_sequence_id}
            return

        last_sequence_id = self._last_source_intent_sequence_id
        if last_sequence_id is not None:
            if source_sequence_id < last_sequence_id:
                self._safety_intent_received_ns = received_at_ns
                self._safety_intent = {
                    "source_sequence_regression": source_sequence_id
                }
                return
            if source_sequence_id == last_sequence_id:
                if payload != self._last_source_intent_payload:
                    self._safety_intent_received_ns = received_at_ns
                    self._safety_intent = {
                        "source_sequence_payload_mutation": source_sequence_id
                    }
                # An identical replay is deliberately not freshened.
                return

        self._last_source_intent_sequence_id = source_sequence_id
        self._last_source_intent_payload = payload
        self._safety_intent_received_ns = received_at_ns
        state = _intent_state(int(message.state))
        if state is None or not correlation_id.strip():
            self._safety_intent = {"invalid_safety_intent": payload}
            return

        self._safety_intent = {
            "state": state,
            "timestamp_ns": timestamp_ns,
            "correlation_id": correlation_id,
            # Commands and intents deliberately share this local ordering
            # domain; publisher sequence IDs are validated separately above.
            "sequence_id": self._next_ingress_sequence_id(),
        }

    def _on_motion_envelope(
        self,
        message: PermittedMotionEnvelopeMsg,
    ) -> None:
        self._motion_envelope = {
            "policy_version": str(message.policy_version),
            "timestamp_ns": _envelope_timestamp_ns(message),
            "frame_id": str(message.header.frame_id),
            "correlation_id": str(message.correlation_id),
            "sequence_id": int(message.sequence_id),
            "min_linear_x_mps": float(message.min_linear_x_mps),
            "max_linear_x_mps": float(message.max_linear_x_mps),
            "min_linear_y_mps": float(message.min_linear_y_mps),
            "max_linear_y_mps": float(message.max_linear_y_mps),
            "min_angular_z_radps": float(message.min_angular_z_radps),
            "max_angular_z_radps": float(message.max_angular_z_radps),
        }

    def _warning_limit(self, warning: float | None, nominal: float) -> float:
        return nominal if warning is None else warning

    def _publish_result(self, result: ArbitrationResult) -> None:
        command = _twist_from_result(result)
        self._command_publisher.publish(command)
        published_at_ns = self.get_clock().now().nanoseconds
        config = self._arbiter.config
        timing = measure_arbitration_latency(
            intent_received_ns=self._safety_intent_received_ns,
            command_published_ns=published_at_ns,
            budget_ns=self._arbitration_latency_budget_ns,
        )
        status = ArbitrationStatusMsg()
        status.header.stamp.sec = published_at_ns // _NANOSECONDS_PER_SECOND
        status.header.stamp.nanosec = published_at_ns % _NANOSECONDS_PER_SECOND
        status.header.frame_id = config.output_frame_id
        status.mode = _status_mode(result)
        status.reason = result.reason.value
        status.policy_version = result.policy_version
        status.correlation_id = (
            result.correlation_id or self._audit_correlation_id
        )
        status.is_safe = result.mode is not ArbitrationMode.FORCED_ZERO
        status.command_sequence_id = self._command_sequence_id
        status.safety_intent_sequence_id = self._source_intent_sequence_id
        status.arbitration_latency_status = timing.status.value
        status.arbitration_timing_valid = timing.timing_valid
        status.arbitration_within_budget = timing.within_budget
        status.intent_received_timestamp_ns = timing.intent_received_ns or 0
        status.command_published_timestamp_ns = timing.command_published_ns or 0
        status.arbitration_latency_ns = timing.latency_ns or 0
        status.arbitration_latency_budget_ns = timing.budget_ns
        status.max_abs_linear_x = config.max_abs_linear_x
        status.max_abs_linear_y = config.max_abs_linear_y
        status.max_abs_angular_z = config.max_abs_angular_z
        status.warning_max_abs_linear_x = self._warning_limit(
            config.warning_max_abs_linear_x,
            config.max_abs_linear_x,
        )
        status.warning_max_abs_linear_y = self._warning_limit(
            config.warning_max_abs_linear_y,
            config.max_abs_linear_y,
        )
        status.warning_max_abs_angular_z = self._warning_limit(
            config.warning_max_abs_angular_z,
            config.max_abs_angular_z,
        )
        status.commanded_twist = command

        self._status_publisher.publish(status)
        self._last_result = result
        self._last_status = status

    def _on_timer(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        result = self._arbiter.evaluate_with_envelope(
            self._desired_velocity,
            self._safety_intent,
            self._motion_envelope,
            now_ns=now_ns,
        )
        self._publish_result(result)


def main(args: list[str] | None = None) -> None:
    """Run the authoritative ROS 2 velocity arbiter until shutdown."""
    rclpy.init(args=args)
    node: VelocityArbiterNode | None = None
    try:
        node = VelocityArbiterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
