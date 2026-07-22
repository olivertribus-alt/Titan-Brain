"""ROS 2 adapter for TB-ACT-001B stop acknowledgement monitoring."""

from __future__ import annotations

import math
from collections.abc import Mapping

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
from titan_brain_msgs.msg import (
    ActuatorFeedback as ActuatorFeedbackMsg,
)
from titan_brain_msgs.msg import (
    ArbitrationStatus as ArbitrationStatusMsg,
)
from titan_brain_msgs.msg import (
    StopAcknowledgement as StopAcknowledgementMsg,
)

from core.actuator_feedback import ActuatorFeedbackConfig, FeedbackInput
from core.stop_ack_monitor import (
    StopAckMonitor,
    StopMonitorConfig,
    StopMonitorResult,
)

try:
    from rclpy.exceptions import RCLError
except ImportError:  # ROS 2 Jazzy exposes the type from its pybind module.
    from rclpy._rclpy_pybind11 import RCLError

_NANOSECONDS_PER_SECOND = 1_000_000_000
_ZERO_EPSILON = 1e-12


def feedback_qos_profile() -> QoSProfile:
    """Return a bounded best-effort profile for high-rate actuator samples."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def control_qos_profile() -> QoSProfile:
    """Return a reliable profile for authoritative stop requests."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def status_qos_profile() -> QoSProfile:
    """Return a reliable profile for acknowledgement and latch diagnostics."""
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


def _required_positive_float_parameter(node: Node, name: str) -> float:
    value = node.declare_parameter(name, Parameter.Type.DOUBLE).value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"ROS parameter {name!r} must be numeric")
    checked = float(value)
    if not math.isfinite(checked) or checked <= 0.0:
        raise ValueError(f"ROS parameter {name!r} must be finite and positive")
    return checked


def _required_nonnegative_float_parameter(node: Node, name: str) -> float:
    value = node.declare_parameter(name, Parameter.Type.DOUBLE).value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"ROS parameter {name!r} must be numeric")
    checked = float(value)
    if not math.isfinite(checked) or checked < 0.0:
        raise ValueError(
            f"ROS parameter {name!r} must be finite and non-negative"
        )
    return checked


def _seconds_to_ns(value: float, *, name: str) -> int:
    nanoseconds = round(value * _NANOSECONDS_PER_SECOND)
    if nanoseconds <= 0:
        raise ValueError(f"ROS parameter {name!r} must round to at least 1 ns")
    return nanoseconds


def _stamp_ns(message: object) -> int:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return 0
    seconds = int(getattr(stamp, "sec", 0))
    nanoseconds = int(getattr(stamp, "nanosec", 0))
    if seconds < 0 or nanoseconds < 0:
        return -1
    return seconds * _NANOSECONDS_PER_SECOND + nanoseconds


def _set_stamp(message: object, timestamp_ns: int) -> None:
    header = message.header
    header.stamp.sec = timestamp_ns // _NANOSECONDS_PER_SECOND
    header.stamp.nanosec = timestamp_ns % _NANOSECONDS_PER_SECOND


def _feedback_payload(message: ActuatorFeedbackMsg) -> Mapping[str, object]:
    """Convert the ROS message to a strict core payload.

    Transport-invalid state flags are represented by an extra key so the core
    parser rejects them instead of accidentally trusting measured values.
    """
    payload: dict[str, object] = {
        "correlation_id": str(message.correlation_id),
        "sequence_id": int(message.sequence_id),
        "timestamp_ns": _stamp_ns(message),
        "measured_linear_x": float(message.measured_linear_x),
        "measured_linear_y": float(message.measured_linear_y),
        "measured_angular_z": float(message.measured_angular_z),
    }
    if (
        not bool(message.is_valid)
        or int(message.state)
        in {
            ActuatorFeedbackMsg.STATE_INVALID_DATA,
            ActuatorFeedbackMsg.STATE_STALE_DATA,
        }
    ):
        payload["transport_feedback_invalid"] = True
    return payload


def _is_zero_command(message: ArbitrationStatusMsg) -> bool:
    command = message.commanded_twist
    return all(
        abs(value) <= _ZERO_EPSILON
        for value in (
            float(command.linear.x),
            float(command.linear.y),
            float(command.angular.z),
        )
    )


def _state_code(result: StopMonitorResult) -> int:
    return {
        "idle": StopAcknowledgementMsg.STATE_IDLE,
        "stop_pending": StopAcknowledgementMsg.STATE_STOP_PENDING,
        "stop_acknowledged": StopAcknowledgementMsg.STATE_STOP_ACKNOWLEDGED,
        "hardware_fault_latch": (
            StopAcknowledgementMsg.STATE_HARDWARE_FAULT_LATCH
        ),
    }[result.state.value]


def _shutdown_rclpy() -> None:
    """Shut down the global ROS context without turning teardown into a fault."""
    if not rclpy.ok():
        return
    try:
        rclpy.shutdown()
    except RCLError:
        # launch_testing may already have shut down the shared context while
        # this process is unwinding its own finally block.
        return


class ActuatorFeedbackMonitorNode(Node):
    """Bridge actuator feedback and authoritative control stop requests."""

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
    ) -> None:
        super().__init__(
            "actuator_feedback_monitor_node",
            parameter_overrides=parameter_overrides,
        )

        policy_version = _required_text_parameter(self, "policy_version")
        output_frame_id = _required_text_parameter(self, "output_frame_id")
        stop_budget_ns = _seconds_to_ns(
            _required_positive_float_parameter(self, "stop_budget_sec"),
            name="stop_budget_sec",
        )
        stale_threshold_ns = _seconds_to_ns(
            _required_positive_float_parameter(
                self,
                "feedback_stale_threshold_sec",
            ),
            name="feedback_stale_threshold_sec",
        )
        epsilon_stop_linear = _required_nonnegative_float_parameter(
            self,
            "epsilon_stop_linear",
        )
        epsilon_stop_angular = _required_nonnegative_float_parameter(
            self,
            "epsilon_stop_angular",
        )
        timer_period_sec = _required_positive_float_parameter(
            self,
            "timer_period_sec",
        )

        config = StopMonitorConfig(
            stop_budget_ns=stop_budget_ns,
            feedback_config=ActuatorFeedbackConfig(
                epsilon_stop_linear=epsilon_stop_linear,
                epsilon_stop_angular=epsilon_stop_angular,
                stale_threshold_ns=stale_threshold_ns,
            ),
        )
        self._monitor = StopAckMonitor(config)
        self._policy_version = policy_version
        self._output_frame_id = output_frame_id
        self._last_feedback: FeedbackInput = None
        self._feedback_pending = False
        self._last_control_status: ArbitrationStatusMsg | None = None
        self._last_result: StopMonitorResult | None = None
        self._last_status: StopAcknowledgementMsg | None = None
        self._last_acknowledgement: StopAcknowledgementMsg | None = None

        self._acknowledgement_publisher = self.create_publisher(
            StopAcknowledgementMsg,
            "/actuator/stop_acknowledgement",
            status_qos_profile(),
        )
        self._status_publisher = self.create_publisher(
            StopAcknowledgementMsg,
            "/actuator/status",
            status_qos_profile(),
        )
        self._feedback_subscription = self.create_subscription(
            ActuatorFeedbackMsg,
            "/actuator/feedback",
            self._on_feedback,
            feedback_qos_profile(),
        )
        self._arbitration_subscription = self.create_subscription(
            ArbitrationStatusMsg,
            "/safety/arbitration_status",
            self._on_arbitration_status,
            control_qos_profile(),
        )
        self._cmd_vel_subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self._on_cmd_vel,
            control_qos_profile(),
        )
        self._timer = self.create_timer(timer_period_sec, self._on_timer)

        self._publish_result(self._monitor.tick(now_ns=self._now_ns()))
        self.get_logger().info(
            "ActuatorFeedbackMonitorNode initialized "
            f"(policy={policy_version!r}, frame={output_frame_id!r})"
        )

    @property
    def monitor(self) -> StopAckMonitor:
        """Expose the dependency-free monitor for diagnostics and tests."""
        return self._monitor

    @property
    def last_result(self) -> StopMonitorResult | None:
        """Return the latest core monitor result."""
        return self._last_result

    @property
    def last_status(self) -> StopAcknowledgementMsg | None:
        """Return the latest published actuator status message."""
        return self._last_status

    @property
    def last_acknowledgement(self) -> StopAcknowledgementMsg | None:
        """Return the latest status message carrying a stop acknowledgement."""
        return self._last_acknowledgement

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _publish_result(self, result: StopMonitorResult) -> None:
        timestamp_ns = result.evaluated_timestamp_ns
        message = StopAcknowledgementMsg()
        _set_stamp(message, timestamp_ns)
        message.header.frame_id = self._output_frame_id
        message.schema_version = "0.1"
        message.state = _state_code(result)
        message.reason = result.reason.value
        message.correlation_id = result.correlation_id or ""
        message.request_sequence_id = result.request_sequence_id or 0
        message.is_stopped = False
        message.latched_fault = result.is_latched
        message.critical = result.is_latched
        message.priority = (
            StopAcknowledgementMsg.PRIORITY_CRITICAL
            if result.is_latched
            else StopAcknowledgementMsg.PRIORITY_NORMAL
        )
        message.stop_elapsed_ns = 0
        message.evaluated_timestamp_ns = timestamp_ns
        if result.acknowledgement is not None:
            acknowledgement = result.acknowledgement
            message.feedback_sequence_id = acknowledgement.feedback_sequence_id
            message.is_stopped = True
            message.stop_elapsed_ns = acknowledgement.stop_latency_ns
            self._last_acknowledgement = message

        self._last_result = result
        self._last_status = message
        self._acknowledgement_publisher.publish(message)
        self._status_publisher.publish(message)

    def _on_feedback(self, message: ActuatorFeedbackMsg) -> None:
        self._last_feedback = _feedback_payload(message)
        self._feedback_pending = True
        if self._monitor.state.value not in {
            "stop_pending",
            "stop_acknowledged",
        }:
            return
        result = self._monitor.observe_feedback(
            self._last_feedback,
            now_ns=self._now_ns(),
        )
        self._feedback_pending = False
        self._publish_result(result)

    def _stop_request_from_status(
        self,
        message: ArbitrationStatusMsg,
        *,
        now_ns: int,
    ) -> Mapping[str, object]:
        requested_timestamp_ns = _stamp_ns(message)
        if requested_timestamp_ns <= 0:
            requested_timestamp_ns = now_ns
        return {
            "correlation_id": str(message.correlation_id),
            "sequence_id": int(message.command_sequence_id),
            "requested_timestamp_ns": requested_timestamp_ns,
        }

    def _on_arbitration_status(self, message: ArbitrationStatusMsg) -> None:
        self._last_control_status = message
        command_is_stop = (
            int(message.mode) == ArbitrationStatusMsg.MODE_FORCED_ZERO
            or _is_zero_command(message)
        )
        if not command_is_stop:
            return
        now_ns = self._now_ns()
        result = self._monitor.request_stop(
            self._stop_request_from_status(message, now_ns=now_ns),
            now_ns=now_ns,
        )
        self._publish_result(result)
        self._consume_pending_feedback(now_ns)

    def _on_cmd_vel(self, message: Twist) -> None:
        """Use /cmd_vel as a conservative fallback when status is unavailable."""
        linear = getattr(getattr(message, "linear", None), "x", math.inf)
        lateral = getattr(getattr(message, "linear", None), "y", math.inf)
        angular = getattr(getattr(message, "angular", None), "z", math.inf)
        if not all(
            abs(float(value)) <= _ZERO_EPSILON
            for value in (linear, lateral, angular)
        ):
            return
        if self._monitor.state.value != "idle":
            # ArbitrationStatus is authoritative when a stop window is already
            # active; do not restart the same window on every zero /cmd_vel.
            return
        status = self._last_control_status
        if status is None:
            return
        now_ns = self._now_ns()
        result = self._monitor.request_stop(
            self._stop_request_from_status(status, now_ns=now_ns),
            now_ns=now_ns,
        )
        self._publish_result(result)
        self._consume_pending_feedback(now_ns)

    def _consume_pending_feedback(self, now_ns: int) -> None:
        if not self._feedback_pending or self._last_feedback is None:
            return
        result = self._monitor.observe_feedback(
            self._last_feedback,
            now_ns=now_ns,
        )
        self._feedback_pending = False
        self._publish_result(result)

    def _on_timer(self) -> None:
        now_ns = self._now_ns()
        self._consume_pending_feedback(now_ns)
        if self._monitor.state.value == "stop_pending":
            result = self._monitor.tick(now_ns=now_ns)
            self._publish_result(result)


def main() -> None:
    """Run the ROS 2 actuator feedback monitor process."""
    rclpy.init()
    node = ActuatorFeedbackMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        _shutdown_rclpy()


__all__ = [
    "ActuatorFeedbackMonitorNode",
    "control_qos_profile",
    "feedback_qos_profile",
    "main",
    "status_qos_profile",
]
