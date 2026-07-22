"""ROS 2 adapter for the TB-SAFE-001 external safety-loop supervisor."""

from __future__ import annotations

import math
import time
from collections.abc import Callable

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
    SafetyHeartbeat,
    SafetyRelayStatus,
    SafetySupervisorStatus,
)

from core.safety_supervisor import (
    HeartbeatChannel,
    RelayRequest,
    SafetyReason,
    SafetyState,
    SafetySupervisor,
    SafetySupervisorConfig,
    SafetySupervisorResult,
)

try:
    from rclpy.exceptions import RCLError
except ImportError:  # ROS 2 Jazzy exposes the type from its pybind module.
    from rclpy._rclpy_pybind11 import RCLError

_NANOSECONDS_PER_SECOND = 1_000_000_000
_HEALTHY_STATUS_CODE = 0

HEARTBEAT_TOPICS = {
    HeartbeatChannel.CONTROL_ARBITER: "/safety/heartbeat/control_arbiter",
    HeartbeatChannel.ACTUATOR_MONITOR: "/safety/heartbeat/actuator_monitor",
    HeartbeatChannel.ODOMETRY: "/safety/heartbeat/odometry",
}
RELAY_STATUS_TOPIC = "/safety/relay_status"
RELAY_COMMAND_TOPIC = "/safety/relay_command"
SUPERVISOR_STATUS_TOPIC = "/safety/supervisor_status"


def heartbeat_qos_profile() -> QoSProfile:
    """Return a bounded best-effort profile for liveness pulses."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def control_qos_profile() -> QoSProfile:
    """Return a reliable profile for the relay control plane."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def status_qos_profile() -> QoSProfile:
    """Return a reliable profile for supervisor diagnostics."""
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


def _seconds_to_ns(value: float, *, name: str) -> int:
    nanoseconds = round(value * _NANOSECONDS_PER_SECOND)
    if nanoseconds <= 0:
        raise ValueError(f"ROS parameter {name!r} must round to at least 1 ns")
    return nanoseconds


def _state_code(state: SafetyState) -> int:
    """Map the dependency-free state enum to the generated ROS constants."""
    return {
        SafetyState.INITIALIZING: SafetySupervisorStatus.STATE_INITIALIZING,
        SafetyState.OK: SafetySupervisorStatus.STATE_OK,
        SafetyState.TRIPPED: SafetySupervisorStatus.STATE_TRIPPED,
        SafetyState.HARDWARE_FAULT_LATCH: (
            SafetySupervisorStatus.STATE_HARDWARE_FAULT_LATCH
        ),
    }[state]


def _active_faults(result: SafetySupervisorResult) -> list[str]:
    """Build a deterministic, bounded diagnostic fault list."""
    faults: list[str] = []
    if result.reason not in {
        SafetyReason.INITIALIZING,
        SafetyReason.ALL_HEARTBEATS_HEALTHY,
        SafetyReason.HEARTBEAT_RECEIVED,
    }:
        faults.append(result.reason.value)
    faults.extend(
        f"missing_heartbeat:{channel.value}" for channel in result.missing_channels
    )
    faults.extend(
        f"failed_heartbeat:{channel.value}" for channel in result.failed_channels
    )
    return list(dict.fromkeys(faults))


def _shutdown_rclpy() -> None:
    """Shut down the shared ROS context without masking the real result."""
    if not rclpy.ok():
        return
    try:
        rclpy.shutdown()
    except RCLError:
        # A launch/pytest fixture may have already shut down the context.
        return


class SafetyLoopSupervisorNode(Node):
    """Bridge ROS heartbeats and relay feedback to :class:`SafetySupervisor`."""

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
    ) -> None:
        super().__init__(
            "safety_loop_supervisor_node",
            parameter_overrides=parameter_overrides,
        )

        heartbeat_timeout_ns = _seconds_to_ns(
            _required_positive_float_parameter(self, "heartbeat_timeout_sec"),
            name="heartbeat_timeout_sec",
        )
        initialization_timeout_ns = _seconds_to_ns(
            _required_positive_float_parameter(
                self,
                "initialization_timeout_sec",
            ),
            name="initialization_timeout_sec",
        )
        relay_budget_ns = _seconds_to_ns(
            _required_positive_float_parameter(self, "relay_budget_sec"),
            name="relay_budget_sec",
        )
        timer_period_sec = _required_positive_float_parameter(
            self,
            "timer_period_sec",
        )
        self._policy_version = _required_text_parameter(self, "policy_version")
        self._output_frame_id = _required_text_parameter(self, "output_frame_id")
        self._reset_authorization_token = _required_text_parameter(
            self,
            "reset_authorization_token",
        )

        config = SafetySupervisorConfig(
            control_arbiter_timeout_ns=heartbeat_timeout_ns,
            actuator_monitor_timeout_ns=heartbeat_timeout_ns,
            odometry_timeout_ns=heartbeat_timeout_ns,
            initialization_timeout_ns=initialization_timeout_ns,
            relay_budget_ns=relay_budget_ns,
            reset_authorization_token=self._reset_authorization_token,
        )
        self._supervisor = SafetySupervisor(
            config,
            started_at_ns=self._now_ns(),
        )
        self._last_sequences: dict[HeartbeatChannel, int] = {}
        self._last_result: SafetySupervisorResult | None = None
        self._last_status: SafetySupervisorStatus | None = None
        self._last_relay_command: SafetyRelayStatus | None = None

        self._status_publisher = self.create_publisher(
            SafetySupervisorStatus,
            SUPERVISOR_STATUS_TOPIC,
            status_qos_profile(),
        )
        self._relay_command_publisher = self.create_publisher(
            SafetyRelayStatus,
            RELAY_COMMAND_TOPIC,
            control_qos_profile(),
        )
        self._heartbeat_subscriptions = [
            self.create_subscription(
                SafetyHeartbeat,
                topic,
                self._heartbeat_callback(channel),
                heartbeat_qos_profile(),
            )
            for channel, topic in HEARTBEAT_TOPICS.items()
        ]
        self._relay_subscription = self.create_subscription(
            SafetyRelayStatus,
            RELAY_STATUS_TOPIC,
            self._on_relay_status,
            control_qos_profile(),
        )
        self._timer = self.create_timer(timer_period_sec, self._on_timer)

        self._publish_result(self._supervisor.last_result)
        self.get_logger().info(
            "SafetyLoopSupervisorNode initialized "
            f"(policy={self._policy_version!r}, "
            f"heartbeat_timeout={heartbeat_timeout_ns} ns)"
        )

    @property
    def supervisor(self) -> SafetySupervisor:
        """Expose the dependency-free core for diagnostics and tests."""
        return self._supervisor

    @property
    def last_result(self) -> SafetySupervisorResult | None:
        """Return the most recently evaluated supervisor result."""
        return self._last_result

    @property
    def last_status(self) -> SafetySupervisorStatus | None:
        """Return the most recently published supervisor status."""
        return self._last_status

    @property
    def last_relay_command(self) -> SafetyRelayStatus | None:
        """Return the most recently published relay command."""
        return self._last_relay_command

    def _now_ns(self) -> int:
        """Use a process-monotonic clock; ROS time may jump in simulation."""
        return time.monotonic_ns()

    def _heartbeat_callback(
        self,
        channel: HeartbeatChannel,
    ) -> Callable[[SafetyHeartbeat], None]:
        def callback(message: SafetyHeartbeat) -> None:
            self._on_heartbeat(message, channel)

        return callback

    def _on_heartbeat(
        self,
        message: SafetyHeartbeat,
        channel: HeartbeatChannel,
    ) -> None:
        """Validate one transport pulse before passing it to the core."""
        now_ns = self._now_ns()
        sender_id = str(message.sender_id)
        sequence_number = int(message.sequence_number)
        previous_sequence = self._last_sequences.get(channel)
        error: str | None = None
        healthy = int(message.status_code) == _HEALTHY_STATUS_CODE
        if sender_id != channel.value:
            healthy = False
            error = "heartbeat sender_id does not match its channel"
        elif previous_sequence is not None and sequence_number <= previous_sequence:
            healthy = False
            error = "heartbeat sequence_number did not increase"
        elif not healthy:
            error = f"component status_code={int(message.status_code)}"
        if healthy:
            self._last_sequences[channel] = sequence_number
        result = self._supervisor.receive_heartbeat(
            channel,
            timestamp_ns=now_ns,
            healthy=healthy,
            error=error,
        )
        self._publish_result(result)

    def _on_control_heartbeat(self, message: SafetyHeartbeat) -> None:
        self._on_heartbeat(message, HeartbeatChannel.CONTROL_ARBITER)

    def _on_actuator_heartbeat(self, message: SafetyHeartbeat) -> None:
        self._on_heartbeat(message, HeartbeatChannel.ACTUATOR_MONITOR)

    def _on_odometry_heartbeat(self, message: SafetyHeartbeat) -> None:
        self._on_heartbeat(message, HeartbeatChannel.ODOMETRY)

    def _on_relay_status(self, message: SafetyRelayStatus) -> None:
        """Validate physical relay feedback and latch transport faults."""
        now_ns = self._now_ns()
        if bool(message.is_latched):
            result = self._supervisor.latch_hardware_fault(
                now_ns=now_ns,
                detail="Relay driver reported a hardware fault latch.",
            )
        else:
            feedback_state = int(message.feedback_state)
            if feedback_state == SafetyRelayStatus.FEEDBACK_CLOSED:
                is_closed: bool | None = True
            elif feedback_state == SafetyRelayStatus.FEEDBACK_OPEN:
                is_closed = False
            else:
                is_closed = None
            result = self._supervisor.observe_relay_feedback(
                is_closed,
                timestamp_ns=now_ns,
                valid=is_closed is not None,
            )
        self._publish_result(result)

    def _on_timer(self) -> None:
        self._publish_result(self._supervisor.tick(now_ns=self._now_ns()))

    def _publish_result(self, result: SafetySupervisorResult) -> None:
        """Publish both the physical command and auditable supervisor status."""
        status = SafetySupervisorStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self._output_frame_id
        status.supervisor_state = _state_code(result.state)
        status.active_faults = _active_faults(result)
        status.relay_closed_request = (
            result.relay_request is RelayRequest.REQUEST_SAFETY_CLOSED
        )

        command = SafetyRelayStatus()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = self._output_frame_id
        command.commanded_state = (
            SafetyRelayStatus.COMMANDED_CLOSED
            if status.relay_closed_request
            else SafetyRelayStatus.COMMANDED_OPEN
        )
        command.feedback_state = SafetyRelayStatus.FEEDBACK_UNKNOWN
        command.is_latched = result.state is SafetyState.HARDWARE_FAULT_LATCH

        self._last_result = result
        self._last_status = status
        self._last_relay_command = command
        self._status_publisher.publish(status)
        self._relay_command_publisher.publish(command)


def main() -> None:
    """Run the ROS 2 safety-loop supervisor process."""
    rclpy.init()
    node: SafetyLoopSupervisorNode | None = None
    try:
        node = SafetyLoopSupervisorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        _shutdown_rclpy()


# Short compatibility alias for launch/test callers using the contract name.
SafetySupervisorNode = SafetyLoopSupervisorNode


__all__ = [
    "HEARTBEAT_TOPICS",
    "RELAY_COMMAND_TOPIC",
    "RELAY_STATUS_TOPIC",
    "SUPERVISOR_STATUS_TOPIC",
    "SafetyLoopSupervisorNode",
    "SafetySupervisorNode",
    "control_qos_profile",
    "heartbeat_qos_profile",
    "main",
    "status_qos_profile",
]
