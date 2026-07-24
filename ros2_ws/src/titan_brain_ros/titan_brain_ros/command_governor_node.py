"""ROS 2 adapter for the TB-EVAL-006 command governor."""

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
from titan_brain_msgs.msg import SafetySupervisorStatus

from core.command_governor import (
    NANOSECONDS_PER_SECOND,
    CommandGovernor,
    GovernorCommand,
    GovernorConfig,
    GovernorResult,
)

try:
    from rclpy.exceptions import RCLError
except ImportError:  # ROS 2 Jazzy exposes the type from its pybind module.
    from rclpy._rclpy_pybind11 import RCLError

RAW_COMMAND_TOPIC = "/cmd_vel_raw"
GOVERNED_COMMAND_TOPIC = "/cmd_vel_governed"
SAFETY_STATUS_TOPIC = "/safety/supervisor_status"


def command_qos_profile() -> QoSProfile:
    """Return a bounded reliable profile for velocity commands."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def safety_qos_profile() -> QoSProfile:
    """Return a reliable profile for the external safety supervisor."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _parameter_float(node: Node, name: str, default: float) -> float:
    value = node.declare_parameter(name, default).value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"ROS parameter {name!r} must be numeric")
    checked = float(value)
    if not math.isfinite(checked) or checked <= 0.0:
        raise ValueError(f"ROS parameter {name!r} must be finite and positive")
    return checked


def _seconds_parameter(node: Node, name: str, default: float) -> float:
    return _parameter_float(node, name, default)


def _parameter_bool(node: Node, name: str, default: bool) -> bool:
    value = node.declare_parameter(name, default).value
    if not isinstance(value, bool):
        raise ValueError(f"ROS parameter {name!r} must be boolean")
    return value


def _finite_component(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    checked = float(value)
    return checked if math.isfinite(checked) else None


def _safety_state_is_trip(status: SafetySupervisorStatus) -> bool:
    """Treat every state except healthy closed-loop operation as unsafe."""
    return (
        int(status.supervisor_state) != SafetySupervisorStatus.STATE_OK
        or not bool(status.relay_closed_request)
        or bool(status.active_faults)
    )


def _shutdown_rclpy() -> None:
    """Shut down the shared ROS context without masking a test failure."""
    if not rclpy.ok():
        return
    try:
        rclpy.shutdown()
    except RCLError:
        return


class CommandGovernorNode(Node):
    """Profile raw velocity commands before they reach the safety arbiter."""

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
    ) -> None:
        super().__init__(
            "command_governor_node",
            parameter_overrides=parameter_overrides,
        )

        self._timer_period_sec = _seconds_parameter(
            self,
            "timer_period_sec",
            0.02,
        )
        self._cmd_timeout_ns = self._seconds_to_ns(
            _seconds_parameter(self, "cmd_timeout_sec", 0.20),
            name="cmd_timeout_sec",
        )
        self._safety_timeout_ns = self._seconds_to_ns(
            _seconds_parameter(self, "safety_timeout_sec", 0.25),
            name="safety_timeout_sec",
        )
        self._stale_command_emergency_stop = _parameter_bool(
            self,
            "stale_command_emergency_stop",
            True,
        )
        config = GovernorConfig(
            max_linear_velocity_mps=_parameter_float(
                self, "max_linear_velocity_mps", 1.0
            ),
            max_angular_velocity_radps=_parameter_float(
                self, "max_angular_velocity_radps", 1.0
            ),
            max_linear_acceleration_mps2=_parameter_float(
                self, "max_linear_acceleration_mps2", 1.0
            ),
            max_linear_deceleration_mps2=_parameter_float(
                self, "max_linear_deceleration_mps2", 2.0
            ),
            max_angular_acceleration_radps2=_parameter_float(
                self, "max_angular_acceleration_radps2", 1.0
            ),
            max_angular_deceleration_radps2=_parameter_float(
                self, "max_angular_deceleration_radps2", 2.0
            ),
            max_linear_jerk_mps3=_parameter_float(self, "max_linear_jerk_mps3", 5.0),
            max_angular_jerk_radps3=_parameter_float(
                self, "max_angular_jerk_radps3", 5.0
            ),
        )

        now_ns = self._now_ns()
        self._governor = CommandGovernor(
            config,
            initial_timestamp_ns=max(0, now_ns - 1),
        )
        self._last_command: GovernorCommand | None = None
        self._last_command_received_ns: int | None = None
        self._last_safety_received_ns: int | None = None
        self._safety_trip = True
        self._last_result: GovernorResult | None = None

        self._governed_publisher = self.create_publisher(
            Twist,
            GOVERNED_COMMAND_TOPIC,
            command_qos_profile(),
        )
        self._raw_subscription = self.create_subscription(
            Twist,
            RAW_COMMAND_TOPIC,
            self._on_raw_command,
            command_qos_profile(),
        )
        self._safety_subscription = self.create_subscription(
            SafetySupervisorStatus,
            SAFETY_STATUS_TOPIC,
            self._on_safety_status,
            safety_qos_profile(),
        )
        self._timer = self.create_timer(
            self._timer_period_sec,
            self._on_timer,
        )

        # No command or safety proof exists at startup: publish a hard zero.
        self._publish_emergency(now_ns)
        self.get_logger().info(
            "CommandGovernorNode initialized at 50 Hz with fail-closed startup"
        )

    @staticmethod
    def _seconds_to_ns(value: float, *, name: str) -> int:
        nanoseconds = round(value * NANOSECONDS_PER_SECOND)
        if nanoseconds <= 0:
            raise ValueError(f"ROS parameter {name!r} must round to at least 1 ns")
        return nanoseconds

    def _now_ns(self) -> int:
        """Use the ROS clock so simulation and executor tests are deterministic."""
        return int(self.get_clock().now().nanoseconds)

    @property
    def governor(self) -> CommandGovernor:
        """Expose the dependency-free governor for diagnostics and tests."""
        return self._governor

    @property
    def last_result(self) -> GovernorResult | None:
        """Return the most recently published governed result."""
        return self._last_result

    @property
    def last_command_received_ns(self) -> int | None:
        """Return the ingress time of the most recent raw command."""
        return self._last_command_received_ns

    def _on_raw_command(self, message: Twist) -> None:
        now_ns = self._now_ns()
        linear_x = _finite_component(message.linear.x)
        angular_z = _finite_component(message.angular.z)
        lateral_y = _finite_component(message.linear.y)
        if linear_x is None or angular_z is None or lateral_y is None:
            self._last_command = None
            self._last_command_received_ns = now_ns
            self._publish_emergency(now_ns)
            return
        # The 006A core governs one translational axis.  Do not silently pass
        # an unsupported lateral component into a drivetrain command path.
        if lateral_y != 0.0:
            self._last_command = None
            self._last_command_received_ns = now_ns
            self._publish_emergency(now_ns)
            return
        self._last_command = GovernorCommand(
            linear_velocity_mps=linear_x,
            angular_velocity_radps=angular_z,
            timestamp_ns=now_ns,
        )
        self._last_command_received_ns = now_ns

    def _on_safety_status(self, message: SafetySupervisorStatus) -> None:
        now_ns = self._now_ns()
        self._last_safety_received_ns = now_ns
        self._safety_trip = _safety_state_is_trip(message)
        if self._safety_trip:
            self._publish_emergency(now_ns)

    def _command_is_stale(self, now_ns: int) -> bool:
        received = self._last_command_received_ns
        return (
            received is None
            or now_ns < received
            or now_ns - received > self._cmd_timeout_ns
        )

    def _safety_is_stale(self, now_ns: int) -> bool:
        received = self._last_safety_received_ns
        return (
            received is None
            or now_ns < received
            or now_ns - received > self._safety_timeout_ns
        )

    def _publish_emergency(self, now_ns: int) -> None:
        # Add one nanosecond when a safety callback and timer share a clock
        # tick; this preserves the core's strict monotonic timestamp contract.
        timestamp_ns = max(now_ns, self._governor.last_timestamp_ns + 1)
        result = self._governor.step(
            GovernorCommand(emergency_stop=True),
            timestamp_ns=timestamp_ns,
        )
        self._publish_result(result)

    def _publish_stale_command_stop(self, now_ns: int) -> None:
        if self._stale_command_emergency_stop:
            self._publish_emergency(now_ns)
            return
        timestamp_ns = max(now_ns, self._governor.last_timestamp_ns + 1)
        result = self._governor.step(
            GovernorCommand(),
            timestamp_ns=timestamp_ns,
        )
        self._publish_result(result)

    def _publish_result(self, result: GovernorResult) -> None:
        message = Twist()
        message.linear.x = result.linear_velocity_mps
        message.linear.y = 0.0
        message.angular.z = result.angular_velocity_radps
        self._governed_publisher.publish(message)
        self._last_result = result

    def _on_timer(self) -> None:
        now_ns = self._now_ns()
        if self._safety_trip or self._safety_is_stale(now_ns):
            self._publish_emergency(now_ns)
            return
        if self._command_is_stale(now_ns) or self._last_command is None:
            self._publish_stale_command_stop(now_ns)
            return
        result = self._governor.step(
            self._last_command,
            timestamp_ns=now_ns,
        )
        self._publish_result(result)


# A narrow alias keeps launch/test code readable across the 006B–006D slices.
CommandGovernorRosNode = CommandGovernorNode


def main(args: list[str] | None = None) -> None:
    """Run the governor node until ROS requests shutdown."""
    rclpy.init(args=args)
    node: CommandGovernorNode | None = None
    try:
        node = CommandGovernorNode()
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
    "CommandGovernorNode",
    "CommandGovernorRosNode",
    "GOVERNED_COMMAND_TOPIC",
    "RAW_COMMAND_TOPIC",
    "SAFETY_STATUS_TOPIC",
    "command_qos_profile",
    "main",
    "safety_qos_profile",
]
