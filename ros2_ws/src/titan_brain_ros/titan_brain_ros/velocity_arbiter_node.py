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
from titan_brain_msgs.msg import SafetyEvaluationStatus as SafetyEvaluationStatusMsg

from core.arbitrator import (
    ArbitrationMode,
    ArbitrationResult,
    EvaluationAction,
    SafetyInput,
    VelocityArbiter,
    VelocityArbiterConfig,
    VelocityInput,
)

_NANOSECONDS_PER_SECOND = 1_000_000_000
_SAFETY_STATUS_SCHEMA_VERSION = "0.1"


def command_qos_profile() -> QoSProfile:
    """Return the explicit low-depth reliable velocity-command QoS contract."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def status_qos_profile() -> QoSProfile:
    """Return the reliable, non-latched safety-status QoS contract."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _required_text_parameter(node: Node, name: str) -> str:
    value = node.declare_parameter(name).value
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"ROS parameter {name!r} must be a non-blank string")
    return value


def _required_finite_parameter(
    node: Node,
    name: str,
    *,
    allow_zero: bool,
) -> float:
    value = node.declare_parameter(name).value
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


def _message_stamp_ns(message: SafetyEvaluationStatusMsg) -> int:
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


class VelocityArbiterNode(Node):
    """Apply the pure arbiter and exclusively publish its checked command."""

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
        )

        self._arbiter = VelocityArbiter(config)
        self._desired_velocity: VelocityInput = None
        self._safety_state: SafetyInput = None
        self._last_evaluation_action: str | None = None
        self._last_evaluation_timestamp_ns: int | None = None
        self._transport_fault_latched = False
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
            "/cmd_vel_nav",
            self._on_desired_velocity,
            command_qos_profile(),
        )
        self._safety_subscription = self.create_subscription(
            SafetyEvaluationStatusMsg,
            "/safety/evaluation_status",
            self._on_safety_status,
            status_qos_profile(),
        )
        self._arbitration_timer = self.create_timer(
            timer_period_sec,
            self._on_timer,
        )

        # Publish a fail-closed command during startup instead of waiting for
        # the first timer tick or either input stream.
        self._on_timer()
        self.get_logger().info(
            "VelocityArbiterNode initialized as /cmd_vel authority "
            f"(frame={output_frame_id!r}, policy={policy_version!r})"
        )

    @property
    def arbiter(self) -> VelocityArbiter:
        """Expose the immutable policy for runtime diagnostics and tests."""
        return self._arbiter

    @property
    def last_result(self) -> ArbitrationResult | None:
        """Return the most recent authoritative arbitration result."""
        return self._last_result

    @property
    def last_status(self) -> ArbitrationStatusMsg | None:
        """Return the diagnostic message paired with the latest command."""
        return self._last_status

    def _on_desired_velocity(self, message: Twist) -> None:
        received_at_ns = self.get_clock().now().nanoseconds
        self._desired_velocity = {
            "linear_x": float(message.linear.x),
            "linear_y": float(message.linear.y),
            "angular_z": float(message.angular.z),
            "timestamp_ns": received_at_ns,
            "frame_id": self._arbiter.config.output_frame_id,
        }

    def _on_safety_status(self, message: SafetyEvaluationStatusMsg) -> None:
        if message.schema_version != _SAFETY_STATUS_SCHEMA_VERSION:
            self._transport_fault_latched = True
            self._safety_state = {"unsupported_schema": message.schema_version}
            return
        if message.observation_accepted:
            self._last_evaluation_action = message.action
            self._last_evaluation_timestamp_ns = _message_stamp_ns(message)
            self._transport_fault_latched = False
        elif message.adapter_status != "watchdog":
            # A TF, validation, or persistence failure cannot be cleared by a
            # later heartbeat; only a newly accepted evaluation can clear it.
            self._transport_fault_latched = True
            self._safety_state = {"transport_fault": True}
            return

        action = self._last_evaluation_action
        timestamp_ns = self._last_evaluation_timestamp_ns
        if self._transport_fault_latched:
            self._safety_state = {"transport_fault": True}
            return
        if action is None or timestamp_ns is None:
            self._safety_state = None
            return

        self._safety_state = {
            "is_safe": (
                bool(message.watchdog_healthy)
                and action
                in {
                    EvaluationAction.PROCEED.value,
                    EvaluationAction.CLAMP.value,
                }
            ),
            "watchdog_state": message.watchdog_status,
            "eval_action": action,
            "timestamp_ns": timestamp_ns,
        }

    def _publish_result(self, result: ArbitrationResult, *, now_ns: int) -> None:
        command = _twist_from_result(result)
        status = ArbitrationStatusMsg()
        status.header.stamp.sec = now_ns // _NANOSECONDS_PER_SECOND
        status.header.stamp.nanosec = now_ns % _NANOSECONDS_PER_SECOND
        status.header.frame_id = self._arbiter.config.output_frame_id
        status.mode = _status_mode(result)
        status.reason = result.reason.value
        status.commanded_twist = command

        self._command_publisher.publish(command)
        self._status_publisher.publish(status)
        self._last_result = result
        self._last_status = status

    def _on_timer(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        result = self._arbiter.arbitrate(
            self._desired_velocity,
            self._safety_state,
            now_ns=now_ns,
        )
        self._publish_result(result, now_ns=now_ns)


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
