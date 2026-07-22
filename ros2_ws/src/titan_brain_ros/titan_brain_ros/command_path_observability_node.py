"""ROS 2 observability-plane correlation for the complete command path."""

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
from titan_brain_msgs.msg import ArbitrationStatus as ArbitrationStatusMsg
from titan_brain_msgs.msg import (
    CommandPathObservabilityStatus as CommandPathObservabilityStatusMsg,
)
from titan_brain_msgs.msg import (
    EvaluatorObservabilityStatus as EvaluatorObservabilityStatusMsg,
)

from core.command_observability import (
    ArbitrationTimingSample,
    CommandPathObservability,
    CommandPathObservabilityConfig,
    CommandPathObservabilityReport,
    EvaluatorTimingSample,
    measure_arbitration_latency,
)

_NANOSECONDS_PER_SECOND = 1_000_000_000


def status_qos_profile() -> QoSProfile:
    """Return the reliable, volatile observability QoS contract."""
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


def _required_positive_integer_parameter(node: Node, name: str) -> int:
    value = node.declare_parameter(name, Parameter.Type.INTEGER).value
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"ROS parameter {name!r} must be a positive integer")
    return value


def _seconds_to_ns(value: float, *, name: str) -> int:
    nanoseconds = round(value * _NANOSECONDS_PER_SECOND)
    if nanoseconds <= 0:
        raise ValueError(f"ROS parameter {name!r} must round to at least 1 ns")
    return nanoseconds


def _message_stamp_ns(message: ArbitrationStatusMsg) -> int:
    return (
        int(message.header.stamp.sec) * _NANOSECONDS_PER_SECOND
        + int(message.header.stamp.nanosec)
    )


def _evaluator_message_stamp_ns(
    message: EvaluatorObservabilityStatusMsg,
) -> int:
    return (
        int(message.header.stamp.sec) * _NANOSECONDS_PER_SECOND
        + int(message.header.stamp.nanosec)
    )


class CommandPathObservabilityNode(Node):
    """Pair evaluator and arbiter telemetry without actuator authority."""

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
    ) -> None:
        super().__init__(
            "command_path_observability_node",
            parameter_overrides=parameter_overrides,
        )
        arbitration_budget_ns = _seconds_to_ns(
            _required_positive_float_parameter(
                self,
                "arbitration_latency_budget_sec",
            ),
            name="arbitration_latency_budget_sec",
        )
        config = CommandPathObservabilityConfig(
            policy_version=_required_text_parameter(
                self,
                "command_path_policy_version",
            ),
            arbitration_budget_ns=arbitration_budget_ns,
            observation_to_command_budget_ns=_seconds_to_ns(
                _required_positive_float_parameter(
                    self,
                    "observation_to_command_budget_sec",
                ),
                name="observation_to_command_budget_sec",
            ),
            max_correlations=_required_positive_integer_parameter(
                self,
                "max_correlation_entries",
            ),
            max_pending_per_correlation=_required_positive_integer_parameter(
                self,
                "max_pending_per_correlation",
            ),
        )
        self._observability = CommandPathObservability(config)
        self._last_report: CommandPathObservabilityReport | None = None
        self._last_status: CommandPathObservabilityStatusMsg | None = None

        self._publisher = self.create_publisher(
            CommandPathObservabilityStatusMsg,
            "/safety/command_path_observability",
            status_qos_profile(),
        )
        self._evaluator_subscription = self.create_subscription(
            EvaluatorObservabilityStatusMsg,
            "/safety/evaluator_observability",
            self._on_evaluator_status,
            status_qos_profile(),
        )
        self._arbitration_subscription = self.create_subscription(
            ArbitrationStatusMsg,
            "/safety/arbitration_status",
            self._on_arbitration_status,
            status_qos_profile(),
        )
        self.get_logger().info(
            "CommandPathObservabilityNode initialized "
            f"(policy={config.policy_version!r})"
        )

    @property
    def observability(self) -> CommandPathObservability:
        """Expose the dependency-free correlator for diagnostics/tests."""
        return self._observability

    @property
    def last_report(self) -> CommandPathObservabilityReport | None:
        """Return the most recently correlated core report."""
        return self._last_report

    @property
    def last_status(self) -> CommandPathObservabilityStatusMsg | None:
        """Return the most recently published ROS diagnostic."""
        return self._last_status

    def _evaluator_sample(
        self,
        message: EvaluatorObservabilityStatusMsg,
    ) -> EvaluatorTimingSample:
        if str(message.schema_version) != "0.1":
            raise ValueError("Unsupported evaluator observability schema")
        timing_valid = bool(message.timing_valid)
        latency_status = str(message.latency_status)
        within_budget = bool(message.within_budget)
        exceeded_budgets = tuple(message.exceeded_budgets)
        detail = str(message.detail)
        timing_values = (
            int(message.observation_timestamp_ns),
            int(message.received_timestamp_ns),
            int(message.decision_timestamp_ns),
            int(message.published_timestamp_ns),
            int(message.observation_to_receive_ns),
            int(message.receive_to_decision_ns),
            int(message.decision_to_publish_ns),
            int(message.end_to_end_ns),
        )
        if timing_valid:
            observation_ns, received_ns, decision_ns, published_ns = (
                timing_values[:4]
            )
            expected_latencies = (
                received_ns - observation_ns,
                decision_ns - received_ns,
                published_ns - decision_ns,
                published_ns - observation_ns,
            )
            expected_budget_shape = (
                latency_status == "within_budget"
                and within_budget
                and not exceeded_budgets
            ) or (
                latency_status == "budget_exceeded"
                and not within_budget
                and bool(exceeded_budgets)
            )
            if (
                timing_values[4:] != expected_latencies
                or published_ns != _evaluator_message_stamp_ns(message)
                or detail != "none"
                or not expected_budget_shape
            ):
                raise ValueError("Evaluator status contains inconsistent timing")
        elif (
            latency_status not in {"clock_regression", "invalid_timestamp"}
            or within_budget
            or any(timing_values)
            or exceeded_budgets
            or not detail.strip()
            or detail == "none"
        ):
            raise ValueError("Evaluator status contains inconsistent invalid timing")
        return EvaluatorTimingSample(
            correlation_id=str(message.correlation_id),
            decision_id=str(message.decision_id) or "none",
            outcome=str(message.outcome),
            latency_status=latency_status,
            timing_valid=timing_valid,
            observation_timestamp_ns=(
                int(message.observation_timestamp_ns)
                if timing_valid
                else None
            ),
            published_timestamp_ns=(
                int(message.published_timestamp_ns) if timing_valid else None
            ),
            end_to_end_ns=(int(message.end_to_end_ns) if timing_valid else None),
            exceeded_budgets=exceeded_budgets,
            detail=None if timing_valid else detail,
        )

    def _arbitration_sample(
        self,
        message: ArbitrationStatusMsg,
    ) -> ArbitrationTimingSample:
        config = self._observability.config
        if int(message.arbitration_latency_budget_ns) != config.arbitration_budget_ns:
            raise ValueError("Arbitration status uses an unexpected latency budget")
        if int(message.command_published_timestamp_ns) != _message_stamp_ns(message):
            raise ValueError("Arbitration publication timestamp does not match header")
        transmitted_status = str(message.arbitration_latency_status)
        received_timestamp = int(message.intent_received_timestamp_ns)
        timing = measure_arbitration_latency(
            intent_received_ns=(
                None
                if transmitted_status == "invalid_timing"
                and received_timestamp == 0
                else received_timestamp
            ),
            command_published_ns=int(message.command_published_timestamp_ns),
            budget_ns=config.arbitration_budget_ns,
        )
        transmitted = (
            transmitted_status,
            bool(message.arbitration_timing_valid),
            bool(message.arbitration_within_budget),
            int(message.arbitration_latency_ns),
        )
        expected = (
            timing.status.value,
            timing.timing_valid,
            timing.within_budget,
            timing.latency_ns or 0,
        )
        if transmitted != expected:
            raise ValueError("Arbitration status contains inconsistent timing")
        return ArbitrationTimingSample(
            correlation_id=str(message.correlation_id),
            reason=str(message.reason),
            mode=int(message.mode),
            command_sequence_id=int(message.command_sequence_id),
            safety_intent_sequence_id=int(message.safety_intent_sequence_id),
            timing=timing,
        )

    def _on_evaluator_status(
        self,
        message: EvaluatorObservabilityStatusMsg,
    ) -> None:
        try:
            reports = self._observability.record_evaluator(
                self._evaluator_sample(message)
            )
        except ValueError as error:
            self.get_logger().error(f"Evaluator telemetry rejected: {error}")
            return
        for report in reports:
            self._publish_report(report)

    def _on_arbitration_status(self, message: ArbitrationStatusMsg) -> None:
        if not str(message.correlation_id).strip():
            return
        try:
            reports = self._observability.record_arbitration(
                self._arbitration_sample(message)
            )
        except ValueError as error:
            self.get_logger().error(f"Arbitration telemetry rejected: {error}")
            return
        for report in reports:
            self._publish_report(report)

    def _publish_report(self, report: CommandPathObservabilityReport) -> None:
        config = self._observability.config
        published_ns = report.command_published_timestamp_ns
        if published_ns is None:
            published_ns = self.get_clock().now().nanoseconds
        message = CommandPathObservabilityStatusMsg()
        message.header.stamp.sec = published_ns // _NANOSECONDS_PER_SECOND
        message.header.stamp.nanosec = published_ns % _NANOSECONDS_PER_SECOND
        message.schema_version = report.schema_version
        message.policy_version = report.policy_version
        message.correlation_id = report.correlation_id
        message.decision_id = report.decision_id
        message.outcome = report.outcome
        message.arbitration_reason = report.arbitration_reason
        message.arbitration_mode = report.arbitration_mode
        message.command_sequence_id = report.command_sequence_id
        message.safety_intent_sequence_id = report.safety_intent_sequence_id
        message.latency_status = report.latency_status.value
        message.timing_valid = report.timing_valid
        message.within_budget = report.within_budget
        message.observation_timestamp_ns = report.observation_timestamp_ns or 0
        message.evaluator_published_timestamp_ns = (
            report.evaluator_published_timestamp_ns or 0
        )
        message.intent_received_timestamp_ns = (
            report.intent_received_timestamp_ns or 0
        )
        message.command_published_timestamp_ns = (
            report.command_published_timestamp_ns or 0
        )
        message.evaluator_end_to_end_ns = report.evaluator_end_to_end_ns or 0
        message.arbitration_latency_ns = report.arbitration_latency_ns or 0
        message.observation_to_command_ns = report.observation_to_command_ns or 0
        message.arbitration_latency_budget_ns = config.arbitration_budget_ns
        message.observation_to_command_budget_ns = (
            config.observation_to_command_budget_ns
        )
        message.exceeded_budgets = list(report.exceeded_budgets)
        message.detail = report.detail or "none"
        self._publisher.publish(message)
        self._last_report = report
        self._last_status = message


def main(args: list[str] | None = None) -> None:
    """Run command-path observability until shutdown."""
    rclpy.init(args=args)
    node: CommandPathObservabilityNode | None = None
    try:
        node = CommandPathObservabilityNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        # During launch shutdown, rclpy can race a queued DDS conversion with
        # destruction of the subscription's native message storage.  This
        # node is observability-only, so the conversion race must not turn a
        # clean SIGINT shutdown into a failed process exit.  Other runtime
        # errors remain actionable and are re-raised.
        if "Unable to convert call argument" not in str(error):
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
