"""Dependency-free command-path correlation and latency diagnostics."""

from __future__ import annotations

from collections import OrderedDict
from enum import StrEnum
from typing import Self

from pydantic import Field, model_validator

from core.types.incident import StrictFrozenModel


class CommandPathLatencyStatus(StrEnum):
    """Validity and budget state of one command-path timing sample."""

    WITHIN_BUDGET = "within_budget"
    BUDGET_EXCEEDED = "budget_exceeded"
    CLOCK_REGRESSION = "clock_regression"
    INVALID_TIMING = "invalid_timing"


class CommandPathObservabilityConfig(StrictFrozenModel):
    """Versioned budgets and bounded correlation storage."""

    policy_version: str = Field(default="TB-EVAL-004D-0.1.0", min_length=1)
    arbitration_budget_ns: int = Field(default=30_000_000, gt=0)
    observation_to_command_budget_ns: int = Field(default=100_000_000, gt=0)
    max_correlations: int = Field(default=256, gt=0)
    max_pending_per_correlation: int = Field(default=16, gt=0)


class ArbitrationLatencyMeasurement(StrictFrozenModel):
    """Validated duration from SafetyIntent receipt to command publication."""

    status: CommandPathLatencyStatus
    intent_received_ns: int | None = Field(default=None, ge=0)
    command_published_ns: int | None = Field(default=None, ge=0)
    latency_ns: int | None = Field(default=None, ge=0)
    budget_ns: int = Field(gt=0)
    detail: str | None = None

    @property
    def timing_valid(self) -> bool:
        """Return whether both timestamps formed a monotonic interval."""
        return self.status in {
            CommandPathLatencyStatus.WITHIN_BUDGET,
            CommandPathLatencyStatus.BUDGET_EXCEEDED,
        }

    @property
    def within_budget(self) -> bool:
        """Return whether the measured interval met its configured budget."""
        return self.status is CommandPathLatencyStatus.WITHIN_BUDGET

    @model_validator(mode="after")
    def validate_shape(self) -> Self:
        """Keep valid measurements and controlled failures unambiguous."""
        if self.timing_valid:
            if (
                self.intent_received_ns is None
                or self.command_published_ns is None
                or self.latency_ns is None
                or self.detail is not None
            ):
                raise ValueError("valid timing requires both timestamps and latency")
            if (
                self.command_published_ns - self.intent_received_ns
                != self.latency_ns
            ):
                raise ValueError("latency must equal publish minus intent receipt")
            if self.within_budget != (self.latency_ns <= self.budget_ns):
                raise ValueError("latency status must match the configured budget")
        elif self.latency_ns is not None or self.detail is None:
            raise ValueError("invalid timing requires detail and no latency")
        return self


def _checked_timestamp(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def measure_arbitration_latency(
    *,
    intent_received_ns: object,
    command_published_ns: object,
    budget_ns: int,
) -> ArbitrationLatencyMeasurement:
    """Measure one arbitration interval without depending on ROS clocks."""
    if isinstance(budget_ns, bool) or not isinstance(budget_ns, int) or budget_ns <= 0:
        raise ValueError("budget_ns must be a positive integer")
    received = _checked_timestamp(intent_received_ns)
    published = _checked_timestamp(command_published_ns)
    if received is None or published is None:
        return ArbitrationLatencyMeasurement(
            status=CommandPathLatencyStatus.INVALID_TIMING,
            intent_received_ns=received,
            command_published_ns=published,
            budget_ns=budget_ns,
            detail="Both arbitration timestamps must be non-negative integers.",
        )
    if published < received:
        return ArbitrationLatencyMeasurement(
            status=CommandPathLatencyStatus.CLOCK_REGRESSION,
            intent_received_ns=received,
            command_published_ns=published,
            budget_ns=budget_ns,
            detail="Command publication cannot precede SafetyIntent receipt.",
        )
    latency_ns = published - received
    return ArbitrationLatencyMeasurement(
        status=(
            CommandPathLatencyStatus.BUDGET_EXCEEDED
            if latency_ns > budget_ns
            else CommandPathLatencyStatus.WITHIN_BUDGET
        ),
        intent_received_ns=received,
        command_published_ns=published,
        latency_ns=latency_ns,
        budget_ns=budget_ns,
    )


class EvaluatorTimingSample(StrictFrozenModel):
    """Evaluator timing fields required for command-path correlation."""

    correlation_id: str = Field(min_length=1)
    decision_id: str = Field(min_length=1)
    outcome: str = Field(min_length=1)
    latency_status: str = Field(min_length=1)
    timing_valid: bool
    observation_timestamp_ns: int | None = Field(default=None, ge=0)
    published_timestamp_ns: int | None = Field(default=None, ge=0)
    end_to_end_ns: int | None = Field(default=None, ge=0)
    exceeded_budgets: tuple[str, ...] = ()
    detail: str | None = None

    @model_validator(mode="after")
    def validate_shape(self) -> Self:
        """Require exact absolute endpoints for every valid evaluator sample."""
        values = (
            self.observation_timestamp_ns,
            self.published_timestamp_ns,
            self.end_to_end_ns,
        )
        if self.timing_valid:
            if any(value is None for value in values) or self.detail is not None:
                raise ValueError("valid evaluator timing requires both endpoints")
            observation_ns, published_ns, end_to_end_ns = values
            assert observation_ns is not None
            assert published_ns is not None
            assert end_to_end_ns is not None
            if published_ns - observation_ns != end_to_end_ns:
                raise ValueError("evaluator latency must match its timestamps")
            if self.latency_status not in {"within_budget", "budget_exceeded"}:
                raise ValueError("valid evaluator timing requires a valid status")
            if (self.latency_status == "within_budget") == bool(
                self.exceeded_budgets
            ):
                raise ValueError("evaluator status must match exceeded budgets")
        elif any(value is not None for value in values) or self.detail is None:
            raise ValueError("invalid evaluator timing requires only detail")
        elif (
            self.latency_status not in {"clock_regression", "invalid_timestamp"}
            or self.exceeded_budgets
        ):
            raise ValueError("invalid evaluator timing requires an invalid status")
        return self


class ArbitrationTimingSample(StrictFrozenModel):
    """One command publication and its propagated control-plane identity."""

    correlation_id: str = Field(min_length=1)
    reason: str = Field(min_length=1)
    mode: int = Field(ge=0)
    command_sequence_id: int = Field(ge=0)
    safety_intent_sequence_id: int = Field(ge=0)
    timing: ArbitrationLatencyMeasurement


class CommandPathObservabilityReport(StrictFrozenModel):
    """Exact observation-to-command audit record for one publication."""

    schema_version: str = "0.1"
    policy_version: str = Field(min_length=1)
    correlation_id: str = Field(min_length=1)
    decision_id: str = Field(min_length=1)
    outcome: str = Field(min_length=1)
    arbitration_reason: str = Field(min_length=1)
    arbitration_mode: int = Field(ge=0)
    command_sequence_id: int = Field(ge=0)
    safety_intent_sequence_id: int = Field(ge=0)
    latency_status: CommandPathLatencyStatus
    observation_timestamp_ns: int | None = Field(default=None, ge=0)
    evaluator_published_timestamp_ns: int | None = Field(default=None, ge=0)
    intent_received_timestamp_ns: int | None = Field(default=None, ge=0)
    command_published_timestamp_ns: int | None = Field(default=None, ge=0)
    evaluator_end_to_end_ns: int | None = Field(default=None, ge=0)
    arbitration_latency_ns: int | None = Field(default=None, ge=0)
    observation_to_command_ns: int | None = Field(default=None, ge=0)
    exceeded_budgets: tuple[str, ...] = ()
    detail: str | None = None

    @property
    def timing_valid(self) -> bool:
        """Return whether the entire chain has monotonic timestamps."""
        return self.latency_status in {
            CommandPathLatencyStatus.WITHIN_BUDGET,
            CommandPathLatencyStatus.BUDGET_EXCEEDED,
        }

    @property
    def within_budget(self) -> bool:
        """Return whether every evaluator and command budget was met."""
        return self.latency_status is CommandPathLatencyStatus.WITHIN_BUDGET

    @model_validator(mode="after")
    def validate_shape(self) -> Self:
        """Prevent a partially valid end-to-end report from escaping."""
        timestamps = (
            self.observation_timestamp_ns,
            self.evaluator_published_timestamp_ns,
            self.intent_received_timestamp_ns,
            self.command_published_timestamp_ns,
        )
        latencies = (
            self.evaluator_end_to_end_ns,
            self.arbitration_latency_ns,
            self.observation_to_command_ns,
        )
        if self.timing_valid:
            if (
                any(value is None for value in timestamps + latencies)
                or self.detail is not None
            ):
                raise ValueError("valid command timing requires the complete chain")
            observation_ns, evaluator_ns, intent_ns, command_ns = timestamps
            evaluator_latency, arbitration_latency, total_latency = latencies
            assert observation_ns is not None
            assert evaluator_ns is not None
            assert intent_ns is not None
            assert command_ns is not None
            assert evaluator_latency is not None
            assert arbitration_latency is not None
            assert total_latency is not None
            if not observation_ns <= evaluator_ns <= intent_ns <= command_ns:
                raise ValueError("valid command timestamps must be monotonic")
            if (
                evaluator_ns - observation_ns != evaluator_latency
                or command_ns - intent_ns != arbitration_latency
                or command_ns - observation_ns != total_latency
            ):
                raise ValueError("command-path latencies must match timestamps")
            if self.within_budget and self.exceeded_budgets:
                raise ValueError("within-budget command timing cannot name failures")
            if not self.within_budget and not self.exceeded_budgets:
                raise ValueError("budget failure must name an exceeded budget")
        elif self.observation_to_command_ns is not None or self.detail is None:
            raise ValueError("invalid command timing requires detail and no total")
        return self


class CommandPathObservability:
    """Pair evaluator and arbitration samples in either delivery order."""

    def __init__(self, config: CommandPathObservabilityConfig) -> None:
        self._config = config
        self._evaluators: OrderedDict[str, EvaluatorTimingSample] = OrderedDict()
        self._pending: OrderedDict[
            str, list[ArbitrationTimingSample]
        ] = OrderedDict()

    @property
    def config(self) -> CommandPathObservabilityConfig:
        """Return the immutable budgets and storage bounds."""
        return self._config

    def _bound_evaluators(self) -> None:
        while len(self._evaluators) > self._config.max_correlations:
            self._evaluators.popitem(last=False)

    def _bound_pending(self) -> None:
        while len(self._pending) > self._config.max_correlations:
            self._pending.popitem(last=False)

    def record_evaluator(
        self,
        sample: EvaluatorTimingSample,
    ) -> tuple[CommandPathObservabilityReport, ...]:
        """Store one evaluator sample and resolve queued command samples."""
        existing = self._evaluators.get(sample.correlation_id)
        if existing is not None and existing != sample:
            raise ValueError("correlation_id cannot identify mutated evaluator data")
        self._evaluators[sample.correlation_id] = sample
        self._evaluators.move_to_end(sample.correlation_id)
        self._bound_evaluators()
        pending = self._pending.pop(sample.correlation_id, [])
        return tuple(self._correlate(sample, arbitration) for arbitration in pending)

    def record_arbitration(
        self,
        sample: ArbitrationTimingSample,
    ) -> tuple[CommandPathObservabilityReport, ...]:
        """Correlate immediately or retain a bounded out-of-order sample."""
        evaluator = self._evaluators.get(sample.correlation_id)
        if evaluator is not None:
            self._evaluators.move_to_end(sample.correlation_id)
            return (self._correlate(evaluator, sample),)
        pending = self._pending.setdefault(sample.correlation_id, [])
        if len(pending) >= self._config.max_pending_per_correlation:
            pending.pop(0)
        pending.append(sample)
        self._pending.move_to_end(sample.correlation_id)
        self._bound_pending()
        return ()

    def _correlate(
        self,
        evaluator: EvaluatorTimingSample,
        arbitration: ArbitrationTimingSample,
    ) -> CommandPathObservabilityReport:
        timing = arbitration.timing
        if not evaluator.timing_valid or not timing.timing_valid:
            invalid_sources = []
            if not evaluator.timing_valid:
                invalid_sources.append("evaluator")
            if not timing.timing_valid:
                invalid_sources.append("arbitration")
            latency_status = (
                CommandPathLatencyStatus.CLOCK_REGRESSION
                if (
                    evaluator.latency_status == "clock_regression"
                    or timing.status is CommandPathLatencyStatus.CLOCK_REGRESSION
                )
                else CommandPathLatencyStatus.INVALID_TIMING
            )
            return self._report(
                evaluator,
                arbitration,
                latency_status=latency_status,
                detail=(
                    "Invalid timing in: " + ", ".join(invalid_sources) + "."
                ),
            )

        observation_ns = evaluator.observation_timestamp_ns
        evaluator_ns = evaluator.published_timestamp_ns
        intent_ns = timing.intent_received_ns
        command_ns = timing.command_published_ns
        assert observation_ns is not None
        assert evaluator_ns is not None
        assert intent_ns is not None
        assert command_ns is not None
        if not observation_ns <= evaluator_ns <= intent_ns <= command_ns:
            return self._report(
                evaluator,
                arbitration,
                latency_status=CommandPathLatencyStatus.CLOCK_REGRESSION,
                detail="Command-path timestamps must be monotonic.",
            )

        observation_to_command_ns = command_ns - observation_ns
        exceeded = [
            f"evaluator.{name}" for name in evaluator.exceeded_budgets
        ]
        if not timing.within_budget:
            exceeded.append("arbitration")
        if (
            observation_to_command_ns
            > self._config.observation_to_command_budget_ns
        ):
            exceeded.append("observation_to_command")
        return self._report(
            evaluator,
            arbitration,
            latency_status=(
                CommandPathLatencyStatus.BUDGET_EXCEEDED
                if exceeded
                else CommandPathLatencyStatus.WITHIN_BUDGET
            ),
            observation_to_command_ns=observation_to_command_ns,
            exceeded_budgets=tuple(exceeded),
        )

    def _report(
        self,
        evaluator: EvaluatorTimingSample,
        arbitration: ArbitrationTimingSample,
        *,
        latency_status: CommandPathLatencyStatus,
        observation_to_command_ns: int | None = None,
        exceeded_budgets: tuple[str, ...] = (),
        detail: str | None = None,
    ) -> CommandPathObservabilityReport:
        timing = arbitration.timing
        return CommandPathObservabilityReport(
            policy_version=self._config.policy_version,
            correlation_id=evaluator.correlation_id,
            decision_id=evaluator.decision_id,
            outcome=evaluator.outcome,
            arbitration_reason=arbitration.reason,
            arbitration_mode=arbitration.mode,
            command_sequence_id=arbitration.command_sequence_id,
            safety_intent_sequence_id=arbitration.safety_intent_sequence_id,
            latency_status=latency_status,
            observation_timestamp_ns=evaluator.observation_timestamp_ns,
            evaluator_published_timestamp_ns=evaluator.published_timestamp_ns,
            intent_received_timestamp_ns=timing.intent_received_ns,
            command_published_timestamp_ns=timing.command_published_ns,
            evaluator_end_to_end_ns=evaluator.end_to_end_ns,
            arbitration_latency_ns=timing.latency_ns,
            observation_to_command_ns=observation_to_command_ns,
            exceeded_budgets=exceeded_budgets,
            detail=detail,
        )
