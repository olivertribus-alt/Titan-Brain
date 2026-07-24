"""Dependency-free evaluator observability contract for TB-OBS-004."""

from __future__ import annotations

import hashlib
import json
from enum import StrEnum
from typing import Self

from pydantic import Field, model_validator

from core.types.incident import StrictFrozenModel


class EvaluationOutcome(StrEnum):
    """Normalized outcome used by metrics independently of transport names."""

    NORMAL = "normal"
    WARNING = "warning"
    E_STOP = "e_stop"
    REJECTED = "rejected"


class LatencyStatus(StrEnum):
    """Validity and budget state of one end-to-end timing sample."""

    WITHIN_BUDGET = "within_budget"
    BUDGET_EXCEEDED = "budget_exceeded"
    CLOCK_REGRESSION = "clock_regression"
    INVALID_TIMESTAMP = "invalid_timestamp"


class EvaluatorObservabilityConfig(StrictFrozenModel):
    """Versioned latency budgets for the evaluator pipeline."""

    policy_version: str = Field(default="TB-OBS-004-0.1.0", min_length=1)
    receive_to_decision_budget_ns: int = Field(default=50_000_000, gt=0)
    decision_to_publish_budget_ns: int = Field(default=20_000_000, gt=0)
    end_to_end_budget_ns: int = Field(default=70_000_000, gt=0)

    @model_validator(mode="after")
    def validate_end_to_end_budget(self) -> Self:
        """Ensure the aggregate budget covers both measured processing stages."""
        minimum = (
            self.receive_to_decision_budget_ns + self.decision_to_publish_budget_ns
        )
        if self.end_to_end_budget_ns < minimum:
            raise ValueError("end_to_end_budget_ns must cover both stage budgets")
        return self


DEFAULT_EVALUATOR_OBSERVABILITY_CONFIG = EvaluatorObservabilityConfig()


class EvaluationTimestamps(StrictFrozenModel):
    """Four clock readings that delimit the evaluator publishing pipeline."""

    observation_ns: object
    received_ns: object
    decided_ns: object
    published_ns: object


class EvaluationCounters(StrictFrozenModel):
    """Monotonic counters for evaluator outcomes and timing failures."""

    total: int = Field(ge=0)
    normal: int = Field(ge=0)
    warning: int = Field(ge=0)
    e_stop: int = Field(ge=0)
    rejected: int = Field(ge=0)
    budget_exceeded: int = Field(ge=0)
    invalid_timing: int = Field(ge=0)

    @model_validator(mode="after")
    def validate_totals(self) -> Self:
        """Keep the outcome partition internally consistent."""
        if self.total != self.normal + self.warning + self.e_stop + self.rejected:
            raise ValueError("total must equal the sum of outcome counters")
        if self.budget_exceeded + self.invalid_timing > self.total:
            raise ValueError("timing failure counters cannot exceed total")
        return self


class EvaluationObservabilityReport(StrictFrozenModel):
    """One audit-correlated latency sample with a cumulative counter snapshot."""

    schema_version: str = "0.1"
    policy_version: str = Field(min_length=1)
    correlation_id: str = Field(min_length=1)
    decision_id: str | None = None
    outcome: EvaluationOutcome
    latency_status: LatencyStatus
    observation_timestamp_ns: int | None = Field(default=None, ge=0)
    received_timestamp_ns: int | None = Field(default=None, ge=0)
    decision_timestamp_ns: int | None = Field(default=None, ge=0)
    published_timestamp_ns: int | None = Field(default=None, ge=0)
    observation_to_receive_ns: int | None = Field(default=None, ge=0)
    receive_to_decision_ns: int | None = Field(default=None, ge=0)
    decision_to_publish_ns: int | None = Field(default=None, ge=0)
    end_to_end_ns: int | None = Field(default=None, ge=0)
    exceeded_budgets: tuple[str, ...] = ()
    detail: str | None = None
    counters: EvaluationCounters

    @property
    def timing_valid(self) -> bool:
        """Return whether all timestamps formed a monotonic pipeline."""
        return self.latency_status in {
            LatencyStatus.WITHIN_BUDGET,
            LatencyStatus.BUDGET_EXCEEDED,
        }

    @property
    def within_budget(self) -> bool:
        """Return whether every configured latency budget was met."""
        return self.latency_status is LatencyStatus.WITHIN_BUDGET

    @model_validator(mode="after")
    def validate_shape(self) -> Self:
        """Prevent valid and invalid timing report shapes from being mixed."""
        latencies = (
            self.observation_to_receive_ns,
            self.receive_to_decision_ns,
            self.decision_to_publish_ns,
            self.end_to_end_ns,
        )
        timestamps = (
            self.observation_timestamp_ns,
            self.received_timestamp_ns,
            self.decision_timestamp_ns,
            self.published_timestamp_ns,
        )
        if self.timing_valid:
            if (
                any(value is None for value in latencies + timestamps)
                or self.detail is not None
            ):
                raise ValueError(
                    "valid timing requires all timestamps, latencies, and no detail"
                )
            observation_ns, received_ns, decided_ns, published_ns = timestamps
            assert observation_ns is not None
            assert received_ns is not None
            assert decided_ns is not None
            assert published_ns is not None
            expected_latencies = (
                received_ns - observation_ns,
                decided_ns - received_ns,
                published_ns - decided_ns,
                published_ns - observation_ns,
            )
            if expected_latencies != latencies or any(
                value < 0 for value in expected_latencies
            ):
                raise ValueError("latencies must match monotonic timestamps")
            if self.within_budget and self.exceeded_budgets:
                raise ValueError("within-budget timing cannot name exceeded budgets")
            if not self.within_budget and not self.exceeded_budgets:
                raise ValueError("budget failure must name an exceeded budget")
        elif (
            any(value is not None for value in latencies + timestamps)
            or self.detail is None
        ):
            raise ValueError(
                "invalid timing requires detail and no timestamps or latencies"
            )
        return self


def outcome_from_action(action: str | None, *, accepted: bool) -> EvaluationOutcome:
    """Map evaluator actions onto the stable observability outcome vocabulary."""
    if not accepted:
        return EvaluationOutcome.REJECTED
    mapping = {
        "proceed": EvaluationOutcome.NORMAL,
        "protective_stop": EvaluationOutcome.WARNING,
        "emergency_stop": EvaluationOutcome.E_STOP,
    }
    if action is None:
        raise ValueError("accepted evaluation requires a known action")
    try:
        return mapping[action]
    except KeyError as error:
        raise ValueError("accepted evaluation requires a known action") from error


def _checked_timestamp(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _correlation_id(
    *,
    received_ns: int | None,
    outcome: EvaluationOutcome,
    decision_id: str | None,
    sequence: int,
) -> str:
    canonical = json.dumps(
        {
            "decision_id": decision_id,
            "outcome": outcome.value,
            "received_ns": received_ns,
            "sequence": sequence,
        },
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return f"eval_{hashlib.sha256(canonical).hexdigest()[:16]}"


class EvaluatorObservability:
    """Stateful metrics collector with no ROS or actuator dependency."""

    def __init__(
        self,
        config: EvaluatorObservabilityConfig = (DEFAULT_EVALUATOR_OBSERVABILITY_CONFIG),
    ) -> None:
        self._config = config
        self._counters = EvaluationCounters(
            total=0,
            normal=0,
            warning=0,
            e_stop=0,
            rejected=0,
            budget_exceeded=0,
            invalid_timing=0,
        )
        self._last_published_ns: int | None = None

    @property
    def counters(self) -> EvaluationCounters:
        """Return the latest immutable counter snapshot."""
        return self._counters

    def record(
        self,
        timestamps: EvaluationTimestamps,
        *,
        outcome: EvaluationOutcome,
        decision_id: str | None = None,
    ) -> EvaluationObservabilityReport:
        """Validate, classify, and count one completed evaluation pipeline."""
        checked = tuple(
            _checked_timestamp(value)
            for value in (
                timestamps.observation_ns,
                timestamps.received_ns,
                timestamps.decided_ns,
                timestamps.published_ns,
            )
        )
        latency_status: LatencyStatus
        detail: str | None = None
        exceeded: tuple[str, ...] = ()
        latencies: tuple[int | None, int | None, int | None, int | None]
        if any(value is None for value in checked):
            latency_status = LatencyStatus.INVALID_TIMESTAMP
            detail = "All timestamps must be non-negative integers."
            latencies = (None, None, None, None)
        else:
            observation_ns, received_ns, decided_ns, published_ns = checked
            assert observation_ns is not None
            assert received_ns is not None
            assert decided_ns is not None
            assert published_ns is not None
            regressed = not (
                observation_ns <= received_ns <= decided_ns <= published_ns
            ) or (
                self._last_published_ns is not None
                and published_ns < self._last_published_ns
            )
            if regressed:
                latency_status = LatencyStatus.CLOCK_REGRESSION
                detail = "Timestamps must be monotonic within and across samples."
                latencies = (None, None, None, None)
            else:
                observation_to_receive = received_ns - observation_ns
                receive_to_decision = decided_ns - received_ns
                decision_to_publish = published_ns - decided_ns
                end_to_end = published_ns - observation_ns
                exceeded_list: list[str] = []
                if receive_to_decision > self._config.receive_to_decision_budget_ns:
                    exceeded_list.append("receive_to_decision")
                if decision_to_publish > self._config.decision_to_publish_budget_ns:
                    exceeded_list.append("decision_to_publish")
                if end_to_end > self._config.end_to_end_budget_ns:
                    exceeded_list.append("end_to_end")
                exceeded = tuple(exceeded_list)
                latency_status = (
                    LatencyStatus.BUDGET_EXCEEDED
                    if exceeded
                    else LatencyStatus.WITHIN_BUDGET
                )
                latencies = (
                    observation_to_receive,
                    receive_to_decision,
                    decision_to_publish,
                    end_to_end,
                )
                self._last_published_ns = published_ns

        increments = {
            EvaluationOutcome.NORMAL: (1, 0, 0, 0),
            EvaluationOutcome.WARNING: (0, 1, 0, 0),
            EvaluationOutcome.E_STOP: (0, 0, 1, 0),
            EvaluationOutcome.REJECTED: (0, 0, 0, 1),
        }[outcome]
        self._counters = EvaluationCounters(
            total=self._counters.total + 1,
            normal=self._counters.normal + increments[0],
            warning=self._counters.warning + increments[1],
            e_stop=self._counters.e_stop + increments[2],
            rejected=self._counters.rejected + increments[3],
            budget_exceeded=(
                self._counters.budget_exceeded
                + (latency_status is LatencyStatus.BUDGET_EXCEEDED)
            ),
            invalid_timing=(
                self._counters.invalid_timing
                + (
                    latency_status
                    in {
                        LatencyStatus.CLOCK_REGRESSION,
                        LatencyStatus.INVALID_TIMESTAMP,
                    }
                )
            ),
        )
        correlation_id = _correlation_id(
            received_ns=checked[1],
            outcome=outcome,
            decision_id=decision_id,
            sequence=self._counters.total,
        )
        return EvaluationObservabilityReport(
            policy_version=self._config.policy_version,
            correlation_id=correlation_id,
            decision_id=decision_id,
            outcome=outcome,
            latency_status=latency_status,
            observation_timestamp_ns=(checked[0] if latencies[0] is not None else None),
            received_timestamp_ns=(checked[1] if latencies[0] is not None else None),
            decision_timestamp_ns=(checked[2] if latencies[0] is not None else None),
            published_timestamp_ns=(checked[3] if latencies[0] is not None else None),
            observation_to_receive_ns=latencies[0],
            receive_to_decision_ns=latencies[1],
            decision_to_publish_ns=latencies[2],
            end_to_end_ns=latencies[3],
            exceeded_budgets=exceeded,
            detail=detail,
            counters=self._counters,
        )
