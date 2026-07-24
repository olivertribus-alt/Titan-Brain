"""Tests for TB-EVAL-004D command-path audit correlation."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from core.command_observability import (
    ArbitrationLatencyMeasurement,
    ArbitrationTimingSample,
    CommandPathLatencyStatus,
    CommandPathObservability,
    CommandPathObservabilityConfig,
    CommandPathObservabilityReport,
    EvaluatorTimingSample,
    measure_arbitration_latency,
)


def _config(
    *,
    arbitration_budget_ns: int = 30,
    observation_to_command_budget_ns: int = 100,
    max_correlations: int = 4,
    max_pending_per_correlation: int = 3,
) -> CommandPathObservabilityConfig:
    return CommandPathObservabilityConfig(
        arbitration_budget_ns=arbitration_budget_ns,
        observation_to_command_budget_ns=observation_to_command_budget_ns,
        max_correlations=max_correlations,
        max_pending_per_correlation=max_pending_per_correlation,
    )


def _evaluator(
    correlation_id: str = "eval_001",
    *,
    observation_ns: int = 100,
    published_ns: int = 140,
    exceeded_budgets: tuple[str, ...] = (),
) -> EvaluatorTimingSample:
    return EvaluatorTimingSample(
        correlation_id=correlation_id,
        decision_id="decision_001",
        outcome="normal",
        latency_status=("budget_exceeded" if exceeded_budgets else "within_budget"),
        timing_valid=True,
        observation_timestamp_ns=observation_ns,
        published_timestamp_ns=published_ns,
        end_to_end_ns=published_ns - observation_ns,
        exceeded_budgets=exceeded_budgets,
    )


def _arbitration(
    correlation_id: str = "eval_001",
    *,
    received_ns: int = 145,
    published_ns: int = 160,
    budget_ns: int = 30,
    reason: str = "proceed",
    mode: int = 0,
) -> ArbitrationTimingSample:
    return ArbitrationTimingSample(
        correlation_id=correlation_id,
        reason=reason,
        mode=mode,
        command_sequence_id=2,
        safety_intent_sequence_id=1,
        timing=measure_arbitration_latency(
            intent_received_ns=received_ns,
            command_published_ns=published_ns,
            budget_ns=budget_ns,
        ),
    )


def test_arbitration_measurement_is_exact_and_budget_boundary_is_inclusive() -> None:
    boundary = measure_arbitration_latency(
        intent_received_ns=100,
        command_published_ns=130,
        budget_ns=30,
    )
    exceeded = measure_arbitration_latency(
        intent_received_ns=100,
        command_published_ns=131,
        budget_ns=30,
    )

    assert boundary.status is CommandPathLatencyStatus.WITHIN_BUDGET
    assert boundary.timing_valid is True
    assert boundary.within_budget is True
    assert boundary.latency_ns == 30
    assert exceeded.status is CommandPathLatencyStatus.BUDGET_EXCEEDED
    assert exceeded.timing_valid is True
    assert exceeded.within_budget is False
    assert exceeded.latency_ns == 31


@pytest.mark.parametrize(
    ("received", "published"),
    [(-1, 10), (True, 10), (10, 1.5), (10, "20")],
)
def test_invalid_arbitration_timestamps_are_controlled(
    received: object,
    published: object,
) -> None:
    measurement = measure_arbitration_latency(
        intent_received_ns=received,
        command_published_ns=published,
        budget_ns=10,
    )

    assert measurement.status is CommandPathLatencyStatus.INVALID_TIMING
    assert measurement.timing_valid is False
    assert measurement.latency_ns is None
    assert measurement.detail is not None


@pytest.mark.parametrize("budget", [0, -1, True, 1.5])
def test_arbitration_budget_must_be_a_positive_integer(budget: object) -> None:
    with pytest.raises(ValueError, match="positive integer"):
        measure_arbitration_latency(
            intent_received_ns=1,
            command_published_ns=2,
            budget_ns=budget,  # type: ignore[arg-type]
        )


def test_arbitration_clock_regression_is_fail_visible() -> None:
    measurement = measure_arbitration_latency(
        intent_received_ns=11,
        command_published_ns=10,
        budget_ns=10,
    )

    assert measurement.status is CommandPathLatencyStatus.CLOCK_REGRESSION
    assert measurement.timing_valid is False
    assert measurement.intent_received_ns == 11
    assert measurement.command_published_ns == 10


def test_evaluator_first_correlation_builds_exact_command_path() -> None:
    observability = CommandPathObservability(_config())
    assert observability.config.policy_version == "TB-EVAL-004D-0.1.0"
    assert observability.record_evaluator(_evaluator()) == ()

    reports = observability.record_arbitration(_arbitration())

    assert len(reports) == 1
    report = reports[0]
    assert report.correlation_id == "eval_001"
    assert report.decision_id == "decision_001"
    assert report.timing_valid is True
    assert report.within_budget is True
    assert report.evaluator_end_to_end_ns == 40
    assert report.arbitration_latency_ns == 15
    assert report.observation_to_command_ns == 60
    assert report.command_sequence_id == 2
    assert report.safety_intent_sequence_id == 1


def test_arbitration_first_delivery_resolves_all_bounded_pending_samples() -> None:
    observability = CommandPathObservability(_config(max_pending_per_correlation=2))
    assert observability.record_arbitration(_arbitration(published_ns=150)) == ()
    assert observability.record_arbitration(_arbitration(published_ns=155)) == ()
    assert observability.record_arbitration(_arbitration(published_ns=160)) == ()

    reports = observability.record_evaluator(_evaluator())

    assert [report.command_published_timestamp_ns for report in reports] == [
        155,
        160,
    ]


def test_all_exceeded_budgets_are_namespaced_and_preserved() -> None:
    observability = CommandPathObservability(
        _config(
            arbitration_budget_ns=10,
            observation_to_command_budget_ns=50,
        )
    )
    observability.record_evaluator(
        _evaluator(exceeded_budgets=("receive_to_decision", "end_to_end"))
    )

    report = observability.record_arbitration(_arbitration(budget_ns=10))[0]

    assert report.latency_status is CommandPathLatencyStatus.BUDGET_EXCEEDED
    assert report.within_budget is False
    assert report.exceeded_budgets == (
        "evaluator.receive_to_decision",
        "evaluator.end_to_end",
        "arbitration",
        "observation_to_command",
    )


def test_invalid_component_timing_keeps_available_stage_evidence() -> None:
    observability = CommandPathObservability(_config())
    invalid_evaluator = EvaluatorTimingSample(
        correlation_id="eval_invalid",
        decision_id="none",
        outcome="rejected",
        latency_status="invalid_timestamp",
        timing_valid=False,
        detail="invalid evaluator timestamp",
    )
    observability.record_evaluator(invalid_evaluator)

    report = observability.record_arbitration(_arbitration("eval_invalid"))[0]

    assert report.latency_status is CommandPathLatencyStatus.INVALID_TIMING
    assert report.timing_valid is False
    assert report.arbitration_latency_ns == 15
    assert report.observation_to_command_ns is None
    assert report.detail == "Invalid timing in: evaluator."


def test_component_and_cross_pipeline_clock_regressions_are_distinct() -> None:
    component = CommandPathObservability(_config())
    component.record_evaluator(_evaluator())
    component_report = component.record_arbitration(
        _arbitration(received_ns=170, published_ns=160)
    )[0]

    cross_pipeline = CommandPathObservability(_config())
    cross_pipeline.record_evaluator(_evaluator())
    cross_report = cross_pipeline.record_arbitration(
        _arbitration(received_ns=130, published_ns=160)
    )[0]

    assert component_report.latency_status is CommandPathLatencyStatus.CLOCK_REGRESSION
    assert cross_report.latency_status is CommandPathLatencyStatus.CLOCK_REGRESSION
    assert cross_report.detail == "Command-path timestamps must be monotonic."


def test_mutation_and_correlation_storage_are_bounded_fail_closed() -> None:
    observability = CommandPathObservability(_config(max_correlations=1))
    original = _evaluator("eval_original")
    observability.record_evaluator(original)
    observability.record_evaluator(original)
    with pytest.raises(ValueError, match="mutated evaluator"):
        observability.record_evaluator(
            original.model_copy(update={"outcome": "warning"})
        )

    observability.record_evaluator(_evaluator("eval_new"))
    assert observability.record_arbitration(_arbitration("eval_original")) == ()

    pending = CommandPathObservability(_config(max_correlations=1))
    pending.record_arbitration(_arbitration("pending_old"))
    pending.record_arbitration(_arbitration("pending_new"))
    assert pending.record_evaluator(_evaluator("pending_old")) == ()


def test_models_reject_incoherent_manual_shapes() -> None:
    with pytest.raises(ValidationError, match="both timestamps"):
        ArbitrationLatencyMeasurement(
            status=CommandPathLatencyStatus.WITHIN_BUDGET,
            budget_ns=10,
        )
    with pytest.raises(ValidationError, match="publish minus intent"):
        ArbitrationLatencyMeasurement(
            status=CommandPathLatencyStatus.WITHIN_BUDGET,
            intent_received_ns=1,
            command_published_ns=3,
            latency_ns=1,
            budget_ns=10,
        )
    with pytest.raises(ValidationError, match="configured budget"):
        ArbitrationLatencyMeasurement(
            status=CommandPathLatencyStatus.BUDGET_EXCEEDED,
            intent_received_ns=1,
            command_published_ns=2,
            latency_ns=1,
            budget_ns=10,
        )
    with pytest.raises(ValidationError, match="detail and no latency"):
        ArbitrationLatencyMeasurement(
            status=CommandPathLatencyStatus.INVALID_TIMING,
            latency_ns=1,
            budget_ns=10,
        )
    with pytest.raises(ValidationError, match="both endpoints"):
        EvaluatorTimingSample(
            correlation_id="eval_bad",
            decision_id="decision",
            outcome="normal",
            latency_status="within_budget",
            timing_valid=True,
        )
    with pytest.raises(ValidationError, match="match its timestamps"):
        EvaluatorTimingSample(
            correlation_id="eval_bad",
            decision_id="decision",
            outcome="normal",
            latency_status="within_budget",
            timing_valid=True,
            observation_timestamp_ns=100,
            published_timestamp_ns=140,
            end_to_end_ns=39,
        )
    with pytest.raises(ValidationError, match="requires a valid status"):
        EvaluatorTimingSample(
            correlation_id="eval_bad",
            decision_id="decision",
            outcome="normal",
            latency_status="invalid_timestamp",
            timing_valid=True,
            observation_timestamp_ns=100,
            published_timestamp_ns=140,
            end_to_end_ns=40,
        )
    with pytest.raises(ValidationError, match="match exceeded budgets"):
        EvaluatorTimingSample(
            correlation_id="eval_bad",
            decision_id="decision",
            outcome="normal",
            latency_status="within_budget",
            timing_valid=True,
            observation_timestamp_ns=100,
            published_timestamp_ns=140,
            end_to_end_ns=40,
            exceeded_budgets=("end_to_end",),
        )
    with pytest.raises(ValidationError, match="only detail"):
        EvaluatorTimingSample(
            correlation_id="eval_bad",
            decision_id="decision",
            outcome="normal",
            latency_status="invalid_timestamp",
            timing_valid=False,
            observation_timestamp_ns=1,
            detail="invalid",
        )
    with pytest.raises(ValidationError, match="requires an invalid status"):
        EvaluatorTimingSample(
            correlation_id="eval_bad",
            decision_id="decision",
            outcome="normal",
            latency_status="within_budget",
            timing_valid=False,
            detail="invalid",
        )
    with pytest.raises(ValidationError, match="complete chain"):
        CommandPathObservabilityReport(
            policy_version="policy",
            correlation_id="eval_bad",
            decision_id="decision",
            outcome="normal",
            arbitration_reason="proceed",
            arbitration_mode=0,
            command_sequence_id=1,
            safety_intent_sequence_id=1,
            latency_status=CommandPathLatencyStatus.WITHIN_BUDGET,
        )
    with pytest.raises(ValidationError, match="detail and no total"):
        CommandPathObservabilityReport(
            policy_version="policy",
            correlation_id="eval_bad",
            decision_id="decision",
            outcome="normal",
            arbitration_reason="proceed",
            arbitration_mode=0,
            command_sequence_id=1,
            safety_intent_sequence_id=1,
            latency_status=CommandPathLatencyStatus.INVALID_TIMING,
            observation_to_command_ns=1,
        )
    valid_report = {
        "policy_version": "policy",
        "correlation_id": "eval_bad",
        "decision_id": "decision",
        "outcome": "normal",
        "arbitration_reason": "proceed",
        "arbitration_mode": 0,
        "command_sequence_id": 2,
        "safety_intent_sequence_id": 1,
        "latency_status": CommandPathLatencyStatus.WITHIN_BUDGET,
        "observation_timestamp_ns": 100,
        "evaluator_published_timestamp_ns": 140,
        "intent_received_timestamp_ns": 145,
        "command_published_timestamp_ns": 160,
        "evaluator_end_to_end_ns": 40,
        "arbitration_latency_ns": 15,
        "observation_to_command_ns": 60,
    }
    with pytest.raises(ValidationError, match="must be monotonic"):
        CommandPathObservabilityReport.model_validate(
            {
                **valid_report,
                "intent_received_timestamp_ns": 130,
                "arbitration_latency_ns": 30,
            }
        )
    with pytest.raises(ValidationError, match="must match timestamps"):
        CommandPathObservabilityReport.model_validate(
            {**valid_report, "observation_to_command_ns": 59}
        )
    with pytest.raises(ValidationError, match="cannot name failures"):
        CommandPathObservabilityReport.model_validate(
            {**valid_report, "exceeded_budgets": ("arbitration",)}
        )
    with pytest.raises(ValidationError, match="must name an exceeded"):
        CommandPathObservabilityReport.model_validate(
            {
                **valid_report,
                "latency_status": CommandPathLatencyStatus.BUDGET_EXCEEDED,
            }
        )
