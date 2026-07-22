"""Tests for the TB-OBS-004 evaluator observability contract."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from core.adapters.ros_diagnostics import to_ros_evaluator_diagnostic
from core.observability import (
    EvaluationCounters,
    EvaluationObservabilityReport,
    EvaluationOutcome,
    EvaluationTimestamps,
    EvaluatorObservability,
    EvaluatorObservabilityConfig,
    LatencyStatus,
    outcome_from_action,
)


def _timestamps(
    *,
    observation_ns: object = 100,
    received_ns: object = 110,
    decided_ns: object = 130,
    published_ns: object = 140,
) -> EvaluationTimestamps:
    return EvaluationTimestamps(
        observation_ns=observation_ns,
        received_ns=received_ns,
        decided_ns=decided_ns,
        published_ns=published_ns,
    )


def _collector() -> EvaluatorObservability:
    return EvaluatorObservability(
        EvaluatorObservabilityConfig(
            receive_to_decision_budget_ns=20,
            decision_to_publish_budget_ns=10,
            end_to_end_budget_ns=40,
        )
    )


def test_valid_pipeline_records_exact_latencies_and_audit_correlation() -> None:
    collector = _collector()
    assert collector.counters.total == 0
    report = collector.record(
        _timestamps(),
        outcome=EvaluationOutcome.NORMAL,
        decision_id="decision_123",
    )

    assert report.latency_status is LatencyStatus.WITHIN_BUDGET
    assert report.timing_valid is True
    assert report.within_budget is True
    assert report.observation_timestamp_ns == 100
    assert report.received_timestamp_ns == 110
    assert report.decision_timestamp_ns == 130
    assert report.published_timestamp_ns == 140
    assert report.observation_to_receive_ns == 10
    assert report.receive_to_decision_ns == 20
    assert report.decision_to_publish_ns == 10
    assert report.end_to_end_ns == 40
    assert report.correlation_id.startswith("eval_")
    assert report.decision_id == "decision_123"
    assert report.counters == EvaluationCounters(
        total=1,
        normal=1,
        warning=0,
        e_stop=0,
        rejected=0,
        budget_exceeded=0,
        invalid_timing=0,
    )


def test_each_budget_boundary_is_inclusive() -> None:
    report = _collector().record(
        _timestamps(),
        outcome=EvaluationOutcome.WARNING,
    )

    assert report.within_budget is True
    assert report.exceeded_budgets == ()


def test_exceeded_budgets_are_named_and_counted_once_per_sample() -> None:
    report = _collector().record(
        _timestamps(decided_ns=131, published_ns=142),
        outcome=EvaluationOutcome.E_STOP,
    )

    assert report.latency_status is LatencyStatus.BUDGET_EXCEEDED
    assert report.exceeded_budgets == (
        "receive_to_decision",
        "decision_to_publish",
        "end_to_end",
    )
    assert report.counters.e_stop == 1
    assert report.counters.budget_exceeded == 1
    assert report.counters.invalid_timing == 0


@pytest.mark.parametrize(
    "timestamps",
    [
        _timestamps(observation_ns=-1),
        _timestamps(received_ns=True),
        _timestamps(decided_ns=1.5),
        _timestamps(published_ns="140"),
        _timestamps(received_ns=object()),
    ],
)
def test_invalid_timestamps_are_controlled_diagnostics(
    timestamps: EvaluationTimestamps,
) -> None:
    report = _collector().record(
        timestamps,
        outcome=EvaluationOutcome.REJECTED,
    )

    assert report.latency_status is LatencyStatus.INVALID_TIMESTAMP
    assert report.timing_valid is False
    assert report.detail == "All timestamps must be non-negative integers."
    assert report.end_to_end_ns is None
    assert report.counters.rejected == 1
    assert report.counters.invalid_timing == 1


def test_intra_sample_clock_regression_is_fail_visible() -> None:
    report = _collector().record(
        _timestamps(decided_ns=109),
        outcome=EvaluationOutcome.NORMAL,
    )

    assert report.latency_status is LatencyStatus.CLOCK_REGRESSION
    assert report.detail is not None
    assert report.counters.normal == 1
    assert report.counters.invalid_timing == 1


def test_cross_sample_clock_regression_is_detected_without_poisoning_clock() -> None:
    collector = _collector()
    first = collector.record(_timestamps(), outcome=EvaluationOutcome.NORMAL)
    regressed = collector.record(
        _timestamps(
            observation_ns=80,
            received_ns=90,
            decided_ns=100,
            published_ns=120,
        ),
        outcome=EvaluationOutcome.WARNING,
    )
    recovered = collector.record(
        _timestamps(
            observation_ns=140,
            received_ns=150,
            decided_ns=160,
            published_ns=170,
        ),
        outcome=EvaluationOutcome.E_STOP,
    )

    assert first.latency_status is LatencyStatus.WITHIN_BUDGET
    assert regressed.latency_status is LatencyStatus.CLOCK_REGRESSION
    assert recovered.latency_status is LatencyStatus.WITHIN_BUDGET
    assert recovered.counters.total == 3
    assert recovered.counters.normal == 1
    assert recovered.counters.warning == 1
    assert recovered.counters.e_stop == 1


@pytest.mark.parametrize(
    ("action", "accepted", "expected"),
    [
        ("proceed", True, EvaluationOutcome.NORMAL),
        ("protective_stop", True, EvaluationOutcome.WARNING),
        ("emergency_stop", True, EvaluationOutcome.E_STOP),
        (None, False, EvaluationOutcome.REJECTED),
    ],
)
def test_evaluator_actions_map_to_stable_metric_outcomes(
    action: str | None,
    accepted: bool,
    expected: EvaluationOutcome,
) -> None:
    assert outcome_from_action(action, accepted=accepted) is expected


def test_unknown_accepted_action_is_rejected() -> None:
    with pytest.raises(ValueError, match="known action"):
        outcome_from_action("reverse", accepted=True)
    with pytest.raises(ValueError, match="known action"):
        outcome_from_action(None, accepted=True)


def test_ros_projection_uses_explicit_validity_flags_for_optional_latencies() -> None:
    collector = _collector()
    valid = to_ros_evaluator_diagnostic(
        collector.record(_timestamps(), outcome=EvaluationOutcome.NORMAL)
    )
    invalid = to_ros_evaluator_diagnostic(
        collector.record(
            _timestamps(received_ns=True),
            outcome=EvaluationOutcome.REJECTED,
        )
    )

    assert valid.timing_valid is True
    assert valid.observation_timestamp_ns == 100
    assert valid.published_timestamp_ns == 140
    assert valid.end_to_end_ns == 40
    assert valid.normal_count == 1
    assert invalid.timing_valid is False
    assert invalid.observation_timestamp_ns == 0
    assert invalid.published_timestamp_ns == 0
    assert invalid.end_to_end_ns == 0
    assert invalid.rejected_count == 1
    assert invalid.invalid_timing_count == 1


def test_config_and_report_models_reject_incoherent_shapes() -> None:
    with pytest.raises(ValidationError, match="cover both stage budgets"):
        EvaluatorObservabilityConfig(
            receive_to_decision_budget_ns=20,
            decision_to_publish_budget_ns=10,
            end_to_end_budget_ns=29,
        )
    with pytest.raises(ValidationError, match="sum of outcome counters"):
        EvaluationCounters(
            total=1,
            normal=0,
            warning=0,
            e_stop=0,
            rejected=0,
            budget_exceeded=0,
            invalid_timing=0,
        )
    with pytest.raises(ValidationError, match="cannot exceed total"):
        EvaluationCounters(
            total=1,
            normal=1,
            warning=0,
            e_stop=0,
            rejected=0,
            budget_exceeded=1,
            invalid_timing=1,
        )
    counters = EvaluationCounters(
        total=1,
        normal=1,
        warning=0,
        e_stop=0,
        rejected=0,
        budget_exceeded=0,
        invalid_timing=0,
    )
    with pytest.raises(ValidationError, match="all timestamps, latencies"):
        EvaluationObservabilityReport(
            policy_version="TB-OBS-004-0.1.0",
            correlation_id="eval_1",
            outcome=EvaluationOutcome.NORMAL,
            latency_status=LatencyStatus.WITHIN_BUDGET,
            counters=counters,
        )
    with pytest.raises(ValidationError, match="cannot name exceeded"):
        EvaluationObservabilityReport(
            policy_version="TB-OBS-004-0.1.0",
            correlation_id="eval_2",
            outcome=EvaluationOutcome.NORMAL,
            latency_status=LatencyStatus.WITHIN_BUDGET,
            observation_timestamp_ns=0,
            received_timestamp_ns=1,
            decision_timestamp_ns=2,
            published_timestamp_ns=3,
            observation_to_receive_ns=1,
            receive_to_decision_ns=1,
            decision_to_publish_ns=1,
            end_to_end_ns=3,
            exceeded_budgets=("end_to_end",),
            counters=counters,
        )
    with pytest.raises(ValidationError, match="must name an exceeded"):
        EvaluationObservabilityReport(
            policy_version="TB-OBS-004-0.1.0",
            correlation_id="eval_3",
            outcome=EvaluationOutcome.NORMAL,
            latency_status=LatencyStatus.BUDGET_EXCEEDED,
            observation_timestamp_ns=0,
            received_timestamp_ns=1,
            decision_timestamp_ns=2,
            published_timestamp_ns=3,
            observation_to_receive_ns=1,
            receive_to_decision_ns=1,
            decision_to_publish_ns=1,
            end_to_end_ns=3,
            counters=counters,
        )
    with pytest.raises(ValidationError, match="no timestamps or latencies"):
        EvaluationObservabilityReport(
            policy_version="TB-OBS-004-0.1.0",
            correlation_id="eval_4",
            outcome=EvaluationOutcome.NORMAL,
            latency_status=LatencyStatus.CLOCK_REGRESSION,
            observation_to_receive_ns=1,
            detail="clock regressed",
            counters=counters,
        )
    with pytest.raises(ValidationError, match="must match monotonic timestamps"):
        EvaluationObservabilityReport(
            policy_version="TB-OBS-004-0.1.0",
            correlation_id="eval_5",
            outcome=EvaluationOutcome.NORMAL,
            latency_status=LatencyStatus.WITHIN_BUDGET,
            observation_timestamp_ns=0,
            received_timestamp_ns=1,
            decision_timestamp_ns=2,
            published_timestamp_ns=3,
            observation_to_receive_ns=1,
            receive_to_decision_ns=1,
            decision_to_publish_ns=1,
            end_to_end_ns=4,
            counters=counters,
        )
