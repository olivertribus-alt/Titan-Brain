"""TB-EVAL-006C deterministic governor fault-injection acceptance tests."""

from __future__ import annotations

import math

import pytest

from core.command_governor import GovernorConfig, GovernorReason
from core.command_governor_fault_injection import (
    GovernorFaultCase,
    GovernorFaultEvent,
    GovernorFaultReport,
    GovernorFaultScenario,
    run_fault_injection,
    standard_fault_cases,
)


@pytest.mark.parametrize(
    "case",
    standard_fault_cases(),
    ids=lambda case: case.scenario.value,
)
def test_standard_matrix_passes(case: GovernorFaultCase) -> None:
    report = run_fault_injection(case)

    assert report.scenario is case.scenario
    assert report.passed is True
    assert report.events_processed == len(case.events)
    assert report.final_result is report.results[-1]


def test_standard_matrix_contains_six_required_scenarios() -> None:
    assert {case.scenario for case in standard_fault_cases()} == {
        GovernorFaultScenario.JERK_LIMIT,
        GovernorFaultScenario.ASYMMETRIC_RAMP,
        GovernorFaultScenario.EMERGENCY_CUTOFF,
        GovernorFaultScenario.STALE_COMMAND,
        GovernorFaultScenario.SAFETY_TIMEOUT,
        GovernorFaultScenario.INVALID_INPUT,
    }


def test_jerk_metric_is_bounded_by_configured_limit() -> None:
    case = standard_fault_cases()[0]
    report = run_fault_injection(case)

    assert report.max_observed_jerk_mps3 <= 5.0 + 1e-9
    assert report.final_result.linear_velocity_mps > 0.0


def test_asymmetric_profile_uses_stronger_braking_limit() -> None:
    report = run_fault_injection(standard_fault_cases()[1])

    assert report.max_positive_acceleration_mps2 == pytest.approx(1.0)
    assert report.max_braking_acceleration_mps2 == pytest.approx(2.0)
    assert report.final_result.linear_velocity_mps == 0.0


@pytest.mark.parametrize(
    "scenario",
    (
        GovernorFaultScenario.EMERGENCY_CUTOFF,
        GovernorFaultScenario.STALE_COMMAND,
        GovernorFaultScenario.SAFETY_TIMEOUT,
    ),
)
def test_stop_paths_are_immediate_and_zero(scenario: GovernorFaultScenario) -> None:
    case = next(case for case in standard_fault_cases() if case.scenario is scenario)
    report = run_fault_injection(case)

    assert report.final_result.reason is GovernorReason.EMERGENCY_STOP
    assert report.final_result.emergency_override is True
    assert report.final_result.linear_velocity_mps == 0.0
    assert report.final_result.angular_velocity_radps == 0.0


def test_invalid_input_is_fail_closed() -> None:
    report = run_fault_injection(standard_fault_cases()[-1])

    assert report.final_result.reason is GovernorReason.INVALID_COMMAND
    assert report.final_result.is_safe is False
    assert report.final_result.linear_velocity_mps == 0.0


def test_event_rejects_negative_and_boolean_timestamps() -> None:
    with pytest.raises(ValueError, match="non-negative integer"):
        GovernorFaultEvent(timestamp_ns=-1, command=None)
    with pytest.raises(ValueError, match="non-negative integer"):
        GovernorFaultEvent(timestamp_ns=True, command=None)


def test_case_rejects_empty_or_invalid_events() -> None:
    with pytest.raises(ValueError, match="contain events"):
        GovernorFaultCase(
            scenario=GovernorFaultScenario.JERK_LIMIT,
            config=GovernorConfig(),
            events=(),
        )
    with pytest.raises(ValueError, match="scenario"):
        GovernorFaultCase(
            scenario="jerk_limit",  # type: ignore[arg-type]
            config=GovernorConfig(),
            events=(GovernorFaultEvent(timestamp_ns=1, command=None),),
        )
    with pytest.raises(ValueError, match="config"):
        GovernorFaultCase(
            scenario=GovernorFaultScenario.JERK_LIMIT,
            config=object(),  # type: ignore[arg-type]
            events=(GovernorFaultEvent(timestamp_ns=1, command=None),),
        )
    with pytest.raises(ValueError, match="events"):
        GovernorFaultCase(
            scenario=GovernorFaultScenario.JERK_LIMIT,
            config=GovernorConfig(),
            events=(object(),),  # type: ignore[arg-type]
        )


def test_timeout_empty_command_is_rejected_for_non_timeout_case() -> None:
    case = GovernorFaultCase(
        scenario=GovernorFaultScenario.JERK_LIMIT,
        config=GovernorConfig(),
        events=(GovernorFaultEvent(timestamp_ns=1, command=None),),
    )

    with pytest.raises(ValueError, match="timeout scenarios"):
        run_fault_injection(case)


def test_report_is_immutable_and_rejects_invalid_report_data() -> None:
    report = run_fault_injection(standard_fault_cases()[0])
    with pytest.raises(AttributeError):
        report.passed = False  # type: ignore[misc]

    with pytest.raises(ValueError, match="at least one result"):
        GovernorFaultReport(
            scenario=GovernorFaultScenario.JERK_LIMIT,
            results=(),
            max_observed_jerk_mps3=0.0,
            max_positive_acceleration_mps2=0.0,
            max_braking_acceleration_mps2=0.0,
            final_result=report.final_result,
            events_processed=0,
            passed=False,
        )

    with pytest.raises(ValueError, match="scenario"):
        GovernorFaultReport(
            scenario="jerk_limit",  # type: ignore[arg-type]
            results=report.results,
            max_observed_jerk_mps3=report.max_observed_jerk_mps3,
            max_positive_acceleration_mps2=report.max_positive_acceleration_mps2,
            max_braking_acceleration_mps2=report.max_braking_acceleration_mps2,
            final_result=report.final_result,
            events_processed=report.events_processed,
            passed=False,
        )
    with pytest.raises(ValueError, match="match result count"):
        GovernorFaultReport(
            scenario=report.scenario,
            results=report.results,
            max_observed_jerk_mps3=report.max_observed_jerk_mps3,
            max_positive_acceleration_mps2=report.max_positive_acceleration_mps2,
            max_braking_acceleration_mps2=report.max_braking_acceleration_mps2,
            final_result=report.final_result,
            events_processed=report.events_processed + 1,
            passed=False,
        )
    with pytest.raises(ValueError, match="last result"):
        GovernorFaultReport(
            scenario=report.scenario,
            results=report.results,
            max_observed_jerk_mps3=report.max_observed_jerk_mps3,
            max_positive_acceleration_mps2=report.max_positive_acceleration_mps2,
            max_braking_acceleration_mps2=report.max_braking_acceleration_mps2,
            final_result=report.final_result.model_copy(),
            events_processed=report.events_processed,
            passed=False,
        )


def test_non_finite_report_metric_is_rejected() -> None:
    report = run_fault_injection(standard_fault_cases()[0])

    with pytest.raises(ValueError, match="finite"):
        GovernorFaultReport(
            scenario=report.scenario,
            results=report.results,
            max_observed_jerk_mps3=math.inf,
            max_positive_acceleration_mps2=report.max_positive_acceleration_mps2,
            max_braking_acceleration_mps2=report.max_braking_acceleration_mps2,
            final_result=report.final_result,
            events_processed=report.events_processed,
            passed=False,
        )


def test_invalid_case_argument_is_rejected() -> None:
    with pytest.raises(ValueError, match="GovernorFaultCase"):
        run_fault_injection(object())  # type: ignore[arg-type]
