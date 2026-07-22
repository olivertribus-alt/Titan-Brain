"""Fault-injection acceptance matrix for TB-ACT-001D."""

from __future__ import annotations

import pytest

from core.actuator_fault_injection import (
    FaultInjectionCase,
    FaultInjectionEvent,
    FaultInjectionScenario,
    run_fault_injection,
    standard_fault_cases,
)
from core.actuator_feedback import ActuatorFeedbackConfig
from core.stop_ack_monitor import (
    StopAckReason,
    StopMonitorConfig,
    StopMonitorState,
    StopRequest,
)


@pytest.fixture
def config() -> StopMonitorConfig:
    return StopMonitorConfig(
        stop_budget_ns=100,
        feedback_config=ActuatorFeedbackConfig(
            epsilon_stop_linear=0.01,
            epsilon_stop_angular=0.02,
            stale_threshold_ns=50,
        ),
    )


@pytest.mark.parametrize(
    "case", standard_fault_cases(), ids=lambda case: case.scenario
)
def test_standard_fault_matrix_latches_fail_closed(
    case: FaultInjectionCase,
    config: StopMonitorConfig,
) -> None:
    report = run_fault_injection(case, config=config)

    assert report.scenario is case.scenario
    assert report.final_state is StopMonitorState.HARDWARE_FAULT_LATCH
    assert report.final_reason is case.expected_reason
    assert report.is_latched is True
    assert report.events_processed == len(case.events)


def test_standard_matrix_covers_all_required_faults() -> None:
    assert {case.scenario for case in standard_fault_cases()} == {
        FaultInjectionScenario.ENCODER_DRIFT,
        FaultInjectionScenario.FROZEN_FEEDBACK,
        FaultInjectionScenario.CORRELATION_DESYNC,
        FaultInjectionScenario.SEQUENCE_REPLAY,
        FaultInjectionScenario.SEQUENCE_GAP,
        FaultInjectionScenario.NON_FINITE_FEEDBACK,
        FaultInjectionScenario.SPURIOUS_MOVEMENT,
    }


def test_event_contract_rejects_negative_and_boolean_timestamps() -> None:
    with pytest.raises(ValueError, match="non-negative integer"):
        FaultInjectionEvent(now_ns=-1, payload=None)
    with pytest.raises(ValueError, match="non-negative integer"):
        FaultInjectionEvent(now_ns=True, payload=None)


def test_case_contract_rejects_empty_event_stream() -> None:
    request = StopRequest(
        correlation_id="act-001d",
        sequence_id=1,
        requested_timestamp_ns=100,
    )

    with pytest.raises(ValueError, match="at least one event"):
        FaultInjectionCase(
            scenario=FaultInjectionScenario.FROZEN_FEEDBACK,
            request=request,
            events=(),
            expected_reason=StopAckReason.STALE_FEEDBACK,
        )


def test_infinite_measurement_is_fail_closed(config: StopMonitorConfig) -> None:
    case = FaultInjectionCase(
        scenario=FaultInjectionScenario.NON_FINITE_FEEDBACK,
        request=StopRequest(
            correlation_id="act-001d",
            sequence_id=1,
            requested_timestamp_ns=100,
        ),
        events=(
            FaultInjectionEvent(
                now_ns=110,
                payload={
                    "correlation_id": "act-001d",
                    "sequence_id": 1,
                    "timestamp_ns": 110,
                    "measured_linear_x": 0.0,
                    "measured_linear_y": 0.0,
                    "measured_angular_z": float("-inf"),
                },
            ),
        ),
        expected_reason=StopAckReason.INVALID_FEEDBACK,
    )

    report = run_fault_injection(case, config=config)

    assert report.final_reason is StopAckReason.INVALID_FEEDBACK
    assert report.is_latched is True


def test_report_is_immutable(config: StopMonitorConfig) -> None:
    report = run_fault_injection(standard_fault_cases()[0], config=config)

    assert report.scenario.value == "encoder_drift"
    with pytest.raises(AttributeError):
        report.is_latched = False  # type: ignore[misc]
