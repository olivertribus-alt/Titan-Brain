"""Fault-injection acceptance matrix for TB-SAFE-001D."""

from __future__ import annotations

import pytest

from core.safety_fault_injection import (
    RelayFaultMode,
    SafetyFaultCase,
    SafetyFaultScenario,
    SafetyRelayEmulator,
    run_safety_fault_injection,
    standard_fault_cases,
)
from core.safety_supervisor import (
    RelayRequest,
    SafetyReason,
    SafetyState,
    SafetySupervisorConfig,
)


@pytest.mark.parametrize(
    "case",
    standard_fault_cases(),
    ids=lambda case: case.scenario.value,
)
def test_standard_fault_matrix_is_fail_closed(case: SafetyFaultCase) -> None:
    report = run_safety_fault_injection(case)

    assert report.scenario is case.scenario
    assert report.final_state is case.expected_state
    assert report.final_reason is case.expected_reason
    assert report.is_latched is (
        case.expected_state is SafetyState.HARDWARE_FAULT_LATCH
    )
    assert report.events_processed > 0


def test_standard_matrix_covers_all_required_faults() -> None:
    assert {case.scenario for case in standard_fault_cases()} == {
        SafetyFaultScenario.MISSING_HEARTBEAT,
        SafetyFaultScenario.STALE_HEARTBEAT,
        SafetyFaultScenario.WELDED_RELAY_CONTACTS,
        SafetyFaultScenario.CLOCK_REGRESSION,
        SafetyFaultScenario.SEQUENCE_REPLAY,
        SafetyFaultScenario.UNAUTHORIZED_RESET,
    }


def test_relay_emulator_tracks_nominal_open_and_close_commands() -> None:
    relay = SafetyRelayEmulator()
    assert relay.apply(RelayRequest.REQUEST_SAFETY_OPEN) is False
    assert relay.apply(RelayRequest.REQUEST_SAFETY_CLOSED) is True


def test_welded_relay_ignores_open_request() -> None:
    relay = SafetyRelayEmulator(
        is_closed=True,
        fault_mode=RelayFaultMode.WELDED_CLOSED,
    )
    assert relay.apply(RelayRequest.REQUEST_SAFETY_OPEN) is True


def test_unintended_open_relay_is_distinct_fault_mode() -> None:
    relay = SafetyRelayEmulator(
        is_closed=True,
        fault_mode=RelayFaultMode.UNINTENDED_OPEN,
    )
    assert relay.apply(RelayRequest.REQUEST_SAFETY_CLOSED) is False


def test_fault_contracts_reject_invalid_types() -> None:
    with pytest.raises(ValueError, match="scenario"):
        SafetyFaultCase(
            scenario="clock_regression",  # type: ignore[arg-type]
            config=SafetySupervisorConfig(),
            expected_state=SafetyState.TRIPPED,
            expected_reason=SafetyReason.CLOCK_REGRESSION,
        )
    with pytest.raises(ValueError, match="fault_mode"):
        SafetyRelayEmulator(fault_mode="welded_closed")  # type: ignore[arg-type]


def test_report_is_immutable() -> None:
    report = run_safety_fault_injection(standard_fault_cases()[0])

    with pytest.raises(AttributeError):
        report.is_latched = False  # type: ignore[misc]


def test_invalid_case_input_is_fail_closed() -> None:
    with pytest.raises(ValueError, match="SafetyFaultCase"):
        run_safety_fault_injection(object())  # type: ignore[arg-type]
