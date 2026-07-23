"""TB-EVAL-007A tests for the dependency-free priority selector."""

from __future__ import annotations

import math

import pytest

from core.priority_selector import (
    CommandSourcePriority,
    PrioritySelectorCore,
    RawCommandFrame,
    SystemFaultState,
)


def _frame(
    *,
    priority: CommandSourcePriority,
    source_id: str,
    timestamp_ns: int = 900,
    linear_x: float = 0.4,
    angular_z: float = 0.1,
) -> RawCommandFrame:
    return RawCommandFrame(
        linear_x=linear_x,
        angular_z=angular_z,
        priority=priority,
        timestamp_ns=timestamp_ns,
        source_id=source_id,
    )


def test_priority_and_default_contracts() -> None:
    selector = PrioritySelectorCore()

    assert selector.command_timeout_ns == 100_000_000
    assert selector.last_processed_time_ns == 0
    assert selector.timing_latched is False
    assert CommandSourcePriority.TELEOPERATION > CommandSourcePriority.AUTONOMY
    assert SystemFaultState.LATCHED_SAFETY_FAULT > SystemFaultState.OK


@pytest.mark.parametrize("timeout", (0, -1, True, 1.5))
def test_timeout_must_be_positive_integer(timeout: object) -> None:
    with pytest.raises(ValueError, match="positive integer"):
        PrioritySelectorCore(timeout)  # type: ignore[arg-type]


def test_teleoperation_has_precedence_over_autonomy() -> None:
    selector = PrioritySelectorCore(command_timeout_ns=100)
    teleop = _frame(
        priority=CommandSourcePriority.TELEOPERATION,
        source_id="teleop",
        linear_x=0.2,
    )
    autonomy = _frame(
        priority=CommandSourcePriority.AUTONOMY,
        source_id="autonomy",
        linear_x=0.8,
    )

    result = selector.select_source(
        SystemFaultState.OK,
        teleop,
        autonomy,
        current_time_ns=1_000,
    )

    assert result.selected_frame is teleop
    assert result.active_priority is CommandSourcePriority.TELEOPERATION
    assert result.fault_state is SystemFaultState.OK
    assert result.rejection_reason is None


def test_autonomy_is_selected_when_teleop_is_missing_or_stale() -> None:
    selector = PrioritySelectorCore(command_timeout_ns=100)
    autonomy = _frame(
        priority=CommandSourcePriority.AUTONOMY,
        source_id="navigation",
        linear_x=0.8,
    )
    result = selector.select_source(
        SystemFaultState.OK,
        None,
        autonomy,
        current_time_ns=1_000,
    )
    assert result.selected_frame is autonomy
    assert result.active_priority is CommandSourcePriority.AUTONOMY

    stale_teleop = _frame(
        priority=CommandSourcePriority.TELEOPERATION,
        source_id="teleop",
        timestamp_ns=800,
    )
    result = selector.select_source(
        SystemFaultState.OK,
        stale_teleop,
        autonomy,
        current_time_ns=1_000,
    )
    assert result.selected_frame is autonomy


def test_exact_timeout_boundary_is_fresh_and_after_boundary_is_stale() -> None:
    selector = PrioritySelectorCore(command_timeout_ns=100)
    autonomy = _frame(
        priority=CommandSourcePriority.AUTONOMY,
        source_id="autonomy",
    )
    result = selector.select_source(
        SystemFaultState.OK,
        None,
        autonomy,
        current_time_ns=1_000,
    )
    assert result.selected_frame is autonomy

    stale = _frame(
        priority=CommandSourcePriority.AUTONOMY,
        source_id="autonomy",
        timestamp_ns=899,
    )
    result = selector.select_source(
        SystemFaultState.OK,
        None,
        stale,
        current_time_ns=1_000,
    )
    assert result.selected_frame is None
    assert result.rejection_reason == "COMMAND_TIMEOUT"


@pytest.mark.parametrize(
    "fault_state",
    (
        SystemFaultState.E_STOP_ACTIVE,
        SystemFaultState.HARDWARE_FAULT,
        SystemFaultState.LATCHED_SAFETY_FAULT,
    ),
)
def test_any_explicit_fault_forces_zero(fault_state: SystemFaultState) -> None:
    selector = PrioritySelectorCore()
    result = selector.select_source(
        fault_state,
        _frame(priority=CommandSourcePriority.TELEOPERATION, source_id="teleop"),
        _frame(priority=CommandSourcePriority.AUTONOMY, source_id="autonomy"),
        current_time_ns=1_000,
    )

    assert result.selected_frame is None
    assert result.active_priority is CommandSourcePriority.NONE
    assert result.fault_state is fault_state
    assert result.rejection_reason == f"SYSTEM_FAULT_{fault_state.name}"


def test_no_inputs_returns_idle_fail_closed_selection() -> None:
    selector = PrioritySelectorCore()

    result = selector.select_source(
        SystemFaultState.OK,
        None,
        None,
        current_time_ns=1,
    )

    assert result.selected_frame is None
    assert result.active_priority is CommandSourcePriority.NONE
    assert result.fault_state is SystemFaultState.OK
    assert result.rejection_reason == "NO_VALID_COMMAND_SOURCE"


@pytest.mark.parametrize(
    ("frame", "reason"),
    (
        (object(), "INVALID_COMMAND_FRAME"),
        (
            _frame(
                priority=CommandSourcePriority.AUTONOMY,
                source_id="autonomy",
                linear_x=math.nan,
            ),
            "NON_FINITE_COMMAND",
        ),
        (
            _frame(
                priority=CommandSourcePriority.AUTONOMY,
                source_id="autonomy",
                timestamp_ns=True,
            ),
            "INVALID_COMMAND_TIMESTAMP",
        ),
        (
            _frame(
                priority=CommandSourcePriority.AUTONOMY,
                source_id="",
            ),
            "INVALID_SOURCE_ID",
        ),
        (
            _frame(
                priority=CommandSourcePriority.AUTONOMY,
                source_id="unknown-source",
            ),
            "UNKNOWN_SOURCE_ID",
        ),
        (
            _frame(
                priority=CommandSourcePriority.TELEOPERATION,
                source_id="teleop",
            ),
            "SOURCE_PRIORITY_MISMATCH",
        ),
    ),
)
def test_invalid_source_frames_fail_closed(frame: object, reason: str) -> None:
    selector = PrioritySelectorCore()
    result = selector.select_source(
        SystemFaultState.OK,
        None,
        frame,  # type: ignore[arg-type]
        current_time_ns=1_000,
    )

    assert result.selected_frame is None
    assert result.rejection_reason == reason


def test_unknown_priority_fails_closed() -> None:
    selector = PrioritySelectorCore()
    frame = _frame(
        priority=99,  # type: ignore[arg-type]
        source_id="autonomy",
    )
    result = selector.select_source(
        SystemFaultState.OK,
        None,
        frame,
        current_time_ns=1_000,
    )

    assert result.rejection_reason == "UNKNOWN_SOURCE_PRIORITY"


def test_future_timestamp_is_rejected() -> None:
    selector = PrioritySelectorCore()
    result = selector.select_source(
        SystemFaultState.OK,
        _frame(
            priority=CommandSourcePriority.TELEOPERATION,
            source_id="teleop",
            timestamp_ns=1_001,
        ),
        None,
        current_time_ns=1_000,
    )

    assert result.selected_frame is None
    assert result.rejection_reason == "FUTURE_TIMESTAMP"


def test_clock_regression_latches_and_cannot_recover_implicitly() -> None:
    selector = PrioritySelectorCore()
    selected = selector.select_source(
        SystemFaultState.OK,
        _frame(priority=CommandSourcePriority.TELEOPERATION, source_id="teleop"),
        None,
        current_time_ns=1_000,
    )
    assert selected.selected_frame is not None

    regression = selector.select_source(
        SystemFaultState.OK,
        None,
        None,
        current_time_ns=999,
    )
    assert regression.fault_state is SystemFaultState.LATCHED_SAFETY_FAULT
    assert regression.rejection_reason == "CLOCK_REGRESSION_DETECTED"
    assert selector.timing_latched is True

    later = selector.select_source(
        SystemFaultState.OK,
        _frame(priority=CommandSourcePriority.TELEOPERATION, source_id="teleop"),
        None,
        current_time_ns=2_000,
    )
    assert later.rejection_reason == "TIMING_FAULT_LATCHED"
    assert later.selected_frame is None


def test_invalid_current_time_and_fault_state_are_fail_closed() -> None:
    selector = PrioritySelectorCore()
    invalid_time = selector.select_source(
        SystemFaultState.OK,
        None,
        None,
        current_time_ns=-1,
    )
    assert invalid_time.rejection_reason == "INVALID_CURRENT_TIME"
    assert invalid_time.fault_state is SystemFaultState.LATCHED_SAFETY_FAULT

    other = PrioritySelectorCore()
    invalid_fault = other.select_source(
        object(),  # type: ignore[arg-type]
        None,
        None,
        current_time_ns=1,
    )
    assert invalid_fault.rejection_reason == "INVALID_SYSTEM_FAULT_STATE"
    assert invalid_fault.fault_state is SystemFaultState.LATCHED_SAFETY_FAULT


def test_result_and_frame_are_immutable() -> None:
    selector = PrioritySelectorCore()
    frame = _frame(priority=CommandSourcePriority.AUTONOMY, source_id="autonomy")
    result = selector.select_source(SystemFaultState.OK, None, frame, 1_000)

    with pytest.raises(AttributeError):
        frame.linear_x = 1.0  # type: ignore[misc]
    with pytest.raises(AttributeError):
        result.rejection_reason = "changed"  # type: ignore[misc]
