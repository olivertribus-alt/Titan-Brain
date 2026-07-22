"""Deterministic actuator fault-injection scenarios for TB-ACT-001D.

The harness deliberately models only the feedback boundary.  It does not
publish commands or emulate hardware I/O; each case feeds timestamped samples
to :class:`~core.stop_ack_monitor.StopAckMonitor` and records the fail-closed
decision.  This keeps fault regression tests fast and runnable without ROS 2.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import cast

from core.actuator_feedback import FeedbackInput
from core.stop_ack_monitor import (
    StopAckMonitor,
    StopAckReason,
    StopMonitorConfig,
    StopMonitorResult,
    StopMonitorState,
    StopRequest,
)


class FaultInjectionScenario(StrEnum):
    """Faults covered by the hardware-emulation regression suite."""

    ENCODER_DRIFT = "encoder_drift"
    FROZEN_FEEDBACK = "frozen_feedback"
    CORRELATION_DESYNC = "correlation_desync"
    SEQUENCE_REPLAY = "sequence_replay"
    SEQUENCE_GAP = "sequence_gap"
    NON_FINITE_FEEDBACK = "non_finite_feedback"
    SPURIOUS_MOVEMENT = "spurious_movement"


# Shorter name for callers that already have a scenario-oriented API.
FaultScenario = FaultInjectionScenario


@dataclass(frozen=True, slots=True)
class FaultInjectionEvent:
    """One timestamped sample or malformed payload sent to the monitor."""

    now_ns: int
    payload: object

    def __post_init__(self) -> None:
        if (
            isinstance(self.now_ns, bool)
            or not isinstance(self.now_ns, int)
            or self.now_ns < 0
        ):
            raise ValueError("fault event time must be a non-negative integer")


@dataclass(frozen=True, slots=True)
class FaultInjectionCase:
    """A complete stop request plus deterministic injected feedback events."""

    scenario: FaultInjectionScenario
    request: StopRequest
    events: tuple[FaultInjectionEvent, ...]
    expected_reason: StopAckReason

    def __post_init__(self) -> None:
        if not self.events:
            raise ValueError("fault case must contain at least one event")


@dataclass(frozen=True, slots=True)
class FaultInjectionReport:
    """Final monitor evidence produced by one injected fault case."""

    scenario: FaultInjectionScenario
    final_state: StopMonitorState
    final_reason: StopAckReason
    is_latched: bool
    events_processed: int


def run_fault_injection(
    case: FaultInjectionCase,
    *,
    config: StopMonitorConfig,
) -> FaultInjectionReport:
    """Execute a case against a fresh monitor and return its final evidence."""
    monitor = StopAckMonitor(config)
    monitor.request_stop(case.request, now_ns=case.request.requested_timestamp_ns)
    result: StopMonitorResult | None = None
    for event in case.events:
        result = monitor.observe_feedback(
            cast(FeedbackInput, event.payload),
            now_ns=event.now_ns,
        )
    if result is None:  # pragma: no cover - cases reject empty event tuples
        raise ValueError("fault case must contain at least one event")
    return FaultInjectionReport(
        scenario=case.scenario,
        final_state=result.state,
        final_reason=result.reason,
        is_latched=result.is_latched,
        events_processed=len(case.events),
    )


def _feedback(
    *,
    correlation_id: str,
    sequence_id: int,
    timestamp_ns: int,
    measured_linear_x: float = 0.0,
) -> dict[str, object]:
    return {
        "correlation_id": correlation_id,
        "sequence_id": sequence_id,
        "timestamp_ns": timestamp_ns,
        "measured_linear_x": measured_linear_x,
        "measured_linear_y": 0.0,
        "measured_angular_z": 0.0,
    }


def standard_fault_cases(
    *,
    correlation_id: str = "act-001d",
    request_sequence_id: int = 1,
    request_timestamp_ns: int = 100,
) -> tuple[FaultInjectionCase, ...]:
    """Build the canonical 001D fault matrix used by unit and ROS tests."""
    request = StopRequest(
        correlation_id=correlation_id,
        sequence_id=request_sequence_id,
        requested_timestamp_ns=request_timestamp_ns,
    )
    t0 = request_timestamp_ns
    return (
        FaultInjectionCase(
            scenario=FaultInjectionScenario.ENCODER_DRIFT,
            request=request,
            events=(
                FaultInjectionEvent(
                    now_ns=t0 + 50,
                    payload=_feedback(
                        correlation_id=correlation_id,
                        sequence_id=1,
                        timestamp_ns=t0 + 50,
                        measured_linear_x=0.5,
                    ),
                ),
                FaultInjectionEvent(
                    now_ns=t0 + 100,
                    payload=_feedback(
                        correlation_id=correlation_id,
                        sequence_id=2,
                        timestamp_ns=t0 + 100,
                        measured_linear_x=0.5,
                    ),
                ),
            ),
            expected_reason=StopAckReason.STOP_TIMEOUT,
        ),
        FaultInjectionCase(
            scenario=FaultInjectionScenario.FROZEN_FEEDBACK,
            request=request,
            events=(
                FaultInjectionEvent(
                    now_ns=t0 + 60,
                    payload=_feedback(
                        correlation_id=correlation_id,
                        sequence_id=1,
                        timestamp_ns=t0,
                    ),
                ),
            ),
            expected_reason=StopAckReason.STALE_FEEDBACK,
        ),
        FaultInjectionCase(
            scenario=FaultInjectionScenario.CORRELATION_DESYNC,
            request=request,
            events=(
                FaultInjectionEvent(
                    now_ns=t0 + 10,
                    payload=_feedback(
                        correlation_id="unexpected-actuator",
                        sequence_id=1,
                        timestamp_ns=t0 + 10,
                    ),
                ),
            ),
            expected_reason=StopAckReason.FEEDBACK_CORRELATION_MISMATCH,
        ),
        FaultInjectionCase(
            scenario=FaultInjectionScenario.SEQUENCE_REPLAY,
            request=request,
            events=(
                FaultInjectionEvent(
                    now_ns=t0 + 10,
                    payload=_feedback(
                        correlation_id=correlation_id,
                        sequence_id=1,
                        timestamp_ns=t0 + 10,
                        measured_linear_x=0.5,
                    ),
                ),
                FaultInjectionEvent(
                    now_ns=t0 + 20,
                    payload=_feedback(
                        correlation_id=correlation_id,
                        sequence_id=1,
                        timestamp_ns=t0 + 20,
                    ),
                ),
            ),
            expected_reason=StopAckReason.FEEDBACK_SEQUENCE_REGRESSION,
        ),
        FaultInjectionCase(
            scenario=FaultInjectionScenario.SEQUENCE_GAP,
            request=request,
            events=(
                FaultInjectionEvent(
                    now_ns=t0 + 10,
                    payload=_feedback(
                        correlation_id=correlation_id,
                        sequence_id=1,
                        timestamp_ns=t0 + 10,
                        measured_linear_x=0.5,
                    ),
                ),
                FaultInjectionEvent(
                    now_ns=t0 + 20,
                    payload=_feedback(
                        correlation_id=correlation_id,
                        sequence_id=3,
                        timestamp_ns=t0 + 20,
                    ),
                ),
            ),
            expected_reason=StopAckReason.FEEDBACK_SEQUENCE_GAP,
        ),
        FaultInjectionCase(
            scenario=FaultInjectionScenario.NON_FINITE_FEEDBACK,
            request=request,
            events=(
                FaultInjectionEvent(
                    now_ns=t0 + 10,
                    payload=_feedback(
                        correlation_id=correlation_id,
                        sequence_id=1,
                        timestamp_ns=t0 + 10,
                        measured_linear_x=float("nan"),
                    ),
                ),
            ),
            expected_reason=StopAckReason.INVALID_FEEDBACK,
        ),
        FaultInjectionCase(
            scenario=FaultInjectionScenario.SPURIOUS_MOVEMENT,
            request=request,
            events=(
                FaultInjectionEvent(
                    now_ns=t0 + 10,
                    payload=_feedback(
                        correlation_id=correlation_id,
                        sequence_id=1,
                        timestamp_ns=t0 + 10,
                    ),
                ),
                FaultInjectionEvent(
                    now_ns=t0 + 20,
                    payload=_feedback(
                        correlation_id=correlation_id,
                        sequence_id=2,
                        timestamp_ns=t0 + 20,
                        measured_linear_x=0.2,
                    ),
                ),
            ),
            expected_reason=StopAckReason.SPURIOUS_MOVEMENT_AFTER_ACK,
        ),
    )


__all__ = [
    "FaultInjectionCase",
    "FaultInjectionEvent",
    "FaultInjectionReport",
    "FaultInjectionScenario",
    "FaultScenario",
    "run_fault_injection",
    "standard_fault_cases",
]
