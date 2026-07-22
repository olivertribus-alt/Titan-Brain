"""Deterministic TB-SAFE-001D supervisor and relay fault injection harness.

The harness models only the safety boundary.  It deliberately avoids ROS 2,
DDS and hardware dependencies so every failure mode can be replayed with
integer timestamps in a unit-test process.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from core.safety_supervisor import (
    HeartbeatChannel,
    RelayRequest,
    SafetyReason,
    SafetyState,
    SafetySupervisor,
    SafetySupervisorConfig,
    SafetySupervisorResult,
)


class SafetyFaultScenario(StrEnum):
    """Faults covered by the TB-SAFE-001D regression matrix."""

    MISSING_HEARTBEAT = "missing_heartbeat"
    STALE_HEARTBEAT = "stale_heartbeat"
    WELDED_RELAY_CONTACTS = "welded_relay_contacts"
    CLOCK_REGRESSION = "clock_regression"
    SEQUENCE_REPLAY = "sequence_replay"
    UNAUTHORIZED_RESET = "unauthorized_reset"


FaultScenario = SafetyFaultScenario


class RelayFaultMode(StrEnum):
    """Physical relay behavior used by the deterministic emulator."""

    NOMINAL = "nominal"
    WELDED_CLOSED = "welded_closed"
    UNINTENDED_OPEN = "unintended_open"


@dataclass(slots=True)
class SafetyRelayEmulator:
    """Minimal auxiliary-contact model for supervisor boundary tests."""

    is_closed: bool = False
    fault_mode: RelayFaultMode = RelayFaultMode.NOMINAL

    def __post_init__(self) -> None:
        if not isinstance(self.is_closed, bool):
            raise ValueError("relay is_closed must be boolean")
        if not isinstance(self.fault_mode, RelayFaultMode):
            raise ValueError("relay fault_mode must be a RelayFaultMode")

    def apply(self, request: RelayRequest) -> bool:
        """Apply a command and return the resulting auxiliary feedback."""
        if not isinstance(request, RelayRequest):
            raise ValueError("relay request must be a RelayRequest")
        requested_closed = request is RelayRequest.REQUEST_SAFETY_CLOSED
        if self.fault_mode is RelayFaultMode.WELDED_CLOSED:
            self.is_closed = self.is_closed or requested_closed
        elif self.fault_mode is RelayFaultMode.UNINTENDED_OPEN:
            self.is_closed = False
        else:
            self.is_closed = requested_closed
        return self.feedback()

    def feedback(self) -> bool:
        """Return the physical contact state observed by the supervisor."""
        return self.is_closed


@dataclass(frozen=True, slots=True)
class SafetyFaultCase:
    """Expected evidence and deterministic budget for one injected fault."""

    scenario: SafetyFaultScenario
    config: SafetySupervisorConfig
    expected_state: SafetyState
    expected_reason: SafetyReason

    def __post_init__(self) -> None:
        if not isinstance(self.scenario, SafetyFaultScenario):
            raise ValueError("scenario must be a SafetyFaultScenario")
        if not isinstance(self.config, SafetySupervisorConfig):
            raise ValueError("config must be a SafetySupervisorConfig")
        if not isinstance(self.expected_state, SafetyState):
            raise ValueError("expected_state must be a SafetyState")
        if not isinstance(self.expected_reason, SafetyReason):
            raise ValueError("expected_reason must be a SafetyReason")


@dataclass(frozen=True, slots=True)
class SafetyFaultReport:
    """Auditable result of one fault-injection execution."""

    scenario: SafetyFaultScenario
    final_state: SafetyState
    final_reason: SafetyReason
    relay_feedback_closed: bool | None
    is_latched: bool
    events_processed: int


def _ready_supervisor(
    supervisor: SafetySupervisor,
    relay: SafetyRelayEmulator,
) -> int:
    """Register all channels and acknowledge a nominal closed relay."""
    result: SafetySupervisorResult | None = None
    for offset, channel in enumerate(HeartbeatChannel, start=1):
        result = supervisor.receive_heartbeat(channel, timestamp_ns=offset)
    if result is None:  # pragma: no cover - the channel enum is non-empty
        raise RuntimeError("supervisor channel matrix is empty")
    feedback = relay.apply(result.relay_request)
    supervisor.observe_relay_feedback(feedback, timestamp_ns=4)
    return 4


def standard_fault_cases() -> tuple[SafetyFaultCase, ...]:
    """Return the canonical TB-SAFE-001D fault matrix."""
    default = SafetySupervisorConfig(
        control_arbiter_timeout_ns=100,
        actuator_monitor_timeout_ns=100,
        odometry_timeout_ns=100,
        initialization_timeout_ns=100,
        relay_budget_ns=50,
    )
    short = SafetySupervisorConfig(
        control_arbiter_timeout_ns=10,
        actuator_monitor_timeout_ns=10,
        odometry_timeout_ns=10,
        initialization_timeout_ns=10,
        relay_budget_ns=50,
    )
    missing_heartbeat = SafetySupervisorConfig(
        control_arbiter_timeout_ns=100,
        actuator_monitor_timeout_ns=100,
        odometry_timeout_ns=100,
        initialization_timeout_ns=100,
        relay_budget_ns=200,
    )
    return (
        SafetyFaultCase(
            scenario=SafetyFaultScenario.MISSING_HEARTBEAT,
            config=missing_heartbeat,
            expected_state=SafetyState.TRIPPED,
            expected_reason=SafetyReason.INITIALIZATION_TIMEOUT,
        ),
        SafetyFaultCase(
            scenario=SafetyFaultScenario.STALE_HEARTBEAT,
            config=default,
            expected_state=SafetyState.TRIPPED,
            expected_reason=SafetyReason.HEARTBEAT_TIMEOUT,
        ),
        SafetyFaultCase(
            scenario=SafetyFaultScenario.WELDED_RELAY_CONTACTS,
            config=short,
            expected_state=SafetyState.HARDWARE_FAULT_LATCH,
            expected_reason=SafetyReason.WELDED_CONTACTS,
        ),
        SafetyFaultCase(
            scenario=SafetyFaultScenario.CLOCK_REGRESSION,
            config=default,
            expected_state=SafetyState.TRIPPED,
            expected_reason=SafetyReason.CLOCK_REGRESSION,
        ),
        SafetyFaultCase(
            scenario=SafetyFaultScenario.SEQUENCE_REPLAY,
            config=default,
            expected_state=SafetyState.TRIPPED,
            expected_reason=SafetyReason.HEARTBEAT_ERROR,
        ),
        SafetyFaultCase(
            scenario=SafetyFaultScenario.UNAUTHORIZED_RESET,
            config=default,
            expected_state=SafetyState.HARDWARE_FAULT_LATCH,
            expected_reason=SafetyReason.RESET_REJECTED,
        ),
    )


def run_safety_fault_injection(case: SafetyFaultCase) -> SafetyFaultReport:
    """Execute one deterministic fault case against a fresh supervisor."""
    if not isinstance(case, SafetyFaultCase):
        raise ValueError("case must be a SafetyFaultCase")
    supervisor = SafetySupervisor(case.config, started_at_ns=0)
    relay = SafetyRelayEmulator()
    result: SafetySupervisorResult
    events_processed = 0

    if case.scenario is SafetyFaultScenario.MISSING_HEARTBEAT:
        result = supervisor.evaluate(now_ns=case.config.initialization_timeout_ns + 1)
        events_processed = 1
    elif case.scenario is SafetyFaultScenario.STALE_HEARTBEAT:
        _ready_supervisor(supervisor, relay)
        result = supervisor.evaluate(now_ns=104)
        events_processed = 5
    elif case.scenario is SafetyFaultScenario.WELDED_RELAY_CONTACTS:
        _ready_supervisor(supervisor, relay)
        relay.fault_mode = RelayFaultMode.WELDED_CLOSED
        relay.apply(RelayRequest.REQUEST_SAFETY_OPEN)
        supervisor.evaluate(now_ns=14)
        result = supervisor.observe_relay_feedback(
            relay.feedback(),
            timestamp_ns=65,
        )
        events_processed = 7
    elif case.scenario is SafetyFaultScenario.CLOCK_REGRESSION:
        supervisor.receive_heartbeat(
            HeartbeatChannel.CONTROL_ARBITER,
            timestamp_ns=10,
        )
        result = supervisor.evaluate(now_ns=9)
        events_processed = 2
    elif case.scenario is SafetyFaultScenario.SEQUENCE_REPLAY:
        supervisor.receive_heartbeat(
            HeartbeatChannel.CONTROL_ARBITER,
            timestamp_ns=1,
        )
        result = supervisor.receive_heartbeat(
            HeartbeatChannel.CONTROL_ARBITER,
            timestamp_ns=2,
            healthy=False,
            error="heartbeat sequence replay",
        )
        events_processed = 2
    else:
        supervisor.latch_hardware_fault(now_ns=1, detail="injected relay fault")
        supervisor.observe_relay_feedback(False, timestamp_ns=2)
        for offset, channel in enumerate(HeartbeatChannel, start=10):
            supervisor.receive_heartbeat(channel, timestamp_ns=offset)
        result = supervisor.reset_hardware_fault(
            authorization="unauthorized",
            sequence_id=1,
            now_ns=13,
        )
        events_processed = 6

    return SafetyFaultReport(
        scenario=case.scenario,
        final_state=result.state,
        final_reason=result.reason,
        relay_feedback_closed=result.relay_feedback_closed,
        is_latched=result.state is SafetyState.HARDWARE_FAULT_LATCH,
        events_processed=events_processed,
    )


# Concise alias for scripts and callers using the generic harness name.
run_fault_injection = run_safety_fault_injection


__all__ = [
    "FaultScenario",
    "RelayFaultMode",
    "SafetyFaultCase",
    "SafetyFaultReport",
    "SafetyFaultScenario",
    "SafetyRelayEmulator",
    "run_fault_injection",
    "run_safety_fault_injection",
    "standard_fault_cases",
]
