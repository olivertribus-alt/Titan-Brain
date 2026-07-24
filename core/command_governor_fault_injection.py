"""Deterministic TB-EVAL-006C governor verification harness.

The harness deliberately stays dependency-free.  It drives the same
``CommandGovernor`` used by the production adapter with integer monotonic
timestamps and records enough evidence to verify jerk, asymmetric ramping,
emergency bypass, and fail-closed timeout behavior without requiring ROS 2.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum
from typing import cast

from core.command_governor import (
    CommandGovernor,
    CommandInput,
    GovernorCommand,
    GovernorConfig,
    GovernorReason,
    GovernorResult,
)

NANOSECONDS_PER_SECOND = 1_000_000_000
METRIC_EPSILON = 1e-9


class GovernorFaultScenario(StrEnum):
    """Scenarios covered by the TB-EVAL-006C acceptance matrix."""

    JERK_LIMIT = "jerk_limit"
    ASYMMETRIC_RAMP = "asymmetric_ramp"
    EMERGENCY_CUTOFF = "emergency_cutoff"
    STALE_COMMAND = "stale_command"
    SAFETY_TIMEOUT = "safety_timeout"
    INVALID_INPUT = "invalid_input"


FaultScenario = GovernorFaultScenario


@dataclass(frozen=True, slots=True)
class GovernorFaultEvent:
    """One timestamped command or fail-closed timeout observation.

    ``command=None`` is intentionally reserved for the stale-command and
    safety-timeout cases.  The harness converts it to the explicit emergency
    stop path used by the ROS adapter after its watchdog detects the fault.
    """

    timestamp_ns: int
    command: object | None

    def __post_init__(self) -> None:
        if (
            isinstance(self.timestamp_ns, bool)
            or not isinstance(self.timestamp_ns, int)
            or self.timestamp_ns < 0
        ):
            raise ValueError("fault event time must be a non-negative integer")


@dataclass(frozen=True, slots=True)
class GovernorFaultCase:
    """A complete deterministic governor scenario."""

    scenario: GovernorFaultScenario
    config: GovernorConfig
    events: tuple[GovernorFaultEvent, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.scenario, GovernorFaultScenario):
            raise ValueError("scenario must be a GovernorFaultScenario")
        if not isinstance(self.config, GovernorConfig):
            raise ValueError("config must be a GovernorConfig")
        if not isinstance(self.events, tuple) or not self.events:
            raise ValueError("governor fault case must contain events")
        if not all(isinstance(event, GovernorFaultEvent) for event in self.events):
            raise ValueError("events must contain GovernorFaultEvent values")


@dataclass(frozen=True, slots=True)
class GovernorFaultReport:
    """Auditable result and measured safety evidence for one scenario."""

    scenario: GovernorFaultScenario
    results: tuple[GovernorResult, ...]
    max_observed_jerk_mps3: float
    max_positive_acceleration_mps2: float
    max_braking_acceleration_mps2: float
    final_result: GovernorResult
    events_processed: int
    passed: bool

    def __post_init__(self) -> None:
        if not isinstance(self.scenario, GovernorFaultScenario):
            raise ValueError("scenario must be a GovernorFaultScenario")
        if not self.results:
            raise ValueError("report must contain at least one result")
        if self.events_processed != len(self.results):
            raise ValueError("events_processed must match result count")
        if self.final_result is not self.results[-1]:
            raise ValueError("final_result must be the last result")
        for metric in (
            self.max_observed_jerk_mps3,
            self.max_positive_acceleration_mps2,
            self.max_braking_acceleration_mps2,
        ):
            if not math.isfinite(metric) or metric < 0.0:
                raise ValueError("report metrics must be finite and non-negative")


def _default_config() -> GovernorConfig:
    return GovernorConfig(
        max_linear_velocity_mps=2.0,
        max_angular_velocity_radps=2.0,
        max_linear_acceleration_mps2=1.0,
        max_linear_deceleration_mps2=2.0,
        max_angular_acceleration_radps2=1.0,
        max_angular_deceleration_radps2=2.0,
        max_linear_jerk_mps3=5.0,
        max_angular_jerk_radps3=5.0,
    )


def standard_fault_cases() -> tuple[GovernorFaultCase, ...]:
    """Return the canonical six-scenario 006C verification matrix."""
    default = _default_config()
    asym_config = GovernorConfig(
        max_linear_velocity_mps=2.0,
        max_angular_velocity_radps=2.0,
        max_linear_acceleration_mps2=1.0,
        max_linear_deceleration_mps2=2.0,
        max_angular_acceleration_radps2=1.0,
        max_angular_deceleration_radps2=2.0,
        max_linear_jerk_mps3=100.0,
        max_angular_jerk_radps3=100.0,
    )
    step_ns = 20_000_000
    jerk_events = tuple(
        GovernorFaultEvent(
            timestamp_ns=index * step_ns,
            command=GovernorCommand(linear_velocity_mps=1.0),
        )
        for index in range(1, 13)
    )
    return (
        GovernorFaultCase(
            scenario=GovernorFaultScenario.JERK_LIMIT,
            config=default,
            events=jerk_events,
        ),
        GovernorFaultCase(
            scenario=GovernorFaultScenario.ASYMMETRIC_RAMP,
            config=asym_config,
            events=(
                GovernorFaultEvent(
                    timestamp_ns=NANOSECONDS_PER_SECOND,
                    command=GovernorCommand(linear_velocity_mps=2.0),
                ),
                GovernorFaultEvent(
                    timestamp_ns=2 * NANOSECONDS_PER_SECOND,
                    command=GovernorCommand(linear_velocity_mps=2.0),
                ),
                GovernorFaultEvent(
                    timestamp_ns=3 * NANOSECONDS_PER_SECOND,
                    command=GovernorCommand(linear_velocity_mps=0.0),
                ),
            ),
        ),
        GovernorFaultCase(
            scenario=GovernorFaultScenario.EMERGENCY_CUTOFF,
            config=default,
            events=(
                GovernorFaultEvent(
                    timestamp_ns=NANOSECONDS_PER_SECOND,
                    command=GovernorCommand(linear_velocity_mps=1.0),
                ),
                GovernorFaultEvent(
                    timestamp_ns=1_100_000_000,
                    command=GovernorCommand(
                        emergency_stop=True,
                        correlation_id="safety-trip",
                    ),
                ),
            ),
        ),
        GovernorFaultCase(
            scenario=GovernorFaultScenario.STALE_COMMAND,
            config=default,
            events=(
                GovernorFaultEvent(
                    timestamp_ns=NANOSECONDS_PER_SECOND,
                    command=GovernorCommand(linear_velocity_mps=1.0),
                ),
                GovernorFaultEvent(timestamp_ns=1_200_000_000, command=None),
            ),
        ),
        GovernorFaultCase(
            scenario=GovernorFaultScenario.SAFETY_TIMEOUT,
            config=default,
            events=(
                GovernorFaultEvent(
                    timestamp_ns=NANOSECONDS_PER_SECOND,
                    command=GovernorCommand(linear_velocity_mps=1.0),
                ),
                GovernorFaultEvent(timestamp_ns=1_300_000_000, command=None),
            ),
        ),
        GovernorFaultCase(
            scenario=GovernorFaultScenario.INVALID_INPUT,
            config=default,
            events=(
                GovernorFaultEvent(
                    timestamp_ns=NANOSECONDS_PER_SECOND,
                    command={"linear_velocity_mps": math.nan},
                ),
            ),
        ),
    )


def _command_for_event(
    event: GovernorFaultEvent,
    scenario: GovernorFaultScenario,
) -> CommandInput:
    if event.command is None:
        if scenario not in (
            GovernorFaultScenario.STALE_COMMAND,
            GovernorFaultScenario.SAFETY_TIMEOUT,
        ):
            raise ValueError("only timeout scenarios may contain an empty command")
        return GovernorCommand(
            emergency_stop=True,
            correlation_id=f"{scenario.value}-fail-closed",
        )
    return cast(CommandInput, event.command)


def _metrics(
    results: tuple[GovernorResult, ...],
) -> tuple[float, float, float]:
    previous_acceleration = 0.0
    max_jerk = 0.0
    max_positive_acceleration = 0.0
    max_braking_acceleration = 0.0
    for result in results:
        dt_s = result.dt_ns / NANOSECONDS_PER_SECOND
        if dt_s > 0.0:
            jerk = abs((result.linear_acceleration_mps2 - previous_acceleration) / dt_s)
            max_jerk = max(max_jerk, jerk)
        max_positive_acceleration = max(
            max_positive_acceleration,
            result.linear_acceleration_mps2,
        )
        max_braking_acceleration = max(
            max_braking_acceleration,
            -result.linear_acceleration_mps2,
        )
        previous_acceleration = result.linear_acceleration_mps2
    return max_jerk, max_positive_acceleration, max_braking_acceleration


def _scenario_passed(
    case: GovernorFaultCase,
    report_values: tuple[float, float, float],
    final: GovernorResult,
) -> bool:
    max_jerk, max_acceleration, max_braking = report_values
    if case.scenario is GovernorFaultScenario.JERK_LIMIT:
        return (
            max_jerk <= case.config.max_linear_jerk_mps3 + METRIC_EPSILON
            and final.linear_velocity_mps > 0.0
            and final.is_safe
        )
    if case.scenario is GovernorFaultScenario.ASYMMETRIC_RAMP:
        return (
            max_acceleration
            <= case.config.max_linear_acceleration_mps2 + METRIC_EPSILON
            and max_braking <= case.config.max_linear_deceleration_mps2 + METRIC_EPSILON
            and max_braking > max_acceleration
            and math.isclose(final.linear_velocity_mps, 0.0, abs_tol=METRIC_EPSILON)
        )
    if case.scenario in (
        GovernorFaultScenario.EMERGENCY_CUTOFF,
        GovernorFaultScenario.STALE_COMMAND,
        GovernorFaultScenario.SAFETY_TIMEOUT,
    ):
        return (
            final.reason is GovernorReason.EMERGENCY_STOP
            and final.emergency_override
            and final.is_safe
            and final.linear_velocity_mps == 0.0
            and final.angular_velocity_radps == 0.0
        )
    return (
        final.reason is GovernorReason.INVALID_COMMAND
        and not final.is_safe
        and final.linear_velocity_mps == 0.0
        and final.angular_velocity_radps == 0.0
    )


def run_fault_injection(case: GovernorFaultCase) -> GovernorFaultReport:
    """Execute one case and return immutable, auditable evidence."""
    if not isinstance(case, GovernorFaultCase):
        raise ValueError("case must be a GovernorFaultCase")
    governor = CommandGovernor(case.config)
    results: list[GovernorResult] = []
    for event in case.events:
        results.append(
            governor.step(
                _command_for_event(event, case.scenario),
                timestamp_ns=event.timestamp_ns,
            )
        )
    result_tuple = tuple(results)
    metrics = _metrics(result_tuple)
    final = result_tuple[-1]
    return GovernorFaultReport(
        scenario=case.scenario,
        results=result_tuple,
        max_observed_jerk_mps3=metrics[0],
        max_positive_acceleration_mps2=metrics[1],
        max_braking_acceleration_mps2=metrics[2],
        final_result=final,
        events_processed=len(result_tuple),
        passed=_scenario_passed(case, metrics, final),
    )


run_governor_fault_injection = run_fault_injection


__all__ = [
    "FaultScenario",
    "GovernorFaultCase",
    "GovernorFaultEvent",
    "GovernorFaultReport",
    "GovernorFaultScenario",
    "run_fault_injection",
    "run_governor_fault_injection",
    "standard_fault_cases",
]
