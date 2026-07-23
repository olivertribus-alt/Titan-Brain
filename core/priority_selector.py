"""Dependency-free O(1) source-priority selector for TB-EVAL-007A.

The selector chooses one already-formed command frame.  It does not create
motion, clamp against a permitted-motion envelope, or publish to ROS 2.  The
existing TB-EVAL-004/005 arbiter remains the single authority for envelope
validation and the TB-EVAL-006 governor remains responsible for ramp/jerk
shaping downstream of this selection step.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

DEFAULT_COMMAND_TIMEOUT_NS = 100_000_000


class SystemFaultState(IntEnum):
    """Explicit system fault state consumed by the priority selector."""

    OK = 0
    E_STOP_ACTIVE = 1
    HARDWARE_FAULT = 2
    LATCHED_SAFETY_FAULT = 3


class CommandSourcePriority(IntEnum):
    """Allowed source precedence, from autonomy to operator control."""

    NONE = 0
    AUTONOMY = 10
    TELEOPERATION = 20


@dataclass(frozen=True, slots=True)
class RawCommandFrame:
    """Immutable command candidate received from one source."""

    linear_x: float
    angular_z: float
    priority: CommandSourcePriority
    timestamp_ns: int
    source_id: str


@dataclass(frozen=True, slots=True)
class SelectionResult:
    """Immutable decision and audit reason produced by the selector."""

    selected_frame: RawCommandFrame | None
    fault_state: SystemFaultState
    active_priority: CommandSourcePriority
    rejection_reason: str | None


_SOURCE_IDS: dict[CommandSourcePriority, frozenset[str]] = {
    CommandSourcePriority.AUTONOMY: frozenset(
        {"autonomy", "auto", "navigation", "navigator"}
    ),
    CommandSourcePriority.TELEOPERATION: frozenset(
        {"teleoperation", "teleop", "manual", "operator", "remote"}
    ),
}


def _checked_timestamp(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


class PrioritySelectorCore:
    """Select teleoperation over autonomy with fail-closed timing checks."""

    def __init__(self, command_timeout_ns: int = DEFAULT_COMMAND_TIMEOUT_NS) -> None:
        checked_timeout = _checked_timestamp(command_timeout_ns)
        if checked_timeout is None or checked_timeout <= 0:
            raise ValueError("command_timeout_ns must be a positive integer")
        self._timeout_ns = checked_timeout
        self._last_processed_time_ns = 0
        self._timing_latched = False

    @property
    def command_timeout_ns(self) -> int:
        """Return the immutable command freshness budget."""
        return self._timeout_ns

    @property
    def last_processed_time_ns(self) -> int:
        """Return the last accepted evaluation time."""
        return self._last_processed_time_ns

    @property
    def timing_latched(self) -> bool:
        """Return whether a timing-integrity fault permanently stopped selection."""
        return self._timing_latched

    @staticmethod
    def _zero_result(
        *,
        fault_state: SystemFaultState,
        rejection_reason: str,
    ) -> SelectionResult:
        return SelectionResult(
            selected_frame=None,
            fault_state=fault_state,
            active_priority=CommandSourcePriority.NONE,
            rejection_reason=rejection_reason,
        )

    def _validate_frame(
        self,
        frame: object,
        *,
        expected_priority: CommandSourcePriority,
        now_ns: int,
    ) -> tuple[bool, str | None]:
        if not isinstance(frame, RawCommandFrame):
            return False, "INVALID_COMMAND_FRAME"
        if not isinstance(frame.priority, CommandSourcePriority):
            return False, "UNKNOWN_SOURCE_PRIORITY"
        if frame.priority is not expected_priority:
            return False, "SOURCE_PRIORITY_MISMATCH"
        if not _finite_number(frame.linear_x) or not _finite_number(
            frame.angular_z
        ):
            return False, "NON_FINITE_COMMAND"
        if not isinstance(frame.source_id, str) or not frame.source_id.strip():
            return False, "INVALID_SOURCE_ID"
        source_id = frame.source_id.strip().casefold()
        if source_id not in _SOURCE_IDS[expected_priority]:
            return False, "UNKNOWN_SOURCE_ID"
        timestamp_ns = _checked_timestamp(frame.timestamp_ns)
        if timestamp_ns is None:
            return False, "INVALID_COMMAND_TIMESTAMP"
        if timestamp_ns > now_ns:
            return False, "FUTURE_TIMESTAMP"
        if now_ns - timestamp_ns > self._timeout_ns:
            return False, "COMMAND_TIMEOUT"
        return True, None

    def select_source(
        self,
        fault_state: SystemFaultState,
        teleop_cmd: RawCommandFrame | None,
        autonomy_cmd: RawCommandFrame | None,
        current_time_ns: int,
    ) -> SelectionResult:
        """Return the highest-priority fresh command or a fail-closed result."""
        checked_now_ns = _checked_timestamp(current_time_ns)
        if checked_now_ns is None:
            self._timing_latched = True
            return self._zero_result(
                fault_state=SystemFaultState.LATCHED_SAFETY_FAULT,
                rejection_reason="INVALID_CURRENT_TIME",
            )
        if self._timing_latched:
            return self._zero_result(
                fault_state=SystemFaultState.LATCHED_SAFETY_FAULT,
                rejection_reason="TIMING_FAULT_LATCHED",
            )
        if checked_now_ns < self._last_processed_time_ns:
            self._timing_latched = True
            return self._zero_result(
                fault_state=SystemFaultState.LATCHED_SAFETY_FAULT,
                rejection_reason="CLOCK_REGRESSION_DETECTED",
            )
        self._last_processed_time_ns = checked_now_ns

        if not isinstance(fault_state, SystemFaultState):
            self._timing_latched = True
            return self._zero_result(
                fault_state=SystemFaultState.LATCHED_SAFETY_FAULT,
                rejection_reason="INVALID_SYSTEM_FAULT_STATE",
            )
        if fault_state is not SystemFaultState.OK:
            return self._zero_result(
                fault_state=fault_state,
                rejection_reason=f"SYSTEM_FAULT_{fault_state.name}",
            )

        rejection_reason: str | None = None
        if teleop_cmd is not None:
            valid, rejection_reason = self._validate_frame(
                teleop_cmd,
                expected_priority=CommandSourcePriority.TELEOPERATION,
                now_ns=checked_now_ns,
            )
            if valid:
                return SelectionResult(
                    selected_frame=teleop_cmd,
                    fault_state=SystemFaultState.OK,
                    active_priority=CommandSourcePriority.TELEOPERATION,
                    rejection_reason=None,
                )

        if autonomy_cmd is not None:
            autonomy_valid, autonomy_reason = self._validate_frame(
                autonomy_cmd,
                expected_priority=CommandSourcePriority.AUTONOMY,
                now_ns=checked_now_ns,
            )
            if autonomy_valid:
                return SelectionResult(
                    selected_frame=autonomy_cmd,
                    fault_state=SystemFaultState.OK,
                    active_priority=CommandSourcePriority.AUTONOMY,
                    rejection_reason=None,
                )
            rejection_reason = autonomy_reason

        return self._zero_result(
            fault_state=SystemFaultState.OK,
            rejection_reason=rejection_reason or "NO_VALID_COMMAND_SOURCE",
        )


__all__ = [
    "CommandSourcePriority",
    "DEFAULT_COMMAND_TIMEOUT_NS",
    "PrioritySelectorCore",
    "RawCommandFrame",
    "SelectionResult",
    "SystemFaultState",
]
