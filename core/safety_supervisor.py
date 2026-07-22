"""Dependency-free external safety-loop supervisor for TB-SAFE-001A.

The supervisor owns no ROS or hardware resources.  It only evaluates a
monotonic heartbeat matrix and emits an immutable relay request.  Any invalid
input, clock regression, heartbeat timeout, or component-reported error is
fail-closed and cannot release a trip automatically.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import StrEnum
from types import MappingProxyType
from typing import Mapping

DEFAULT_HEARTBEAT_TIMEOUT_NS = 200_000_000


class HeartbeatChannel(StrEnum):
    """Critical software components supervised by the safety loop."""

    CONTROL_ARBITER = "control_arbiter"
    ACTUATOR_MONITOR = "actuator_monitor"
    ODOMETRY = "odometry"


class SafetyState(StrEnum):
    """Fail-closed supervisor state."""

    INITIALIZING = "initializing"
    OK = "ok"
    TRIPPED = "tripped"
    HARDWARE_FAULT_LATCH = "hardware_fault_latch"


class RelayRequest(StrEnum):
    """Requested physical state of the external safety loop."""

    REQUEST_SAFETY_CLOSED = "request_safety_closed"
    REQUEST_SAFETY_OPEN = "request_safety_open"


class SafetyReason(StrEnum):
    """Machine-readable reason for the latest supervisor result."""

    INITIALIZING = "initializing"
    ALL_HEARTBEATS_HEALTHY = "all_heartbeats_healthy"
    HEARTBEAT_RECEIVED = "heartbeat_received"
    HEARTBEAT_TIMEOUT = "heartbeat_timeout"
    INITIALIZATION_TIMEOUT = "initialization_timeout"
    INVALID_HEARTBEAT = "invalid_heartbeat"
    INVALID_TIMESTAMP = "invalid_timestamp"
    HEARTBEAT_ERROR = "heartbeat_error"
    CLOCK_REGRESSION = "clock_regression"
    HARDWARE_FAULT = "hardware_fault"
    HARDWARE_FAULT_LATCHED = "hardware_fault_latched"


@dataclass(frozen=True, slots=True)
class SafetySupervisorConfig:
    """Per-channel heartbeat budgets in monotonic nanoseconds."""

    control_arbiter_timeout_ns: int = DEFAULT_HEARTBEAT_TIMEOUT_NS
    actuator_monitor_timeout_ns: int = DEFAULT_HEARTBEAT_TIMEOUT_NS
    odometry_timeout_ns: int = DEFAULT_HEARTBEAT_TIMEOUT_NS
    initialization_timeout_ns: int = DEFAULT_HEARTBEAT_TIMEOUT_NS

    def __post_init__(self) -> None:
        """Reject booleans, negative values, and non-integral budgets."""
        values = (
            self.control_arbiter_timeout_ns,
            self.actuator_monitor_timeout_ns,
            self.odometry_timeout_ns,
            self.initialization_timeout_ns,
        )
        if any(
            isinstance(value, bool) or not isinstance(value, int) or value <= 0
            for value in values
        ):
            raise ValueError("all safety heartbeat budgets must be positive integers")

    def timeout_for(self, channel: HeartbeatChannel | str) -> int:
        """Return the configured timeout for one channel."""
        checked_channel = _coerce_channel(channel)
        if checked_channel is None:
            raise ValueError(f"unknown heartbeat channel: {channel!r}")
        return {
            HeartbeatChannel.CONTROL_ARBITER: self.control_arbiter_timeout_ns,
            HeartbeatChannel.ACTUATOR_MONITOR: self.actuator_monitor_timeout_ns,
            HeartbeatChannel.ODOMETRY: self.odometry_timeout_ns,
        }[checked_channel]

    @property
    def channel_timeouts(self) -> Mapping[HeartbeatChannel, int]:
        """Return an immutable snapshot of all channel budgets."""
        return MappingProxyType(
            {
                channel: self.timeout_for(channel)
                for channel in HeartbeatChannel
            }
        )


@dataclass(frozen=True, slots=True)
class Heartbeat:
    """Validated heartbeat evidence accepted by the supervisor."""

    channel: HeartbeatChannel
    timestamp_ns: int
    healthy: bool = True

    def __post_init__(self) -> None:
        """Keep heartbeat evidence strict and monotonic-ready."""
        if not isinstance(self.channel, HeartbeatChannel):
            raise ValueError("heartbeat channel must be a HeartbeatChannel")
        if (
            isinstance(self.timestamp_ns, bool)
            or not isinstance(self.timestamp_ns, int)
            or self.timestamp_ns < 0
        ):
            raise ValueError("heartbeat timestamp must be a non-negative integer")
        if not isinstance(self.healthy, bool):
            raise ValueError("heartbeat healthy flag must be boolean")


@dataclass(frozen=True, slots=True)
class SafetySupervisorResult:
    """Immutable, auditable supervisor decision and relay request."""

    state: SafetyState
    relay_request: RelayRequest
    reason: SafetyReason
    evaluated_at_ns: int
    is_safe: bool
    missing_channels: tuple[HeartbeatChannel, ...] = ()
    failed_channels: tuple[HeartbeatChannel, ...] = ()
    detail: str | None = None

    def __post_init__(self) -> None:
        """Enforce the fail-closed state/relay invariant."""
        if not isinstance(self.state, SafetyState):
            raise ValueError("state must be a SafetyState")
        if not isinstance(self.relay_request, RelayRequest):
            raise ValueError("relay_request must be a RelayRequest")
        if not isinstance(self.reason, SafetyReason):
            raise ValueError("reason must be a SafetyReason")
        if (
            isinstance(self.evaluated_at_ns, bool)
            or not isinstance(self.evaluated_at_ns, int)
            or self.evaluated_at_ns < 0
        ):
            raise ValueError("evaluated_at_ns must be a non-negative integer")
        if not isinstance(self.is_safe, bool):
            raise ValueError("is_safe must be boolean")
        expected_safe = self.state is SafetyState.OK
        expected_relay = (
            RelayRequest.REQUEST_SAFETY_CLOSED
            if expected_safe
            else RelayRequest.REQUEST_SAFETY_OPEN
        )
        if self.is_safe is not expected_safe:
            raise ValueError("is_safe must match the supervisor state")
        if self.relay_request is not expected_relay:
            raise ValueError("relay request must be fail-closed outside OK")
        if not isinstance(self.missing_channels, tuple) or any(
            not isinstance(channel, HeartbeatChannel)
            for channel in self.missing_channels
        ):
            raise ValueError("missing_channels must contain heartbeat channels")
        if not isinstance(self.failed_channels, tuple) or any(
            not isinstance(channel, HeartbeatChannel)
            for channel in self.failed_channels
        ):
            raise ValueError("failed_channels must contain heartbeat channels")
        if self.detail is not None and (
            not isinstance(self.detail, str) or not self.detail.strip()
        ):
            raise ValueError("detail must not be blank")


def _coerce_channel(value: object) -> HeartbeatChannel | None:
    """Convert a public channel value without accepting arbitrary objects."""
    if isinstance(value, HeartbeatChannel):
        return value
    if not isinstance(value, str):
        return None
    try:
        return HeartbeatChannel(value)
    except ValueError:
        return None


def _checked_timestamp(value: object) -> int | None:
    """Return a strict non-negative monotonic timestamp, if valid."""
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


class SafetySupervisor:
    """Stateful heartbeat matrix with sticky fail-closed transitions."""

    _CHANNELS = tuple(HeartbeatChannel)

    def __init__(
        self,
        config: SafetySupervisorConfig | None = None,
        *,
        started_at_ns: object | None = None,
    ) -> None:
        self._config = config or SafetySupervisorConfig()
        start_value = (
            time.monotonic_ns() if started_at_ns is None else started_at_ns
        )
        start_ns = _checked_timestamp(start_value)
        if start_ns is None:
            raise ValueError("started_at_ns must be a non-negative integer")
        self._initializing_since_ns = start_ns
        self._last_now_ns = start_ns
        self._last_heartbeat_ns: dict[HeartbeatChannel, int | None] = {
            channel: None for channel in self._CHANNELS
        }
        self._state = SafetyState.INITIALIZING
        self._trip_reason: SafetyReason | None = None
        self._trip_detail: str | None = None
        self._trip_channels: tuple[HeartbeatChannel, ...] = ()
        self._last_result = self._make_result(
            evaluated_at_ns=start_ns,
            reason=SafetyReason.INITIALIZING,
            missing_channels=self._CHANNELS,
        )

    @property
    def config(self) -> SafetySupervisorConfig:
        """Return the immutable supervisor configuration."""
        return self._config

    @property
    def state(self) -> SafetyState:
        """Return the current fail-closed state."""
        return self._state

    @property
    def last_result(self) -> SafetySupervisorResult:
        """Return the most recent decision emitted by the supervisor."""
        return self._last_result

    @property
    def started_at_ns(self) -> int:
        """Return the monotonic start boundary used for initialization."""
        return self._initializing_since_ns

    @property
    def last_heartbeats(self) -> Mapping[HeartbeatChannel, int | None]:
        """Return an immutable snapshot of the heartbeat matrix."""
        return MappingProxyType(dict(self._last_heartbeat_ns))

    def last_heartbeat_ns(self, channel: HeartbeatChannel | str) -> int | None:
        """Return the last accepted timestamp for one channel."""
        checked_channel = _coerce_channel(channel)
        if checked_channel is None:
            raise ValueError(f"unknown heartbeat channel: {channel!r}")
        return self._last_heartbeat_ns[checked_channel]

    def evaluate(self, now_ns: object | None = None) -> SafetySupervisorResult:
        """Evaluate freshness using a monotonic timestamp or the system clock."""
        checked_now = _checked_timestamp(
            time.monotonic_ns() if now_ns is None else now_ns
        )
        if checked_now is None:
            return self._trip(
                reason=SafetyReason.INVALID_TIMESTAMP,
                evaluated_at_ns=self._last_now_ns,
                detail="Supervisor time must be a non-negative integer.",
            )
        if checked_now < self._last_now_ns:
            return self._trip(
                reason=SafetyReason.CLOCK_REGRESSION,
                evaluated_at_ns=self._last_now_ns,
                detail="Monotonic supervisor time moved backwards.",
            )
        self._last_now_ns = checked_now

        if self._state is SafetyState.HARDWARE_FAULT_LATCH:
            return self._remember(
                self._make_result(
                    evaluated_at_ns=checked_now,
                    reason=SafetyReason.HARDWARE_FAULT_LATCHED,
                    detail=self._trip_detail,
                    failed_channels=self._trip_channels,
                )
            )
        if self._state is SafetyState.TRIPPED:
            return self._remember(
                self._make_result(
                    evaluated_at_ns=checked_now,
                    reason=self._trip_reason or SafetyReason.HEARTBEAT_TIMEOUT,
                    detail=self._trip_detail,
                    failed_channels=self._trip_channels,
                )
            )

        missing = tuple(
            channel
            for channel in self._CHANNELS
            if self._last_heartbeat_ns[channel] is None
        )
        stale = self._stale_channels(checked_now)
        if self._state is SafetyState.INITIALIZING:
            if stale:
                return self._trip(
                    reason=SafetyReason.HEARTBEAT_TIMEOUT,
                    evaluated_at_ns=checked_now,
                    failed_channels=stale,
                    detail="A registered heartbeat exceeded its channel budget.",
                )
            if (
                checked_now - self._initializing_since_ns
                > self._config.initialization_timeout_ns
            ):
                return self._trip(
                    reason=SafetyReason.INITIALIZATION_TIMEOUT,
                    evaluated_at_ns=checked_now,
                    failed_channels=missing,
                    detail="Not all required heartbeat channels registered in time.",
                )
            if not missing:
                self._state = SafetyState.OK
                return self._remember(
                    self._make_result(
                        evaluated_at_ns=checked_now,
                        reason=SafetyReason.ALL_HEARTBEATS_HEALTHY,
                    )
                )
            return self._remember(
                self._make_result(
                    evaluated_at_ns=checked_now,
                    reason=SafetyReason.INITIALIZING,
                    missing_channels=missing,
                )
            )

        if stale:
            return self._trip(
                reason=SafetyReason.HEARTBEAT_TIMEOUT,
                evaluated_at_ns=checked_now,
                failed_channels=stale,
                detail="A heartbeat exceeded its channel budget.",
            )
        return self._remember(
            self._make_result(
                evaluated_at_ns=checked_now,
                reason=SafetyReason.ALL_HEARTBEATS_HEALTHY,
            )
        )

    def receive_heartbeat(
        self,
        channel: HeartbeatChannel | str | object,
        *,
        timestamp_ns: object | None = None,
        healthy: object = True,
        error: object | None = None,
    ) -> SafetySupervisorResult:
        """Record one pulse and immediately re-evaluate the matrix."""
        checked_now = _checked_timestamp(
            time.monotonic_ns() if timestamp_ns is None else timestamp_ns
        )
        checked_channel = _coerce_channel(channel)
        if checked_now is None:
            return self._trip(
                reason=SafetyReason.INVALID_TIMESTAMP,
                evaluated_at_ns=self._last_now_ns,
                detail="Heartbeat timestamp must be a non-negative integer.",
                failed_channels=(checked_channel,)
                if checked_channel is not None
                else (),
            )
        if checked_now < self._last_now_ns:
            return self._trip(
                reason=SafetyReason.CLOCK_REGRESSION,
                evaluated_at_ns=self._last_now_ns,
                detail="Heartbeat timestamp moved backwards.",
                failed_channels=(checked_channel,)
                if checked_channel is not None
                else (),
            )
        self._last_now_ns = checked_now
        if checked_channel is None:
            return self._trip(
                reason=SafetyReason.INVALID_HEARTBEAT,
                evaluated_at_ns=checked_now,
                detail="Heartbeat channel is not registered.",
            )
        previous = self._last_heartbeat_ns[checked_channel]
        if previous is not None and checked_now < previous:
            return self._trip(
                reason=SafetyReason.CLOCK_REGRESSION,
                evaluated_at_ns=checked_now,
                failed_channels=(checked_channel,),
                detail="Heartbeat channel timestamp moved backwards.",
            )
        if not isinstance(healthy, bool):
            return self._trip(
                reason=SafetyReason.INVALID_HEARTBEAT,
                evaluated_at_ns=checked_now,
                failed_channels=(checked_channel,),
                detail="Heartbeat healthy flag must be boolean.",
            )
        if error is not None and (
            not isinstance(error, str) or not error.strip()
        ):
            return self._trip(
                reason=SafetyReason.INVALID_HEARTBEAT,
                evaluated_at_ns=checked_now,
                failed_channels=(checked_channel,),
                detail="Heartbeat error detail must be non-blank text.",
            )
        if not healthy or error is not None:
            return self._trip(
                reason=SafetyReason.HEARTBEAT_ERROR,
                evaluated_at_ns=checked_now,
                failed_channels=(checked_channel,),
                detail=(
                    str(error)
                    if error is not None
                    else "Component reported unhealthy."
                ),
            )

        if self._state is SafetyState.HARDWARE_FAULT_LATCH:
            return self.evaluate(now_ns=checked_now)
        if self._state is SafetyState.TRIPPED:
            return self.evaluate(now_ns=checked_now)
        self._last_heartbeat_ns[checked_channel] = checked_now
        result = self.evaluate(now_ns=checked_now)
        if result.reason is SafetyReason.ALL_HEARTBEATS_HEALTHY:
            return result
        return self._remember(
            self._make_result(
                evaluated_at_ns=checked_now,
                reason=(
                    SafetyReason.HEARTBEAT_RECEIVED
                    if result.state is SafetyState.INITIALIZING
                    else result.reason
                ),
                missing_channels=result.missing_channels,
                failed_channels=result.failed_channels,
                detail=result.detail,
            )
        )

    def record_heartbeat(
        self,
        channel: HeartbeatChannel | str | object,
        timestamp_ns: object | None = None,
        *,
        healthy: object = True,
        error: object | None = None,
    ) -> SafetySupervisorResult:
        """Positional-timestamp alias for integrations with callback APIs."""
        return self.receive_heartbeat(
            channel,
            timestamp_ns=timestamp_ns,
            healthy=healthy,
            error=error,
        )

    def heartbeat(
        self,
        channel: HeartbeatChannel | str | object,
        *,
        timestamp_ns: object | None = None,
        healthy: object = True,
        error: object | None = None,
    ) -> SafetySupervisorResult:
        """Short alias for :meth:`receive_heartbeat`."""
        return self.receive_heartbeat(
            channel,
            timestamp_ns=timestamp_ns,
            healthy=healthy,
            error=error,
        )

    def tick(self, now_ns: object | None = None) -> SafetySupervisorResult:
        """Short alias for :meth:`evaluate`."""
        return self.evaluate(now_ns=now_ns)

    def latch_hardware_fault(
        self,
        *,
        now_ns: object | None = None,
        detail: str = "Hardware safety-loop fault latched.",
    ) -> SafetySupervisorResult:
        """Enter the sticky hardware latch; no automatic release is provided."""
        checked_now = _checked_timestamp(
            time.monotonic_ns() if now_ns is None else now_ns
        )
        if checked_now is None:
            checked_now = self._last_now_ns
            detail = "Invalid latch timestamp; hardware fault remains latched."
        elif checked_now < self._last_now_ns:
            checked_now = self._last_now_ns
            detail = "Clock regression while latching hardware fault."
        self._last_now_ns = checked_now
        self._state = SafetyState.HARDWARE_FAULT_LATCH
        self._trip_reason = SafetyReason.HARDWARE_FAULT
        self._trip_detail = detail
        self._trip_channels = self._CHANNELS
        return self._remember(
            self._make_result(
                evaluated_at_ns=checked_now,
                reason=SafetyReason.HARDWARE_FAULT,
                failed_channels=self._CHANNELS,
                detail=detail,
            )
        )

    def _stale_channels(self, now_ns: int) -> tuple[HeartbeatChannel, ...]:
        stale: list[HeartbeatChannel] = []
        for channel in self._CHANNELS:
            last_heartbeat_ns = self._last_heartbeat_ns[channel]
            if (
                last_heartbeat_ns is not None
                and now_ns - last_heartbeat_ns > self._config.timeout_for(channel)
            ):
                stale.append(channel)
        return tuple(stale)

    def _trip(
        self,
        *,
        reason: SafetyReason,
        evaluated_at_ns: int,
        detail: str,
        failed_channels: tuple[HeartbeatChannel, ...] = (),
    ) -> SafetySupervisorResult:
        if self._state is SafetyState.HARDWARE_FAULT_LATCH:
            return self._remember(
                self._make_result(
                    evaluated_at_ns=evaluated_at_ns,
                    reason=SafetyReason.HARDWARE_FAULT_LATCHED,
                    failed_channels=self._trip_channels,
                    detail=self._trip_detail,
                )
            )
        if self._state is not SafetyState.TRIPPED:
            self._state = SafetyState.TRIPPED
            self._trip_reason = reason
            self._trip_detail = detail
            self._trip_channels = failed_channels
        return self._remember(
            self._make_result(
                evaluated_at_ns=evaluated_at_ns,
                reason=self._trip_reason or reason,
                failed_channels=self._trip_channels,
                detail=self._trip_detail or detail,
            )
        )

    def _make_result(
        self,
        *,
        evaluated_at_ns: int,
        reason: SafetyReason,
        missing_channels: tuple[HeartbeatChannel, ...] = (),
        failed_channels: tuple[HeartbeatChannel, ...] = (),
        detail: str | None = None,
    ) -> SafetySupervisorResult:
        return SafetySupervisorResult(
            state=self._state,
            relay_request=(
                RelayRequest.REQUEST_SAFETY_CLOSED
                if self._state is SafetyState.OK
                else RelayRequest.REQUEST_SAFETY_OPEN
            ),
            reason=reason,
            evaluated_at_ns=evaluated_at_ns,
            is_safe=self._state is SafetyState.OK,
            missing_channels=missing_channels,
            failed_channels=failed_channels,
            detail=detail,
        )

    def _remember(self, result: SafetySupervisorResult) -> SafetySupervisorResult:
        self._last_result = result
        return result


# Compatibility aliases for callers that use the longer contract names.
SupervisorConfig = SafetySupervisorConfig
SupervisorResult = SafetySupervisorResult
SafetySupervisorState = SafetyState
SafetyRelayRequest = RelayRequest
HeartbeatSource = HeartbeatChannel


__all__ = [
    "DEFAULT_HEARTBEAT_TIMEOUT_NS",
    "Heartbeat",
    "HeartbeatChannel",
    "HeartbeatSource",
    "RelayRequest",
    "SafetyReason",
    "SafetyRelayRequest",
    "SafetyState",
    "SafetySupervisor",
    "SafetySupervisorConfig",
    "SafetySupervisorResult",
    "SafetySupervisorState",
    "SupervisorConfig",
    "SupervisorResult",
]
