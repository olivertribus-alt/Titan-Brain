"""Fail-closed stop acknowledgement monitoring for TB-ACT-001B.

This module owns the temporal boundary between a control-plane stop request and
the actuator feedback that confirms the vehicle has actually stopped.  It is
deliberately free of ROS and hardware I/O: callers feed immutable contracts to
``StopAckMonitor`` and publish or actuate the resulting decision elsewhere.
Once a stop execution fault is detected, the monitor remains latched until an
explicit hardware-reset protocol is accepted.
"""

from __future__ import annotations

from collections.abc import Mapping
from enum import StrEnum
from typing import Literal, Self, TypeAlias

from pydantic import Field, ValidationError, field_validator, model_validator

from core.actuator_feedback import (
    ActuatorFeedback,
    ActuatorFeedbackConfig,
    ActuatorState,
    FeedbackInput,
    evaluate_actuator_feedback,
)
from core.types.incident import StrictFrozenModel


class StopMonitorState(StrEnum):
    """Lifecycle state of the stop-execution monitor."""

    IDLE = "idle"
    STOP_PENDING = "stop_pending"
    STOP_ACKNOWLEDGED = "stop_acknowledged"
    HARDWARE_FAULT_LATCH = "hardware_fault_latch"


class StopAckReason(StrEnum):
    """Stable machine-readable explanation for a monitor result."""

    NO_STOP_REQUEST = "no_stop_request"
    STOP_REQUESTED = "stop_requested"
    FEEDBACK_MISSING = "feedback_missing"
    FEEDBACK_MOVING = "feedback_moving"
    STOP_ACKNOWLEDGED = "stop_acknowledged"
    INVALID_FEEDBACK = "invalid_feedback"
    STALE_FEEDBACK = "stale_feedback"
    FEEDBACK_BEFORE_STOP_REQUEST = "feedback_before_stop_request"
    FEEDBACK_CORRELATION_MISMATCH = "feedback_correlation_mismatch"
    FEEDBACK_SEQUENCE_REGRESSION = "feedback_sequence_regression"
    FEEDBACK_SEQUENCE_GAP = "feedback_sequence_gap"
    SPURIOUS_MOVEMENT_AFTER_ACK = "spurious_movement_after_ack"
    STOP_TIMEOUT = "stop_timeout"
    CLOCK_REGRESSION = "clock_regression"
    HARDWARE_FAULT_LATCHED = "hardware_fault_latched"
    STOP_REQUEST_INVALID = "stop_request_invalid"
    RESET_INVALID = "reset_invalid"
    RESET_NOT_REQUIRED = "reset_not_required"
    LATCH_RESET = "latch_reset"


class StopRequest(StrictFrozenModel):
    """Immutable request that starts one stop-execution budget."""

    schema_version: Literal["0.1"] = "0.1"
    correlation_id: str = Field(min_length=1)
    sequence_id: int = Field(gt=0)
    requested_timestamp_ns: int = Field(ge=0)

    @model_validator(mode="before")
    @classmethod
    def normalize_wire_aliases(cls, value: object) -> object:
        """Accept compact wire names without weakening strict validation."""
        if not isinstance(value, Mapping):
            return value
        payload = dict(value)
        if "sequence" in payload and "sequence_id" not in payload:
            payload["sequence_id"] = payload.pop("sequence")
        if "timestamp_ns" not in payload:
            if "requested_timestamp" in payload:
                payload["requested_timestamp_ns"] = payload.pop("requested_timestamp")
            elif "timestamp" in payload:
                payload["requested_timestamp_ns"] = payload.pop("timestamp")
        return payload

    @property
    def sequence(self) -> int:
        """Compatibility alias for the wire contract's sequence field."""
        return self.sequence_id

    @property
    def requested_timestamp(self) -> int:
        """Compatibility alias for the wire contract timestamp field."""
        return self.requested_timestamp_ns


class HardwareResetRequest(StrictFrozenModel):
    """Explicit reset protocol required to clear a hardware fault latch."""

    schema_version: Literal["0.1"] = "0.1"
    reset_id: str = Field(min_length=1)
    timestamp_ns: int = Field(ge=0)


class StopAcknowledgement(StrictFrozenModel):
    """Immutable evidence that an actuator stop was observed in time."""

    schema_version: Literal["0.1"] = "0.1"
    correlation_id: str = Field(min_length=1)
    request_sequence_id: int = Field(gt=0)
    feedback_sequence_id: int = Field(gt=0)
    requested_timestamp_ns: int = Field(ge=0)
    acknowledged_timestamp_ns: int = Field(ge=0)
    measured_linear_x: float
    measured_linear_y: float
    measured_angular_z: float

    @property
    def sequence_id(self) -> int:
        """Return the actuator sequence carried by the acknowledgement."""
        return self.feedback_sequence_id

    @property
    def stop_latency_ns(self) -> int:
        """Return elapsed time from stop request to observed stop."""
        return self.acknowledged_timestamp_ns - self.requested_timestamp_ns

    @model_validator(mode="after")
    def validate_timing(self) -> Self:
        """Prevent impossible negative acknowledgement latency."""
        if self.acknowledged_timestamp_ns < self.requested_timestamp_ns:
            raise ValueError("acknowledgement cannot precede stop request")
        return self


class StopMonitorConfig(StrictFrozenModel):
    """Timing and actuator-feedback policy for one monitor instance."""

    schema_version: Literal["0.1"] = "0.1"
    stop_budget_ns: int = Field(gt=0)
    feedback_config: ActuatorFeedbackConfig


class StopMonitorResult(StrictFrozenModel):
    """Immutable monitor output suitable for audit and ROS adaptation."""

    schema_version: Literal["0.1"] = "0.1"
    state: StopMonitorState
    reason: StopAckReason
    is_latched: bool
    correlation_id: str | None = None
    request_sequence_id: int | None = Field(default=None, gt=0)
    evaluated_timestamp_ns: int = Field(ge=0)
    acknowledgement: StopAcknowledgement | None = None

    @field_validator("state", mode="before")
    @classmethod
    def parse_state(cls, value: object) -> object:
        """Accept exact state wire values without broad coercion."""
        return StopMonitorState(value) if isinstance(value, str) else value

    @field_validator("reason", mode="before")
    @classmethod
    def parse_reason(cls, value: object) -> object:
        """Accept exact reason wire values without broad coercion."""
        return StopAckReason(value) if isinstance(value, str) else value

    @model_validator(mode="after")
    def validate_result_shape(self) -> Self:
        """Keep latch, acknowledgement, and state flags mutually consistent."""
        if self.state is StopMonitorState.HARDWARE_FAULT_LATCH:
            if not self.is_latched or self.acknowledgement is not None:
                raise ValueError("hardware fault results must remain latched")
        elif self.is_latched:
            raise ValueError("only hardware fault state may be latched")

        has_ack = self.acknowledgement is not None
        if (self.state is StopMonitorState.STOP_ACKNOWLEDGED) != has_ack:
            raise ValueError("acknowledgement must match monitor state")
        return self


StopRequestInput: TypeAlias = StopRequest | Mapping[str, object] | None
ResetInput: TypeAlias = HardwareResetRequest | Mapping[str, object] | None


def _checked_now_ns(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _parse_stop_request(
    value: StopRequestInput,
) -> tuple[StopRequest | None, bool]:
    if value is None:
        return None, False
    if isinstance(value, StopRequest):
        return value, True
    try:
        return StopRequest.model_validate(value), True
    except (TypeError, ValidationError):
        return None, True


def _parse_reset_request(
    value: ResetInput,
) -> tuple[HardwareResetRequest | None, bool]:
    if value is None:
        return None, False
    if isinstance(value, HardwareResetRequest):
        return value, True
    try:
        return HardwareResetRequest.model_validate(value), True
    except (TypeError, ValidationError):
        return None, True


def _extract_correlation(value: FeedbackInput) -> str | None:
    if isinstance(value, ActuatorFeedback):
        return value.correlation_id
    if isinstance(value, Mapping):
        candidate = value.get("correlation_id")
        return candidate if isinstance(candidate, str) else None
    return None


class StopAckMonitor:
    """Stateful, dependency-free stop acknowledgement monitor.

    Invalid or stale feedback received during a pending stop immediately
    latches a hardware fault.  A moving but valid actuator remains pending until
    the stop budget expires, at which point it is latched as a timeout.  The
    latch is never cleared by feedback or time passing; movement observed after
    an acknowledgement is treated as spurious motion and latched immediately.
    Only ``reset_fault`` accepts an explicit reset protocol.
    """

    def __init__(self, config: StopMonitorConfig) -> None:
        self._config = config
        self._state = StopMonitorState.IDLE
        self._request: StopRequest | None = None
        self._acknowledgement: StopAcknowledgement | None = None
        self._last_feedback_sequence_id: int | None = None
        self._last_now_ns: int | None = None
        self._fault_reason: StopAckReason | None = None
        self._fault_timestamp_ns = 0

    @property
    def config(self) -> StopMonitorConfig:
        """Return the immutable monitor policy."""
        return self._config

    @property
    def state(self) -> StopMonitorState:
        """Return the current monitor lifecycle state."""
        return self._state

    @property
    def is_latched(self) -> bool:
        """Return whether a persistent hardware fault is active."""
        return self._state is StopMonitorState.HARDWARE_FAULT_LATCH

    @property
    def acknowledgement(self) -> StopAcknowledgement | None:
        """Return the immutable acknowledgement, if one was issued."""
        return self._acknowledgement

    @property
    def fault_reason(self) -> StopAckReason | None:
        """Return the specific reason that caused the persistent latch."""
        return self._fault_reason

    def _result(
        self,
        reason: StopAckReason,
        *,
        timestamp_ns: int,
    ) -> StopMonitorResult:
        request = self._request
        return StopMonitorResult(
            state=self._state,
            reason=reason,
            is_latched=self.is_latched,
            correlation_id=request.correlation_id if request else None,
            request_sequence_id=request.sequence_id if request else None,
            evaluated_timestamp_ns=timestamp_ns,
            acknowledgement=self._acknowledgement,
        )

    def _latch(
        self,
        reason: StopAckReason,
        *,
        timestamp_ns: int,
    ) -> StopMonitorResult:
        self._state = StopMonitorState.HARDWARE_FAULT_LATCH
        self._acknowledgement = None
        self._fault_reason = reason
        self._fault_timestamp_ns = timestamp_ns
        self._last_now_ns = timestamp_ns
        return self._result(
            reason,
            timestamp_ns=timestamp_ns,
        )

    def _latched_result(self) -> StopMonitorResult:
        return self._result(
            self._fault_reason or StopAckReason.HARDWARE_FAULT_LATCHED,
            timestamp_ns=self._fault_timestamp_ns,
        )

    def _clock_checked(self, now_ns: object) -> int | None:
        checked = _checked_now_ns(now_ns)
        if checked is None:
            if self._request is not None:
                self._latch(
                    StopAckReason.CLOCK_REGRESSION,
                    timestamp_ns=self._last_now_ns or 0,
                )
            return None
        if self._last_now_ns is not None and checked < self._last_now_ns:
            if self._request is not None:
                self._latch(
                    StopAckReason.CLOCK_REGRESSION,
                    timestamp_ns=self._last_now_ns,
                )
            return None
        self._last_now_ns = checked
        return checked

    def _budget_expired(self, now_ns: int) -> bool:
        request = self._request
        if request is None:
            return False
        return now_ns - request.requested_timestamp_ns >= self._config.stop_budget_ns

    def _pending_result(
        self,
        reason: StopAckReason,
        *,
        timestamp_ns: int,
    ) -> StopMonitorResult:
        if self._budget_expired(timestamp_ns):
            return self._latch(StopAckReason.STOP_TIMEOUT, timestamp_ns=timestamp_ns)
        return self._result(reason, timestamp_ns=timestamp_ns)

    def _sequence_fault_reason(self, sequence_id: int) -> StopAckReason | None:
        """Reject replay and forward gaps at the actuator feedback boundary."""
        previous = self._last_feedback_sequence_id
        if previous is None:
            return None
        if sequence_id <= previous:
            return StopAckReason.FEEDBACK_SEQUENCE_REGRESSION
        if sequence_id != previous + 1:
            return StopAckReason.FEEDBACK_SEQUENCE_GAP
        return None

    def request_stop(
        self,
        request: StopRequestInput,
        *,
        now_ns: object,
    ) -> StopMonitorResult:
        """Start or replace a stop window unless a hardware fault is latched."""
        if self.is_latched:
            return self._latched_result()

        checked_now = self._clock_checked(now_ns)
        if checked_now is None:
            return self._result(
                StopAckReason.CLOCK_REGRESSION,
                timestamp_ns=self._last_now_ns or 0,
            )

        parsed_request, _ = _parse_stop_request(request)
        if parsed_request is None:
            return self._result(
                StopAckReason.STOP_REQUEST_INVALID,
                timestamp_ns=checked_now,
            )
        if parsed_request.requested_timestamp_ns > checked_now:
            return self._result(
                StopAckReason.STOP_REQUEST_INVALID,
                timestamp_ns=checked_now,
            )

        self._request = parsed_request
        self._state = StopMonitorState.STOP_PENDING
        self._acknowledgement = None
        self._last_feedback_sequence_id = None
        self._fault_reason = None
        self._fault_timestamp_ns = 0
        return self._result(
            StopAckReason.STOP_REQUESTED,
            timestamp_ns=checked_now,
        )

    def observe_feedback(
        self,
        feedback: FeedbackInput,
        *,
        now_ns: object,
    ) -> StopMonitorResult:
        """Consume one actuator sample and acknowledge or latch the stop."""
        if self.is_latched:
            return self._latched_result()
        if self._state is StopMonitorState.STOP_ACKNOWLEDGED:
            return self._observe_acknowledged_feedback(feedback, now_ns=now_ns)
        if self._request is None:
            checked_now = _checked_now_ns(now_ns) or 0
            return self._result(
                StopAckReason.NO_STOP_REQUEST,
                timestamp_ns=checked_now,
            )

        checked_feedback_now = self._clock_checked(now_ns)
        if checked_feedback_now is None:
            return self._result(
                StopAckReason.CLOCK_REGRESSION,
                timestamp_ns=self._last_now_ns or 0,
            )
        if feedback is None:
            return self._pending_result(
                StopAckReason.FEEDBACK_MISSING,
                timestamp_ns=checked_feedback_now,
            )

        request = self._request
        status = evaluate_actuator_feedback(
            feedback,
            now_ns=checked_feedback_now,
            expected_correlation_id=request.correlation_id,
            config=self._config.feedback_config,
        )
        if status.state is ActuatorState.STALE_DATA:
            return self._latch(
                StopAckReason.STALE_FEEDBACK,
                timestamp_ns=checked_feedback_now,
            )
        if status.state is ActuatorState.INVALID_DATA:
            if _extract_correlation(feedback) not in {
                None,
                request.correlation_id,
            }:
                reason = StopAckReason.FEEDBACK_CORRELATION_MISMATCH
            else:
                reason = StopAckReason.INVALID_FEEDBACK
            return self._latch(reason, timestamp_ns=checked_feedback_now)

        parsed_feedback = (
            feedback
            if isinstance(feedback, ActuatorFeedback)
            else ActuatorFeedback.model_validate(feedback)
        )
        if parsed_feedback.timestamp_ns < request.requested_timestamp_ns:
            return self._latch(
                StopAckReason.FEEDBACK_BEFORE_STOP_REQUEST,
                timestamp_ns=checked_feedback_now,
            )
        sequence_fault = self._sequence_fault_reason(parsed_feedback.sequence_id)
        if sequence_fault is not None:
            return self._latch(
                sequence_fault,
                timestamp_ns=checked_feedback_now,
            )
        self._last_feedback_sequence_id = parsed_feedback.sequence_id

        if status.state is ActuatorState.STOPPED:
            self._acknowledgement = StopAcknowledgement(
                correlation_id=request.correlation_id,
                request_sequence_id=request.sequence_id,
                feedback_sequence_id=parsed_feedback.sequence_id,
                requested_timestamp_ns=request.requested_timestamp_ns,
                acknowledged_timestamp_ns=checked_feedback_now,
                measured_linear_x=parsed_feedback.measured_linear_x,
                measured_linear_y=parsed_feedback.measured_linear_y,
                measured_angular_z=parsed_feedback.measured_angular_z,
            )
            self._state = StopMonitorState.STOP_ACKNOWLEDGED
            return self._result(
                StopAckReason.STOP_ACKNOWLEDGED,
                timestamp_ns=checked_feedback_now,
            )

        return self._pending_result(
            StopAckReason.FEEDBACK_MOVING,
            timestamp_ns=checked_feedback_now,
        )

    def _observe_acknowledged_feedback(
        self,
        feedback: FeedbackInput,
        *,
        now_ns: object,
    ) -> StopMonitorResult:
        """Continue validating feedback after a stop acknowledgement.

        Acknowledgement is not permission to stop monitoring the actuator.  A
        fresh moving sample after the acknowledged stop is evidence of
        uncommanded motion and therefore enters the sticky hardware-fault
        latch.  Replayed, stale, desynchronised, or malformed samples are
        treated as faults as well.
        """
        if feedback is None:
            return self._result(
                StopAckReason.STOP_ACKNOWLEDGED,
                timestamp_ns=self._last_now_ns or 0,
            )

        request = self._request
        if request is None:  # pragma: no cover - acknowledgement requires one
            return self._result(
                StopAckReason.NO_STOP_REQUEST,
                timestamp_ns=self._last_now_ns or 0,
            )

        checked_feedback_now = self._clock_checked(now_ns)
        if checked_feedback_now is None:
            return self._result(
                StopAckReason.CLOCK_REGRESSION,
                timestamp_ns=self._last_now_ns or 0,
            )

        status = evaluate_actuator_feedback(
            feedback,
            now_ns=checked_feedback_now,
            expected_correlation_id=request.correlation_id,
            config=self._config.feedback_config,
        )
        if status.state is ActuatorState.STALE_DATA:
            return self._latch(
                StopAckReason.STALE_FEEDBACK,
                timestamp_ns=checked_feedback_now,
            )
        if status.state is ActuatorState.INVALID_DATA:
            if _extract_correlation(feedback) not in {
                None,
                request.correlation_id,
            }:
                reason = StopAckReason.FEEDBACK_CORRELATION_MISMATCH
            else:
                reason = StopAckReason.INVALID_FEEDBACK
            return self._latch(reason, timestamp_ns=checked_feedback_now)

        parsed_feedback = (
            feedback
            if isinstance(feedback, ActuatorFeedback)
            else ActuatorFeedback.model_validate(feedback)
        )
        if parsed_feedback.timestamp_ns < request.requested_timestamp_ns:
            return self._latch(
                StopAckReason.FEEDBACK_BEFORE_STOP_REQUEST,
                timestamp_ns=checked_feedback_now,
            )
        sequence_fault = self._sequence_fault_reason(parsed_feedback.sequence_id)
        if sequence_fault is not None:
            return self._latch(
                sequence_fault,
                timestamp_ns=checked_feedback_now,
            )
        self._last_feedback_sequence_id = parsed_feedback.sequence_id

        if status.state is ActuatorState.MOVING:
            return self._latch(
                StopAckReason.SPURIOUS_MOVEMENT_AFTER_ACK,
                timestamp_ns=checked_feedback_now,
            )
        return self._result(
            StopAckReason.STOP_ACKNOWLEDGED,
            timestamp_ns=checked_feedback_now,
        )

    def tick(self, *, now_ns: object) -> StopMonitorResult:
        """Advance the stop budget when no new feedback message is available."""
        if self.is_latched:
            return self._latched_result()
        if self._state is StopMonitorState.STOP_ACKNOWLEDGED:
            return self._result(
                StopAckReason.STOP_ACKNOWLEDGED,
                timestamp_ns=self._last_now_ns or 0,
            )
        if self._request is None:
            checked_now = _checked_now_ns(now_ns) or 0
            return self._result(
                StopAckReason.NO_STOP_REQUEST,
                timestamp_ns=checked_now,
            )
        checked_tick_now = self._clock_checked(now_ns)
        if checked_tick_now is None:
            return self._result(
                StopAckReason.CLOCK_REGRESSION,
                timestamp_ns=self._last_now_ns or 0,
            )
        return self._pending_result(
            StopAckReason.FEEDBACK_MISSING,
            timestamp_ns=checked_tick_now,
        )

    def reset_fault(
        self,
        reset: ResetInput,
        *,
        now_ns: object,
    ) -> StopMonitorResult:
        """Clear the latch only after a valid, explicit hardware reset."""
        if not self.is_latched:
            checked_now = _checked_now_ns(now_ns) or 0
            return self._result(
                StopAckReason.RESET_NOT_REQUIRED,
                timestamp_ns=checked_now,
            )

        checked_reset_now = _checked_now_ns(now_ns)
        parsed_reset, was_supplied = _parse_reset_request(reset)
        if (
            checked_reset_now is None
            or parsed_reset is None
            or not was_supplied
            or parsed_reset.timestamp_ns > checked_reset_now
            or checked_reset_now < self._fault_timestamp_ns
        ):
            return self._result(
                StopAckReason.RESET_INVALID,
                timestamp_ns=self._fault_timestamp_ns,
            )

        self._state = StopMonitorState.IDLE
        self._request = None
        self._acknowledgement = None
        self._last_feedback_sequence_id = None
        self._fault_reason = None
        self._fault_timestamp_ns = 0
        self._last_now_ns = checked_reset_now
        return self._result(
            StopAckReason.LATCH_RESET,
            timestamp_ns=checked_reset_now,
        )

    def reset(self, *, reset_id: str, now_ns: object) -> StopMonitorResult:
        """Convenience wrapper for an explicit reset protocol."""
        return self.reset_fault(
            {"reset_id": reset_id, "timestamp_ns": now_ns},
            now_ns=now_ns,
        )


__all__ = [
    "HardwareResetRequest",
    "ResetInput",
    "StopAckMonitor",
    "StopAckReason",
    "StopAcknowledgement",
    "StopMonitorConfig",
    "StopMonitorResult",
    "StopMonitorState",
    "StopRequest",
    "StopRequestInput",
]
