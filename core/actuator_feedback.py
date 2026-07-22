"""Dependency-free actuator feedback validation for TB-ACT-001A.

The module deliberately stops at *evaluating* feedback.  Timing windows,
hardware latching, and escalation belong to TB-ACT-001B and are therefore not
part of this contract.  A valid, fresh sample is classified using the three
measured body-frame axes; every rejected sample is fail-closed and is never
reported as stopped.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from enum import StrEnum
from typing import Literal, Self, TypeAlias

from pydantic import Field, ValidationError, field_validator, model_validator

from core.types.incident import StrictFrozenModel


class ActuatorState(StrEnum):
    """Stable classification of one actuator feedback sample."""

    STOPPED = "stopped"
    MOVING = "moving"
    INVALID_DATA = "invalid_data"
    STALE_DATA = "stale_data"


class ActuatorFeedbackConfig(StrictFrozenModel):
    """Thresholds used to classify fresh actuator feedback."""

    schema_version: Literal["0.1"] = "0.1"
    epsilon_stop_linear: float = Field(ge=0.0)
    epsilon_stop_angular: float = Field(ge=0.0)
    stale_threshold_ns: int = Field(gt=0)


class ActuatorFeedback(StrictFrozenModel):
    """Immutable measured actuator response in the robot body frame."""

    schema_version: Literal["0.1"] = "0.1"
    measured_linear_x: float
    measured_linear_y: float
    measured_angular_z: float
    correlation_id: str = Field(min_length=1)
    sequence_id: int = Field(gt=0)
    timestamp_ns: int = Field(ge=0)

    @model_validator(mode="before")
    @classmethod
    def normalize_wire_aliases(cls, value: object) -> object:
        """Accept compact wire names while retaining canonical Python fields."""
        if not isinstance(value, Mapping):
            return value
        payload = dict(value)
        if "sequence" in payload and "sequence_id" not in payload:
            payload["sequence_id"] = payload.pop("sequence")
        if "timestamp" in payload and "timestamp_ns" not in payload:
            payload["timestamp_ns"] = payload.pop("timestamp")
        return payload

    @property
    def sequence(self) -> int:
        """Compatibility alias for the wire contract's sequence field."""
        return self.sequence_id

    @property
    def timestamp(self) -> int:
        """Compatibility alias for the wire contract's timestamp field."""
        return self.timestamp_ns


class ActuatorStatus(StrictFrozenModel):
    """Immutable audit result for one feedback evaluation."""

    schema_version: Literal["0.1"] = "0.1"
    state: ActuatorState
    is_stopped: bool
    is_fresh: bool
    is_valid: bool
    correlation_id: str = Field(min_length=1)
    sequence_id: int = Field(ge=0)
    feedback_timestamp_ns: int = Field(ge=0)
    evaluated_timestamp_ns: int = Field(ge=0)

    @model_validator(mode="before")
    @classmethod
    def normalize_wire_aliases(cls, value: object) -> object:
        """Accept compact audit names while retaining canonical fields."""
        if not isinstance(value, Mapping):
            return value
        payload = dict(value)
        if "sequence" in payload and "sequence_id" not in payload:
            payload["sequence_id"] = payload.pop("sequence")
        if "timestamp" in payload and "feedback_timestamp_ns" not in payload:
            payload["feedback_timestamp_ns"] = payload.pop("timestamp")
        return payload

    @field_validator("state", mode="before")
    @classmethod
    def parse_state(cls, value: object) -> object:
        """Accept exact wire enum values without broad coercion."""
        return ActuatorState(value) if isinstance(value, str) else value

    @property
    def sequence(self) -> int:
        """Compatibility alias for the wire contract's sequence field."""
        return self.sequence_id

    @property
    def timestamp(self) -> int:
        """Return the timestamp of the evaluated feedback sample."""
        return self.feedback_timestamp_ns

    @model_validator(mode="after")
    def validate_state_flags(self) -> Self:
        """Keep the safety flags consistent with the classified state."""
        expected = {
            ActuatorState.STOPPED: (True, True, True),
            ActuatorState.MOVING: (False, True, True),
            ActuatorState.INVALID_DATA: (False, False, False),
            ActuatorState.STALE_DATA: (False, False, True),
        }[self.state]
        observed = (self.is_stopped, self.is_fresh, self.is_valid)
        if observed != expected:
            raise ValueError("actuator state flags do not match state")
        return self


FeedbackInput: TypeAlias = ActuatorFeedback | Mapping[str, object] | None

_INVALID_CORRELATION_ID = "invalid"


def _safe_correlation_id(value: object) -> str:
    if isinstance(value, str) and value.strip():
        return value
    return _INVALID_CORRELATION_ID


def _safe_sequence_id(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return 0
    return value


def _safe_timestamp_ns(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return 0
    return value


def _invalid_status(
    *,
    correlation_id: object = _INVALID_CORRELATION_ID,
    sequence_id: object = 0,
    feedback_timestamp_ns: object = 0,
    evaluated_timestamp_ns: object = 0,
) -> ActuatorStatus:
    return ActuatorStatus(
        state=ActuatorState.INVALID_DATA,
        is_stopped=False,
        is_fresh=False,
        is_valid=False,
        correlation_id=_safe_correlation_id(correlation_id),
        sequence_id=_safe_sequence_id(sequence_id),
        feedback_timestamp_ns=_safe_timestamp_ns(feedback_timestamp_ns),
        evaluated_timestamp_ns=_safe_timestamp_ns(evaluated_timestamp_ns),
    )


def _stale_status(feedback: ActuatorFeedback, *, now_ns: int) -> ActuatorStatus:
    return ActuatorStatus(
        state=ActuatorState.STALE_DATA,
        is_stopped=False,
        is_fresh=False,
        is_valid=True,
        correlation_id=feedback.correlation_id,
        sequence_id=feedback.sequence_id,
        feedback_timestamp_ns=feedback.timestamp_ns,
        evaluated_timestamp_ns=now_ns,
    )


def _parse_feedback(value: FeedbackInput) -> tuple[ActuatorFeedback | None, bool]:
    """Parse a model or wire mapping while retaining supplied/missing intent."""
    if value is None:
        return None, False
    if isinstance(value, ActuatorFeedback):
        return value, True
    try:
        return ActuatorFeedback.model_validate(value), True
    except (TypeError, ValidationError):
        return None, True


def _checked_now_ns(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _is_finite_feedback(feedback: ActuatorFeedback) -> bool:
    try:
        return all(
            math.isfinite(component)
            for component in (
                feedback.measured_linear_x,
                feedback.measured_linear_y,
                feedback.measured_angular_z,
            )
        )
    except (TypeError, ValueError):
        return False


def _is_stopped(
    feedback: ActuatorFeedback,
    config: ActuatorFeedbackConfig,
) -> bool:
    return (
        abs(feedback.measured_linear_x) <= config.epsilon_stop_linear
        and abs(feedback.measured_linear_y) <= config.epsilon_stop_linear
        and abs(feedback.measured_angular_z) <= config.epsilon_stop_angular
    )


def evaluate_actuator_feedback(
    feedback: FeedbackInput,
    *,
    now_ns: object,
    expected_correlation_id: object,
    config: ActuatorFeedbackConfig,
    expected_sequence_id: object | None = None,
) -> ActuatorStatus:
    """Classify one actuator sample using fail-closed validation.

    ``stale_threshold_ns`` is inclusive: a sample exactly at the budget is no
    longer fresh.  Correlation and optional sequence checks are performed before
    freshness so a desynchronised sample is always reported as invalid data.
    """
    checked_now_ns = _checked_now_ns(now_ns)
    expected_id = _safe_correlation_id(expected_correlation_id)
    if (
        checked_now_ns is None
        or expected_id == _INVALID_CORRELATION_ID
        or not isinstance(config, ActuatorFeedbackConfig)
    ):
        return _invalid_status(evaluated_timestamp_ns=checked_now_ns or 0)

    expected_sequence: int | None
    if expected_sequence_id is None:
        expected_sequence = None
    elif (
        isinstance(expected_sequence_id, bool)
        or not isinstance(expected_sequence_id, int)
        or expected_sequence_id <= 0
    ):
        return _invalid_status(evaluated_timestamp_ns=checked_now_ns)
    else:
        expected_sequence = expected_sequence_id

    parsed_feedback, was_supplied = _parse_feedback(feedback)
    if parsed_feedback is None:
        if isinstance(feedback, Mapping):
            return _invalid_status(
                correlation_id=feedback.get("correlation_id"),
                sequence_id=feedback.get("sequence_id", feedback.get("sequence")),
                feedback_timestamp_ns=feedback.get(
                    "timestamp_ns", feedback.get("timestamp")
                ),
                evaluated_timestamp_ns=checked_now_ns,
            )
        return _invalid_status(evaluated_timestamp_ns=checked_now_ns)
    if not was_supplied:  # pragma: no cover - guarded by the parser contract
        return _invalid_status(evaluated_timestamp_ns=checked_now_ns)
    if not _is_finite_feedback(parsed_feedback):
        return _invalid_status(
            correlation_id=parsed_feedback.correlation_id,
            sequence_id=parsed_feedback.sequence_id,
            feedback_timestamp_ns=parsed_feedback.timestamp_ns,
            evaluated_timestamp_ns=checked_now_ns,
        )
    if parsed_feedback.correlation_id != expected_id:
        return _invalid_status(
            correlation_id=parsed_feedback.correlation_id,
            sequence_id=parsed_feedback.sequence_id,
            feedback_timestamp_ns=parsed_feedback.timestamp_ns,
            evaluated_timestamp_ns=checked_now_ns,
        )
    if (
        expected_sequence is not None
        and parsed_feedback.sequence_id != expected_sequence
    ):
        return _invalid_status(
            correlation_id=parsed_feedback.correlation_id,
            sequence_id=parsed_feedback.sequence_id,
            feedback_timestamp_ns=parsed_feedback.timestamp_ns,
            evaluated_timestamp_ns=checked_now_ns,
        )
    if parsed_feedback.timestamp_ns > checked_now_ns:
        return _invalid_status(
            correlation_id=parsed_feedback.correlation_id,
            sequence_id=parsed_feedback.sequence_id,
            feedback_timestamp_ns=parsed_feedback.timestamp_ns,
            evaluated_timestamp_ns=checked_now_ns,
        )

    age_ns = checked_now_ns - parsed_feedback.timestamp_ns
    if age_ns >= config.stale_threshold_ns:
        return _stale_status(parsed_feedback, now_ns=checked_now_ns)

    stopped = _is_stopped(parsed_feedback, config)
    return ActuatorStatus(
        state=ActuatorState.STOPPED if stopped else ActuatorState.MOVING,
        is_stopped=stopped,
        is_fresh=True,
        is_valid=True,
        correlation_id=parsed_feedback.correlation_id,
        sequence_id=parsed_feedback.sequence_id,
        feedback_timestamp_ns=parsed_feedback.timestamp_ns,
        evaluated_timestamp_ns=checked_now_ns,
    )


def evaluate_feedback(
    feedback: FeedbackInput,
    *,
    now_ns: object,
    expected_correlation_id: object,
    epsilon_stop_linear: float,
    epsilon_stop_angular: float,
    stale_threshold_ns: int,
    expected_sequence_id: object | None = None,
) -> ActuatorStatus:
    """Convenience API for callers that provide scalar policy thresholds."""
    try:
        config = ActuatorFeedbackConfig(
            epsilon_stop_linear=epsilon_stop_linear,
            epsilon_stop_angular=epsilon_stop_angular,
            stale_threshold_ns=stale_threshold_ns,
        )
    except ValidationError:
        return _invalid_status(
            evaluated_timestamp_ns=_checked_now_ns(now_ns) or 0,
        )
    return evaluate_actuator_feedback(
        feedback,
        now_ns=now_ns,
        expected_correlation_id=expected_correlation_id,
        config=config,
        expected_sequence_id=expected_sequence_id,
    )


__all__ = [
    "ActuatorFeedback",
    "ActuatorFeedbackConfig",
    "ActuatorState",
    "ActuatorStatus",
    "FeedbackInput",
    "evaluate_actuator_feedback",
    "evaluate_feedback",
]
