"""Deterministic, dependency-free velocity command arbitration."""

from __future__ import annotations

from collections.abc import Mapping
from enum import StrEnum
from typing import Self, TypeAlias

from pydantic import Field, ValidationError, field_validator, model_validator

from core.types.incident import StrictFrozenModel


class WatchdogState(StrEnum):
    """Transport health consumed by the velocity safety policy."""

    HEALTHY = "healthy"
    TIMED_OUT = "timed_out"
    NO_VALID_OBSERVATION = "no_valid_observation"
    CLOCK_REGRESSION = "clock_regression"


class EvaluationAction(StrEnum):
    """Safety evaluator action relevant to actuator arbitration."""

    PROCEED = "proceed"
    CLAMP = "clamp"
    PROTECTIVE_STOP = "protective_stop"
    EMERGENCY_STOP = "emergency_stop"


class ArbitrationMode(StrEnum):
    """How the desired velocity was converted to the output command."""

    PASS_THROUGH = "pass_through"
    CLAMPED = "clamped"
    FORCED_ZERO = "forced_zero"


class ArbitrationReason(StrEnum):
    """Stable machine-readable explanation for one arbitration result."""

    PROCEED = "proceed"
    CLAMP_POLICY = "clamp_policy"
    EMERGENCY_STOP = "emergency_stop"
    PROTECTIVE_STOP = "protective_stop"
    WATCHDOG_TIMED_OUT = "watchdog_timed_out"
    WATCHDOG_NO_VALID_OBSERVATION = "watchdog_no_valid_observation"
    WATCHDOG_CLOCK_REGRESSION = "watchdog_clock_regression"
    SAFETY_STATE_UNSAFE = "safety_state_unsafe"
    SAFETY_STATE_MISSING = "safety_state_missing"
    SAFETY_STATE_INVALID = "safety_state_invalid"
    SAFETY_STATE_STALE = "safety_state_stale"
    SAFETY_CLOCK_REGRESSION = "safety_clock_regression"
    COMMAND_MISSING = "command_missing"
    COMMAND_INVALID = "command_invalid"
    COMMAND_STALE = "command_stale"
    COMMAND_CLOCK_REGRESSION = "command_clock_regression"
    COMMAND_FRAME_MISMATCH = "command_frame_mismatch"
    CURRENT_TIME_INVALID = "current_time_invalid"


class DesiredVelocity(StrictFrozenModel):
    """One desired planar velocity together with its source timestamp."""

    linear_x: float
    linear_y: float
    angular_z: float
    timestamp_ns: int = Field(ge=0)
    frame_id: str = Field(min_length=1)


class SafetyState(StrictFrozenModel):
    """Direct evaluator and watchdog state used by the live control path."""

    is_safe: bool
    watchdog_state: WatchdogState
    eval_action: EvaluationAction
    timestamp_ns: int = Field(ge=0)

    @field_validator("watchdog_state", mode="before")
    @classmethod
    def parse_watchdog_state(cls, value: object) -> object:
        """Accept exact enum values without enabling general coercion."""
        if isinstance(value, str):
            return WatchdogState(value)
        return value

    @field_validator("eval_action", mode="before")
    @classmethod
    def parse_evaluation_action(cls, value: object) -> object:
        """Accept exact enum values without enabling general coercion."""
        if isinstance(value, str):
            return EvaluationAction(value)
        return value


class VelocityArbiterConfig(StrictFrozenModel):
    """Explicit timing, frame, and clamp policy for one robot deployment."""

    policy_version: str = Field(min_length=1)
    output_frame_id: str = Field(min_length=1)
    command_stale_threshold_ns: int = Field(gt=0)
    safety_stale_threshold_ns: int = Field(gt=0)
    max_abs_linear_x: float = Field(ge=0.0)
    max_abs_linear_y: float = Field(ge=0.0)
    max_abs_angular_z: float = Field(ge=0.0)


class ArbitrationResult(StrictFrozenModel):
    """Velocity output plus stable evidence describing its policy path."""

    command: DesiredVelocity
    mode: ArbitrationMode
    reason: ArbitrationReason
    policy_version: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_result_shape(self) -> Self:
        """Keep mode, reason, and forced-zero output mutually consistent."""
        if self.mode is ArbitrationMode.PASS_THROUGH:
            if self.reason is not ArbitrationReason.PROCEED:
                raise ValueError("PASS_THROUGH requires the PROCEED reason")
        elif self.mode is ArbitrationMode.CLAMPED:
            if self.reason is not ArbitrationReason.CLAMP_POLICY:
                raise ValueError("CLAMPED requires the CLAMP_POLICY reason")
        else:
            if self.reason in {
                ArbitrationReason.PROCEED,
                ArbitrationReason.CLAMP_POLICY,
            }:
                raise ValueError("FORCED_ZERO requires a fail-safe reason")
            if any(
                component != 0.0
                for component in (
                    self.command.linear_x,
                    self.command.linear_y,
                    self.command.angular_z,
                )
            ):
                raise ValueError("FORCED_ZERO requires an exactly zero command")
        return self


VelocityInput: TypeAlias = DesiredVelocity | Mapping[str, object] | None
SafetyInput: TypeAlias = SafetyState | Mapping[str, object] | None


def _checked_now_ns(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _parse_velocity(value: VelocityInput) -> tuple[DesiredVelocity | None, bool]:
    if value is None:
        return None, False
    if isinstance(value, DesiredVelocity):
        return value, True
    try:
        return DesiredVelocity.model_validate(value), True
    except ValidationError:
        return None, True


def _parse_safety_state(value: SafetyInput) -> tuple[SafetyState | None, bool]:
    if value is None:
        return None, False
    if isinstance(value, SafetyState):
        return value, True
    try:
        return SafetyState.model_validate(value), True
    except (ValidationError, ValueError):
        return None, True


def _clamp(value: float, limit: float) -> float:
    clamped = max(-limit, min(limit, value))
    return 0.0 if clamped == 0.0 else clamped


class VelocityArbiter:
    """Apply a fail-closed velocity policy without I/O or mutable state."""

    def __init__(self, config: VelocityArbiterConfig) -> None:
        self._config = config

    @property
    def config(self) -> VelocityArbiterConfig:
        """Return the immutable policy used for every arbitration."""
        return self._config

    def _forced_zero(
        self,
        reason: ArbitrationReason,
        *,
        timestamp_ns: int,
    ) -> ArbitrationResult:
        return ArbitrationResult(
            command=DesiredVelocity(
                linear_x=0.0,
                linear_y=0.0,
                angular_z=0.0,
                timestamp_ns=timestamp_ns,
                frame_id=self._config.output_frame_id,
            ),
            mode=ArbitrationMode.FORCED_ZERO,
            reason=reason,
            policy_version=self._config.policy_version,
        )

    def arbitrate(
        self,
        desired_velocity: VelocityInput,
        safety_state: SafetyInput,
        *,
        now_ns: object,
    ) -> ArbitrationResult:
        """Return a deterministic pass-through, clamp, or fail-safe command."""
        checked_now_ns = _checked_now_ns(now_ns)
        if checked_now_ns is None:
            return self._forced_zero(
                ArbitrationReason.CURRENT_TIME_INVALID,
                timestamp_ns=0,
            )

        parsed_safety, safety_was_supplied = _parse_safety_state(safety_state)
        if parsed_safety is None:
            reason = (
                ArbitrationReason.SAFETY_STATE_INVALID
                if safety_was_supplied
                else ArbitrationReason.SAFETY_STATE_MISSING
            )
            return self._forced_zero(reason, timestamp_ns=checked_now_ns)

        parsed_velocity, velocity_was_supplied = _parse_velocity(desired_velocity)

        if parsed_safety.eval_action is EvaluationAction.EMERGENCY_STOP:
            return self._forced_zero(
                ArbitrationReason.EMERGENCY_STOP,
                timestamp_ns=checked_now_ns,
            )
        if parsed_safety.eval_action is EvaluationAction.PROTECTIVE_STOP:
            return self._forced_zero(
                ArbitrationReason.PROTECTIVE_STOP,
                timestamp_ns=checked_now_ns,
            )

        unhealthy_reason = {
            WatchdogState.TIMED_OUT: ArbitrationReason.WATCHDOG_TIMED_OUT,
            WatchdogState.NO_VALID_OBSERVATION: (
                ArbitrationReason.WATCHDOG_NO_VALID_OBSERVATION
            ),
            WatchdogState.CLOCK_REGRESSION: (
                ArbitrationReason.WATCHDOG_CLOCK_REGRESSION
            ),
        }.get(parsed_safety.watchdog_state)
        if unhealthy_reason is not None:
            return self._forced_zero(
                unhealthy_reason,
                timestamp_ns=checked_now_ns,
            )
        if not parsed_safety.is_safe:
            return self._forced_zero(
                ArbitrationReason.SAFETY_STATE_UNSAFE,
                timestamp_ns=checked_now_ns,
            )

        safety_age_ns = checked_now_ns - parsed_safety.timestamp_ns
        if safety_age_ns < 0:
            return self._forced_zero(
                ArbitrationReason.SAFETY_CLOCK_REGRESSION,
                timestamp_ns=checked_now_ns,
            )
        if safety_age_ns >= self._config.safety_stale_threshold_ns:
            return self._forced_zero(
                ArbitrationReason.SAFETY_STATE_STALE,
                timestamp_ns=checked_now_ns,
            )

        if parsed_velocity is None:
            reason = (
                ArbitrationReason.COMMAND_INVALID
                if velocity_was_supplied
                else ArbitrationReason.COMMAND_MISSING
            )
            return self._forced_zero(reason, timestamp_ns=checked_now_ns)
        if parsed_velocity.frame_id != self._config.output_frame_id:
            return self._forced_zero(
                ArbitrationReason.COMMAND_FRAME_MISMATCH,
                timestamp_ns=checked_now_ns,
            )

        command_age_ns = checked_now_ns - parsed_velocity.timestamp_ns
        if command_age_ns < 0:
            return self._forced_zero(
                ArbitrationReason.COMMAND_CLOCK_REGRESSION,
                timestamp_ns=checked_now_ns,
            )
        if command_age_ns >= self._config.command_stale_threshold_ns:
            return self._forced_zero(
                ArbitrationReason.COMMAND_STALE,
                timestamp_ns=checked_now_ns,
            )

        if parsed_safety.eval_action is EvaluationAction.PROCEED:
            return ArbitrationResult(
                command=parsed_velocity,
                mode=ArbitrationMode.PASS_THROUGH,
                reason=ArbitrationReason.PROCEED,
                policy_version=self._config.policy_version,
            )

        clamped_command = DesiredVelocity(
            linear_x=_clamp(
                parsed_velocity.linear_x,
                self._config.max_abs_linear_x,
            ),
            linear_y=_clamp(
                parsed_velocity.linear_y,
                self._config.max_abs_linear_y,
            ),
            angular_z=_clamp(
                parsed_velocity.angular_z,
                self._config.max_abs_angular_z,
            ),
            timestamp_ns=parsed_velocity.timestamp_ns,
            frame_id=parsed_velocity.frame_id,
        )
        return ArbitrationResult(
            command=clamped_command,
            mode=ArbitrationMode.CLAMPED,
            reason=ArbitrationReason.CLAMP_POLICY,
            policy_version=self._config.policy_version,
        )
