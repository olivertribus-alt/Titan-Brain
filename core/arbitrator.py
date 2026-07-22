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
    SAFETY_INTENT_MISSING = "safety_intent_missing"
    SAFETY_INTENT_INVALID = "safety_intent_invalid"
    SAFETY_INTENT_TIMEOUT = "safety_intent_timeout"
    SAFETY_INTENT_SEQUENCE_REGRESSION = "safety_intent_sequence_regression"
    COMMAND_TIMEOUT = "command_timeout"
    COMMAND_SEQUENCE_REGRESSION = "command_sequence_regression"
    ARBITER_CLOCK_REGRESSION = "arbiter_clock_regression"
    E_STOP_ACTIVE = "e_stop_active"
    RECOVERY_HOLDING = "recovery_holding"
    RECOVERY_COMMAND_REQUIRED = "recovery_command_required"
    WARNING_TEMPORARY_ZERO = "warning_temporary_zero"
    WARNING_SHAPED = "warning_shaped"
    WARNING_UNMODIFIED = "warning_unmodified"


class SafetyIntentState(StrEnum):
    """Authoritative control-plane state consumed by TB-EVAL-004."""

    NORMAL = "normal"
    WARNING = "warning"
    E_STOP = "e_stop"
    RECOVERY_HOLDING = "recovery_holding"


class DesiredVelocity(StrictFrozenModel):
    """One desired planar velocity together with its source timestamp."""

    linear_x: float
    linear_y: float
    angular_z: float
    timestamp_ns: int = Field(ge=0)
    frame_id: str = Field(min_length=1)
    sequence_id: int = Field(default=0, ge=0)


class SafetyIntent(StrictFrozenModel):
    """Fresh, globally ordered evaluator authority independent of diagnostics."""

    state: SafetyIntentState
    timestamp_ns: int = Field(ge=0)
    correlation_id: str = Field(min_length=1)
    sequence_id: int = Field(gt=0)

    @field_validator("state", mode="before")
    @classmethod
    def parse_state(cls, value: object) -> object:
        """Accept exact wire values without enabling broad coercion."""
        if isinstance(value, str):
            return SafetyIntentState(value)
        return value


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
    warning_max_abs_linear_x: float | None = Field(default=None, ge=0.0)
    warning_max_abs_linear_y: float | None = Field(default=None, ge=0.0)
    warning_max_abs_angular_z: float | None = Field(default=None, ge=0.0)

    @model_validator(mode="after")
    def validate_warning_limits(self) -> Self:
        """Prevent WARNING policy from exceeding nominal component limits."""
        pairs = (
            (self.warning_max_abs_linear_x, self.max_abs_linear_x),
            (self.warning_max_abs_linear_y, self.max_abs_linear_y),
            (self.warning_max_abs_angular_z, self.max_abs_angular_z),
        )
        if any(
            warning is not None and warning > nominal
            for warning, nominal in pairs
        ):
            raise ValueError("WARNING limits must not exceed nominal limits")
        return self


class ArbitrationResult(StrictFrozenModel):
    """Velocity output plus stable evidence describing its policy path."""

    command: DesiredVelocity
    mode: ArbitrationMode
    reason: ArbitrationReason
    policy_version: str = Field(min_length=1)
    correlation_id: str | None = None

    @model_validator(mode="after")
    def validate_result_shape(self) -> Self:
        """Keep mode, reason, and forced-zero output mutually consistent."""
        if self.mode is ArbitrationMode.PASS_THROUGH:
            if self.reason not in {
                ArbitrationReason.PROCEED,
                ArbitrationReason.WARNING_UNMODIFIED,
            }:
                raise ValueError(
                    "PASS_THROUGH requires a pass-through policy reason"
                )
        elif self.mode is ArbitrationMode.CLAMPED:
            if self.reason not in {
                ArbitrationReason.CLAMP_POLICY,
                ArbitrationReason.WARNING_SHAPED,
            }:
                raise ValueError("CLAMPED requires a shaping policy reason")
        else:
            if self.reason in {
                ArbitrationReason.PROCEED,
                ArbitrationReason.CLAMP_POLICY,
                ArbitrationReason.WARNING_SHAPED,
                ArbitrationReason.WARNING_UNMODIFIED,
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
IntentInput: TypeAlias = SafetyIntent | Mapping[str, object] | None


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


def _parse_safety_intent(
    value: IntentInput,
) -> tuple[SafetyIntent | None, bool]:
    if value is None:
        return None, False
    if isinstance(value, SafetyIntent):
        return value, True
    try:
        return SafetyIntent.model_validate(value), True
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


class DynamicSafetyCommandArbiter:
    """Stateful fail-closed SafetyIntent arbiter for TB-EVAL-004A.

    Commands and intents use one global ingress ``sequence_id`` domain. A
    command can resume motion only when its sequence follows the NORMAL intent
    that explicitly released the last stop.
    """

    def __init__(self, config: VelocityArbiterConfig) -> None:
        self._config = config
        self._last_now_ns: int | None = None
        self._last_intent_sequence_id: int | None = None
        self._last_command_sequence_id: int | None = None
        self._last_intent: SafetyIntent | None = None
        self._last_command: DesiredVelocity | None = None
        self._blocked_after_intent_sequence_id = 0
        self._release_sequence_id: int | None = None
        self._requires_new_normal = True
        self._last_output: DesiredVelocity | None = None

    @property
    def config(self) -> VelocityArbiterConfig:
        """Return the immutable timing and frame policy."""
        return self._config

    @property
    def recovery_latched(self) -> bool:
        """Return whether a new NORMAL intent is required before motion."""
        return self._requires_new_normal

    @property
    def last_output(self) -> DesiredVelocity | None:
        """Return the last authoritative output used by the warning guard."""
        return self._last_output

    def _zero(
        self,
        reason: ArbitrationReason,
        *,
        timestamp_ns: int,
        correlation_id: str | None = None,
    ) -> ArbitrationResult:
        command = DesiredVelocity(
            linear_x=0.0,
            linear_y=0.0,
            angular_z=0.0,
            timestamp_ns=timestamp_ns,
            frame_id=self._config.output_frame_id,
        )
        self._last_output = command
        return ArbitrationResult(
            command=command,
            mode=ArbitrationMode.FORCED_ZERO,
            reason=reason,
            policy_version=self._config.policy_version,
            correlation_id=correlation_id,
        )

    @staticmethod
    def _warning_component(
        value: float,
        *,
        limit: float,
        previous: float,
    ) -> float:
        saturated = _clamp(value, limit)
        magnitude = min(abs(saturated), abs(previous))
        if magnitude == 0.0:
            return 0.0
        return -magnitude if saturated < 0.0 else magnitude

    def _warning_result(
        self,
        velocity: DesiredVelocity,
        *,
        correlation_id: str,
    ) -> ArbitrationResult:
        previous = self._last_output
        previous_linear_x = previous.linear_x if previous is not None else 0.0
        previous_linear_y = previous.linear_y if previous is not None else 0.0
        previous_angular_z = previous.angular_z if previous is not None else 0.0
        config = self._config
        command = DesiredVelocity(
            linear_x=self._warning_component(
                velocity.linear_x,
                limit=(
                    config.warning_max_abs_linear_x
                    if config.warning_max_abs_linear_x is not None
                    else config.max_abs_linear_x
                ),
                previous=previous_linear_x,
            ),
            linear_y=self._warning_component(
                velocity.linear_y,
                limit=(
                    config.warning_max_abs_linear_y
                    if config.warning_max_abs_linear_y is not None
                    else config.max_abs_linear_y
                ),
                previous=previous_linear_y,
            ),
            angular_z=self._warning_component(
                velocity.angular_z,
                limit=(
                    config.warning_max_abs_angular_z
                    if config.warning_max_abs_angular_z is not None
                    else config.max_abs_angular_z
                ),
                previous=previous_angular_z,
            ),
            timestamp_ns=velocity.timestamp_ns,
            frame_id=velocity.frame_id,
            sequence_id=velocity.sequence_id,
        )
        shaped = (
            command.linear_x,
            command.linear_y,
            command.angular_z,
        ) != (
            velocity.linear_x,
            velocity.linear_y,
            velocity.angular_z,
        )
        self._last_output = command
        return ArbitrationResult(
            command=command,
            mode=(ArbitrationMode.CLAMPED if shaped else ArbitrationMode.PASS_THROUGH),
            reason=(
                ArbitrationReason.WARNING_SHAPED
                if shaped
                else ArbitrationReason.WARNING_UNMODIFIED
            ),
            policy_version=config.policy_version,
            correlation_id=correlation_id,
        )

    def _latch(self, intent: SafetyIntent | None) -> None:
        self._requires_new_normal = True
        self._release_sequence_id = None
        sequence_id = (
            intent.sequence_id
            if intent is not None
            else self._last_intent_sequence_id
        )
        if sequence_id is not None:
            self._blocked_after_intent_sequence_id = max(
                self._blocked_after_intent_sequence_id,
                sequence_id,
            )

    def evaluate(
        self,
        desired_velocity: VelocityInput,
        safety_intent: IntentInput,
        *,
        now_ns: object,
    ) -> ArbitrationResult:
        """Evaluate ordered control inputs and retain a fail-closed latch."""
        checked_now_ns = _checked_now_ns(now_ns)
        if checked_now_ns is None:
            self._latch(None)
            return self._zero(
                ArbitrationReason.CURRENT_TIME_INVALID,
                timestamp_ns=0,
            )
        if self._last_now_ns is not None and checked_now_ns < self._last_now_ns:
            self._latch(None)
            return self._zero(
                ArbitrationReason.ARBITER_CLOCK_REGRESSION,
                timestamp_ns=checked_now_ns,
            )
        self._last_now_ns = checked_now_ns

        intent, intent_was_supplied = _parse_safety_intent(safety_intent)
        if intent is None:
            self._latch(None)
            return self._zero(
                (
                    ArbitrationReason.SAFETY_INTENT_INVALID
                    if intent_was_supplied
                    else ArbitrationReason.SAFETY_INTENT_MISSING
                ),
                timestamp_ns=checked_now_ns,
            )
        correlation_id = intent.correlation_id
        if (
            self._last_intent_sequence_id is not None
            and (
                intent.sequence_id < self._last_intent_sequence_id
                or (
                    intent.sequence_id == self._last_intent_sequence_id
                    and intent != self._last_intent
                )
            )
        ):
            self._latch(intent)
            return self._zero(
                ArbitrationReason.SAFETY_INTENT_SEQUENCE_REGRESSION,
                timestamp_ns=checked_now_ns,
                correlation_id=correlation_id,
            )
        self._last_intent_sequence_id = intent.sequence_id
        self._last_intent = intent

        intent_age_ns = checked_now_ns - intent.timestamp_ns
        if intent_age_ns < 0:
            self._latch(intent)
            return self._zero(
                ArbitrationReason.SAFETY_CLOCK_REGRESSION,
                timestamp_ns=checked_now_ns,
                correlation_id=correlation_id,
            )
        if intent_age_ns >= self._config.safety_stale_threshold_ns:
            self._latch(intent)
            return self._zero(
                ArbitrationReason.SAFETY_INTENT_TIMEOUT,
                timestamp_ns=checked_now_ns,
                correlation_id=correlation_id,
            )

        stop_reason = {
            SafetyIntentState.E_STOP: ArbitrationReason.E_STOP_ACTIVE,
            SafetyIntentState.RECOVERY_HOLDING: ArbitrationReason.RECOVERY_HOLDING,
        }.get(intent.state)
        if stop_reason is not None:
            self._latch(intent)
            return self._zero(
                stop_reason,
                timestamp_ns=checked_now_ns,
                correlation_id=correlation_id,
            )

        if intent.state is SafetyIntentState.NORMAL and self._requires_new_normal:
            if intent.sequence_id <= self._blocked_after_intent_sequence_id:
                return self._zero(
                    ArbitrationReason.RECOVERY_HOLDING,
                    timestamp_ns=checked_now_ns,
                    correlation_id=correlation_id,
                )
            self._requires_new_normal = False
            self._release_sequence_id = intent.sequence_id

        velocity, velocity_was_supplied = _parse_velocity(desired_velocity)
        if velocity is None:
            self._latch(intent)
            return self._zero(
                (
                    ArbitrationReason.COMMAND_INVALID
                    if velocity_was_supplied
                    else ArbitrationReason.COMMAND_MISSING
                ),
                timestamp_ns=checked_now_ns,
                correlation_id=correlation_id,
            )
        if velocity.frame_id != self._config.output_frame_id:
            self._latch(intent)
            return self._zero(
                ArbitrationReason.COMMAND_FRAME_MISMATCH,
                timestamp_ns=checked_now_ns,
                correlation_id=correlation_id,
            )
        if (
            self._last_command_sequence_id is not None
            and (
                velocity.sequence_id < self._last_command_sequence_id
                or (
                    velocity.sequence_id == self._last_command_sequence_id
                    and velocity != self._last_command
                )
            )
        ):
            self._latch(intent)
            return self._zero(
                ArbitrationReason.COMMAND_SEQUENCE_REGRESSION,
                timestamp_ns=checked_now_ns,
                correlation_id=correlation_id,
            )
        self._last_command_sequence_id = velocity.sequence_id
        self._last_command = velocity

        command_age_ns = checked_now_ns - velocity.timestamp_ns
        if command_age_ns < 0:
            self._latch(intent)
            return self._zero(
                ArbitrationReason.COMMAND_CLOCK_REGRESSION,
                timestamp_ns=checked_now_ns,
                correlation_id=correlation_id,
            )
        if command_age_ns >= self._config.command_stale_threshold_ns:
            self._latch(intent)
            return self._zero(
                ArbitrationReason.COMMAND_TIMEOUT,
                timestamp_ns=checked_now_ns,
                correlation_id=correlation_id,
            )
        release_sequence_id = self._release_sequence_id
        if (
            release_sequence_id is not None
            and velocity.sequence_id <= release_sequence_id
        ):
            return self._zero(
                ArbitrationReason.RECOVERY_COMMAND_REQUIRED,
                timestamp_ns=checked_now_ns,
                correlation_id=correlation_id,
            )

        if intent.state is SafetyIntentState.WARNING:
            return self._warning_result(
                velocity,
                correlation_id=correlation_id,
            )

        self._last_output = velocity
        return ArbitrationResult(
            command=velocity,
            mode=ArbitrationMode.PASS_THROUGH,
            reason=ArbitrationReason.PROCEED,
            policy_version=self._config.policy_version,
            correlation_id=correlation_id,
        )
