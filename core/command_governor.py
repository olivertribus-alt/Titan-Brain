"""Dependency-free kinematic command governor for TB-EVAL-006A.

The governor is the last software stage before a command reaches the control
arbiter.  It limits velocity, acceleration, deceleration, and jerk on the
linear and angular axes while keeping all state transitions deterministic.  A
marked emergency stop is deliberately handled as a hard zero and bypasses the
normal ramp profile.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from enum import StrEnum
from typing import Literal, TypeAlias

from pydantic import Field, ValidationError, model_validator

from core.types.incident import StrictFrozenModel

NANOSECONDS_PER_SECOND = 1_000_000_000
DEFAULT_LINEAR_VELOCITY_MPS = 1.0
DEFAULT_ANGULAR_VELOCITY_RADPS = 1.0
DEFAULT_LINEAR_ACCELERATION_MPS2 = 1.0
DEFAULT_LINEAR_DECELERATION_MPS2 = 2.0
DEFAULT_ANGULAR_ACCELERATION_RADPS2 = 1.0
DEFAULT_ANGULAR_DECELERATION_RADPS2 = 2.0
DEFAULT_LINEAR_JERK_MPS3 = 5.0
DEFAULT_ANGULAR_JERK_RADPS3 = 5.0


class GovernorReason(StrEnum):
    """Primary reason attached to one governed command."""

    NOMINAL = "nominal"
    SPEED_LIMITED = "speed_limited"
    ACCELERATION_LIMITED = "acceleration_limited"
    DECELERATION_LIMITED = "deceleration_limited"
    JERK_LIMITED = "jerk_limited"
    EMERGENCY_STOP = "emergency_stop"
    INVALID_COMMAND = "invalid_command"
    INVALID_TIMESTAMP = "invalid_timestamp"
    NON_POSITIVE_DT = "non_positive_dt"
    CLOCK_REGRESSION = "clock_regression"


class GovernorConfig(StrictFrozenModel):
    """Kinematic limits used by :class:`CommandGovernor`.

    The limits are symmetric around zero.  Acceleration and deceleration are
    intentionally separate so a deployment can permit a sharper controlled
    stop than a normal launch.  All values are strictly positive and finite.
    """

    schema_version: Literal["0.1"] = "0.1"
    max_linear_velocity_mps: float = Field(
        default=DEFAULT_LINEAR_VELOCITY_MPS,
        gt=0.0,
    )
    max_angular_velocity_radps: float = Field(
        default=DEFAULT_ANGULAR_VELOCITY_RADPS,
        gt=0.0,
    )
    max_linear_acceleration_mps2: float = Field(
        default=DEFAULT_LINEAR_ACCELERATION_MPS2,
        gt=0.0,
    )
    max_linear_deceleration_mps2: float = Field(
        default=DEFAULT_LINEAR_DECELERATION_MPS2,
        gt=0.0,
    )
    max_angular_acceleration_radps2: float = Field(
        default=DEFAULT_ANGULAR_ACCELERATION_RADPS2,
        gt=0.0,
    )
    max_angular_deceleration_radps2: float = Field(
        default=DEFAULT_ANGULAR_DECELERATION_RADPS2,
        gt=0.0,
    )
    max_linear_jerk_mps3: float = Field(
        default=DEFAULT_LINEAR_JERK_MPS3,
        gt=0.0,
    )
    max_angular_jerk_radps3: float = Field(
        default=DEFAULT_ANGULAR_JERK_RADPS3,
        gt=0.0,
    )

    @model_validator(mode="before")
    @classmethod
    def normalize_short_names(cls, value: object) -> object:
        """Accept the compact notation used in the engineering contract."""
        if not isinstance(value, Mapping):
            return value
        payload = dict(value)
        aliases = {
            "v_max": "max_linear_velocity_mps",
            "omega_max": "max_angular_velocity_radps",
            "a_accel_max": "max_linear_acceleration_mps2",
            "a_decel_max": "max_linear_deceleration_mps2",
            "alpha_accel_max": "max_angular_acceleration_radps2",
            "alpha_decel_max": "max_angular_deceleration_radps2",
            "j_max": "max_linear_jerk_mps3",
            "j_angular_max": "max_angular_jerk_radps3",
        }
        for alias, canonical in aliases.items():
            if canonical not in payload and alias in payload:
                payload[canonical] = payload[alias]
            payload.pop(alias, None)

        numeric_fields = tuple(aliases.values())
        for field_name in numeric_fields:
            candidate = payload.get(field_name)
            if isinstance(candidate, int) and not isinstance(candidate, bool):
                payload[field_name] = float(candidate)
        return payload

    @property
    def v_max(self) -> float:
        """Return the linear speed limit using the contract notation."""
        return self.max_linear_velocity_mps

    @property
    def omega_max(self) -> float:
        """Return the angular speed limit using the contract notation."""
        return self.max_angular_velocity_radps

    @property
    def a_accel_max(self) -> float:
        """Return the linear launch acceleration limit."""
        return self.max_linear_acceleration_mps2

    @property
    def a_decel_max(self) -> float:
        """Return the linear controlled-deceleration limit."""
        return self.max_linear_deceleration_mps2

    @property
    def j_max(self) -> float:
        """Return the linear jerk limit."""
        return self.max_linear_jerk_mps3


class GovernorCommand(StrictFrozenModel):
    """One requested body-frame command and its optional audit context."""

    schema_version: Literal["0.1"] = "0.1"
    linear_velocity_mps: float = 0.0
    angular_velocity_radps: float = 0.0
    emergency_stop: bool = False
    correlation_id: str = "unknown"
    timestamp_ns: int | None = Field(default=None, ge=0)

    @model_validator(mode="before")
    @classmethod
    def normalize_wire_aliases(cls, value: object) -> object:
        """Accept ROS-style axis names and the explicit stop alias."""
        if not isinstance(value, Mapping):
            return value
        payload = dict(value)
        aliases = {
            "linear_x_mps": "linear_velocity_mps",
            "linear_x": "linear_velocity_mps",
            "angular_z_radps": "angular_velocity_radps",
            "angular_z": "angular_velocity_radps",
            "stop": "emergency_stop",
            "is_emergency_stop": "emergency_stop",
        }
        for alias, canonical in aliases.items():
            if canonical not in payload and alias in payload:
                payload[canonical] = payload[alias]
            payload.pop(alias, None)
        for field_name in ("linear_velocity_mps", "angular_velocity_radps"):
            candidate = payload.get(field_name)
            if isinstance(candidate, int) and not isinstance(candidate, bool):
                payload[field_name] = float(candidate)
        return payload


class GovernorState(StrictFrozenModel):
    """Immutable snapshot of the governor's internal dynamic state."""

    schema_version: Literal["0.1"] = "0.1"
    timestamp_ns: int = Field(ge=0)
    linear_velocity_mps: float
    angular_velocity_radps: float
    linear_acceleration_mps2: float
    angular_acceleration_radps2: float


class GovernorResult(StrictFrozenModel):
    """Auditable output of one governor evaluation."""

    schema_version: Literal["0.1"] = "0.1"
    timestamp_ns: int = Field(ge=0)
    dt_ns: int = Field(ge=0)
    linear_velocity_mps: float
    angular_velocity_radps: float
    linear_acceleration_mps2: float
    angular_acceleration_radps2: float
    reason: GovernorReason
    is_safe: bool
    emergency_override: bool = False
    correlation_id: str = "unknown"

    @property
    def linear_x_mps(self) -> float:
        """Return the output using the body-frame wire name."""
        return self.linear_velocity_mps

    @property
    def angular_z_radps(self) -> float:
        """Return the output using the body-frame wire name."""
        return self.angular_velocity_radps


CommandInput: TypeAlias = GovernorCommand | Mapping[str, object]


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


def _parse_command(value: object) -> GovernorCommand | None:
    if isinstance(value, GovernorCommand):
        return value
    if not isinstance(value, Mapping):
        return None
    try:
        return GovernorCommand.model_validate(value)
    except (TypeError, ValidationError):
        return None


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def _axis_step(
    *,
    current: float,
    previous_acceleration: float,
    target: float,
    dt_s: float,
    acceleration_limit: float,
    deceleration_limit: float,
    jerk_limit: float,
    speed_limit: float,
) -> tuple[float, float, bool, bool, bool, bool]:
    """Advance one axis and report speed/accel/decel/jerk limiting flags."""
    bounded_target = _clamp(target, -speed_limit, speed_limit)
    speed_limited = bounded_target != target
    delta = bounded_target - current
    if delta == 0.0:
        return current, 0.0, speed_limited, False, False, False

    increasing = abs(bounded_target) > abs(current) and (
        current == 0.0 or current * bounded_target > 0.0
    )
    limit = acceleration_limit if increasing else deceleration_limit
    unconstrained_acceleration = delta / dt_s
    requested_acceleration = _clamp(
        unconstrained_acceleration,
        -limit,
        limit,
    )
    accel_limited = increasing and abs(unconstrained_acceleration) > acceleration_limit
    decel_limited = (
        not increasing and abs(unconstrained_acceleration) > deceleration_limit
    )

    max_acceleration_change = jerk_limit * dt_s
    jerk_limited_acceleration = _clamp(
        requested_acceleration,
        previous_acceleration - max_acceleration_change,
        previous_acceleration + max_acceleration_change,
    )
    jerk_limited = not math.isclose(
        jerk_limited_acceleration,
        requested_acceleration,
        rel_tol=0.0,
        abs_tol=1e-12,
    )

    candidate = current + jerk_limited_acceleration * dt_s
    # A jerk-limited acceleration must never move away from its target.  This
    # matters when a prior positive acceleration is being reversed for a stop.
    if (candidate - current) * delta <= 0.0:
        candidate = current
    elif delta > 0.0:
        candidate = min(candidate, bounded_target)
    else:
        candidate = max(candidate, bounded_target)
    actual_acceleration = (candidate - current) / dt_s
    return (
        candidate,
        actual_acceleration,
        speed_limited,
        accel_limited,
        decel_limited,
        jerk_limited,
    )


class CommandGovernor:
    """Stateful deterministic velocity, acceleration, and jerk governor."""

    def __init__(
        self,
        config: GovernorConfig | None = None,
        *,
        initial_timestamp_ns: int = 0,
    ) -> None:
        if config is not None and not isinstance(config, GovernorConfig):
            raise ValueError("config must be a GovernorConfig")
        checked_timestamp = _checked_timestamp(initial_timestamp_ns)
        if checked_timestamp is None:
            raise ValueError("initial_timestamp_ns must be a non-negative integer")
        self._config = config or GovernorConfig()
        self._timestamp_ns = checked_timestamp
        self._linear_velocity = 0.0
        self._angular_velocity = 0.0
        self._linear_acceleration = 0.0
        self._angular_acceleration = 0.0

    @property
    def config(self) -> GovernorConfig:
        """Return the immutable governor configuration."""
        return self._config

    @property
    def last_timestamp_ns(self) -> int:
        """Return the last accepted monotonic timestamp."""
        return self._timestamp_ns

    @property
    def state(self) -> GovernorState:
        """Return a frozen snapshot of the current dynamic state."""
        return GovernorState(
            timestamp_ns=self._timestamp_ns,
            linear_velocity_mps=self._linear_velocity,
            angular_velocity_radps=self._angular_velocity,
            linear_acceleration_mps2=self._linear_acceleration,
            angular_acceleration_radps2=self._angular_acceleration,
        )

    def _reset_baseline(self, timestamp_ns: int | None = None) -> None:
        if timestamp_ns is not None and timestamp_ns >= self._timestamp_ns:
            self._timestamp_ns = timestamp_ns
        self._linear_velocity = 0.0
        self._angular_velocity = 0.0
        self._linear_acceleration = 0.0
        self._angular_acceleration = 0.0

    def _failure(
        self,
        *,
        reason: GovernorReason,
        timestamp_ns: int | None,
        correlation_id: str = "invalid",
    ) -> GovernorResult:
        evaluated_timestamp = (
            self._timestamp_ns if timestamp_ns is None else timestamp_ns
        )
        dt_ns = max(0, evaluated_timestamp - self._timestamp_ns)
        if reason is not GovernorReason.CLOCK_REGRESSION:
            self._reset_baseline(timestamp_ns)
        else:
            self._reset_baseline()
        return GovernorResult(
            timestamp_ns=evaluated_timestamp,
            dt_ns=dt_ns,
            linear_velocity_mps=0.0,
            angular_velocity_radps=0.0,
            linear_acceleration_mps2=0.0,
            angular_acceleration_radps2=0.0,
            reason=reason,
            is_safe=False,
            correlation_id=correlation_id,
        )

    def step(
        self,
        command: CommandInput,
        timestamp_ns: object | None = None,
        *,
        now_ns: object | None = None,
    ) -> GovernorResult:
        """Evaluate one command at a monotonic timestamp.

        ``timestamp_ns`` is accepted positionally for small adapters, while
        ``now_ns`` is a readable keyword alias.  Supplying both is invalid and
        fails closed.  A command can also carry ``timestamp_ns`` in its wire
        mapping when neither argument is supplied.
        """
        parsed = _parse_command(command)
        if timestamp_ns is not None and now_ns is not None:
            return self._failure(
                reason=GovernorReason.INVALID_TIMESTAMP,
                timestamp_ns=None,
            )

        supplied_timestamp = timestamp_ns if timestamp_ns is not None else now_ns
        if supplied_timestamp is None and parsed is not None:
            supplied_timestamp = parsed.timestamp_ns
        checked_timestamp = _checked_timestamp(supplied_timestamp)
        if checked_timestamp is None:
            return self._failure(
                reason=(
                    GovernorReason.INVALID_COMMAND
                    if parsed is None
                    else GovernorReason.INVALID_TIMESTAMP
                ),
                timestamp_ns=None,
            )
        if parsed is None:
            return self._failure(
                reason=GovernorReason.INVALID_COMMAND,
                timestamp_ns=checked_timestamp,
            )
        if checked_timestamp < self._timestamp_ns:
            return self._failure(
                reason=GovernorReason.CLOCK_REGRESSION,
                timestamp_ns=checked_timestamp,
                correlation_id=parsed.correlation_id,
            )
        dt_ns = checked_timestamp - self._timestamp_ns
        if dt_ns <= 0:
            return self._failure(
                reason=GovernorReason.NON_POSITIVE_DT,
                timestamp_ns=checked_timestamp,
                correlation_id=parsed.correlation_id,
            )
        if not _finite_number(parsed.linear_velocity_mps) or not _finite_number(
            parsed.angular_velocity_radps
        ):
            return self._failure(
                reason=GovernorReason.INVALID_COMMAND,
                timestamp_ns=checked_timestamp,
                correlation_id=parsed.correlation_id,
            )

        if parsed.emergency_stop:
            self._reset_baseline(checked_timestamp)
            return GovernorResult(
                timestamp_ns=checked_timestamp,
                dt_ns=dt_ns,
                linear_velocity_mps=0.0,
                angular_velocity_radps=0.0,
                linear_acceleration_mps2=0.0,
                angular_acceleration_radps2=0.0,
                reason=GovernorReason.EMERGENCY_STOP,
                is_safe=True,
                emergency_override=True,
                correlation_id=parsed.correlation_id,
            )

        dt_s = dt_ns / NANOSECONDS_PER_SECOND
        linear = _axis_step(
            current=self._linear_velocity,
            previous_acceleration=self._linear_acceleration,
            target=parsed.linear_velocity_mps,
            dt_s=dt_s,
            acceleration_limit=self._config.max_linear_acceleration_mps2,
            deceleration_limit=self._config.max_linear_deceleration_mps2,
            jerk_limit=self._config.max_linear_jerk_mps3,
            speed_limit=self._config.max_linear_velocity_mps,
        )
        angular = _axis_step(
            current=self._angular_velocity,
            previous_acceleration=self._angular_acceleration,
            target=parsed.angular_velocity_radps,
            dt_s=dt_s,
            acceleration_limit=self._config.max_angular_acceleration_radps2,
            deceleration_limit=self._config.max_angular_deceleration_radps2,
            jerk_limit=self._config.max_angular_jerk_radps3,
            speed_limit=self._config.max_angular_velocity_radps,
        )
        self._timestamp_ns = checked_timestamp
        self._linear_velocity, self._linear_acceleration = linear[0], linear[1]
        self._angular_velocity, self._angular_acceleration = angular[0], angular[1]

        flags = linear[2:] + angular[2:]
        speed_limited = flags[0] or flags[4]
        accel_limited = flags[1] or flags[5]
        decel_limited = flags[2] or flags[6]
        jerk_limited = flags[3] or flags[7]
        if speed_limited:
            reason = GovernorReason.SPEED_LIMITED
        elif jerk_limited:
            reason = GovernorReason.JERK_LIMITED
        elif accel_limited:
            reason = GovernorReason.ACCELERATION_LIMITED
        elif decel_limited:
            reason = GovernorReason.DECELERATION_LIMITED
        else:
            reason = GovernorReason.NOMINAL
        return GovernorResult(
            timestamp_ns=checked_timestamp,
            dt_ns=dt_ns,
            linear_velocity_mps=self._linear_velocity,
            angular_velocity_radps=self._angular_velocity,
            linear_acceleration_mps2=self._linear_acceleration,
            angular_acceleration_radps2=self._angular_acceleration,
            reason=reason,
            is_safe=True,
            correlation_id=parsed.correlation_id,
        )

    def evaluate(
        self,
        command: CommandInput,
        *,
        now_ns: object | None = None,
        timestamp_ns: object | None = None,
    ) -> GovernorResult:
        """Readable alias for :meth:`step`."""
        return self.step(command, timestamp_ns, now_ns=now_ns)

    def govern(
        self,
        command: CommandInput,
        *,
        now_ns: object | None = None,
    ) -> GovernorResult:
        """Compatibility alias used by command-path adapters."""
        return self.step(command, now_ns=now_ns)


def govern_command(
    command: CommandInput,
    *,
    timestamp_ns: object,
    config: GovernorConfig | None = None,
) -> GovernorResult:
    """Convenience function for one-shot command shaping."""
    governor = CommandGovernor(config, initial_timestamp_ns=0)
    return governor.step(command, timestamp_ns)


# Contract-friendly aliases retained for downstream adapters.
KinematicLimits = GovernorConfig
CommandGovernorConfig = GovernorConfig
CommandGovernorInput = GovernorCommand
CommandGovernorOutput = GovernorResult
CommandGovernorReason = GovernorReason


__all__ = [
    "CommandGovernor",
    "CommandGovernorConfig",
    "CommandGovernorInput",
    "CommandGovernorOutput",
    "CommandGovernorReason",
    "CommandInput",
    "GovernorCommand",
    "GovernorConfig",
    "GovernorReason",
    "GovernorResult",
    "GovernorState",
    "KinematicLimits",
    "NANOSECONDS_PER_SECOND",
    "govern_command",
]
