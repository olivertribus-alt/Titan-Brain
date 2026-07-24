"""Deterministic TB-EVAL-009A recovery and degradation lifecycle manager."""

from __future__ import annotations

import math
from enum import StrEnum
from typing import Literal, Self

from pydantic import Field, model_validator

from core.types.incident import StrictFrozenModel


class SafetyLifecycleState(StrEnum):
    """Externally visible safety-authority lifecycle state."""

    NORMAL = "normal"
    DEGRADED = "degraded"
    RECOVERY = "recovery"
    EMERGENCY_STOP = "emergency_stop"


class SafetyLifecycleReason(StrEnum):
    """Stable machine-readable reason for the latest transition."""

    NORMAL_CLEARANCE = "normal_clearance"
    WARNING_ZONE = "warning_zone"
    NONCRITICAL_WARNING = "noncritical_warning"
    SYSTEM_FAULT = "system_fault"
    FAULT_STATUS_INVALID = "fault_status_invalid"
    STOP_MARGIN_BREACH = "stop_margin_breach"
    SENSOR_STALE = "sensor_stale"
    SENSOR_INVALID = "sensor_invalid"
    TIME_INVALID = "time_invalid"
    CLOCK_REGRESSION = "clock_regression"
    RECOVERY_STARTED = "recovery_started"
    RECOVERY_HOLDING = "recovery_holding"
    RECOVERY_COMPLETE_NORMAL = "recovery_complete_normal"
    RECOVERY_COMPLETE_DEGRADED = "recovery_complete_degraded"


class SafetyRecoveryConfig(StrictFrozenModel):
    """Finite policy thresholds and bounded authority caps."""

    schema_version: Literal["0.1"] = "0.1"
    policy_version: str = Field(
        default="TB-EVAL-009A-0.1.0",
        min_length=1,
    )
    stop_margin_m: float = Field(default=0.30, ge=0.0)
    warning_distance_m: float = Field(default=1.00, gt=0.0)
    distance_hysteresis_m: float = Field(default=0.10, ge=0.0)
    recovery_dwell_time_ns: int = Field(default=1_000_000_000, gt=0)
    degraded_linear_speed_limit_mps: float = Field(default=0.50, gt=0.0)
    degraded_angular_speed_limit_radps: float = Field(default=0.50, gt=0.0)
    recovery_linear_speed_limit_mps: float = Field(default=0.20, gt=0.0)
    recovery_angular_speed_limit_radps: float = Field(default=0.50, gt=0.0)

    @model_validator(mode="after")
    def validate_policy(self) -> Self:
        """Reject non-finite or contradictory safety thresholds."""
        values = (
            self.stop_margin_m,
            self.warning_distance_m,
            self.distance_hysteresis_m,
            self.degraded_linear_speed_limit_mps,
            self.degraded_angular_speed_limit_radps,
            self.recovery_linear_speed_limit_mps,
            self.recovery_angular_speed_limit_radps,
        )
        if any(not math.isfinite(value) for value in values):
            raise ValueError("safety recovery configuration must be finite")
        if self.warning_distance_m <= self.stop_margin_m:
            raise ValueError("warning distance must exceed the stop margin")
        if not math.isfinite(self.normal_release_distance_m):
            raise ValueError("normal release distance must be finite")
        if self.recovery_linear_speed_limit_mps > self.degraded_linear_speed_limit_mps:
            raise ValueError(
                "recovery linear speed limit must not exceed degraded limit"
            )
        if (
            self.recovery_angular_speed_limit_radps
            > self.degraded_angular_speed_limit_radps
        ):
            raise ValueError(
                "recovery angular speed limit must not exceed degraded limit"
            )
        return self

    @property
    def normal_release_distance_m(self) -> float:
        """Distance required to leave degraded authority."""
        return self.warning_distance_m + self.distance_hysteresis_m


class SafetyLifecycleEvidence(StrictFrozenModel):
    """Constant-size health and authority evidence for one transition."""

    fault_status_valid: bool
    is_faulted: bool
    sensor_valid: bool
    sensor_fresh: bool
    time_valid: bool
    noncritical_warning: bool = False
    distance_min_m: float | None = Field(default=None, ge=0.0)
    max_linear_velocity_mps: float = Field(ge=0.0)
    max_angular_velocity_radps: float = Field(ge=0.0)

    @model_validator(mode="after")
    def validate_finite_evidence(self) -> Self:
        """Keep corrupt numeric evidence out of transition arithmetic."""
        numeric_values = [
            self.max_linear_velocity_mps,
            self.max_angular_velocity_radps,
        ]
        if self.distance_min_m is not None:
            numeric_values.append(self.distance_min_m)
        if any(not math.isfinite(value) for value in numeric_values):
            raise ValueError("safety lifecycle evidence must be finite")
        if (
            self.fault_status_valid
            and not self.is_faulted
            and self.sensor_valid
            and self.sensor_fresh
            and self.time_valid
            and self.distance_min_m is None
        ):
            raise ValueError("healthy evidence requires a minimum distance")
        return self


class SafetyLifecycleTransition(StrictFrozenModel):
    """Immutable lifecycle state and authority emitted by one update."""

    schema_version: Literal["0.1"] = "0.1"
    policy_version: str = Field(min_length=1)
    state: SafetyLifecycleState
    reason: SafetyLifecycleReason
    evaluated_at_ns: int = Field(ge=0)
    monotonic_time_ns: int = Field(ge=0)
    distance_min_m: float | None = Field(default=None, ge=0.0)
    recovery_started_at_ns: int | None = Field(default=None, ge=0)
    recovery_elapsed_ns: int = Field(default=0, ge=0)
    max_linear_velocity_mps: float = Field(ge=0.0)
    max_angular_velocity_radps: float = Field(ge=0.0)

    @model_validator(mode="after")
    def validate_transition_shape(self) -> Self:
        """Ensure timers and exact-zero authority agree with the state."""
        if self.monotonic_time_ns < self.evaluated_at_ns:
            raise ValueError("monotonic time must cover evaluated time")
        velocities = (
            self.max_linear_velocity_mps,
            self.max_angular_velocity_radps,
        )
        if any(not math.isfinite(value) for value in velocities):
            raise ValueError("lifecycle authority must be finite")
        if self.state is SafetyLifecycleState.EMERGENCY_STOP:
            if velocities != (0.0, 0.0):
                raise ValueError("emergency stop requires exact-zero authority")
            if self.recovery_started_at_ns is not None:
                raise ValueError("emergency stop must not retain recovery timer")
        elif self.state is SafetyLifecycleState.RECOVERY:
            if self.recovery_started_at_ns is None:
                raise ValueError("recovery state requires a dwell start")
            if self.recovery_started_at_ns > self.evaluated_at_ns:
                raise ValueError("recovery cannot start after evaluation")
            if (
                self.recovery_elapsed_ns
                != self.evaluated_at_ns - self.recovery_started_at_ns
            ):
                raise ValueError("recovery elapsed time must match timestamps")
        elif self.recovery_started_at_ns is not None or self.recovery_elapsed_ns != 0:
            raise ValueError("stable states must not retain recovery timer")
        return self

    @property
    def stop_only(self) -> bool:
        """Return whether the transition grants no motion authority."""
        return (
            self.max_linear_velocity_mps == 0.0
            and self.max_angular_velocity_radps == 0.0
        )


def _checked_now_ns(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _emergency_transition(
    *,
    previous: SafetyLifecycleTransition | None,
    evidence: SafetyLifecycleEvidence,
    config: SafetyRecoveryConfig,
    reason: SafetyLifecycleReason,
    now_ns: int,
) -> SafetyLifecycleTransition:
    monotonic_time_ns = max(
        now_ns,
        previous.monotonic_time_ns if previous is not None else 0,
    )
    return SafetyLifecycleTransition(
        policy_version=config.policy_version,
        state=SafetyLifecycleState.EMERGENCY_STOP,
        reason=reason,
        evaluated_at_ns=now_ns,
        monotonic_time_ns=monotonic_time_ns,
        distance_min_m=evidence.distance_min_m,
        max_linear_velocity_mps=0.0,
        max_angular_velocity_radps=0.0,
    )


def _critical_reason(
    evidence: SafetyLifecycleEvidence,
    config: SafetyRecoveryConfig,
) -> SafetyLifecycleReason | None:
    if evidence.is_faulted:
        return SafetyLifecycleReason.SYSTEM_FAULT
    if not evidence.fault_status_valid:
        return SafetyLifecycleReason.FAULT_STATUS_INVALID
    if not evidence.time_valid:
        return SafetyLifecycleReason.TIME_INVALID
    if not evidence.sensor_valid:
        return SafetyLifecycleReason.SENSOR_INVALID
    if not evidence.sensor_fresh:
        return SafetyLifecycleReason.SENSOR_STALE
    if evidence.distance_min_m is None:
        return SafetyLifecycleReason.SENSOR_INVALID
    if evidence.distance_min_m <= config.stop_margin_m:
        return SafetyLifecycleReason.STOP_MARGIN_BREACH
    return None


def _desired_stable_state(
    *,
    previous: SafetyLifecycleTransition | None,
    evidence: SafetyLifecycleEvidence,
    config: SafetyRecoveryConfig,
) -> tuple[SafetyLifecycleState, SafetyLifecycleReason]:
    distance_min_m = evidence.distance_min_m
    if evidence.noncritical_warning:
        return (
            SafetyLifecycleState.DEGRADED,
            SafetyLifecycleReason.NONCRITICAL_WARNING,
        )
    if distance_min_m is None or distance_min_m <= config.warning_distance_m:
        return SafetyLifecycleState.DEGRADED, SafetyLifecycleReason.WARNING_ZONE
    if (
        previous is not None
        and previous.state is not SafetyLifecycleState.NORMAL
        and distance_min_m <= config.normal_release_distance_m
    ):
        return SafetyLifecycleState.DEGRADED, SafetyLifecycleReason.WARNING_ZONE
    return SafetyLifecycleState.NORMAL, SafetyLifecycleReason.NORMAL_CLEARANCE


def _authority(
    state: SafetyLifecycleState,
    evidence: SafetyLifecycleEvidence,
    config: SafetyRecoveryConfig,
) -> tuple[float, float]:
    if state is SafetyLifecycleState.EMERGENCY_STOP:
        return 0.0, 0.0
    if state is SafetyLifecycleState.RECOVERY:
        return (
            min(
                evidence.max_linear_velocity_mps,
                config.recovery_linear_speed_limit_mps,
            ),
            min(
                evidence.max_angular_velocity_radps,
                config.recovery_angular_speed_limit_radps,
            ),
        )
    if state is SafetyLifecycleState.DEGRADED:
        return (
            min(
                evidence.max_linear_velocity_mps,
                config.degraded_linear_speed_limit_mps,
            ),
            min(
                evidence.max_angular_velocity_radps,
                config.degraded_angular_speed_limit_radps,
            ),
        )
    return (
        evidence.max_linear_velocity_mps,
        evidence.max_angular_velocity_radps,
    )


def transition_safety_lifecycle(
    previous: SafetyLifecycleTransition | None,
    evidence: SafetyLifecycleEvidence,
    config: SafetyRecoveryConfig,
    *,
    now_ns: object,
) -> SafetyLifecycleTransition:
    """Apply one immediate-entry, delayed-release O(1) transition."""
    checked_now_ns = _checked_now_ns(now_ns)
    if checked_now_ns is None:
        return _emergency_transition(
            previous=previous,
            evidence=evidence,
            config=config,
            reason=SafetyLifecycleReason.TIME_INVALID,
            now_ns=0,
        )
    if previous is not None and checked_now_ns < previous.monotonic_time_ns:
        return _emergency_transition(
            previous=previous,
            evidence=evidence,
            config=config,
            reason=SafetyLifecycleReason.CLOCK_REGRESSION,
            now_ns=checked_now_ns,
        )

    critical_reason = _critical_reason(evidence, config)
    if critical_reason is not None:
        return _emergency_transition(
            previous=previous,
            evidence=evidence,
            config=config,
            reason=critical_reason,
            now_ns=checked_now_ns,
        )

    desired_state, desired_reason = _desired_stable_state(
        previous=previous,
        evidence=evidence,
        config=config,
    )
    requires_recovery = (
        previous is None
        or previous.state is SafetyLifecycleState.EMERGENCY_STOP
        or previous.state is SafetyLifecycleState.RECOVERY
    )
    if requires_recovery:
        recovery_started_at_ns = (
            previous.recovery_started_at_ns
            if previous is not None and previous.state is SafetyLifecycleState.RECOVERY
            else checked_now_ns
        )
        if recovery_started_at_ns is None:
            recovery_started_at_ns = checked_now_ns
        recovery_elapsed_ns = checked_now_ns - recovery_started_at_ns
        if recovery_elapsed_ns < config.recovery_dwell_time_ns:
            linear_limit, angular_limit = _authority(
                SafetyLifecycleState.RECOVERY,
                evidence,
                config,
            )
            return SafetyLifecycleTransition(
                policy_version=config.policy_version,
                state=SafetyLifecycleState.RECOVERY,
                reason=(
                    SafetyLifecycleReason.RECOVERY_HOLDING
                    if previous is not None
                    and previous.state is SafetyLifecycleState.RECOVERY
                    else SafetyLifecycleReason.RECOVERY_STARTED
                ),
                evaluated_at_ns=checked_now_ns,
                monotonic_time_ns=checked_now_ns,
                distance_min_m=evidence.distance_min_m,
                recovery_started_at_ns=recovery_started_at_ns,
                recovery_elapsed_ns=recovery_elapsed_ns,
                max_linear_velocity_mps=linear_limit,
                max_angular_velocity_radps=angular_limit,
            )
        desired_reason = (
            SafetyLifecycleReason.RECOVERY_COMPLETE_NORMAL
            if desired_state is SafetyLifecycleState.NORMAL
            else SafetyLifecycleReason.RECOVERY_COMPLETE_DEGRADED
        )

    linear_limit, angular_limit = _authority(
        desired_state,
        evidence,
        config,
    )
    return SafetyLifecycleTransition(
        policy_version=config.policy_version,
        state=desired_state,
        reason=desired_reason,
        evaluated_at_ns=checked_now_ns,
        monotonic_time_ns=checked_now_ns,
        distance_min_m=evidence.distance_min_m,
        max_linear_velocity_mps=linear_limit,
        max_angular_velocity_radps=angular_limit,
    )


class SafetyRecoveryManager:
    """Stateful façade over the pure constant-time transition function."""

    def __init__(self, config: SafetyRecoveryConfig | None = None) -> None:
        self._config = config or SafetyRecoveryConfig()
        self._transition: SafetyLifecycleTransition | None = None

    @property
    def config(self) -> SafetyRecoveryConfig:
        """Return the immutable active policy."""
        return self._config

    @property
    def transition(self) -> SafetyLifecycleTransition | None:
        """Return the most recently emitted transition."""
        return self._transition

    def update(
        self,
        evidence: SafetyLifecycleEvidence,
        *,
        now_ns: object,
    ) -> SafetyLifecycleTransition:
        """Advance the lifecycle exactly once."""
        self._transition = transition_safety_lifecycle(
            self._transition,
            evidence,
            self._config,
            now_ns=now_ns,
        )
        return self._transition


__all__ = [
    "SafetyLifecycleEvidence",
    "SafetyLifecycleReason",
    "SafetyLifecycleState",
    "SafetyLifecycleTransition",
    "SafetyRecoveryConfig",
    "SafetyRecoveryManager",
    "transition_safety_lifecycle",
]
