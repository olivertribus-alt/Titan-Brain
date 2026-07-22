"""Deterministic fail-closed safety-state stabilization for TB-EVAL-003."""

from __future__ import annotations

import hashlib
import json
import math
from enum import StrEnum
from typing import Self, cast

from pydantic import Field, model_validator

from core.safety import SafetyDecisionResult
from core.types.incident import DecisionEvidence, EvidenceValue, StrictFrozenModel


class InstantaneousSafetyLevel(StrEnum):
    """Unfiltered safety level produced by one observation evaluation."""

    OK = "ok"
    WARNING = "warning"
    E_STOP = "e_stop"
    INVALID = "invalid"


class EvaluatorState(StrEnum):
    """Externally visible state after hysteresis and recovery timing."""

    OK = "ok"
    WARNING = "warning"
    E_STOP = "e_stop"
    RECOVERY_HOLDING = "recovery_holding"


class StabilityReason(StrEnum):
    """Stable machine-readable cause of one state transition."""

    STABLE_OK = "stable_ok"
    RAW_WARNING = "raw_warning"
    RAW_E_STOP = "raw_e_stop"
    INVALID_EVIDENCE = "invalid_evidence"
    INVALID_TIME = "invalid_time"
    CLOCK_REGRESSION = "clock_regression"
    OBSERVATION_TIMEOUT = "observation_timeout"
    HYSTERESIS_NOT_MET = "hysteresis_not_met"
    HOLD_STARTED = "hold_started"
    HOLD_IN_PROGRESS = "hold_in_progress"
    HOLD_COMPLETED = "hold_completed"


class StabilityConfig(StrictFrozenModel):
    """Explicit hysteresis and recovery assumptions for one deployment."""

    policy_version: str = Field(min_length=1)
    clearance_hysteresis_m: float = Field(ge=0.0)
    recovery_hold_time_ns: int = Field(gt=0)


class InstantaneousSafetySignal(StrictFrozenModel):
    """Typed clearance comparison consumed by the transition function."""

    level: InstantaneousSafetyLevel
    observed_clearance_m: float | None = Field(default=None, ge=0.0)
    required_clearance_m: float | None = Field(default=None, ge=0.0)

    @model_validator(mode="after")
    def validate_clearance_pair(self) -> Self:
        """Require complete clearance evidence for a nominally safe input."""
        comparison = (
            self.observed_clearance_m,
            self.required_clearance_m,
        )
        if (comparison[0] is None) != (comparison[1] is None):
            raise ValueError("clearance evidence must be complete or absent")
        if self.level is InstantaneousSafetyLevel.OK and comparison[0] is None:
            raise ValueError("OK signal requires clearance evidence")
        return self


class StabilityTransition(StrictFrozenModel):
    """Complete deterministic state carried between evaluator invocations."""

    state: EvaluatorState
    instantaneous_level: InstantaneousSafetyLevel
    reason: StabilityReason
    evaluated_at_ns: int = Field(ge=0)
    monotonic_time_ns: int = Field(ge=0)
    latched_unsafe_level: InstantaneousSafetyLevel | None = None
    recovery_started_at_ns: int | None = Field(default=None, ge=0)
    hold_elapsed_ns: int | None = Field(default=None, ge=0)
    release_threshold_m: float | None = Field(default=None, ge=0.0)

    @model_validator(mode="after")
    def validate_shape(self) -> Self:
        """Keep latch, timer, and externally visible state consistent."""
        if self.monotonic_time_ns < self.evaluated_at_ns:
            raise ValueError("monotonic time must cover the evaluated time")
        if self.state is EvaluatorState.OK:
            if self.instantaneous_level is not InstantaneousSafetyLevel.OK:
                raise ValueError("OK state requires an instantaneous safe signal")
            if any(
                value is not None
                for value in (
                    self.latched_unsafe_level,
                    self.recovery_started_at_ns,
                    self.hold_elapsed_ns,
                    self.release_threshold_m,
                )
            ):
                raise ValueError("OK state must not retain recovery state")
            return self

        if self.latched_unsafe_level not in {
            InstantaneousSafetyLevel.WARNING,
            InstantaneousSafetyLevel.E_STOP,
        }:
            raise ValueError("non-OK state requires an unsafe latch")
        if (
            self.state is EvaluatorState.WARNING
            and self.latched_unsafe_level is not InstantaneousSafetyLevel.WARNING
        ):
            raise ValueError("WARNING state requires a warning latch")
        if (
            self.state is EvaluatorState.E_STOP
            and self.latched_unsafe_level is not InstantaneousSafetyLevel.E_STOP
        ):
            raise ValueError("E_STOP state requires an e-stop latch")

        if self.state is EvaluatorState.RECOVERY_HOLDING:
            if any(
                value is None
                for value in (
                    self.recovery_started_at_ns,
                    self.hold_elapsed_ns,
                    self.release_threshold_m,
                )
            ):
                raise ValueError("recovery holding requires timer evidence")
            recovery_started_at_ns = cast(int, self.recovery_started_at_ns)
            hold_elapsed_ns = cast(int, self.hold_elapsed_ns)
            if recovery_started_at_ns > self.evaluated_at_ns:
                raise ValueError("recovery cannot start after evaluation")
            if hold_elapsed_ns != self.evaluated_at_ns - recovery_started_at_ns:
                raise ValueError("hold elapsed time must match recovery timestamps")
        elif (
            self.recovery_started_at_ns is not None
            or self.hold_elapsed_ns is not None
        ):
            raise ValueError("non-holding unsafe state must not retain a timer")
        return self


class StabilizedSafetyResult(StrictFrozenModel):
    """Instantaneous and effective decisions paired with transition evidence."""

    instantaneous: SafetyDecisionResult
    effective: SafetyDecisionResult
    transition: StabilityTransition

    @model_validator(mode="after")
    def validate_effective_incident(self) -> Self:
        """Every non-OK stabilized state must retain stop authority."""
        expected_incident = self.transition.state is not EvaluatorState.OK
        if self.effective.is_incident is not expected_incident:
            raise ValueError("effective incident flag does not match stable state")
        return self


def _unsafe_transition(
    *,
    level: InstantaneousSafetyLevel,
    reason: StabilityReason,
    evaluated_at_ns: int,
    monotonic_time_ns: int,
) -> StabilityTransition:
    state = (
        EvaluatorState.WARNING
        if level is InstantaneousSafetyLevel.WARNING
        else EvaluatorState.E_STOP
    )
    latched = (
        InstantaneousSafetyLevel.WARNING
        if state is EvaluatorState.WARNING
        else InstantaneousSafetyLevel.E_STOP
    )
    return StabilityTransition(
        state=state,
        instantaneous_level=level,
        reason=reason,
        evaluated_at_ns=evaluated_at_ns,
        monotonic_time_ns=monotonic_time_ns,
        latched_unsafe_level=latched,
    )


def _checked_now_ns(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _latched_level(previous: StabilityTransition) -> InstantaneousSafetyLevel:
    return cast(InstantaneousSafetyLevel, previous.latched_unsafe_level)


def transition_stability(
    previous: StabilityTransition | None,
    signal: InstantaneousSafetySignal,
    config: StabilityConfig,
    *,
    now_ns: object,
) -> StabilityTransition:
    """Apply an immediate-entry, delayed-release deterministic transition."""
    checked_now_ns = _checked_now_ns(now_ns)
    monotonic_floor = previous.monotonic_time_ns if previous is not None else 0
    if checked_now_ns is None:
        return _unsafe_transition(
            level=InstantaneousSafetyLevel.INVALID,
            reason=StabilityReason.INVALID_TIME,
            evaluated_at_ns=0,
            monotonic_time_ns=monotonic_floor,
        )
    if previous is not None and checked_now_ns < previous.monotonic_time_ns:
        return _unsafe_transition(
            level=signal.level,
            reason=StabilityReason.CLOCK_REGRESSION,
            evaluated_at_ns=checked_now_ns,
            monotonic_time_ns=previous.monotonic_time_ns,
        )

    if signal.level is InstantaneousSafetyLevel.INVALID:
        return _unsafe_transition(
            level=signal.level,
            reason=StabilityReason.INVALID_EVIDENCE,
            evaluated_at_ns=checked_now_ns,
            monotonic_time_ns=checked_now_ns,
        )
    if signal.level is InstantaneousSafetyLevel.E_STOP:
        return _unsafe_transition(
            level=signal.level,
            reason=StabilityReason.RAW_E_STOP,
            evaluated_at_ns=checked_now_ns,
            monotonic_time_ns=checked_now_ns,
        )
    if signal.level is InstantaneousSafetyLevel.WARNING:
        return _unsafe_transition(
            level=signal.level,
            reason=StabilityReason.RAW_WARNING,
            evaluated_at_ns=checked_now_ns,
            monotonic_time_ns=checked_now_ns,
        )

    observed = cast(float, signal.observed_clearance_m)
    required = cast(float, signal.required_clearance_m)

    if previous is not None and previous.state is EvaluatorState.OK:
        return StabilityTransition(
            state=EvaluatorState.OK,
            instantaneous_level=signal.level,
            reason=StabilityReason.STABLE_OK,
            evaluated_at_ns=checked_now_ns,
            monotonic_time_ns=checked_now_ns,
        )

    release_threshold_m = required + config.clearance_hysteresis_m
    if not math.isfinite(release_threshold_m):
        return _unsafe_transition(
            level=InstantaneousSafetyLevel.INVALID,
            reason=StabilityReason.INVALID_EVIDENCE,
            evaluated_at_ns=checked_now_ns,
            monotonic_time_ns=checked_now_ns,
        )
    latched_level = (
        InstantaneousSafetyLevel.E_STOP
        if previous is None
        else _latched_level(previous)
    )
    if observed < release_threshold_m:
        return StabilityTransition(
            state=(
                EvaluatorState.WARNING
                if latched_level is InstantaneousSafetyLevel.WARNING
                else EvaluatorState.E_STOP
            ),
            instantaneous_level=signal.level,
            reason=StabilityReason.HYSTERESIS_NOT_MET,
            evaluated_at_ns=checked_now_ns,
            monotonic_time_ns=checked_now_ns,
            latched_unsafe_level=latched_level,
            release_threshold_m=release_threshold_m,
        )

    recovery_started_at_ns = (
        cast(int, previous.recovery_started_at_ns)
        if previous is not None
        and previous.state is EvaluatorState.RECOVERY_HOLDING
        else checked_now_ns
    )
    hold_elapsed_ns = checked_now_ns - recovery_started_at_ns
    if hold_elapsed_ns >= config.recovery_hold_time_ns:
        return StabilityTransition(
            state=EvaluatorState.OK,
            instantaneous_level=signal.level,
            reason=StabilityReason.HOLD_COMPLETED,
            evaluated_at_ns=checked_now_ns,
            monotonic_time_ns=checked_now_ns,
        )
    return StabilityTransition(
        state=EvaluatorState.RECOVERY_HOLDING,
        instantaneous_level=signal.level,
        reason=(
            StabilityReason.HOLD_IN_PROGRESS
            if previous is not None
            and previous.state is EvaluatorState.RECOVERY_HOLDING
            else StabilityReason.HOLD_STARTED
        ),
        evaluated_at_ns=checked_now_ns,
        monotonic_time_ns=checked_now_ns,
        latched_unsafe_level=latched_level,
        recovery_started_at_ns=recovery_started_at_ns,
        hold_elapsed_ns=hold_elapsed_ns,
        release_threshold_m=release_threshold_m,
    )


def force_fail_closed_stability(
    previous: StabilityTransition | None,
    *,
    now_ns: object,
    reason: StabilityReason,
) -> StabilityTransition:
    """Latch an explicit external health fault without delaying stop entry."""
    if reason is not StabilityReason.OBSERVATION_TIMEOUT:
        raise ValueError("unsupported external fail-closed reason")
    checked_now_ns = _checked_now_ns(now_ns)
    monotonic_floor = previous.monotonic_time_ns if previous is not None else 0
    if checked_now_ns is None:
        return _unsafe_transition(
            level=InstantaneousSafetyLevel.INVALID,
            reason=StabilityReason.INVALID_TIME,
            evaluated_at_ns=0,
            monotonic_time_ns=monotonic_floor,
        )
    if previous is not None and checked_now_ns < previous.monotonic_time_ns:
        return _unsafe_transition(
            level=InstantaneousSafetyLevel.INVALID,
            reason=StabilityReason.CLOCK_REGRESSION,
            evaluated_at_ns=checked_now_ns,
            monotonic_time_ns=previous.monotonic_time_ns,
        )
    return _unsafe_transition(
        level=InstantaneousSafetyLevel.INVALID,
        reason=reason,
        evaluated_at_ns=checked_now_ns,
        monotonic_time_ns=checked_now_ns,
    )


def _number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    checked = float(value)
    return checked if math.isfinite(checked) and checked >= 0.0 else None


def signal_from_decision(result: SafetyDecisionResult) -> InstantaneousSafetySignal:
    """Extract a strict instantaneous signal from evaluator evidence."""
    action = result.decision.action
    level = {
        "proceed": InstantaneousSafetyLevel.OK,
        "clamp": InstantaneousSafetyLevel.OK,
        "protective_stop": InstantaneousSafetyLevel.WARNING,
        "emergency_stop": InstantaneousSafetyLevel.E_STOP,
    }.get(action, InstantaneousSafetyLevel.INVALID)
    clearance = result.decision.evidence.get("clearance")
    observed = _number(clearance.observed) if clearance is not None else None
    required = _number(clearance.threshold) if clearance is not None else None
    if (observed is None) != (required is None):
        observed = None
        required = None
    if level is InstantaneousSafetyLevel.OK and (
        observed is None or required is None
    ):
        level = InstantaneousSafetyLevel.INVALID
        observed = None
        required = None
    return InstantaneousSafetySignal(
        level=level,
        observed_clearance_m=observed,
        required_clearance_m=required,
    )


def _stable_decision_id(
    instantaneous: SafetyDecisionResult,
    transition: StabilityTransition,
    config: StabilityConfig,
) -> str:
    canonical = json.dumps(
        {
            "instantaneous": instantaneous.model_dump(mode="json"),
            "transition": transition.model_dump(mode="json"),
            "config": config.model_dump(mode="json"),
        },
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return f"incident_{hashlib.sha256(canonical).hexdigest()[:16]}"


def _effective_action(transition: StabilityTransition) -> str:
    if transition.state is EvaluatorState.WARNING:
        return "protective_stop"
    if transition.state is EvaluatorState.E_STOP:
        return "emergency_stop"
    if transition.latched_unsafe_level is InstantaneousSafetyLevel.WARNING:
        return "protective_stop"
    return "emergency_stop"


def _effective_rule(reason: StabilityReason) -> str:
    return {
        StabilityReason.HYSTERESIS_NOT_MET: "EV-STABLE-HYSTERESIS",
        StabilityReason.HOLD_STARTED: "EV-STABLE-RECOVERY-HOLD",
        StabilityReason.HOLD_IN_PROGRESS: "EV-STABLE-RECOVERY-HOLD",
        StabilityReason.CLOCK_REGRESSION: "EV-STABLE-CLOCK",
        StabilityReason.INVALID_TIME: "EV-STABLE-TIME",
        StabilityReason.INVALID_EVIDENCE: "EV-STABLE-INVALID",
    }.get(reason, "EV-STABLE-FAIL-CLOSED")


def apply_stability_transition(
    instantaneous: SafetyDecisionResult,
    transition: StabilityTransition,
    config: StabilityConfig,
) -> SafetyDecisionResult:
    """Convert a transition into the effective actuator-facing decision."""
    if transition.state is EvaluatorState.OK:
        return instantaneous
    if transition.reason in {
        StabilityReason.RAW_WARNING,
        StabilityReason.RAW_E_STOP,
    }:
        return instantaneous

    evidence = dict(instantaneous.decision.evidence)
    evidence.update(
        {
            "stability_state": EvidenceValue(
                label="Evaluator Stability State",
                value=transition.state.value,
            ),
            "stability_reason": EvidenceValue(
                label="Stability Transition Reason",
                value=transition.reason.value,
            ),
            "stability_policy_version": EvidenceValue(
                label="Stability Policy Version",
                value=config.policy_version,
            ),
            "clearance_hysteresis": EvidenceValue(
                label="Clearance Hysteresis",
                value=config.clearance_hysteresis_m,
                unit="m",
            ),
            "recovery_hold_time": EvidenceValue(
                label="Recovery Hold Time",
                value=config.recovery_hold_time_ns,
                unit="ns",
            ),
            "instantaneous_action": EvidenceValue(
                label="Instantaneous Evaluator Action",
                value=instantaneous.decision.action,
            ),
        }
    )
    if transition.release_threshold_m is not None:
        evidence["release_threshold"] = EvidenceValue(
            label="Recovery Release Threshold",
            value=transition.release_threshold_m,
            threshold=transition.release_threshold_m,
            unit="m",
        )
    if transition.hold_elapsed_ns is not None:
        evidence["recovery_hold_elapsed"] = EvidenceValue(
            label="Recovery Hold Elapsed",
            value=transition.hold_elapsed_ns,
            unit="ns",
        )

    raw = instantaneous.decision
    decision = DecisionEvidence(
        decision_id=_stable_decision_id(instantaneous, transition, config),
        occurred_at_unix_ns=raw.occurred_at_unix_ns,
        source_module="safety_stability_filter",
        action=_effective_action(transition),
        rule=_effective_rule(transition.reason),
        evidence=evidence,
        spatial_context=raw.spatial_context,
    )
    return SafetyDecisionResult(decision=decision, is_incident=True)


class SafetyStabilityFilter:
    """Minimal state holder around the pure transition function."""

    def __init__(self, config: StabilityConfig) -> None:
        self._config = config
        self._last_transition: StabilityTransition | None = None

    @property
    def config(self) -> StabilityConfig:
        """Return the immutable stabilization policy."""
        return self._config

    @property
    def last_transition(self) -> StabilityTransition | None:
        """Return the last transition without exposing mutable state."""
        return self._last_transition

    def process(
        self,
        instantaneous: SafetyDecisionResult,
        *,
        now_ns: object,
    ) -> StabilizedSafetyResult:
        """Evaluate and retain exactly one deterministic transition."""
        transition = transition_stability(
            self._last_transition,
            signal_from_decision(instantaneous),
            self._config,
            now_ns=now_ns,
        )
        effective = apply_stability_transition(
            instantaneous,
            transition,
            self._config,
        )
        self._last_transition = transition
        return StabilizedSafetyResult(
            instantaneous=instantaneous,
            effective=effective,
            transition=transition,
        )

    def force_observation_timeout(self, *, now_ns: object) -> StabilityTransition:
        """Reset recovery after a transport gap exceeded its watchdog limit."""
        transition = force_fail_closed_stability(
            self._last_transition,
            now_ns=now_ns,
            reason=StabilityReason.OBSERVATION_TIMEOUT,
        )
        self._last_transition = transition
        return transition
