"""Unit and integration tests for TB-EVAL-003 safety stabilization."""

from __future__ import annotations

import hashlib

import pytest
from pydantic import ValidationError

from core.safety import SafetyDecisionResult
from core.stability import (
    EvaluatorState,
    InstantaneousSafetyLevel,
    InstantaneousSafetySignal,
    SafetyStabilityFilter,
    StabilityConfig,
    StabilityReason,
    StabilityTransition,
    StabilizedSafetyResult,
    apply_stability_transition,
    force_fail_closed_stability,
    signal_from_decision,
    transition_stability,
)
from core.types.incident import DecisionEvidence, EvidenceValue

HOLD_NS = 200_000_000


def _config(
    *,
    hysteresis_m: float = 0.10,
    hold_ns: int = HOLD_NS,
) -> StabilityConfig:
    return StabilityConfig(
        policy_version="TB-STABILITY-0.1.0",
        clearance_hysteresis_m=hysteresis_m,
        recovery_hold_time_ns=hold_ns,
    )


def _decision(
    *,
    action: str,
    observed: object = 0.8,
    threshold: object = 0.5,
    include_clearance: bool = True,
) -> SafetyDecisionResult:
    evidence: dict[str, EvidenceValue] = {}
    if include_clearance:
        evidence["clearance"] = EvidenceValue(
            label="Test Clearance",
            value=observed,  # type: ignore[arg-type]
            observed=observed,  # type: ignore[arg-type]
            threshold=threshold,  # type: ignore[arg-type]
            unit="m",
        )
    decision = DecisionEvidence(
        decision_id="decision_input",
        occurred_at_unix_ns=1_000,
        source_module="test",
        action=action,
        rule="RAW-TEST",
        evidence=evidence,
    )
    return SafetyDecisionResult(
        decision=decision,
        is_incident=action in {"protective_stop", "emergency_stop"},
    )


def _signal(
    level: InstantaneousSafetyLevel,
    *,
    observed: float | None = None,
    required: float | None = None,
) -> InstantaneousSafetySignal:
    return InstantaneousSafetySignal(
        level=level,
        observed_clearance_m=observed,
        required_clearance_m=required,
    )


def test_initial_safe_signal_starts_fail_closed_recovery_hold() -> None:
    result = transition_stability(
        None,
        _signal(InstantaneousSafetyLevel.OK, observed=0.8, required=0.5),
        _config(),
        now_ns=10,
    )

    assert result.state is EvaluatorState.RECOVERY_HOLDING
    assert result.reason is StabilityReason.HOLD_STARTED
    assert result.latched_unsafe_level is InstantaneousSafetyLevel.E_STOP
    assert result.recovery_started_at_ns == 10


@pytest.mark.parametrize(
    ("level", "state", "reason"),
    [
        (
            InstantaneousSafetyLevel.WARNING,
            EvaluatorState.WARNING,
            StabilityReason.RAW_WARNING,
        ),
        (
            InstantaneousSafetyLevel.E_STOP,
            EvaluatorState.E_STOP,
            StabilityReason.RAW_E_STOP,
        ),
        (
            InstantaneousSafetyLevel.INVALID,
            EvaluatorState.E_STOP,
            StabilityReason.INVALID_EVIDENCE,
        ),
    ],
)
def test_unsafe_and_invalid_inputs_enter_stop_without_delay(
    level: InstantaneousSafetyLevel,
    state: EvaluatorState,
    reason: StabilityReason,
) -> None:
    transition = transition_stability(
        None,
        _signal(level),
        _config(),
        now_ns=123,
    )

    assert transition.state is state
    assert transition.reason is reason
    assert transition.evaluated_at_ns == 123
    assert transition.recovery_started_at_ns is None


def test_estop_release_requires_hysteresis_and_full_hold_window() -> None:
    config = _config()
    danger = transition_stability(
        None,
        _signal(InstantaneousSafetyLevel.E_STOP),
        config,
        now_ns=1_000,
    )

    inside_hysteresis = transition_stability(
        danger,
        _signal(InstantaneousSafetyLevel.OK, observed=0.599, required=0.5),
        config,
        now_ns=2_000,
    )
    assert inside_hysteresis.state is EvaluatorState.E_STOP
    assert inside_hysteresis.reason is StabilityReason.HYSTERESIS_NOT_MET
    assert inside_hysteresis.recovery_started_at_ns is None
    assert inside_hysteresis.release_threshold_m == pytest.approx(0.6)

    started = transition_stability(
        inside_hysteresis,
        _signal(InstantaneousSafetyLevel.OK, observed=0.6, required=0.5),
        config,
        now_ns=3_000,
    )
    assert started.state is EvaluatorState.RECOVERY_HOLDING
    assert started.reason is StabilityReason.HOLD_STARTED
    assert started.recovery_started_at_ns == 3_000
    assert started.hold_elapsed_ns == 0

    holding = transition_stability(
        started,
        _signal(InstantaneousSafetyLevel.OK, observed=0.7, required=0.5),
        config,
        now_ns=3_000 + HOLD_NS - 1,
    )
    assert holding.state is EvaluatorState.RECOVERY_HOLDING
    assert holding.reason is StabilityReason.HOLD_IN_PROGRESS
    assert holding.hold_elapsed_ns == HOLD_NS - 1

    released = transition_stability(
        holding,
        _signal(InstantaneousSafetyLevel.OK, observed=0.6, required=0.5),
        config,
        now_ns=3_000 + HOLD_NS,
    )
    assert released.state is EvaluatorState.OK
    assert released.reason is StabilityReason.HOLD_COMPLETED
    assert released.latched_unsafe_level is None

    stable = transition_stability(
        released,
        _signal(InstantaneousSafetyLevel.OK, observed=0.5, required=0.5),
        config,
        now_ns=3_000 + HOLD_NS + 1,
    )
    assert stable.state is EvaluatorState.OK
    assert stable.reason is StabilityReason.STABLE_OK


def test_new_danger_cancels_hold_immediately_and_restarts_full_timer() -> None:
    filter_ = SafetyStabilityFilter(_config(hold_ns=100))
    filter_.process(_decision(action="emergency_stop"), now_ns=0)
    first_hold = filter_.process(
        _decision(action="proceed", observed=0.7),
        now_ns=10,
    )
    assert first_hold.transition.state is EvaluatorState.RECOVERY_HOLDING

    danger = filter_.process(_decision(action="emergency_stop"), now_ns=50)
    assert danger.transition.state is EvaluatorState.E_STOP
    assert danger.transition.reason is StabilityReason.RAW_E_STOP
    assert danger.transition.recovery_started_at_ns is None

    restarted = filter_.process(
        _decision(action="proceed", observed=0.7),
        now_ns=60,
    )
    still_holding = filter_.process(
        _decision(action="proceed", observed=0.7),
        now_ns=159,
    )
    released = filter_.process(
        _decision(action="proceed", observed=0.7),
        now_ns=160,
    )

    assert restarted.transition.recovery_started_at_ns == 60
    assert still_holding.transition.state is EvaluatorState.RECOVERY_HOLDING
    assert released.transition.state is EvaluatorState.OK


def test_warning_recovery_holds_protective_stop_authority() -> None:
    filter_ = SafetyStabilityFilter(_config(hold_ns=100))
    warning = filter_.process(_decision(action="protective_stop"), now_ns=0)
    holding = filter_.process(
        _decision(action="proceed", observed=0.7),
        now_ns=1,
    )

    assert warning.transition.state is EvaluatorState.WARNING
    assert holding.transition.state is EvaluatorState.RECOVERY_HOLDING
    assert holding.effective.decision.action == "protective_stop"
    assert holding.effective.decision.rule == "EV-STABLE-RECOVERY-HOLD"
    assert holding.effective.is_incident is True


@pytest.mark.parametrize(
    ("unsafe_action", "effective_action", "state"),
    [
        ("protective_stop", "protective_stop", EvaluatorState.WARNING),
        ("emergency_stop", "emergency_stop", EvaluatorState.E_STOP),
    ],
)
def test_hysteresis_latch_preserves_original_stop_authority(
    unsafe_action: str,
    effective_action: str,
    state: EvaluatorState,
) -> None:
    filter_ = SafetyStabilityFilter(_config())
    filter_.process(_decision(action=unsafe_action), now_ns=0)

    result = filter_.process(
        _decision(action="proceed", observed=0.55),
        now_ns=1,
    )

    assert result.transition.state is state
    assert result.transition.reason is StabilityReason.HYSTERESIS_NOT_MET
    assert result.effective.decision.action == effective_action
    assert result.effective.decision.rule == "EV-STABLE-HYSTERESIS"


def test_hysteresis_noise_never_starts_or_accumulates_hold() -> None:
    config = _config(hold_ns=100)
    previous = transition_stability(
        None,
        _signal(InstantaneousSafetyLevel.E_STOP),
        config,
        now_ns=0,
    )

    for now_ns, clearance in enumerate((0.59, 0.6, 0.599, 0.6, 0.58), 1):
        previous = transition_stability(
            previous,
            _signal(
                InstantaneousSafetyLevel.OK,
                observed=clearance,
                required=0.5,
            ),
            config,
            now_ns=now_ns,
        )

    assert previous.state is EvaluatorState.E_STOP
    assert previous.reason is StabilityReason.HYSTERESIS_NOT_MET
    assert previous.recovery_started_at_ns is None


def test_clock_regression_fails_closed_until_monotonic_floor_is_reached() -> None:
    config = _config(hold_ns=10)
    ok = transition_stability(
        None,
        _signal(InstantaneousSafetyLevel.OK, observed=1.0, required=0.5),
        config,
        now_ns=100,
    )
    regression = transition_stability(
        ok,
        _signal(InstantaneousSafetyLevel.OK, observed=1.0, required=0.5),
        config,
        now_ns=99,
    )
    repeated = transition_stability(
        regression,
        _signal(InstantaneousSafetyLevel.OK, observed=1.0, required=0.5),
        config,
        now_ns=99,
    )
    caught_up = transition_stability(
        repeated,
        _signal(InstantaneousSafetyLevel.OK, observed=1.0, required=0.5),
        config,
        now_ns=100,
    )

    assert regression.state is EvaluatorState.E_STOP
    assert regression.reason is StabilityReason.CLOCK_REGRESSION
    assert regression.monotonic_time_ns == 100
    assert repeated.reason is StabilityReason.CLOCK_REGRESSION
    assert caught_up.state is EvaluatorState.RECOVERY_HOLDING


def test_external_observation_timeout_resets_recovery_fail_closed() -> None:
    filter_ = SafetyStabilityFilter(_config(hold_ns=100))
    filter_.process(_decision(action="emergency_stop"), now_ns=0)
    filter_.process(_decision(action="proceed", observed=0.7), now_ns=10)

    timeout = filter_.force_observation_timeout(now_ns=50)
    restarted = filter_.process(
        _decision(action="proceed", observed=0.7),
        now_ns=50,
    )

    assert timeout.state is EvaluatorState.E_STOP
    assert timeout.reason is StabilityReason.OBSERVATION_TIMEOUT
    assert restarted.transition.state is EvaluatorState.RECOVERY_HOLDING
    assert restarted.transition.recovery_started_at_ns == 50
    assert restarted.effective.decision.action == "emergency_stop"


def test_external_fail_closed_transition_validates_reason_and_time() -> None:
    with pytest.raises(ValueError, match="unsupported external"):
        force_fail_closed_stability(
            None,
            now_ns=0,
            reason=StabilityReason.RAW_E_STOP,
        )
    invalid_time = force_fail_closed_stability(
        None,
        now_ns=True,
        reason=StabilityReason.OBSERVATION_TIMEOUT,
    )
    previous = transition_stability(
        None,
        _signal(InstantaneousSafetyLevel.E_STOP),
        _config(),
        now_ns=10,
    )
    regression = force_fail_closed_stability(
        previous,
        now_ns=9,
        reason=StabilityReason.OBSERVATION_TIMEOUT,
    )

    assert invalid_time.reason is StabilityReason.INVALID_TIME
    assert regression.reason is StabilityReason.CLOCK_REGRESSION


@pytest.mark.parametrize("now_ns", [-1, True, 1.5, "1"])
def test_invalid_time_is_a_fail_closed_estop(now_ns: object) -> None:
    transition = transition_stability(
        None,
        _signal(InstantaneousSafetyLevel.OK, observed=1.0, required=0.5),
        _config(),
        now_ns=now_ns,
    )

    assert transition.state is EvaluatorState.E_STOP
    assert transition.reason is StabilityReason.INVALID_TIME
    assert transition.monotonic_time_ns == 0


def test_overflowing_release_threshold_fails_closed() -> None:
    previous = transition_stability(
        None,
        _signal(InstantaneousSafetyLevel.E_STOP),
        _config(hysteresis_m=1e308),
        now_ns=0,
    )
    result = transition_stability(
        previous,
        _signal(InstantaneousSafetyLevel.OK, observed=1e308, required=1e308),
        _config(hysteresis_m=1e308),
        now_ns=1,
    )

    assert result.state is EvaluatorState.E_STOP
    assert result.reason is StabilityReason.INVALID_EVIDENCE


@pytest.mark.parametrize(
    ("observed", "threshold"),
    [
        (True, 0.5),
        ("0.8", 0.5),
        (-0.1, 0.5),
        (0.8, False),
        (0.8, "0.5"),
        (0.8, -0.1),
    ],
)
def test_invalid_safe_clearance_evidence_is_fail_closed(
    observed: object,
    threshold: object,
) -> None:
    result = _decision(
        action="proceed",
        observed=observed,
        threshold=threshold,
    )
    signal = signal_from_decision(result)

    assert signal.level is InstantaneousSafetyLevel.INVALID
    transition = transition_stability(None, signal, _config(), now_ns=0)
    assert transition.state is EvaluatorState.E_STOP


@pytest.mark.parametrize(
    ("action", "level"),
    [
        ("protective_stop", InstantaneousSafetyLevel.WARNING),
        ("emergency_stop", InstantaneousSafetyLevel.E_STOP),
    ],
)
def test_partial_clearance_evidence_cannot_break_immediate_stop(
    action: str,
    level: InstantaneousSafetyLevel,
) -> None:
    signal = signal_from_decision(
        _decision(action=action, observed=True, threshold=0.5)
    )

    assert signal.level is level
    assert signal.observed_clearance_m is None
    assert signal.required_clearance_m is None
    transition = transition_stability(None, signal, _config(), now_ns=0)
    assert transition.state is not EvaluatorState.OK


def test_missing_clearance_or_unknown_action_is_invalid() -> None:
    missing = signal_from_decision(_decision(action="proceed", include_clearance=False))
    unknown = signal_from_decision(_decision(action="unexpected"))

    assert missing.level is InstantaneousSafetyLevel.INVALID
    assert unknown.level is InstantaneousSafetyLevel.INVALID


def test_clamp_is_treated_as_nominal_safe_input() -> None:
    signal = signal_from_decision(_decision(action="clamp"))

    assert signal.level is InstantaneousSafetyLevel.OK
    assert signal.observed_clearance_m == 0.8
    assert signal.required_clearance_m == 0.5


def test_effective_holding_decision_contains_auditable_evidence() -> None:
    config = _config(hold_ns=100)
    filter_ = SafetyStabilityFilter(config)
    filter_.process(_decision(action="emergency_stop"), now_ns=0)

    result = filter_.process(
        _decision(action="proceed", observed=0.7),
        now_ns=10,
    )

    evidence = result.effective.decision.evidence
    assert result.effective.decision.source_module == "safety_stability_filter"
    assert result.effective.decision.action == "emergency_stop"
    assert result.effective.decision.decision_id is not None
    assert evidence["stability_state"].value == "recovery_holding"
    assert evidence["stability_reason"].value == "hold_started"
    assert evidence["release_threshold"].value == pytest.approx(0.6)
    assert evidence["recovery_hold_elapsed"].value == 0
    assert evidence["instantaneous_action"].value == "proceed"


def test_raw_unsafe_decision_is_not_rewritten() -> None:
    config = _config()
    raw = _decision(action="emergency_stop")
    transition = transition_stability(
        None,
        signal_from_decision(raw),
        config,
        now_ns=0,
    )

    assert apply_stability_transition(raw, transition, config) == raw


def test_filter_is_deterministic_across_identical_sequences() -> None:
    config = _config(hold_ns=10)
    sequence = (
        (_decision(action="emergency_stop"), 0),
        (_decision(action="proceed", observed=0.7), 1),
        (_decision(action="proceed", observed=0.7), 5),
        (_decision(action="proceed", observed=0.7), 11),
    )
    serialized: list[str] = []
    for _ in range(100):
        filter_ = SafetyStabilityFilter(config)
        final = None
        for decision, now_ns in sequence:
            final = filter_.process(decision, now_ns=now_ns)
        assert final is not None
        assert filter_.config == config
        assert filter_.last_transition == final.transition
        serialized.append(final.model_dump_json())

    hashes = {hashlib.sha256(value.encode("utf-8")).hexdigest() for value in serialized}
    assert len(set(serialized)) == 1
    assert len(hashes) == 1


def test_models_reject_inconsistent_shapes() -> None:
    with pytest.raises(ValidationError, match="complete or absent"):
        InstantaneousSafetySignal(
            level=InstantaneousSafetyLevel.WARNING,
            observed_clearance_m=0.5,
        )
    with pytest.raises(ValidationError, match="OK signal requires"):
        InstantaneousSafetySignal(level=InstantaneousSafetyLevel.OK)
    with pytest.raises(ValidationError, match="OK state"):
        StabilityTransition(
            state=EvaluatorState.OK,
            instantaneous_level=InstantaneousSafetyLevel.OK,
            reason=StabilityReason.STABLE_OK,
            evaluated_at_ns=0,
            monotonic_time_ns=0,
            latched_unsafe_level=InstantaneousSafetyLevel.E_STOP,
        )
    with pytest.raises(ValidationError, match="instantaneous safe"):
        StabilityTransition(
            state=EvaluatorState.OK,
            instantaneous_level=InstantaneousSafetyLevel.E_STOP,
            reason=StabilityReason.STABLE_OK,
            evaluated_at_ns=0,
            monotonic_time_ns=0,
        )
    with pytest.raises(ValidationError, match="unsafe latch"):
        StabilityTransition(
            state=EvaluatorState.E_STOP,
            instantaneous_level=InstantaneousSafetyLevel.E_STOP,
            reason=StabilityReason.RAW_E_STOP,
            evaluated_at_ns=0,
            monotonic_time_ns=0,
        )
    with pytest.raises(ValidationError, match="timer evidence"):
        StabilityTransition(
            state=EvaluatorState.RECOVERY_HOLDING,
            instantaneous_level=InstantaneousSafetyLevel.OK,
            reason=StabilityReason.HOLD_STARTED,
            evaluated_at_ns=0,
            monotonic_time_ns=0,
            latched_unsafe_level=InstantaneousSafetyLevel.E_STOP,
        )
    with pytest.raises(ValidationError, match="must not retain a timer"):
        StabilityTransition(
            state=EvaluatorState.E_STOP,
            instantaneous_level=InstantaneousSafetyLevel.OK,
            reason=StabilityReason.HYSTERESIS_NOT_MET,
            evaluated_at_ns=1,
            monotonic_time_ns=1,
            latched_unsafe_level=InstantaneousSafetyLevel.E_STOP,
            recovery_started_at_ns=0,
            hold_elapsed_ns=1,
        )
    with pytest.raises(ValidationError, match="monotonic time"):
        StabilityTransition(
            state=EvaluatorState.E_STOP,
            instantaneous_level=InstantaneousSafetyLevel.E_STOP,
            reason=StabilityReason.RAW_E_STOP,
            evaluated_at_ns=2,
            monotonic_time_ns=1,
            latched_unsafe_level=InstantaneousSafetyLevel.E_STOP,
        )
    with pytest.raises(ValidationError, match="warning latch"):
        StabilityTransition(
            state=EvaluatorState.WARNING,
            instantaneous_level=InstantaneousSafetyLevel.WARNING,
            reason=StabilityReason.RAW_WARNING,
            evaluated_at_ns=0,
            monotonic_time_ns=0,
            latched_unsafe_level=InstantaneousSafetyLevel.E_STOP,
        )
    with pytest.raises(ValidationError, match="e-stop latch"):
        StabilityTransition(
            state=EvaluatorState.E_STOP,
            instantaneous_level=InstantaneousSafetyLevel.E_STOP,
            reason=StabilityReason.RAW_E_STOP,
            evaluated_at_ns=0,
            monotonic_time_ns=0,
            latched_unsafe_level=InstantaneousSafetyLevel.WARNING,
        )
    with pytest.raises(ValidationError, match="start after evaluation"):
        StabilityTransition(
            state=EvaluatorState.RECOVERY_HOLDING,
            instantaneous_level=InstantaneousSafetyLevel.OK,
            reason=StabilityReason.HOLD_STARTED,
            evaluated_at_ns=1,
            monotonic_time_ns=1,
            latched_unsafe_level=InstantaneousSafetyLevel.E_STOP,
            recovery_started_at_ns=2,
            hold_elapsed_ns=0,
            release_threshold_m=0.6,
        )
    with pytest.raises(ValidationError, match="elapsed time"):
        StabilityTransition(
            state=EvaluatorState.RECOVERY_HOLDING,
            instantaneous_level=InstantaneousSafetyLevel.OK,
            reason=StabilityReason.HOLD_IN_PROGRESS,
            evaluated_at_ns=2,
            monotonic_time_ns=2,
            latched_unsafe_level=InstantaneousSafetyLevel.E_STOP,
            recovery_started_at_ns=1,
            hold_elapsed_ns=0,
            release_threshold_m=0.6,
        )


def test_stabilized_result_rejects_nonincident_stop_state() -> None:
    raw_safe = _decision(action="proceed")
    stop_transition = transition_stability(
        None,
        _signal(InstantaneousSafetyLevel.E_STOP),
        _config(),
        now_ns=0,
    )

    with pytest.raises(ValidationError, match="incident flag"):
        StabilizedSafetyResult(
            instantaneous=raw_safe,
            effective=raw_safe,
            transition=stop_transition,
        )
