"""Acceptance tests for TB-ROS-PoC-002A VelocityArbiter."""

from __future__ import annotations

import hashlib
import json

import pytest
from pydantic import ValidationError

from core.arbitrator import (
    ArbitrationMode,
    ArbitrationReason,
    ArbitrationResult,
    DesiredVelocity,
    EvaluationAction,
    SafetyState,
    VelocityArbiter,
    VelocityArbiterConfig,
    WatchdogState,
)

NOW_NS = 10_000_000_000


@pytest.fixture
def config() -> VelocityArbiterConfig:
    return VelocityArbiterConfig(
        policy_version="TB-VEL-ARB-0.1.0",
        output_frame_id="base_link",
        command_stale_threshold_ns=100_000_000,
        safety_stale_threshold_ns=250_000_000,
        motion_envelope_stale_threshold_ns=50_000_000,
        max_abs_linear_x=0.8,
        max_abs_linear_y=0.2,
        max_abs_angular_z=1.5,
    )


@pytest.fixture
def arbiter(config: VelocityArbiterConfig) -> VelocityArbiter:
    return VelocityArbiter(config)


def _command(
    *,
    timestamp_ns: int = NOW_NS,
    frame_id: str = "base_link",
    linear_x: float = 0.4,
    linear_y: float = 0.1,
    angular_z: float = 0.5,
) -> DesiredVelocity:
    return DesiredVelocity(
        linear_x=linear_x,
        linear_y=linear_y,
        angular_z=angular_z,
        timestamp_ns=timestamp_ns,
        frame_id=frame_id,
    )


def _safety(
    *,
    timestamp_ns: int = NOW_NS,
    is_safe: bool = True,
    watchdog_state: WatchdogState = WatchdogState.HEALTHY,
    eval_action: EvaluationAction = EvaluationAction.PROCEED,
) -> SafetyState:
    return SafetyState(
        is_safe=is_safe,
        watchdog_state=watchdog_state,
        eval_action=eval_action,
        timestamp_ns=timestamp_ns,
    )


def _assert_forced_zero(
    result: ArbitrationResult,
    reason: ArbitrationReason,
) -> None:
    assert result.mode is ArbitrationMode.FORCED_ZERO
    assert result.reason is reason
    assert result.command.linear_x == 0.0
    assert result.command.linear_y == 0.0
    assert result.command.angular_z == 0.0
    assert result.command.frame_id == "base_link"


def test_fresh_healthy_proceed_is_exact_pass_through(
    arbiter: VelocityArbiter,
) -> None:
    desired = _command(linear_x=3.0)

    result = arbiter.arbitrate(desired, _safety(), now_ns=NOW_NS)

    assert result.mode is ArbitrationMode.PASS_THROUGH
    assert result.reason is ArbitrationReason.PROCEED
    assert result.command == desired


def test_clamp_applies_independent_symmetric_limits(
    arbiter: VelocityArbiter,
) -> None:
    result = arbiter.arbitrate(
        _command(linear_x=1.2, linear_y=-0.7, angular_z=2.5),
        _safety(eval_action=EvaluationAction.CLAMP),
        now_ns=NOW_NS,
    )

    assert result.mode is ArbitrationMode.CLAMPED
    assert result.reason is ArbitrationReason.CLAMP_POLICY
    assert result.command.linear_x == 0.8
    assert result.command.linear_y == -0.2
    assert result.command.angular_z == 1.5
    assert result.command.timestamp_ns == NOW_NS


def test_clamp_canonicalizes_negative_zero() -> None:
    config = VelocityArbiterConfig(
        policy_version="TB-VEL-ARB-0.1.0",
        output_frame_id="base_link",
        command_stale_threshold_ns=1,
        safety_stale_threshold_ns=1,
        motion_envelope_stale_threshold_ns=1,
        max_abs_linear_x=0.0,
        max_abs_linear_y=0.0,
        max_abs_angular_z=0.0,
    )
    result = VelocityArbiter(config).arbitrate(
        _command(linear_x=-1.0, linear_y=-1.0, angular_z=-1.0),
        _safety(eval_action=EvaluationAction.CLAMP),
        now_ns=NOW_NS,
    )

    assert result.command.model_dump_json().count("-0.0") == 0


@pytest.mark.parametrize(
    ("action", "reason"),
    [
        (EvaluationAction.PROTECTIVE_STOP, ArbitrationReason.PROTECTIVE_STOP),
        (EvaluationAction.EMERGENCY_STOP, ArbitrationReason.EMERGENCY_STOP),
    ],
)
def test_explicit_stop_overrides_even_an_invalid_command(
    arbiter: VelocityArbiter,
    action: EvaluationAction,
    reason: ArbitrationReason,
) -> None:
    result = arbiter.arbitrate(
        {"linear_x": float("nan")},
        _safety(eval_action=action),
        now_ns=NOW_NS,
    )

    _assert_forced_zero(result, reason)


@pytest.mark.parametrize(
    ("watchdog_state", "reason"),
    [
        (WatchdogState.TIMED_OUT, ArbitrationReason.WATCHDOG_TIMED_OUT),
        (
            WatchdogState.NO_VALID_OBSERVATION,
            ArbitrationReason.WATCHDOG_NO_VALID_OBSERVATION,
        ),
        (
            WatchdogState.CLOCK_REGRESSION,
            ArbitrationReason.WATCHDOG_CLOCK_REGRESSION,
        ),
    ],
)
def test_every_unhealthy_watchdog_state_forces_zero(
    arbiter: VelocityArbiter,
    watchdog_state: WatchdogState,
    reason: ArbitrationReason,
) -> None:
    result = arbiter.arbitrate(
        _command(),
        _safety(watchdog_state=watchdog_state),
        now_ns=NOW_NS,
    )

    _assert_forced_zero(result, reason)


def test_is_safe_false_forces_zero(arbiter: VelocityArbiter) -> None:
    result = arbiter.arbitrate(
        _command(),
        _safety(is_safe=False),
        now_ns=NOW_NS,
    )

    _assert_forced_zero(result, ArbitrationReason.SAFETY_STATE_UNSAFE)


@pytest.mark.parametrize(
    ("age_ns", "expected_mode"),
    [
        (99_999_999, ArbitrationMode.PASS_THROUGH),
        (100_000_000, ArbitrationMode.FORCED_ZERO),
        (100_000_001, ArbitrationMode.FORCED_ZERO),
    ],
)
def test_command_staleness_boundary_is_unambiguous(
    arbiter: VelocityArbiter,
    age_ns: int,
    expected_mode: ArbitrationMode,
) -> None:
    result = arbiter.arbitrate(
        _command(timestamp_ns=NOW_NS - age_ns),
        _safety(),
        now_ns=NOW_NS,
    )

    assert result.mode is expected_mode
    if expected_mode is ArbitrationMode.FORCED_ZERO:
        assert result.reason is ArbitrationReason.COMMAND_STALE


@pytest.mark.parametrize(
    ("age_ns", "expected_mode"),
    [
        (249_999_999, ArbitrationMode.PASS_THROUGH),
        (250_000_000, ArbitrationMode.FORCED_ZERO),
        (250_000_001, ArbitrationMode.FORCED_ZERO),
    ],
)
def test_safety_staleness_boundary_is_unambiguous(
    arbiter: VelocityArbiter,
    age_ns: int,
    expected_mode: ArbitrationMode,
) -> None:
    result = arbiter.arbitrate(
        _command(),
        _safety(timestamp_ns=NOW_NS - age_ns),
        now_ns=NOW_NS,
    )

    assert result.mode is expected_mode
    if expected_mode is ArbitrationMode.FORCED_ZERO:
        assert result.reason is ArbitrationReason.SAFETY_STATE_STALE


def test_future_command_and_safety_timestamps_force_zero(
    arbiter: VelocityArbiter,
) -> None:
    command_result = arbiter.arbitrate(
        _command(timestamp_ns=NOW_NS + 1),
        _safety(),
        now_ns=NOW_NS,
    )
    safety_result = arbiter.arbitrate(
        _command(),
        _safety(timestamp_ns=NOW_NS + 1),
        now_ns=NOW_NS,
    )

    _assert_forced_zero(
        command_result,
        ArbitrationReason.COMMAND_CLOCK_REGRESSION,
    )
    _assert_forced_zero(
        safety_result,
        ArbitrationReason.SAFETY_CLOCK_REGRESSION,
    )


@pytest.mark.parametrize("invalid_now", [-1, 1.0, True, None, "10"])
def test_invalid_current_time_forces_zero(
    arbiter: VelocityArbiter,
    invalid_now: object,
) -> None:
    result = arbiter.arbitrate(
        _command(),
        _safety(),
        now_ns=invalid_now,
    )

    _assert_forced_zero(result, ArbitrationReason.CURRENT_TIME_INVALID)
    assert result.command.timestamp_ns == 0


@pytest.mark.parametrize(
    "invalid_command",
    [
        {"linear_x": float("nan")},
        {
            "linear_x": float("inf"),
            "linear_y": 0.0,
            "angular_z": 0.0,
            "timestamp_ns": NOW_NS,
            "frame_id": "base_link",
        },
        {
            "linear_x": 0.0,
            "linear_y": 0.0,
            "angular_z": 0.0,
            "timestamp_ns": NOW_NS,
            "frame_id": "base_link",
            "unexpected": True,
        },
    ],
)
def test_invalid_velocity_payloads_fail_closed(
    arbiter: VelocityArbiter,
    invalid_command: dict[str, object],
) -> None:
    result = arbiter.arbitrate(
        invalid_command,
        _safety(),
        now_ns=NOW_NS,
    )

    _assert_forced_zero(result, ArbitrationReason.COMMAND_INVALID)


def test_missing_command_and_safety_state_have_distinct_reasons(
    arbiter: VelocityArbiter,
) -> None:
    missing_safety = arbiter.arbitrate(_command(), None, now_ns=NOW_NS)
    missing_command = arbiter.arbitrate(None, _safety(), now_ns=NOW_NS)

    _assert_forced_zero(
        missing_safety,
        ArbitrationReason.SAFETY_STATE_MISSING,
    )
    _assert_forced_zero(missing_command, ArbitrationReason.COMMAND_MISSING)


def test_invalid_safety_state_and_frame_mismatch_fail_closed(
    arbiter: VelocityArbiter,
) -> None:
    invalid_safety = arbiter.arbitrate(
        _command(),
        {
            "is_safe": True,
            "watchdog_state": "unknown",
            "eval_action": "proceed",
            "timestamp_ns": NOW_NS,
        },
        now_ns=NOW_NS,
    )
    wrong_frame = arbiter.arbitrate(
        _command(frame_id="map"),
        _safety(),
        now_ns=NOW_NS,
    )

    _assert_forced_zero(
        invalid_safety,
        ArbitrationReason.SAFETY_STATE_INVALID,
    )
    _assert_forced_zero(
        wrong_frame,
        ArbitrationReason.COMMAND_FRAME_MISMATCH,
    )


@pytest.mark.parametrize(
    "invalid_safety",
    [
        {
            "is_safe": True,
            "watchdog_state": 1,
            "eval_action": "proceed",
            "timestamp_ns": NOW_NS,
        },
        {
            "is_safe": True,
            "watchdog_state": "healthy",
            "eval_action": 1,
            "timestamp_ns": NOW_NS,
        },
    ],
)
def test_non_string_enum_payloads_fail_closed(
    arbiter: VelocityArbiter,
    invalid_safety: dict[str, object],
) -> None:
    result = arbiter.arbitrate(
        _command(),
        invalid_safety,
        now_ns=NOW_NS,
    )

    _assert_forced_zero(result, ArbitrationReason.SAFETY_STATE_INVALID)


def test_exact_string_enums_are_accepted_from_untrusted_mapping(
    arbiter: VelocityArbiter,
) -> None:
    result = arbiter.arbitrate(
        _command(),
        {
            "is_safe": True,
            "watchdog_state": "healthy",
            "eval_action": "proceed",
            "timestamp_ns": NOW_NS,
        },
        now_ns=NOW_NS,
    )

    assert result.mode is ArbitrationMode.PASS_THROUGH


def test_contracts_reject_non_finite_values_and_hidden_defaults() -> None:
    with pytest.raises(ValidationError):
        DesiredVelocity(
            linear_x=float("inf"),
            linear_y=0.0,
            angular_z=0.0,
            timestamp_ns=0,
            frame_id="base_link",
        )
    with pytest.raises(ValidationError):
        VelocityArbiterConfig.model_validate({})


def test_forced_zero_result_cannot_contain_motion() -> None:
    with pytest.raises(ValidationError):
        ArbitrationResult(
            command=_command(),
            mode=ArbitrationMode.FORCED_ZERO,
            reason=ArbitrationReason.COMMAND_STALE,
            policy_version="TB-VEL-ARB-0.1.0",
        )
    with pytest.raises(ValidationError):
        ArbitrationResult(
            command=_command(linear_x=0.0, linear_y=0.0, angular_z=0.0),
            mode=ArbitrationMode.FORCED_ZERO,
            reason=ArbitrationReason.PROCEED,
            policy_version="TB-VEL-ARB-0.1.0",
        )


@pytest.mark.parametrize(
    ("mode", "reason"),
    [
        (ArbitrationMode.PASS_THROUGH, ArbitrationReason.CLAMP_POLICY),
        (ArbitrationMode.CLAMPED, ArbitrationReason.PROCEED),
    ],
)
def test_nonzero_modes_require_their_exact_reason(
    mode: ArbitrationMode,
    reason: ArbitrationReason,
) -> None:
    with pytest.raises(ValidationError):
        ArbitrationResult(
            command=_command(),
            mode=mode,
            reason=reason,
            policy_version="TB-VEL-ARB-0.1.0",
        )


def test_arbiter_exposes_the_exact_immutable_config(
    arbiter: VelocityArbiter,
    config: VelocityArbiterConfig,
) -> None:
    assert arbiter.config is config


def test_100_runs_are_bit_deterministic(arbiter: VelocityArbiter) -> None:
    desired = _command(linear_x=1.2, linear_y=-0.7, angular_z=2.5)
    safety = _safety(eval_action=EvaluationAction.CLAMP)

    results = [
        arbiter.arbitrate(desired, safety, now_ns=NOW_NS) for _ in range(100)
    ]
    canonical_json = [
        json.dumps(
            result.model_dump(mode="json"),
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        )
        for result in results
    ]
    hashes = [
        hashlib.sha256(payload.encode("utf-8")).hexdigest()
        for payload in canonical_json
    ]

    assert all(result == results[0] for result in results)
    assert len(set(canonical_json)) == 1
    assert len(set(hashes)) == 1


def test_fail_safe_invariant_across_state_grid(
    arbiter: VelocityArbiter,
) -> None:
    for watchdog_state in WatchdogState:
        for action in EvaluationAction:
            for is_safe in (False, True):
                result = arbiter.arbitrate(
                    _command(),
                    _safety(
                        watchdog_state=watchdog_state,
                        eval_action=action,
                        is_safe=is_safe,
                    ),
                    now_ns=NOW_NS,
                )
                may_move = (
                    watchdog_state is WatchdogState.HEALTHY
                    and is_safe
                    and action in {EvaluationAction.PROCEED, EvaluationAction.CLAMP}
                )
                if not may_move:
                    assert result.mode is ArbitrationMode.FORCED_ZERO
                    assert result.command.linear_x == 0.0
                    assert result.command.linear_y == 0.0
                    assert result.command.angular_z == 0.0
