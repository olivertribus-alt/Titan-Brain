"""Acceptance tests for TB-EVAL-004A Dynamic Safety Command Arbiter."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from core.arbitrator import (
    ArbitrationMode,
    ArbitrationReason,
    ArbitrationResult,
    DesiredVelocity,
    DynamicSafetyCommandArbiter,
    SafetyIntent,
    SafetyIntentState,
    VelocityArbiterConfig,
)

NOW_NS = 20_000_000_000


@pytest.fixture
def config() -> VelocityArbiterConfig:
    return VelocityArbiterConfig(
        policy_version="TB-EVAL-004A-0.1.0",
        output_frame_id="base_link",
        command_stale_threshold_ns=100,
        safety_stale_threshold_ns=200,
        max_abs_linear_x=0.8,
        max_abs_linear_y=0.2,
        max_abs_angular_z=1.5,
        warning_max_abs_linear_x=0.3,
        warning_max_abs_linear_y=0.1,
        warning_max_abs_angular_z=0.4,
    )


@pytest.fixture
def arbiter(config: VelocityArbiterConfig) -> DynamicSafetyCommandArbiter:
    return DynamicSafetyCommandArbiter(config)


def _intent(
    state: SafetyIntentState = SafetyIntentState.NORMAL,
    *,
    sequence_id: int = 1,
    timestamp_ns: int = NOW_NS,
    correlation_id: str = "eval_trace_001",
) -> SafetyIntent:
    return SafetyIntent(
        state=state,
        timestamp_ns=timestamp_ns,
        correlation_id=correlation_id,
        sequence_id=sequence_id,
    )


def _command(
    *,
    sequence_id: int = 2,
    timestamp_ns: int = NOW_NS,
    frame_id: str = "base_link",
    linear_x: float = 0.4,
    linear_y: float = -0.1,
    angular_z: float = 0.5,
) -> DesiredVelocity:
    return DesiredVelocity(
        linear_x=linear_x,
        linear_y=linear_y,
        angular_z=angular_z,
        timestamp_ns=timestamp_ns,
        frame_id=frame_id,
        sequence_id=sequence_id,
    )


def _assert_zero(
    result: ArbitrationResult,
    reason: ArbitrationReason,
    *,
    correlation_id: str | None,
) -> None:
    assert result.mode is ArbitrationMode.FORCED_ZERO
    assert result.reason is reason
    assert result.command.linear_x == 0.0
    assert result.command.linear_y == 0.0
    assert result.command.angular_z == 0.0
    assert result.correlation_id == correlation_id


def test_initial_fresh_normal_requires_and_accepts_post_intent_command(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    assert arbiter.recovery_latched is True

    result = arbiter.evaluate(_command(), _intent(), now_ns=NOW_NS)

    assert result.mode is ArbitrationMode.PASS_THROUGH
    assert result.command == _command()
    assert result.correlation_id == "eval_trace_001"
    assert arbiter.recovery_latched is False
    assert arbiter.config.policy_version == "TB-EVAL-004A-0.1.0"


@pytest.mark.parametrize(
    ("state", "reason"),
    [
        (SafetyIntentState.E_STOP, ArbitrationReason.E_STOP_ACTIVE),
        (SafetyIntentState.RECOVERY_HOLDING, ArbitrationReason.RECOVERY_HOLDING),
    ],
)
def test_every_non_normal_intent_forces_zero_and_preserves_correlation(
    arbiter: DynamicSafetyCommandArbiter,
    state: SafetyIntentState,
    reason: ArbitrationReason,
) -> None:
    result = arbiter.evaluate(
        {"invalid": "command is deliberately ignored"},
        _intent(state),
        now_ns=NOW_NS,
    )

    _assert_zero(result, reason, correlation_id="eval_trace_001")
    assert arbiter.recovery_latched is True


def test_initial_warning_cannot_accelerate_from_zero(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    result = arbiter.evaluate(
        _command(),
        _intent(SafetyIntentState.WARNING),
        now_ns=NOW_NS,
    )

    assert result.mode is ArbitrationMode.CLAMPED
    assert result.reason is ArbitrationReason.WARNING_SHAPED
    assert result.command.linear_x == 0.0
    assert result.command.linear_y == 0.0
    assert result.command.angular_z == 0.0
    assert result.correlation_id == "eval_trace_001"
    assert arbiter.recovery_latched is True


def test_warning_applies_independent_symmetric_limits_for_both_signs(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    arbiter.evaluate(
        _command(
            sequence_id=2,
            linear_x=0.8,
            linear_y=-0.2,
            angular_z=0.9,
        ),
        _intent(sequence_id=1),
        now_ns=NOW_NS,
    )
    positive = arbiter.evaluate(
        _command(
            sequence_id=4,
            linear_x=2.0,
            linear_y=-2.0,
            angular_z=2.0,
        ),
        _intent(SafetyIntentState.WARNING, sequence_id=3),
        now_ns=NOW_NS,
    )
    negative = arbiter.evaluate(
        _command(
            sequence_id=6,
            linear_x=-2.0,
            linear_y=2.0,
            angular_z=-2.0,
        ),
        _intent(SafetyIntentState.WARNING, sequence_id=5),
        now_ns=NOW_NS,
    )

    assert positive.reason is ArbitrationReason.WARNING_SHAPED
    assert positive.mode is ArbitrationMode.CLAMPED
    assert positive.command.linear_x == 0.3
    assert positive.command.linear_y == -0.1
    assert positive.command.angular_z == 0.4
    assert negative.command.linear_x == -0.3
    assert negative.command.linear_y == 0.1
    assert negative.command.angular_z == -0.4


def test_warning_deceleration_passes_but_acceleration_is_guarded(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    arbiter.evaluate(
        _command(
            sequence_id=2,
            linear_x=0.2,
            linear_y=-0.05,
            angular_z=0.3,
        ),
        _intent(sequence_id=1),
        now_ns=NOW_NS,
    )
    accelerated = arbiter.evaluate(
        _command(
            sequence_id=4,
            linear_x=0.25,
            linear_y=-0.08,
            angular_z=0.35,
        ),
        _intent(SafetyIntentState.WARNING, sequence_id=3),
        now_ns=NOW_NS,
    )
    decelerated = arbiter.evaluate(
        _command(
            sequence_id=6,
            linear_x=0.1,
            linear_y=-0.02,
            angular_z=0.2,
        ),
        _intent(SafetyIntentState.WARNING, sequence_id=5),
        now_ns=NOW_NS,
    )

    assert accelerated.reason is ArbitrationReason.WARNING_SHAPED
    assert accelerated.command.linear_x == 0.2
    assert accelerated.command.linear_y == -0.05
    assert accelerated.command.angular_z == 0.3
    assert decelerated.reason is ArbitrationReason.WARNING_UNMODIFIED
    assert decelerated.mode is ArbitrationMode.PASS_THROUGH
    assert decelerated.command.linear_x == 0.1
    assert decelerated.command.linear_y == -0.02
    assert decelerated.command.angular_z == 0.2
    assert arbiter.last_output == decelerated.command


def test_warning_direction_change_preserves_requested_sign_without_acceleration(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    arbiter.evaluate(
        _command(
            sequence_id=2,
            linear_x=0.2,
            linear_y=-0.05,
            angular_z=0.3,
        ),
        _intent(sequence_id=1),
        now_ns=NOW_NS,
    )

    reversed_result = arbiter.evaluate(
        _command(
            sequence_id=4,
            linear_x=-0.25,
            linear_y=0.08,
            angular_z=-0.35,
        ),
        _intent(SafetyIntentState.WARNING, sequence_id=3),
        now_ns=NOW_NS,
    )

    assert reversed_result.command.linear_x == -0.2
    assert reversed_result.command.linear_y == 0.05
    assert reversed_result.command.angular_z == -0.3


def test_warning_limits_fall_back_to_legacy_clamp_limits(
    config: VelocityArbiterConfig,
) -> None:
    fallback_config = config.model_copy(
        update={
            "warning_max_abs_linear_x": None,
            "warning_max_abs_linear_y": None,
            "warning_max_abs_angular_z": None,
        }
    )
    arbiter = DynamicSafetyCommandArbiter(fallback_config)
    arbiter.evaluate(
        _command(
            sequence_id=2,
            linear_x=2.0,
            linear_y=2.0,
            angular_z=2.0,
        ),
        _intent(sequence_id=1),
        now_ns=NOW_NS,
    )

    result = arbiter.evaluate(
        _command(
            sequence_id=4,
            linear_x=-2.0,
            linear_y=-2.0,
            angular_z=-2.0,
        ),
        _intent(SafetyIntentState.WARNING, sequence_id=3),
        now_ns=NOW_NS,
    )

    assert result.command.linear_x == -0.8
    assert result.command.linear_y == -0.2
    assert result.command.angular_z == -1.5


def test_recovery_rejects_command_received_before_new_normal_intent(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    arbiter.evaluate(
        _command(sequence_id=2),
        _intent(SafetyIntentState.E_STOP, sequence_id=1),
        now_ns=NOW_NS,
    )
    old_command = _command(sequence_id=2, timestamp_ns=NOW_NS + 1)
    normal = _intent(
        sequence_id=3,
        timestamp_ns=NOW_NS + 1,
        correlation_id="eval_trace_002",
    )

    blocked = arbiter.evaluate(old_command, normal, now_ns=NOW_NS + 1)
    released = arbiter.evaluate(
        _command(sequence_id=4, timestamp_ns=NOW_NS + 1),
        normal,
        now_ns=NOW_NS + 1,
    )

    _assert_zero(
        blocked,
        ArbitrationReason.RECOVERY_COMMAND_REQUIRED,
        correlation_id="eval_trace_002",
    )
    assert released.mode is ArbitrationMode.PASS_THROUGH
    assert released.command.sequence_id == 4


def test_same_normal_intent_cannot_release_latch_after_stop(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    stop = _intent(SafetyIntentState.E_STOP, sequence_id=5)
    arbiter.evaluate(_command(sequence_id=6), stop, now_ns=NOW_NS)

    blocked = arbiter.evaluate(
        _command(sequence_id=7),
        _intent(sequence_id=5),
        now_ns=NOW_NS,
    )

    _assert_zero(
        blocked,
        ArbitrationReason.SAFETY_INTENT_SEQUENCE_REGRESSION,
        correlation_id="eval_trace_001",
    )


@pytest.mark.parametrize(
    ("intent", "reason"),
    [
        (None, ArbitrationReason.SAFETY_INTENT_MISSING),
        ({"state": "unknown"}, ArbitrationReason.SAFETY_INTENT_INVALID),
        (
            {
                "state": 1,
                "timestamp_ns": NOW_NS,
                "correlation_id": "eval_trace_001",
                "sequence_id": 1,
            },
            ArbitrationReason.SAFETY_INTENT_INVALID,
        ),
    ],
)
def test_missing_and_invalid_intents_fail_closed(
    arbiter: DynamicSafetyCommandArbiter,
    intent: object,
    reason: ArbitrationReason,
) -> None:
    result = arbiter.evaluate(_command(), intent, now_ns=NOW_NS)  # type: ignore[arg-type]

    _assert_zero(result, reason, correlation_id=None)


@pytest.mark.parametrize(
    ("age_ns", "reason"),
    [
        (199, None),
        (200, ArbitrationReason.SAFETY_INTENT_TIMEOUT),
        (201, ArbitrationReason.SAFETY_INTENT_TIMEOUT),
    ],
)
def test_safety_timeout_boundary(
    arbiter: DynamicSafetyCommandArbiter,
    age_ns: int,
    reason: ArbitrationReason | None,
) -> None:
    intent = _intent(timestamp_ns=NOW_NS - age_ns)
    result = arbiter.evaluate(_command(), intent, now_ns=NOW_NS)

    if reason is None:
        assert result.mode is ArbitrationMode.PASS_THROUGH
    else:
        _assert_zero(result, reason, correlation_id="eval_trace_001")


@pytest.mark.parametrize(
    ("age_ns", "reason"),
    [
        (99, None),
        (100, ArbitrationReason.COMMAND_TIMEOUT),
        (101, ArbitrationReason.COMMAND_TIMEOUT),
    ],
)
def test_command_timeout_boundary(
    arbiter: DynamicSafetyCommandArbiter,
    age_ns: int,
    reason: ArbitrationReason | None,
) -> None:
    result = arbiter.evaluate(
        _command(timestamp_ns=NOW_NS - age_ns),
        _intent(),
        now_ns=NOW_NS,
    )

    if reason is None:
        assert result.mode is ArbitrationMode.PASS_THROUGH
    else:
        _assert_zero(result, reason, correlation_id="eval_trace_001")


def test_command_timeout_requires_new_normal_and_newer_command(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    timed_out = arbiter.evaluate(
        _command(sequence_id=2, timestamp_ns=NOW_NS - 100),
        _intent(sequence_id=1),
        now_ns=NOW_NS,
    )
    same_normal = arbiter.evaluate(
        _command(sequence_id=3),
        _intent(sequence_id=1),
        now_ns=NOW_NS,
    )
    new_normal_old_command = arbiter.evaluate(
        _command(sequence_id=3),
        _intent(sequence_id=4),
        now_ns=NOW_NS,
    )
    recovered = arbiter.evaluate(
        _command(sequence_id=5),
        _intent(sequence_id=4),
        now_ns=NOW_NS,
    )

    _assert_zero(
        timed_out,
        ArbitrationReason.COMMAND_TIMEOUT,
        correlation_id="eval_trace_001",
    )
    _assert_zero(
        same_normal,
        ArbitrationReason.RECOVERY_HOLDING,
        correlation_id="eval_trace_001",
    )
    _assert_zero(
        new_normal_old_command,
        ArbitrationReason.RECOVERY_COMMAND_REQUIRED,
        correlation_id="eval_trace_001",
    )
    assert recovered.mode is ArbitrationMode.PASS_THROUGH


def test_wall_clock_regression_latches_until_new_normal_intent(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    arbiter.evaluate(_command(), _intent(), now_ns=NOW_NS)
    regressed = arbiter.evaluate(
        _command(sequence_id=3),
        _intent(sequence_id=2),
        now_ns=NOW_NS - 1,
    )
    same_intent = arbiter.evaluate(
        _command(sequence_id=3),
        _intent(sequence_id=1),
        now_ns=NOW_NS,
    )

    _assert_zero(
        regressed,
        ArbitrationReason.ARBITER_CLOCK_REGRESSION,
        correlation_id=None,
    )
    _assert_zero(
        same_intent,
        ArbitrationReason.RECOVERY_HOLDING,
        correlation_id="eval_trace_001",
    )


@pytest.mark.parametrize("invalid_now", [-1, True, 1.5, None])
def test_invalid_current_time_fails_closed_and_latches(
    arbiter: DynamicSafetyCommandArbiter,
    invalid_now: object,
) -> None:
    result = arbiter.evaluate(_command(), _intent(), now_ns=invalid_now)

    _assert_zero(
        result,
        ArbitrationReason.CURRENT_TIME_INVALID,
        correlation_id=None,
    )
    assert result.command.timestamp_ns == 0
    assert arbiter.recovery_latched is True


def test_future_intent_and_command_timestamps_fail_closed() -> None:
    future_intent = DynamicSafetyCommandArbiter(
        VelocityArbiterConfig(
            policy_version="TB-EVAL-004A-0.1.0",
            output_frame_id="base_link",
            command_stale_threshold_ns=100,
            safety_stale_threshold_ns=200,
            max_abs_linear_x=1.0,
            max_abs_linear_y=1.0,
            max_abs_angular_z=1.0,
        )
    )
    intent_result = future_intent.evaluate(
        _command(),
        _intent(timestamp_ns=NOW_NS + 1),
        now_ns=NOW_NS,
    )
    command_arbiter = DynamicSafetyCommandArbiter(future_intent.config)
    command_result = command_arbiter.evaluate(
        _command(timestamp_ns=NOW_NS + 1),
        _intent(),
        now_ns=NOW_NS,
    )

    _assert_zero(
        intent_result,
        ArbitrationReason.SAFETY_CLOCK_REGRESSION,
        correlation_id="eval_trace_001",
    )
    _assert_zero(
        command_result,
        ArbitrationReason.COMMAND_CLOCK_REGRESSION,
        correlation_id="eval_trace_001",
    )


def test_intent_and_command_sequence_regressions_fail_closed(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    arbiter.evaluate(_command(sequence_id=2), _intent(sequence_id=1), now_ns=NOW_NS)
    arbiter.evaluate(
        _command(sequence_id=4),
        _intent(sequence_id=3),
        now_ns=NOW_NS,
    )
    intent_regression = arbiter.evaluate(
        _command(sequence_id=5),
        _intent(sequence_id=2),
        now_ns=NOW_NS,
    )

    command_arbiter = DynamicSafetyCommandArbiter(arbiter.config)
    command_arbiter.evaluate(
        _command(sequence_id=5),
        _intent(sequence_id=4),
        now_ns=NOW_NS,
    )
    command_regression = command_arbiter.evaluate(
        _command(sequence_id=3),
        _intent(sequence_id=4),
        now_ns=NOW_NS,
    )

    _assert_zero(
        intent_regression,
        ArbitrationReason.SAFETY_INTENT_SEQUENCE_REGRESSION,
        correlation_id="eval_trace_001",
    )
    _assert_zero(
        command_regression,
        ArbitrationReason.COMMAND_SEQUENCE_REGRESSION,
        correlation_id="eval_trace_001",
    )


def test_same_sequence_cannot_replace_intent_or_command_payload(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    arbiter.evaluate(_command(sequence_id=2), _intent(sequence_id=1), now_ns=NOW_NS)
    changed_intent = arbiter.evaluate(
        _command(sequence_id=3),
        _intent(
            SafetyIntentState.WARNING,
            sequence_id=1,
        ),
        now_ns=NOW_NS,
    )

    command_arbiter = DynamicSafetyCommandArbiter(arbiter.config)
    command_arbiter.evaluate(
        _command(sequence_id=2),
        _intent(sequence_id=1),
        now_ns=NOW_NS,
    )
    changed_command = command_arbiter.evaluate(
        _command(sequence_id=2, linear_x=0.3),
        _intent(sequence_id=1),
        now_ns=NOW_NS,
    )

    _assert_zero(
        changed_intent,
        ArbitrationReason.SAFETY_INTENT_SEQUENCE_REGRESSION,
        correlation_id="eval_trace_001",
    )
    _assert_zero(
        changed_command,
        ArbitrationReason.COMMAND_SEQUENCE_REGRESSION,
        correlation_id="eval_trace_001",
    )


@pytest.mark.parametrize(
    ("command", "reason"),
    [
        (None, ArbitrationReason.COMMAND_MISSING),
        ({"linear_x": float("nan")}, ArbitrationReason.COMMAND_INVALID),
        (_command(frame_id="map"), ArbitrationReason.COMMAND_FRAME_MISMATCH),
    ],
)
def test_command_contract_failures_are_distinct_and_latched(
    arbiter: DynamicSafetyCommandArbiter,
    command: object,
    reason: ArbitrationReason,
) -> None:
    result = arbiter.evaluate(command, _intent(), now_ns=NOW_NS)  # type: ignore[arg-type]

    _assert_zero(result, reason, correlation_id="eval_trace_001")
    assert arbiter.recovery_latched is True


def test_exact_string_intent_state_is_accepted() -> None:
    config = VelocityArbiterConfig(
        policy_version="TB-EVAL-004A-0.1.0",
        output_frame_id="base_link",
        command_stale_threshold_ns=100,
        safety_stale_threshold_ns=200,
        max_abs_linear_x=1.0,
        max_abs_linear_y=1.0,
        max_abs_angular_z=1.0,
    )
    result = DynamicSafetyCommandArbiter(config).evaluate(
        _command(),
        {
            "state": "normal",
            "timestamp_ns": NOW_NS,
            "correlation_id": "eval_trace_001",
            "sequence_id": 1,
        },
        now_ns=NOW_NS,
    )

    assert result.mode is ArbitrationMode.PASS_THROUGH


def test_warning_limits_cannot_exceed_nominal_limits(
    config: VelocityArbiterConfig,
) -> None:
    with pytest.raises(ValidationError, match="must not exceed nominal"):
        VelocityArbiterConfig.model_validate(
            {
                **config.model_dump(mode="python"),
                "warning_max_abs_linear_x": config.max_abs_linear_x + 0.1,
            }
        )
