"""Core acceptance tests for TB-EVAL-005C envelope enforcement."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from core.arbitrator import (
    ArbitrationMode,
    ArbitrationReason,
    ArbitrationResult,
    DesiredVelocity,
    DynamicSafetyCommandArbiter,
    PermittedMotionEnvelope,
    SafetyIntent,
    SafetyIntentState,
    VelocityArbiterConfig,
)

NOW_NS = 1_000_000


@pytest.fixture
def arbiter() -> DynamicSafetyCommandArbiter:
    return DynamicSafetyCommandArbiter(
        VelocityArbiterConfig(
            policy_version="TB-EVAL-005C-0.1.0",
            output_frame_id="base_link",
            command_stale_threshold_ns=100,
            safety_stale_threshold_ns=200,
            max_abs_linear_x=1.0,
            max_abs_linear_y=1.0,
            max_abs_angular_z=1.0,
            warning_max_abs_linear_x=0.3,
            warning_max_abs_linear_y=0.2,
            warning_max_abs_angular_z=0.4,
        )
    )


def _command(
    *,
    linear_x: float = 0.2,
    linear_y: float = 0.1,
    angular_z: float = 0.0,
    sequence_id: int = 2,
) -> DesiredVelocity:
    return DesiredVelocity(
        linear_x=linear_x,
        linear_y=linear_y,
        angular_z=angular_z,
        timestamp_ns=NOW_NS,
        frame_id="base_link",
        sequence_id=sequence_id,
    )


def _intent(
    state: SafetyIntentState = SafetyIntentState.NORMAL,
    *,
    sequence_id: int = 1,
    correlation_id: str = "decision-001",
) -> SafetyIntent:
    return SafetyIntent(
        state=state,
        timestamp_ns=NOW_NS,
        correlation_id=correlation_id,
        sequence_id=sequence_id,
    )


def _envelope(**updates: object) -> PermittedMotionEnvelope:
    values: dict[str, object] = {
        "policy_version": "TB-EVAL-005C-ENVELOPE-0.1.0",
        "timestamp_ns": NOW_NS,
        "frame_id": "base_link",
        "correlation_id": "envelope-001",
        "sequence_id": 1,
        "min_linear_x_mps": -0.2,
        "max_linear_x_mps": 0.4,
        "min_linear_y_mps": -0.3,
        "max_linear_y_mps": 0.1,
        "min_angular_z_radps": 0.0,
        "max_angular_z_radps": 0.0,
    }
    values.update(updates)
    return PermittedMotionEnvelope.model_validate(values)


def _assert_zero(
    result_reason: ArbitrationReason,
    result: ArbitrationResult,
) -> None:
    assert result.reason is result_reason
    assert result.mode is ArbitrationMode.FORCED_ZERO
    assert result.command.linear_x == 0.0
    assert result.command.linear_y == 0.0
    assert result.command.angular_z == 0.0


def test_command_inside_envelope_preserves_base_result(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    result = arbiter.evaluate_with_envelope(
        _command(),
        _intent(),
        _envelope(),
        now_ns=NOW_NS,
    )

    assert result.mode is ArbitrationMode.PASS_THROUGH
    assert result.reason is ArbitrationReason.PROCEED
    assert result.command == _command()
    assert result.correlation_id == "decision-001"
    assert arbiter.last_output == result.command


@pytest.mark.parametrize(
    ("command", "expected"),
    [
        (
            _command(linear_x=2.0, linear_y=-2.0, angular_z=1.0),
            (0.4, -0.3, 0.0),
        ),
        (
            _command(linear_x=-2.0, linear_y=2.0, angular_z=-1.0),
            (-0.2, 0.1, 0.0),
        ),
    ],
)
def test_envelope_clamps_both_signs_and_forces_angular_zero(
    arbiter: DynamicSafetyCommandArbiter,
    command: DesiredVelocity,
    expected: tuple[float, float, float],
) -> None:
    result = arbiter.evaluate_with_envelope(
        command,
        _intent(),
        _envelope().model_dump(mode="python"),
        now_ns=NOW_NS,
    )

    assert result.mode is ArbitrationMode.CLAMPED
    assert result.reason is ArbitrationReason.MOTION_ENVELOPE_CLAMPED
    assert (
        result.command.linear_x,
        result.command.linear_y,
        result.command.angular_z,
    ) == expected
    assert result.command.sequence_id == command.sequence_id
    assert result.correlation_id == "decision-001"
    assert arbiter.last_output == result.command


@pytest.mark.parametrize(
    ("envelope", "reason"),
    [
        (None, ArbitrationReason.MOTION_ENVELOPE_MISSING),
        (
            {"max_angular_z_radps": 1.0},
            ArbitrationReason.MOTION_ENVELOPE_INVALID,
        ),
    ],
)
def test_missing_or_invalid_envelope_forces_zero(
    arbiter: DynamicSafetyCommandArbiter,
    envelope: object,
    reason: ArbitrationReason,
) -> None:
    result = arbiter.evaluate_with_envelope(
        _command(),
        _intent(),
        envelope,  # type: ignore[arg-type]
        now_ns=NOW_NS,
    )

    _assert_zero(reason, result)
    assert result.correlation_id == "decision-001"
    assert arbiter.last_output == result.command


def test_envelope_frame_mismatch_forces_zero(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    result = arbiter.evaluate_with_envelope(
        _command(),
        _intent(),
        _envelope(frame_id="map"),
        now_ns=NOW_NS,
    )

    _assert_zero(ArbitrationReason.MOTION_ENVELOPE_FRAME_MISMATCH, result)


def test_existing_stop_reason_has_priority_over_missing_envelope(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    result = arbiter.evaluate_with_envelope(
        _command(),
        _intent(SafetyIntentState.E_STOP),
        None,
        now_ns=NOW_NS,
    )

    _assert_zero(ArbitrationReason.E_STOP_ACTIVE, result)


def test_envelope_is_intersected_with_warning_non_acceleration_guard(
    arbiter: DynamicSafetyCommandArbiter,
) -> None:
    arbiter.evaluate_with_envelope(
        _command(linear_x=0.2, linear_y=0.1),
        _intent(),
        _envelope(max_linear_x_mps=1.0, max_linear_y_mps=1.0),
        now_ns=NOW_NS,
    )
    result = arbiter.evaluate_with_envelope(
        _command(
            linear_x=0.25,
            linear_y=0.15,
            sequence_id=4,
        ),
        _intent(
            SafetyIntentState.WARNING,
            sequence_id=3,
            correlation_id="decision-warning",
        ),
        _envelope(max_linear_x_mps=0.1, max_linear_y_mps=0.05),
        now_ns=NOW_NS,
    )

    assert result.reason is ArbitrationReason.MOTION_ENVELOPE_CLAMPED
    assert result.command.linear_x == 0.1
    assert result.command.linear_y == 0.05
    assert result.command.angular_z == 0.0
    assert result.correlation_id == "decision-warning"


@pytest.mark.parametrize(
    "update",
    [
        {"min_angular_z_radps": -0.01},
        {"max_angular_z_radps": 0.01},
    ],
)
def test_core_contract_rejects_nonzero_angular_authority(
    update: dict[str, float],
) -> None:
    with pytest.raises(ValidationError, match="angular limits must remain zero"):
        _envelope(**update)


def test_envelope_contract_is_strict_and_frozen() -> None:
    envelope = _envelope()

    with pytest.raises(ValidationError):
        PermittedMotionEnvelope.model_validate(
            {
                **envelope.model_dump(mode="python"),
                "sequence_id": "1",
            }
        )
    with pytest.raises(ValidationError):
        envelope.max_linear_x_mps = 1.0
