"""Acceptance tests for TB-ACT-001A actuator feedback evaluation."""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from core.actuator_feedback import (
    ActuatorFeedback,
    ActuatorFeedbackConfig,
    ActuatorState,
    ActuatorStatus,
    evaluate_actuator_feedback,
    evaluate_feedback,
)


@pytest.fixture
def config() -> ActuatorFeedbackConfig:
    return ActuatorFeedbackConfig(
        epsilon_stop_linear=0.01,
        epsilon_stop_angular=0.02,
        stale_threshold_ns=100,
    )


def _feedback(**overrides: object) -> ActuatorFeedback:
    payload: dict[str, object] = {
        "measured_linear_x": 0.0,
        "measured_linear_y": 0.0,
        "measured_angular_z": 0.0,
        "correlation_id": "obs-001",
        "sequence_id": 7,
        "timestamp_ns": 900,
    }
    payload.update(overrides)
    return ActuatorFeedback.model_validate(payload)


def test_stopped_requires_all_axes_to_be_within_inclusive_threshold(
    config: ActuatorFeedbackConfig,
) -> None:
    status = evaluate_actuator_feedback(
        _feedback(
            measured_linear_x=config.epsilon_stop_linear,
            measured_linear_y=-config.epsilon_stop_linear,
            measured_angular_z=config.epsilon_stop_angular,
        ),
        now_ns=999,
        expected_correlation_id="obs-001",
        config=config,
        expected_sequence_id=7,
    )

    assert status.state is ActuatorState.STOPPED
    assert status.is_stopped is True
    assert status.is_fresh is True
    assert status.is_valid is True
    assert status.correlation_id == "obs-001"
    assert status.sequence == 7
    assert status.timestamp == 900
    assert status.evaluated_timestamp_ns == 999


def test_wire_aliases_and_feedback_properties_are_supported() -> None:
    feedback = ActuatorFeedback.model_validate(
        {
            "measured_linear_x": 0.0,
            "measured_linear_y": 0.0,
            "measured_angular_z": 0.0,
            "correlation_id": "obs-001",
            "sequence": 7,
            "timestamp": 900,
        }
    )

    assert feedback.sequence_id == feedback.sequence == 7
    assert feedback.timestamp_ns == feedback.timestamp == 900


def test_feedback_validation_rejects_non_mapping_payload() -> None:
    with pytest.raises(ValidationError):
        ActuatorFeedback.model_validate(1)


def test_status_state_validator_preserves_enum_values() -> None:
    assert ActuatorStatus.parse_state(ActuatorState.STOPPED) is ActuatorState.STOPPED


def test_status_wire_aliases_are_supported() -> None:
    status = ActuatorStatus.model_validate(
        {
            "state": "stopped",
            "is_stopped": True,
            "is_fresh": True,
            "is_valid": True,
            "correlation_id": "obs-001",
            "sequence": 7,
            "timestamp": 900,
            "evaluated_timestamp_ns": 999,
        }
    )

    assert status.sequence_id == status.sequence == 7
    assert status.feedback_timestamp_ns == status.timestamp == 900


def test_status_validation_rejects_non_mapping_payload() -> None:
    with pytest.raises(ValidationError):
        ActuatorStatus.model_validate(1)


@pytest.mark.parametrize(
    "axis",
    ["measured_linear_x", "measured_linear_y", "measured_angular_z"],
)
def test_any_axis_above_its_threshold_is_moving(
    config: ActuatorFeedbackConfig,
    axis: str,
) -> None:
    threshold = (
        config.epsilon_stop_angular
        if axis == "measured_angular_z"
        else config.epsilon_stop_linear
    )
    status = evaluate_actuator_feedback(
        _feedback(**{axis: math.nextafter(threshold, math.inf)}),
        now_ns=999,
        expected_correlation_id="obs-001",
        config=config,
    )

    assert status.state is ActuatorState.MOVING
    assert status.is_stopped is False
    assert status.is_fresh is True
    assert status.is_valid is True


def test_stale_feedback_is_not_claimed_stopped_at_budget_boundary(
    config: ActuatorFeedbackConfig,
) -> None:
    status = evaluate_actuator_feedback(
        _feedback(timestamp_ns=899),
        now_ns=999,
        expected_correlation_id="obs-001",
        config=config,
    )

    assert status.state is ActuatorState.STALE_DATA
    assert status.is_stopped is False
    assert status.is_fresh is False
    assert status.is_valid is True


def test_feedback_just_inside_budget_remains_fresh(
    config: ActuatorFeedbackConfig,
) -> None:
    status = evaluate_actuator_feedback(
        _feedback(timestamp_ns=900),
        now_ns=999,
        expected_correlation_id="obs-001",
        config=config,
    )

    assert status.state is ActuatorState.STOPPED
    assert status.is_fresh is True


@pytest.mark.parametrize(
    "payload",
    [
        None,
        {"measured_linear_x": 0.0},
        {
            "measured_linear_x": float("nan"),
            "measured_linear_y": 0.0,
            "measured_angular_z": 0.0,
            "correlation_id": "obs-001",
            "sequence_id": 7,
            "timestamp_ns": 900,
        },
        {
            "measured_linear_x": 0.0,
            "measured_linear_y": 0.0,
            "measured_angular_z": 0.0,
            "correlation_id": "obs-001",
            "sequence_id": 7,
            "timestamp_ns": -1,
        },
    ],
)
def test_missing_or_malformed_feedback_is_invalid(
    config: ActuatorFeedbackConfig,
    payload: object,
) -> None:
    status = evaluate_actuator_feedback(
        payload,  # type: ignore[arg-type]
        now_ns=999,
        expected_correlation_id="obs-001",
        config=config,
    )

    assert status.state is ActuatorState.INVALID_DATA
    assert status.is_stopped is False
    assert status.is_fresh is False
    assert status.is_valid is False


def test_invalid_mapping_metadata_is_preserved_safely(
    config: ActuatorFeedbackConfig,
) -> None:
    status = evaluate_actuator_feedback(
        {
            "correlation_id": "obs-001",
            "sequence_id": -1,
            "timestamp_ns": -1,
        },
        now_ns=999,
        expected_correlation_id="obs-001",
        config=config,
    )

    assert status.state is ActuatorState.INVALID_DATA
    assert status.sequence_id == 0
    assert status.feedback_timestamp_ns == 0


def test_model_constructed_non_finite_sample_is_fail_closed(
    config: ActuatorFeedbackConfig,
) -> None:
    feedback = ActuatorFeedback.model_construct(
        measured_linear_x=float("nan"),
        measured_linear_y=0.0,
        measured_angular_z=0.0,
        correlation_id="obs-001",
        sequence_id=7,
        timestamp_ns=900,
    )

    status = evaluate_actuator_feedback(
        feedback,
        now_ns=999,
        expected_correlation_id="obs-001",
        config=config,
    )

    assert status.state is ActuatorState.INVALID_DATA


def test_model_constructed_non_numeric_sample_is_fail_closed(
    config: ActuatorFeedbackConfig,
) -> None:
    feedback = ActuatorFeedback.model_construct(
        measured_linear_x="not-a-number",
        measured_linear_y=0.0,
        measured_angular_z=0.0,
        correlation_id="obs-001",
        sequence_id=7,
        timestamp_ns=900,
    )

    status = evaluate_actuator_feedback(
        feedback,
        now_ns=999,
        expected_correlation_id="obs-001",
        config=config,
    )

    assert status.state is ActuatorState.INVALID_DATA


@pytest.mark.parametrize(
    "expected_correlation_id, expected_sequence_id",
    [("other", None), ("obs-001", 8)],
)
def test_correlation_or_sequence_mismatch_is_invalid(
    config: ActuatorFeedbackConfig,
    expected_correlation_id: str,
    expected_sequence_id: int | None,
) -> None:
    status = evaluate_actuator_feedback(
        _feedback(),
        now_ns=999,
        expected_correlation_id=expected_correlation_id,
        config=config,
        expected_sequence_id=expected_sequence_id,
    )

    assert status.state is ActuatorState.INVALID_DATA
    assert status.correlation_id == "obs-001"
    assert status.sequence_id == 7


def test_future_timestamp_is_invalid_clock_data(
    config: ActuatorFeedbackConfig,
) -> None:
    status = evaluate_actuator_feedback(
        _feedback(timestamp_ns=1_000),
        now_ns=999,
        expected_correlation_id="obs-001",
        config=config,
    )

    assert status.state is ActuatorState.INVALID_DATA
    assert status.feedback_timestamp_ns == 1_000


@pytest.mark.parametrize(
    "now_ns, expected_correlation_id, expected_sequence_id",
    [(-1, "obs-001", None), (999, "", None), (999, "obs-001", 0)],
)
def test_invalid_evaluation_context_is_fail_closed(
    config: ActuatorFeedbackConfig,
    now_ns: int,
    expected_correlation_id: str,
    expected_sequence_id: int | None,
) -> None:
    status = evaluate_actuator_feedback(
        _feedback(),
        now_ns=now_ns,
        expected_correlation_id=expected_correlation_id,
        config=config,
        expected_sequence_id=expected_sequence_id,
    )

    assert status.state is ActuatorState.INVALID_DATA
    assert status.is_stopped is False


def test_scalar_convenience_api_matches_configured_api() -> None:
    status = evaluate_feedback(
        _feedback(measured_linear_x=0.5),
        now_ns=999,
        expected_correlation_id="obs-001",
        epsilon_stop_linear=0.01,
        epsilon_stop_angular=0.02,
        stale_threshold_ns=100,
        expected_sequence_id=7,
    )

    assert status.state is ActuatorState.MOVING


def test_scalar_convenience_api_rejects_invalid_policy() -> None:
    status = evaluate_feedback(
        _feedback(),
        now_ns=999,
        expected_correlation_id="obs-001",
        epsilon_stop_linear=-0.01,
        epsilon_stop_angular=0.02,
        stale_threshold_ns=100,
    )

    assert status.state is ActuatorState.INVALID_DATA


def test_config_and_status_are_immutable(config: ActuatorFeedbackConfig) -> None:
    with pytest.raises(ValidationError):
        config.stale_threshold_ns = 10

    status = evaluate_actuator_feedback(
        _feedback(),
        now_ns=999,
        expected_correlation_id="obs-001",
        config=config,
    )
    with pytest.raises(ValidationError):
        status.is_stopped = False


def test_status_rejects_inconsistent_flags() -> None:
    with pytest.raises(ValidationError, match="flags"):
        ActuatorStatus(
            state=ActuatorState.STOPPED,
            is_stopped=False,
            is_fresh=True,
            is_valid=True,
            correlation_id="obs-001",
            sequence_id=1,
            feedback_timestamp_ns=1,
            evaluated_timestamp_ns=2,
        )
