"""TB-EVAL-008A tests for constant-size dynamic envelope evaluation."""

from __future__ import annotations

import hashlib
import math
import struct

import pytest
from pydantic import ValidationError

from core.dynamic_envelope_evaluator import (
    DynamicEnvelopeEvaluator,
    EnvelopeConfig,
    EnvelopeSource,
    EnvelopeState,
    LimitingZone,
    SensorFrame,
)


def _frame(
    *,
    forward: float = 5.0,
    lateral: float = 5.0,
    confidence: float = 1.0,
    age_s: float = 0.0,
) -> SensorFrame:
    return SensorFrame(
        distance_forward_m=forward,
        distance_lateral_m=lateral,
        confidence=confidence,
        source=EnvelopeSource.LIDAR,
        age_s=age_s,
    )


def test_default_contract_and_nominal_authority() -> None:
    evaluator = DynamicEnvelopeEvaluator()
    result = evaluator.evaluate(_frame())

    assert evaluator.config.policy_version == "TB-EVAL-008A-0.1.0"
    assert result.state is EnvelopeState.NOMINAL
    assert result.limiting_zone is LimitingZone.NONE
    assert result.source is EnvelopeSource.LIDAR
    assert result.max_linear_velocity_mps == 1.0
    assert result.max_angular_velocity_radps == 1.0
    assert result.stop_only is False


@pytest.mark.parametrize(
    ("frame", "reason"),
    [
        (None, "SENSOR_FRAME_MISSING"),
        (_frame(age_s=0.200000001), "SENSOR_FRAME_STALE"),
        (_frame(confidence=0.49), "SENSOR_CONFIDENCE_LOW"),
    ],
)
def test_invalid_or_stale_evidence_fails_closed(
    frame: SensorFrame | None,
    reason: str,
) -> None:
    result = DynamicEnvelopeEvaluator().evaluate(frame)

    assert result.state is EnvelopeState.FAIL_CLOSED
    assert result.stop_only is True
    assert result.source is EnvelopeSource.NONE
    assert result.reason == reason


def test_timeout_boundary_is_inclusive() -> None:
    evaluator = DynamicEnvelopeEvaluator()
    assert evaluator.evaluate(_frame(age_s=0.20)).state is EnvelopeState.NOMINAL


def test_forward_clearance_monotonically_controls_linear_authority() -> None:
    evaluator = DynamicEnvelopeEvaluator()
    values = [
        evaluator.evaluate(_frame(forward=distance)).max_linear_velocity_mps
        for distance in (0.30, 0.50, 1.0, 2.0, 5.0)
    ]

    assert values == sorted(values)
    assert values[0] == 0.0
    assert values[-1] == evaluator.config.nominal_linear_velocity_mps


def test_lateral_clearance_controls_angular_authority() -> None:
    evaluator = DynamicEnvelopeEvaluator()
    limited = evaluator.evaluate(_frame(forward=5.0, lateral=0.35))
    nominal = evaluator.evaluate(_frame(forward=5.0, lateral=5.0))

    assert limited.max_linear_velocity_mps == 1.0
    assert 0.0 < limited.max_angular_velocity_radps < 1.0
    assert limited.limiting_zone is LimitingZone.LATERAL
    assert nominal.max_angular_velocity_radps == 1.0


def test_close_forward_obstacle_blocks_all_swept_motion() -> None:
    result = DynamicEnvelopeEvaluator().evaluate(_frame(forward=0.30, lateral=5.0))

    assert result.state is EnvelopeState.PROTECTIVE_STOP
    assert result.stop_only is True
    assert result.limiting_zone is LimitingZone.FORWARD


def test_float32_margin_roundoff_clamps_to_absolute_zero() -> None:
    encoded_margin = struct.unpack("f", struct.pack("f", 0.30))[0]
    result = DynamicEnvelopeEvaluator().evaluate(
        _frame(forward=encoded_margin, lateral=encoded_margin)
    )

    assert encoded_margin > 0.30
    assert result.max_linear_velocity_mps == 0.0
    assert result.max_angular_velocity_radps == 0.0
    assert result.state is EnvelopeState.PROTECTIVE_STOP
    assert result.stop_only is True


def test_close_lateral_obstacle_blocks_rotation_but_not_forward_motion() -> None:
    result = DynamicEnvelopeEvaluator().evaluate(_frame(forward=5.0, lateral=0.30))

    assert result.state is EnvelopeState.LIMITED
    assert result.max_linear_velocity_mps == 1.0
    assert result.max_angular_velocity_radps == 0.0
    assert result.limiting_zone is LimitingZone.LATERAL


def test_reported_stopping_distance_never_exceeds_evidence() -> None:
    evaluator = DynamicEnvelopeEvaluator()
    result = evaluator.evaluate(_frame(forward=0.85, lateral=0.95))

    assert result.linear_stopping_distance_m <= 0.85
    assert result.angular_stopping_distance_m <= 0.85


def test_explicit_fail_closed_reason_and_zone() -> None:
    result = DynamicEnvelopeEvaluator().fail_closed(
        "SYSTEM_FAULT_HARDWARE_FAULT",
        limiting_zone=LimitingZone.SYSTEM_FAULT,
    )

    assert result.stop_only is True
    assert result.reason == "SYSTEM_FAULT_HARDWARE_FAULT"
    assert result.limiting_zone is LimitingZone.SYSTEM_FAULT


@pytest.mark.parametrize(
    "update",
    [
        {"assured_deceleration_mps2": float("inf")},
        {"clearance_margin_m": float("nan")},
        {"nominal_linear_velocity_mps": float("inf")},
        {"angular_swept_radius_m": float("nan")},
        {"max_sensor_age_s": float("inf")},
    ],
)
def test_nonfinite_configuration_is_rejected(update: dict[str, float]) -> None:
    with pytest.raises(ValidationError, match="finite"):
        EnvelopeConfig.model_validate(update)


@pytest.mark.parametrize(
    "payload",
    [
        {
            "distance_forward_m": float("nan"),
            "distance_lateral_m": 1.0,
            "confidence": 1.0,
            "source": EnvelopeSource.LIDAR,
            "age_s": 0.0,
        },
        {
            "distance_forward_m": 1.0,
            "distance_lateral_m": float("inf"),
            "confidence": 1.0,
            "source": EnvelopeSource.LIDAR,
            "age_s": 0.0,
        },
        {
            "distance_forward_m": 1.0,
            "distance_lateral_m": 1.0,
            "confidence": 1.0,
            "source": EnvelopeSource.NONE,
            "age_s": 0.0,
        },
    ],
)
def test_invalid_sensor_frame_is_rejected(payload: dict[str, object]) -> None:
    with pytest.raises(ValidationError):
        SensorFrame.model_validate(payload)


def test_invalid_stopping_speed_is_rejected() -> None:
    evaluator = DynamicEnvelopeEvaluator()
    for invalid in (-1.0, float("nan"), float("inf")):
        with pytest.raises((ValidationError, ValueError)):
            evaluator.stopping_distance(invalid)


def test_result_is_immutable_and_bit_deterministic() -> None:
    evaluator = DynamicEnvelopeEvaluator()
    results = [
        evaluator.evaluate(_frame(forward=1.23456789, lateral=0.98765432))
        for _ in range(100)
    ]
    serialized = [result.model_dump_json() for result in results]
    hashes = {hashlib.sha256(item.encode("utf-8")).hexdigest() for item in serialized}

    assert all(result == results[0] for result in results)
    assert len(set(serialized)) == 1
    assert len(hashes) == 1
    with pytest.raises(ValidationError):
        results[0].max_linear_velocity_mps = 9.0


def test_angular_limit_is_finite_for_tiny_positive_radius() -> None:
    evaluator = DynamicEnvelopeEvaluator(
        EnvelopeConfig(angular_swept_radius_m=math.nextafter(0.0, 1.0))
    )
    result = evaluator.evaluate(_frame())

    assert math.isfinite(result.max_angular_velocity_radps)
    assert result.max_angular_velocity_radps == 1.0
