"""Acceptance tests for TB-EVAL-005B directional motion authority."""

from __future__ import annotations

import hashlib

import pytest
from pydantic import ValidationError

from core.braking import (
    BrakingEnvelopeConfig,
    DirectionalClearances,
    DirectionalSector,
    calculate_stopping_distance,
)
from core.motion_envelope import (
    AngularMotionPolicyStatus,
    DirectionalMotionEnvelope,
    DirectionalSpeedLimit,
    PlanarVelocityLimits,
    calculate_directional_motion_envelope,
    calculate_permitted_speed_limit,
)


@pytest.fixture
def config() -> BrakingEnvelopeConfig:
    return BrakingEnvelopeConfig(
        policy_version="TB-EVAL-005B-0.1.0",
        reaction_time_ns=250_000_000,
        assured_deceleration_mps2=2.0,
        clearance_margin_m=0.10,
    )


def _clearance_for_speed(
    speed_mps: float,
    config: BrakingEnvelopeConfig,
) -> float:
    return calculate_stopping_distance(
        speed_mps,
        config,
    ).required_clearance_m


def _clearances(
    *,
    forward_m: float = 2.0,
    reverse_m: float = 2.0,
    left_m: float = 2.0,
    right_m: float = 2.0,
) -> DirectionalClearances:
    return DirectionalClearances(
        forward_m=forward_m,
        reverse_m=reverse_m,
        left_m=left_m,
        right_m=right_m,
    )


def test_directional_limits_map_to_signed_ros_body_axes(
    config: BrakingEnvelopeConfig,
) -> None:
    envelope = calculate_directional_motion_envelope(
        _clearances(
            forward_m=_clearance_for_speed(1.0, config),
            reverse_m=_clearance_for_speed(0.5, config),
            left_m=_clearance_for_speed(0.25, config),
            right_m=_clearance_for_speed(0.75, config),
        ),
        config,
    )

    assert envelope.schema_version == "0.1"
    assert tuple(item.sector for item in envelope.sector_limits) == tuple(
        DirectionalSector
    )
    assert tuple(
        item.speed_limit.max_closing_speed_mps
        for item in envelope.sector_limits
    ) == (1.0, 0.5, 0.25, 0.75)
    assert envelope.velocity_limits == PlanarVelocityLimits(
        min_linear_x_mps=-0.5,
        max_linear_x_mps=1.0,
        min_linear_y_mps=-0.75,
        max_linear_y_mps=0.25,
        max_abs_angular_z_radps=0.0,
    )


@pytest.mark.parametrize(
    ("field", "sector", "axis_field"),
    [
        ("forward_m", DirectionalSector.FORWARD, "max_linear_x_mps"),
        ("reverse_m", DirectionalSector.REVERSE, "min_linear_x_mps"),
        ("left_m", DirectionalSector.LEFT, "max_linear_y_mps"),
        ("right_m", DirectionalSector.RIGHT, "min_linear_y_mps"),
    ],
)
def test_each_clearance_controls_only_its_direction(
    config: BrakingEnvelopeConfig,
    field: str,
    sector: DirectionalSector,
    axis_field: str,
) -> None:
    baseline = calculate_directional_motion_envelope(_clearances(), config)
    changed_clearances = _clearances().model_copy(update={field: 0.10})
    changed = calculate_directional_motion_envelope(
        changed_clearances,
        config,
    )
    baseline_by_sector = {
        item.sector: item.speed_limit for item in baseline.sector_limits
    }
    changed_by_sector = {
        item.sector: item.speed_limit for item in changed.sector_limits
    }

    assert changed_by_sector[sector].stop_only is True
    assert getattr(changed.velocity_limits, axis_field) == 0.0
    assert math_copysign_is_positive_zero(
        getattr(changed.velocity_limits, axis_field)
    )
    for unchanged_sector in DirectionalSector:
        if unchanged_sector is not sector:
            assert (
                changed_by_sector[unchanged_sector]
                == baseline_by_sector[unchanged_sector]
            )


def math_copysign_is_positive_zero(value: float) -> bool:
    return value.hex() == "0x0.0p+0"


def test_insufficient_omnidirectional_clearance_blocks_rotation(
    config: BrakingEnvelopeConfig,
) -> None:
    envelope = calculate_directional_motion_envelope(
        _clearances(right_m=config.clearance_margin_m),
        config,
    )

    assert envelope.translation_permitted_in_all_directions is False
    assert envelope.velocity_limits.max_abs_angular_z_radps == 0.0
    assert (
        envelope.angular_policy_status
        is AngularMotionPolicyStatus.BLOCKED_INSUFFICIENT_CLEARANCE
    )


def test_rotation_remains_blocked_without_swept_footprint_model(
    config: BrakingEnvelopeConfig,
) -> None:
    envelope = calculate_directional_motion_envelope(
        _clearances(),
        config,
    )

    assert envelope.translation_permitted_in_all_directions is True
    assert envelope.velocity_limits.max_abs_angular_z_radps == 0.0
    assert (
        envelope.angular_policy_status
        is AngularMotionPolicyStatus.BLOCKED_SWEPT_FOOTPRINT_UNAVAILABLE
    )


def test_directional_result_is_bit_stable(
    config: BrakingEnvelopeConfig,
) -> None:
    results = [
        calculate_directional_motion_envelope(
            _clearances(
                forward_m=1.23456789,
                reverse_m=0.98765432,
                left_m=0.45678901,
                right_m=0.34567890,
            ),
            config,
        )
        for _ in range(100)
    ]
    serialized = [result.model_dump_json() for result in results]
    hashes = {
        hashlib.sha256(item.encode("utf-8")).hexdigest()
        for item in serialized
    }

    assert all(result == results[0] for result in results)
    assert len(set(serialized)) == 1
    assert len(hashes) == 1


def test_directional_contract_rejects_forged_evidence(
    config: BrakingEnvelopeConfig,
) -> None:
    valid = calculate_directional_motion_envelope(_clearances(), config)
    reversed_limits = tuple(reversed(valid.sector_limits))
    wrong_policy_limit = DirectionalSpeedLimit(
        sector=DirectionalSector.FORWARD,
        speed_limit=calculate_permitted_speed_limit(
            2.0,
            config.model_copy(update={"policy_version": "other-policy"}),
        ),
    )
    wrong_physics_limit = DirectionalSpeedLimit(
        sector=DirectionalSector.FORWARD,
        speed_limit=calculate_permitted_speed_limit(
            2.0,
            config.model_copy(update={"reaction_time_ns": 300_000_000}),
        ),
    )

    with pytest.raises(ValidationError, match="every sector in order"):
        DirectionalMotionEnvelope.model_validate(
            valid.model_copy(
                update={"sector_limits": reversed_limits}
            ).model_dump(mode="python")
        )
    with pytest.raises(ValidationError, match="policy versions"):
        DirectionalMotionEnvelope.model_validate(
            valid.model_copy(
                update={
                    "sector_limits": (
                        wrong_policy_limit,
                        *valid.sector_limits[1:],
                    )
                }
            ).model_dump(mode="python")
        )
    with pytest.raises(ValidationError, match="physical assumptions"):
        DirectionalMotionEnvelope.model_validate(
            valid.model_copy(
                update={
                    "sector_limits": (
                        wrong_physics_limit,
                        *valid.sector_limits[1:],
                    )
                }
            ).model_dump(mode="python")
        )
    with pytest.raises(ValidationError, match="velocity limits"):
        DirectionalMotionEnvelope.model_validate(
            valid.model_copy(
                update={
                    "velocity_limits": valid.velocity_limits.model_copy(
                        update={"max_linear_x_mps": 99.0}
                    )
                }
            ).model_dump(mode="python")
        )
    with pytest.raises(ValidationError, match="velocity limits"):
        DirectionalMotionEnvelope.model_validate(
            valid.model_copy(
                update={
                    "velocity_limits": valid.velocity_limits.model_copy(
                        update={"max_abs_angular_z_radps": 0.01}
                    )
                }
            ).model_dump(mode="python")
        )
    with pytest.raises(ValidationError, match="translation status"):
        DirectionalMotionEnvelope.model_validate(
            valid.model_copy(
                update={
                    "translation_permitted_in_all_directions": False
                }
            ).model_dump(mode="python")
        )
    with pytest.raises(ValidationError, match="angular policy status"):
        DirectionalMotionEnvelope.model_validate(
            valid.model_copy(
                update={
                    "angular_policy_status": (
                        AngularMotionPolicyStatus.BLOCKED_INSUFFICIENT_CLEARANCE
                    )
                }
            ).model_dump(mode="python")
        )


def test_directional_models_are_strict_and_frozen(
    config: BrakingEnvelopeConfig,
) -> None:
    valid = calculate_directional_motion_envelope(_clearances(), config)

    with pytest.raises(ValidationError):
        PlanarVelocityLimits.model_validate(
            {
                **valid.velocity_limits.model_dump(mode="python"),
                "max_linear_x_mps": "1.0",
            }
        )
    with pytest.raises(ValidationError):
        valid.velocity_limits.max_linear_x_mps = 1.0
