"""Acceptance tests for TB-EVAL-005A inverse braking mathematics."""

from __future__ import annotations

import hashlib
import math
from decimal import Decimal, localcontext

import pytest
from pydantic import ValidationError

from core.braking import BrakingEnvelopeConfig, calculate_stopping_distance
from core.motion_envelope import (
    PermittedSpeedLimit,
    PermittedSpeedStatus,
    calculate_permitted_speed_limit,
)


@pytest.fixture
def config() -> BrakingEnvelopeConfig:
    return BrakingEnvelopeConfig(
        policy_version="TB-EVAL-005A-0.1.0",
        reaction_time_ns=250_000_000,
        assured_deceleration_mps2=2.0,
        clearance_margin_m=0.10,
    )


def _exact_required_clearance(
    speed_mps: float,
    config: BrakingEnvelopeConfig,
) -> Decimal:
    with localcontext() as context:
        context.prec = 100
        speed = Decimal(str(speed_mps))
        reaction_time = Decimal(config.reaction_time_ns) / Decimal(
            1_000_000_000
        )
        deceleration = Decimal(str(config.assured_deceleration_mps2))
        return (
            speed * reaction_time
            + speed * speed / (Decimal(2) * deceleration)
            + Decimal(str(config.clearance_margin_m))
        )


def test_inverse_limit_matches_the_existing_exact_boundary(
    config: BrakingEnvelopeConfig,
) -> None:
    limit = calculate_permitted_speed_limit(0.60, config)

    assert limit.schema_version == "0.1"
    assert limit.policy_version == "TB-EVAL-005A-0.1.0"
    assert limit.observed_clearance_m == 0.60
    assert limit.available_clearance_m == 0.50
    assert limit.max_closing_speed_mps == 1.0
    assert limit.status is PermittedSpeedStatus.MOVEMENT_PERMITTED
    assert limit.stop_only is False
    assert (
        calculate_stopping_distance(
            limit.max_closing_speed_mps,
            config,
        ).required_clearance_m
        == limit.observed_clearance_m
    )


@pytest.mark.parametrize("clearance_m", [0.0, 0.05, 0.10])
def test_exhausted_margin_grants_stop_only(
    config: BrakingEnvelopeConfig,
    clearance_m: float,
) -> None:
    limit = calculate_permitted_speed_limit(clearance_m, config)

    assert limit.available_clearance_m == 0.0
    assert limit.max_closing_speed_mps == 0.0
    assert limit.status is PermittedSpeedStatus.STOP_ONLY
    assert limit.stop_only is True


def test_larger_clearance_monotonically_increases_authority(
    config: BrakingEnvelopeConfig,
) -> None:
    limits = [
        calculate_permitted_speed_limit(clearance, config)
        for clearance in (0.11, 0.20, 0.60, 2.0)
    ]

    speeds = [limit.max_closing_speed_mps for limit in limits]
    assert speeds == sorted(speeds)
    assert len(set(speeds)) == len(speeds)


@pytest.mark.parametrize("speed_mps", [0.1, 0.5, 1.0, 3.0, 20.0])
def test_forward_then_inverse_model_never_authorizes_more_than_boundary(
    config: BrakingEnvelopeConfig,
    speed_mps: float,
) -> None:
    clearance = calculate_stopping_distance(
        speed_mps,
        config,
    ).required_clearance_m
    limit = calculate_permitted_speed_limit(clearance, config)
    next_speed = math.nextafter(limit.max_closing_speed_mps, math.inf)

    assert limit.max_closing_speed_mps <= speed_mps
    assert calculate_stopping_distance(
        limit.max_closing_speed_mps,
        config,
    ).required_clearance_m <= clearance
    assert _exact_required_clearance(
        next_speed,
        config,
    ) > Decimal(str(clearance))


@pytest.mark.parametrize(
    "clearance",
    [-0.01, float("nan"), float("inf"), True, "1.0"],
)
def test_invalid_clearance_is_rejected(
    config: BrakingEnvelopeConfig,
    clearance: object,
) -> None:
    with pytest.raises(ValueError, match="non-negative finite"):
        calculate_permitted_speed_limit(
            clearance,  # type: ignore[arg-type]
            config,
        )


def test_integer_too_large_for_float_is_rejected(
    config: BrakingEnvelopeConfig,
) -> None:
    with pytest.raises(ValueError, match="non-negative finite"):
        calculate_permitted_speed_limit(
            10**400,
            config,
        )


def test_reaction_time_conversion_overflow_is_rejected() -> None:
    config = BrakingEnvelopeConfig(
        policy_version="TB-EVAL-005A-0.1.0",
        reaction_time_ns=10**400,
        assured_deceleration_mps2=2.0,
        clearance_margin_m=0.10,
    )

    with pytest.raises(ValueError, match="reaction-time conversion"):
        calculate_permitted_speed_limit(1.0, config)


def test_non_finite_calculated_limit_is_rejected() -> None:
    max_float = float.fromhex("0x1.fffffffffffffp+1023")
    config = BrakingEnvelopeConfig(
        policy_version="TB-EVAL-005A-0.1.0",
        reaction_time_ns=1,
        assured_deceleration_mps2=max_float,
        clearance_margin_m=0.0,
    )

    with pytest.raises(ValueError, match="result must be finite"):
        calculate_permitted_speed_limit(max_float, config)


def test_forward_model_overflow_fails_closed() -> None:
    config = BrakingEnvelopeConfig(
        policy_version="TB-EVAL-005A-0.1.0",
        reaction_time_ns=1,
        assured_deceleration_mps2=1e308,
        clearance_margin_m=0.0,
    )

    with pytest.raises(ValueError, match="conservative speed limit"):
        calculate_permitted_speed_limit(1e308, config)


def test_positive_clearance_that_underflows_speed_grants_stop_only() -> None:
    config = BrakingEnvelopeConfig(
        policy_version="TB-EVAL-005A-0.1.0",
        reaction_time_ns=10**317,
        assured_deceleration_mps2=1.0,
        clearance_margin_m=0.0,
    )

    limit = calculate_permitted_speed_limit(
        float.fromhex("0x0.0000000000001p-1022"),
        config,
    )

    assert limit.max_closing_speed_mps == 0.0
    assert limit.status is PermittedSpeedStatus.STOP_ONLY


def test_result_is_bit_stable_for_identical_inputs(
    config: BrakingEnvelopeConfig,
) -> None:
    results = [
        calculate_permitted_speed_limit(1.23456789, config)
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


def test_immutable_contract_rejects_inconsistent_evidence(
    config: BrakingEnvelopeConfig,
) -> None:
    valid = calculate_permitted_speed_limit(0.60, config)
    baseline = valid.model_dump(mode="python")

    with pytest.raises(ValidationError, match="available clearance"):
        PermittedSpeedLimit.model_validate(
            {**baseline, "available_clearance_m": 0.49}
        )
    with pytest.raises(ValidationError, match="maximum speed"):
        PermittedSpeedLimit.model_validate(
            {**baseline, "max_closing_speed_mps": 0.99}
        )
    with pytest.raises(ValidationError, match="speed status"):
        PermittedSpeedLimit.model_validate(
            {**baseline, "status": PermittedSpeedStatus.STOP_ONLY}
        )
    with pytest.raises(ValidationError):
        PermittedSpeedLimit.model_validate(
            {**baseline, "schema_version": "0.2"}
        )


def test_models_are_strict_and_frozen(
    config: BrakingEnvelopeConfig,
) -> None:
    limit = calculate_permitted_speed_limit(0.60, config)

    with pytest.raises(ValidationError):
        PermittedSpeedLimit.model_validate(
            {
                **limit.model_dump(mode="python"),
                "observed_clearance_m": "0.60",
            }
        )
    with pytest.raises(ValidationError):
        limit.max_closing_speed_mps = 0.0
