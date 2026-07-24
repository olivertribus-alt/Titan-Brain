"""Acceptance tests for TB-EVAL-002A directional braking mathematics."""

from __future__ import annotations

import hashlib

import pytest
from pydantic import ValidationError

from core.braking import (
    BrakingEnvelopeAssessment,
    BrakingEnvelopeConfig,
    DirectionalClearances,
    DirectionalClosingSpeeds,
    DirectionalSector,
    SectorBrakingAssessment,
    StoppingDistanceBreakdown,
    assess_braking_envelope,
    calculate_stopping_distance,
)


@pytest.fixture
def config() -> BrakingEnvelopeConfig:
    return BrakingEnvelopeConfig(
        policy_version="TB-BRAKE-0.1.0",
        reaction_time_ns=250_000_000,
        assured_deceleration_mps2=2.0,
        clearance_margin_m=0.10,
    )


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


def _speeds(
    *,
    forward_mps: float = 0.0,
    reverse_mps: float = 0.0,
    left_mps: float = 0.0,
    right_mps: float = 0.0,
) -> DirectionalClosingSpeeds:
    return DirectionalClosingSpeeds(
        forward_mps=forward_mps,
        reverse_mps=reverse_mps,
        left_mps=left_mps,
        right_mps=right_mps,
    )


def test_stopping_distance_exposes_every_formula_term(
    config: BrakingEnvelopeConfig,
) -> None:
    distance = calculate_stopping_distance(1.0, config)

    assert distance.reaction_distance_m == 0.25
    assert distance.braking_distance_m == 0.25
    assert distance.clearance_margin_m == 0.10
    assert distance.required_clearance_m == 0.60


@pytest.mark.parametrize(
    ("clearance_m", "expected_safe"),
    [(0.60, True), (0.599999999, False)],
)
def test_required_clearance_boundary_is_unambiguous(
    config: BrakingEnvelopeConfig,
    clearance_m: float,
    expected_safe: bool,
) -> None:
    result = assess_braking_envelope(
        _clearances(forward_m=clearance_m),
        _speeds(forward_mps=1.0),
        config,
    )

    assert result.safe_to_proceed is expected_safe
    assert result.limiting_sector is DirectionalSector.FORWARD


def test_obstacle_behind_does_not_block_forward_motion(
    config: BrakingEnvelopeConfig,
) -> None:
    result = assess_braking_envelope(
        _clearances(forward_m=1.0, reverse_m=0.01),
        _speeds(forward_mps=1.0),
        config,
    )

    reverse = result.assessments[1]
    assert result.safe_to_proceed is True
    assert reverse.sector is DirectionalSector.REVERSE
    assert reverse.active is False
    assert reverse.stopping_distance is None
    assert reverse.clearance_sufficient is None


def test_reverse_motion_uses_only_reverse_clearance(
    config: BrakingEnvelopeConfig,
) -> None:
    result = assess_braking_envelope(
        _clearances(forward_m=0.01, reverse_m=0.50),
        _speeds(reverse_mps=1.0),
        config,
    )

    assert result.safe_to_proceed is False
    assert result.violated_sectors == (DirectionalSector.REVERSE,)
    assert result.limiting_sector is DirectionalSector.REVERSE


def test_lateral_and_diagonal_motion_assess_each_active_sector(
    config: BrakingEnvelopeConfig,
) -> None:
    result = assess_braking_envelope(
        _clearances(forward_m=1.0, left_m=0.20),
        _speeds(forward_mps=1.0, left_mps=0.5),
        config,
    )

    assert result.safe_to_proceed is False
    assert result.violated_sectors == (DirectionalSector.LEFT,)
    assert result.limiting_sector is DirectionalSector.LEFT


def test_stationary_motion_has_no_active_or_limiting_sector(
    config: BrakingEnvelopeConfig,
) -> None:
    result = assess_braking_envelope(
        _clearances(
            forward_m=0.0,
            reverse_m=0.0,
            left_m=0.0,
            right_m=0.0,
        ),
        _speeds(),
        config,
    )

    assert result.safe_to_proceed is True
    assert result.limiting_sector is None
    assert result.violated_sectors == ()
    assert all(not item.active for item in result.assessments)


@pytest.mark.parametrize(
    "payload",
    [
        {"forward_mps": 1.0, "reverse_mps": 1.0},
        {"left_mps": 1.0, "right_mps": 1.0},
    ],
)
def test_opposing_closing_speeds_are_rejected(
    payload: dict[str, float],
) -> None:
    values = _speeds().model_dump()
    values.update(payload)

    with pytest.raises(ValidationError):
        DirectionalClosingSpeeds.model_validate(values)


@pytest.mark.parametrize(
    ("model", "field", "value"),
    [
        (DirectionalClearances, "forward_m", float("nan")),
        (DirectionalClearances, "reverse_m", float("inf")),
        (DirectionalClosingSpeeds, "left_mps", float("-inf")),
        (DirectionalClosingSpeeds, "right_mps", -0.01),
        (BrakingEnvelopeConfig, "assured_deceleration_mps2", 0.0),
        (BrakingEnvelopeConfig, "clearance_margin_m", -0.01),
        (BrakingEnvelopeConfig, "reaction_time_ns", 0),
    ],
)
def test_invalid_physical_inputs_are_rejected(
    model: type[
        DirectionalClearances | DirectionalClosingSpeeds | BrakingEnvelopeConfig
    ],
    field: str,
    value: float | int,
    config: BrakingEnvelopeConfig,
) -> None:
    baseline = {
        DirectionalClearances: _clearances().model_dump(),
        DirectionalClosingSpeeds: _speeds().model_dump(),
        BrakingEnvelopeConfig: config.model_dump(),
    }[model]
    baseline[field] = value

    with pytest.raises(ValidationError):
        model.model_validate(baseline)


@pytest.mark.parametrize("speed", [0.0, -0.1, float("nan"), float("inf")])
def test_stopping_distance_rejects_non_positive_or_non_finite_speed(
    config: BrakingEnvelopeConfig,
    speed: float,
) -> None:
    with pytest.raises(ValueError):
        calculate_stopping_distance(speed, config)


def test_stopping_distance_rejects_finite_input_that_overflows(
    config: BrakingEnvelopeConfig,
) -> None:
    with pytest.raises(ValueError):
        calculate_stopping_distance(1e308, config)


def test_stopping_distance_rejects_reaction_time_conversion_overflow() -> None:
    config = BrakingEnvelopeConfig(
        policy_version="TB-BRAKE-0.1.0",
        reaction_time_ns=10**400,
        assured_deceleration_mps2=2.0,
        clearance_margin_m=0.10,
    )

    with pytest.raises(ValueError):
        calculate_stopping_distance(1.0, config)


def test_all_deployment_assumptions_are_required() -> None:
    with pytest.raises(ValidationError):
        BrakingEnvelopeConfig.model_validate({})


def test_assessment_is_bit_stable_for_identical_inputs(
    config: BrakingEnvelopeConfig,
) -> None:
    clearances = _clearances(forward_m=0.7, right_m=0.3)
    speeds = _speeds(forward_mps=0.8, right_mps=0.2)

    results = [assess_braking_envelope(clearances, speeds, config) for _ in range(100)]
    serialized = [result.model_dump_json() for result in results]
    hashes = {hashlib.sha256(item.encode("utf-8")).hexdigest() for item in serialized}

    assert all(result == results[0] for result in results)
    assert len(set(serialized)) == 1
    assert len(hashes) == 1


def test_derived_contracts_reject_internally_inconsistent_evidence(
    config: BrakingEnvelopeConfig,
) -> None:
    distance = calculate_stopping_distance(1.0, config)
    with pytest.raises(ValidationError):
        StoppingDistanceBreakdown(
            closing_speed_mps=1.0,
            reaction_distance_m=0.25,
            braking_distance_m=0.25,
            clearance_margin_m=0.10,
            required_clearance_m=0.61,
        )

    with pytest.raises(ValidationError):
        SectorBrakingAssessment(
            sector=DirectionalSector.FORWARD,
            observed_clearance_m=1.0,
            active=True,
        )
    with pytest.raises(ValidationError):
        SectorBrakingAssessment(
            sector=DirectionalSector.FORWARD,
            observed_clearance_m=1.0,
            active=False,
            stopping_distance=distance,
        )

    valid = assess_braking_envelope(
        _clearances(),
        _speeds(forward_mps=1.0),
        config,
    )
    with pytest.raises(ValidationError):
        BrakingEnvelopeAssessment(
            policy_version=valid.policy_version,
            assessments=tuple(reversed(valid.assessments)),
            safe_to_proceed=valid.safe_to_proceed,
            limiting_sector=valid.limiting_sector,
        )
    with pytest.raises(ValidationError):
        BrakingEnvelopeAssessment(
            policy_version=valid.policy_version,
            assessments=valid.assessments,
            safe_to_proceed=False,
            limiting_sector=valid.limiting_sector,
        )
    with pytest.raises(ValidationError):
        BrakingEnvelopeAssessment(
            policy_version=valid.policy_version,
            assessments=valid.assessments,
            safe_to_proceed=valid.safe_to_proceed,
            limiting_sector=DirectionalSector.REVERSE,
        )
