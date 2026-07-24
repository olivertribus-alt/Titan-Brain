"""Integration tests for TB-EVAL-002B dynamic safety evaluation."""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
from pydantic import ValidationError

from core.braking import BrakingEnvelopeConfig, DirectionalClearances
from core.incident_store import FileIncidentStore
from core.safety import (
    DirectionalSafetyData,
    PlanarVelocity,
    SafetyObservation,
    SafetyRuleConfig,
    evaluate_safety,
    run_safety_decision_loop,
)
from core.stability import (
    EvaluatorState,
    SafetyStabilityFilter,
    StabilityConfig,
    StabilityReason,
)
from core.types.incident import Pose2D


def _rules() -> SafetyRuleConfig:
    return SafetyRuleConfig(
        policy_version="TB-SAFE-0.2.0",
        clearance_threshold_m=0.50,
        confidence_threshold=0.70,
        braking_envelope=BrakingEnvelopeConfig(
            policy_version="TB-BRAKE-0.2.0",
            reaction_time_ns=250_000_000,
            assured_deceleration_mps2=1.0,
            clearance_margin_m=0.50,
        ),
    )


def _observation(
    *,
    clearance_m: float = 0.80,
    confidence: float = 0.95,
    linear_x_mps: float = 0.10,
    linear_y_mps: float = 0.0,
    angular_z_radps: float = 0.0,
    forward_m: float | None = None,
    reverse_m: float = 0.10,
    left_m: float = 0.10,
    right_m: float = 0.10,
    include_directional_data: bool = True,
) -> SafetyObservation:
    directional_data = None
    if include_directional_data:
        directional_data = DirectionalSafetyData(
            clearances=DirectionalClearances(
                forward_m=clearance_m if forward_m is None else forward_m,
                reverse_m=reverse_m,
                left_m=left_m,
                right_m=right_m,
            ),
            velocity=PlanarVelocity(
                linear_x_mps=linear_x_mps,
                linear_y_mps=linear_y_mps,
                angular_z_radps=angular_z_radps,
            ),
        )
    return SafetyObservation(
        timestamp_ns=1_784_389_338_392_123_456,
        map_id="warehouse_zone_c",
        frame_id="map",
        pose=Pose2D(x=42.15, y=-12.33, yaw=1.5708),
        clearance_m=clearance_m,
        confidence=confidence,
        sensor_id="front_lidar",
        directional_data=directional_data,
    )


def test_higher_forward_speed_requires_more_clearance() -> None:
    slow = evaluate_safety(_observation(linear_x_mps=0.10), _rules())
    fast = evaluate_safety(_observation(linear_x_mps=0.80), _rules())

    assert slow.decision.action == "proceed"
    assert slow.decision.rule == "EV-SAFE-DYN-00"
    assert fast.decision.action == "emergency_stop"
    assert fast.decision.rule == "EV-SAFE-DYN-01"
    assert slow.decision.evidence["clearance"].threshold == pytest.approx(0.53)
    assert fast.decision.evidence["clearance"].threshold == pytest.approx(1.02)


def test_obstacles_in_inactive_sectors_do_not_block_forward_motion() -> None:
    result = evaluate_safety(
        _observation(
            clearance_m=1.20,
            linear_x_mps=0.40,
            reverse_m=0.01,
            left_m=0.01,
            right_m=0.01,
        ),
        _rules(),
    )

    assert result.decision.action == "proceed"
    assert result.decision.evidence["limiting_sector"].value == "forward"


def test_lateral_motion_uses_only_matching_lateral_sector() -> None:
    result = evaluate_safety(
        _observation(
            clearance_m=0.01,
            forward_m=0.01,
            linear_x_mps=0.0,
            linear_y_mps=0.50,
            left_m=2.0,
            right_m=0.01,
        ),
        _rules(),
    )

    assert result.decision.action == "proceed"
    assert result.decision.evidence["limiting_sector"].value == "left"


@pytest.mark.parametrize(
    ("linear_x_mps", "linear_y_mps", "clearance_updates", "sector"),
    [
        (-0.5, 0.0, {"reverse_m": 0.2}, "reverse"),
        (0.0, -0.5, {"right_m": 0.2}, "right"),
    ],
)
def test_signed_velocity_maps_to_reverse_and_right_sectors(
    linear_x_mps: float,
    linear_y_mps: float,
    clearance_updates: dict[str, float],
    sector: str,
) -> None:
    result = evaluate_safety(
        _observation(
            clearance_m=2.0,
            forward_m=2.0,
            reverse_m=clearance_updates.get("reverse_m", 2.0),
            left_m=2.0,
            right_m=clearance_updates.get("right_m", 2.0),
            linear_x_mps=linear_x_mps,
            linear_y_mps=linear_y_mps,
        ),
        _rules(),
    )

    assert result.decision.action == "emergency_stop"
    assert result.decision.evidence["limiting_sector"].value == sector


@pytest.mark.parametrize(
    ("clearance_m", "expected_action"),
    [(1.02, "proceed"), (1.019999999, "emergency_stop")],
)
def test_dynamic_clearance_boundary_is_inclusive(
    clearance_m: float,
    expected_action: str,
) -> None:
    result = evaluate_safety(
        _observation(clearance_m=clearance_m, linear_x_mps=0.8),
        _rules(),
    )

    assert result.decision.action == expected_action


def test_low_confidence_dynamic_violation_is_protective_stop() -> None:
    result = evaluate_safety(
        _observation(linear_x_mps=0.80, confidence=0.69),
        _rules(),
    )

    assert result.decision.action == "protective_stop"
    assert result.decision.rule == "EV-SAFE-DYN-02"
    assert result.is_incident is True


def test_missing_directional_input_fails_closed_when_dynamic_mode_is_enabled() -> None:
    result = evaluate_safety(
        _observation(include_directional_data=False),
        _rules(),
    )

    assert result.decision.action == "protective_stop"
    assert result.decision.rule == "EV-SAFE-DYN-03"
    assert result.decision.evidence["dynamic_input"].value == "missing"


def test_rotation_fails_closed_until_swept_footprint_model_exists() -> None:
    result = evaluate_safety(
        _observation(angular_z_radps=0.01),
        _rules(),
    )

    assert result.decision.action == "protective_stop"
    assert result.decision.rule == "EV-SAFE-DYN-04"
    assert result.decision.evidence["angular_velocity"].threshold == 0.0


@pytest.mark.parametrize(
    ("clearance_m", "expected_action", "expected_rule"),
    [
        (0.49, "emergency_stop", "EV-SAFE-01"),
        (0.50, "proceed", "EV-SAFE-00"),
    ],
)
def test_stationary_dynamic_input_uses_legacy_clearance_floor(
    clearance_m: float,
    expected_action: str,
    expected_rule: str,
) -> None:
    result = evaluate_safety(
        _observation(clearance_m=clearance_m, linear_x_mps=0.0),
        _rules(),
    )

    assert result.decision.action == expected_action
    assert result.decision.rule == expected_rule


def test_legacy_default_path_preserves_existing_decision_id() -> None:
    result = evaluate_safety(
        _observation(
            clearance_m=0.32,
            include_directional_data=False,
        )
    )

    assert result.decision.decision_id == "incident_c7f4873047b14e43"
    assert result.decision.rule == "EV-SAFE-01"


def test_legacy_rules_ignore_opt_in_data_until_dynamic_config_is_present() -> None:
    result = evaluate_safety(_observation(clearance_m=0.80, linear_x_mps=10.0))

    assert result.decision.action == "proceed"
    assert result.decision.rule == "EV-SAFE-00"


def test_dynamic_incident_evidence_is_persisted_atomically(tmp_path: Path) -> None:
    store = FileIncidentStore(tmp_path)

    result = run_safety_decision_loop(
        _observation(linear_x_mps=0.80),
        store,
        _rules(),
    )

    assert result.decision.decision_id is not None
    assert store.load(result.decision.decision_id) == result.decision
    assert result.decision.evidence["reaction_distance"].value == pytest.approx(0.20)
    assert result.decision.evidence["braking_distance"].value == pytest.approx(0.32)
    assert not list(tmp_path.glob("*.tmp"))


def test_dynamic_evaluation_is_bit_deterministic() -> None:
    observation = _observation(linear_x_mps=0.80)
    rules = _rules()

    results = [evaluate_safety(observation, rules) for _ in range(100)]
    serialized = [result.model_dump_json() for result in results]
    hashes = {hashlib.sha256(value.encode("utf-8")).hexdigest() for value in serialized}

    assert all(result == results[0] for result in results)
    assert len(set(serialized)) == 1
    assert len(hashes) == 1


def test_dynamic_required_clearance_drives_hysteresis_release_threshold() -> None:
    filter_ = SafetyStabilityFilter(
        StabilityConfig(
            policy_version="TB-STABILITY-0.1.0",
            clearance_hysteresis_m=0.1,
            recovery_hold_time_ns=200,
        )
    )
    danger = filter_.process(
        evaluate_safety(
            _observation(clearance_m=0.8, linear_x_mps=0.8),
            _rules(),
        ),
        now_ns=0,
    )
    noisy_safe = filter_.process(
        evaluate_safety(
            _observation(clearance_m=0.62, linear_x_mps=0.1),
            _rules(),
        ),
        now_ns=1,
    )
    holding = filter_.process(
        evaluate_safety(
            _observation(clearance_m=0.63, linear_x_mps=0.1),
            _rules(),
        ),
        now_ns=2,
    )
    released = filter_.process(
        evaluate_safety(
            _observation(clearance_m=0.63, linear_x_mps=0.1),
            _rules(),
        ),
        now_ns=202,
    )

    assert danger.transition.state is EvaluatorState.E_STOP
    assert noisy_safe.transition.reason is StabilityReason.HYSTERESIS_NOT_MET
    assert noisy_safe.transition.release_threshold_m == pytest.approx(0.63)
    assert holding.transition.state is EvaluatorState.RECOVERY_HOLDING
    assert released.transition.state is EvaluatorState.OK


@pytest.mark.parametrize(
    "field",
    ["linear_x_mps", "linear_y_mps", "angular_z_radps"],
)
def test_non_finite_dynamic_velocity_is_rejected(field: str) -> None:
    payload = _observation().model_dump(mode="python")
    directional = payload["directional_data"]
    assert isinstance(directional, dict)
    velocity = directional["velocity"]
    assert isinstance(velocity, dict)
    velocity[field] = float("nan")

    with pytest.raises(ValidationError):
        SafetyObservation.model_validate(payload)


def test_contradictory_frontal_clearance_alias_is_rejected() -> None:
    payload = _observation().model_dump(mode="python")
    directional = payload["directional_data"]
    assert isinstance(directional, dict)
    clearances = directional["clearances"]
    assert isinstance(clearances, dict)
    clearances["forward_m"] = 0.81

    with pytest.raises(ValidationError, match="clearance_m must equal"):
        SafetyObservation.model_validate(payload)
