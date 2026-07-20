"""Acceptance tests for TB-PoC-001 Safety Decision Loop."""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
from pydantic import ValidationError

from core.cli import main
from core.incident_store import FileIncidentStore, IncidentConflictError
from core.safety import (
    SafetyObservation,
    evaluate_safety,
    run_safety_decision_loop,
)
from core.types.incident import Pose2D


def _observation(
    *,
    clearance_m: float = 0.32,
    confidence: float = 0.95,
) -> SafetyObservation:
    return SafetyObservation(
        timestamp_ns=1_784_389_338_392_123_456,
        map_id="warehouse_zone_c",
        frame_id="map",
        pose=Pose2D(x=42.15, y=-12.33, yaw=1.5708),
        clearance_m=clearance_m,
        confidence=confidence,
        sensor_id="front_lidar",
    )


def test_emergency_stop_triggered_and_persisted(tmp_path: Path) -> None:
    store = FileIncidentStore(tmp_path)

    result = run_safety_decision_loop(_observation(), store)

    assert result.is_incident is True
    assert result.decision.action == "emergency_stop"
    assert result.decision.rule == "EV-SAFE-01"
    assert result.decision.evidence["clearance"].observed == 0.32
    assert result.decision.evidence["clearance"].threshold == 0.50
    assert result.decision.decision_id is not None
    assert store.load(result.decision.decision_id) == result.decision
    assert not list(tmp_path.glob("*.tmp"))


def test_no_stop_when_clearance_is_safe_and_no_incident_is_stored(
    tmp_path: Path,
) -> None:
    result = run_safety_decision_loop(
        _observation(clearance_m=1.20),
        FileIncidentStore(tmp_path),
    )

    assert result.is_incident is False
    assert result.decision.action == "proceed"
    assert result.decision.rule == "EV-SAFE-00"
    assert not list(tmp_path.iterdir())


def test_low_confidence_near_obstacle_uses_unambiguous_fail_safe(
    tmp_path: Path,
) -> None:
    result = run_safety_decision_loop(
        _observation(clearance_m=0.32, confidence=0.69),
        FileIncidentStore(tmp_path),
    )

    assert result.is_incident is True
    assert result.decision.action == "protective_stop"
    assert result.decision.rule == "EV-SAFE-02"


@pytest.mark.parametrize(
    ("clearance_m", "confidence", "expected_action", "expected_rule"),
    [
        (0.50, 0.70, "proceed", "EV-SAFE-00"),
        (0.4999999, 0.70, "emergency_stop", "EV-SAFE-01"),
        (0.4999999, 0.6999999, "protective_stop", "EV-SAFE-02"),
    ],
)
def test_boundary_conditions(
    clearance_m: float,
    confidence: float,
    expected_action: str,
    expected_rule: str,
) -> None:
    result = evaluate_safety(
        _observation(clearance_m=clearance_m, confidence=confidence)
    )

    assert result.decision.action == expected_action
    assert result.decision.rule == expected_rule


@pytest.mark.parametrize(
    "updates",
    [
        {"clearance_m": float("nan")},
        {"clearance_m": float("inf")},
        {"confidence": float("-inf")},
        {"timestamp_ns": -1},
        {"sensor_id": " "},
    ],
)
def test_invalid_inputs_are_rejected(updates: dict[str, object]) -> None:
    payload = _observation().model_dump()
    payload.update(updates)

    with pytest.raises(ValidationError):
        SafetyObservation.model_validate(payload)


def test_missing_input_is_rejected() -> None:
    payload = _observation().model_dump()
    del payload["clearance_m"]

    with pytest.raises(ValidationError):
        SafetyObservation.model_validate(payload)


def test_evaluation_is_deterministic_and_side_effect_free() -> None:
    observation = _observation()

    results = [evaluate_safety(observation) for _ in range(100)]
    serialized_decisions = [
        result.decision.model_dump_json() for result in results
    ]
    decision_hashes = {
        hashlib.sha256(serialized.encode("utf-8")).hexdigest()
        for serialized in serialized_decisions
    }

    assert all(result == results[0] for result in results)
    assert len(set(serialized_decisions)) == 1
    assert len(decision_hashes) == 1


def test_store_is_idempotent_but_rejects_silent_overwrite(tmp_path: Path) -> None:
    store = FileIncidentStore(tmp_path)
    decision = evaluate_safety(_observation()).decision

    first_path = store.save(decision)
    second_path = store.save(decision)

    assert second_path == first_path
    conflicting = decision.model_copy(update={"action": "different_action"})
    with pytest.raises(IncidentConflictError):
        store.save(conflicting)
    assert store.load(decision.decision_id or "") == decision


def test_end_to_end_generated_incident_is_replayable(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    observation_path = tmp_path / "observation.json"
    incident_store = tmp_path / "incidents"
    observation = _observation()
    observation_path.write_text(observation.model_dump_json(), encoding="utf-8")
    expected = evaluate_safety(observation)

    evaluate_exit = main(
        [
            "safety-evaluate",
            str(observation_path),
            "--store",
            str(incident_store),
        ]
    )
    evaluate_output = capsys.readouterr()

    assert evaluate_exit == 0
    assert f"[DECISION] {expected.decision.decision_id}" in evaluate_output.out
    assert "ACTION:   emergency_stop" in evaluate_output.out
    assert evaluate_output.err == ""

    replay_exit = main(
        [
            "replay",
            expected.decision.decision_id or "",
            "--store",
            str(incident_store),
        ]
    )
    replay_output = capsys.readouterr()

    assert replay_exit == 0
    assert f"[INCIDENT] {expected.decision.decision_id}" in replay_output.out
    assert "Action:   emergency_stop" in replay_output.out
    assert "Rule:     EV-SAFE-01" in replay_output.out
    assert "Frontal Clearance: 0.32 m (Threshold: 0.50 m)" in replay_output.out
    assert replay_output.err == ""


def test_invalid_json_returns_controlled_cli_error(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    observation_path = tmp_path / "invalid.json"
    observation_path.write_text('{"clearance_m": NaN}', encoding="utf-8")

    result = main(
        [
            "safety-evaluate",
            str(observation_path),
            "--store",
            str(tmp_path / "incidents"),
        ]
    )
    output = capsys.readouterr()

    assert result == 2
    assert output.out == ""
    assert output.err.startswith("tb safety-evaluate: error:")
    assert not (tmp_path / "incidents").exists()
