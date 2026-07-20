"""Integration tests for the Titan Brain replay CLI."""

from __future__ import annotations

import pytest

from core.cli import format_decision_replay, load_bundled_decision, main

EXPECTED_INCIDENT_042 = """[INCIDENT] incident_042
--------------------------------------------------
TIME:     2026-07-18 15:42:18.392123 UTC
LOCATION: warehouse_zone_c (x: 42.15, y: -12.33, yaw: 1.57)

DECISION:
Module:   local_planner
Action:   emergency_stop
Rule:     min_clearance_violated

EVIDENCE:
- Frontal Clearance: 0.42 m (Threshold: 0.50 m)
- Obstacle Confidence: 0.95 probability (Threshold: 0.80 probability)
--------------------------------------------------"""


def test_incident_042_matches_contract_and_expected_replay() -> None:
    decision = load_bundled_decision("incident_042")

    assert decision.decision_id == "incident_042"
    assert format_decision_replay(decision) == EXPECTED_INCIDENT_042


def test_replay_command_prints_expected_output(
    capsys: pytest.CaptureFixture[str],
) -> None:
    result = main(["replay", "incident_042"])

    assert result == 0
    captured = capsys.readouterr()
    assert captured.out == EXPECTED_INCIDENT_042 + "\n"
    assert captured.err == ""


def test_replay_reports_unknown_incident(
    capsys: pytest.CaptureFixture[str],
) -> None:
    result = main(["replay", "incident_999"])

    assert result == 2
    captured = capsys.readouterr()
    assert captured.out == ""
    assert captured.err == "tb replay: error: Incident not found: incident_999\n"


def test_replay_rejects_path_traversal(
    capsys: pytest.CaptureFixture[str],
) -> None:
    result = main(["replay", "../incident_042"])

    assert result == 2
    captured = capsys.readouterr()
    assert captured.out == ""
    assert captured.err == (
        "tb replay: error: Invalid incident ID: '../incident_042'\n"
    )
