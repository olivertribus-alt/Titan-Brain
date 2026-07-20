"""Tests for v0.2 decision evidence and v0.1 migration."""

from __future__ import annotations

from datetime import UTC, datetime

import pytest
from pydantic import ValidationError

from core.types.incident import (
    DecisionEvidence,
    EvidenceValue,
    Pose2D,
    SpatialContext,
    iso8601_to_unix_ns,
    load_decision_evidence,
)


def _complete_decision() -> DecisionEvidence:
    return DecisionEvidence(
        decision_id="incident_042",
        occurred_at_unix_ns=1_752_851_738_392_123_456,
        source_module="local_planner",
        action="stop",
        rule="obstacle_confidence_below_threshold",
        evidence={
            "obstacle_confidence": EvidenceValue(
                label="Obstacle confidence",
                description="Confidence assigned to the nearest obstacle",
                value=0.65,
                expected=0.9,
                observed=0.65,
                threshold=0.8,
                unit="ratio",
            )
        },
        spatial_context=SpatialContext(
            map_id="warehouse_a",
            frame_id="map",
            pose=Pose2D(x=14.8, y=6.3, yaw=0.25),
        ),
    )


def test_complete_v02_decision() -> None:
    decision = _complete_decision()

    assert decision.schema_version == "0.2"
    assert decision.evidence["obstacle_confidence"].threshold == 0.8
    assert decision.spatial_context is not None
    assert decision.spatial_context.map_id == "warehouse_a"


def test_spatial_context_is_optional() -> None:
    decision = DecisionEvidence(
        occurred_at_unix_ns=1,
        source_module="supervisor",
        action="wait",
        rule="queue_not_ready",
        evidence={},
    )

    assert decision.decision_id is None
    assert decision.spatial_context is None


def test_zero_is_distinct_from_missing_data() -> None:
    evidence = EvidenceValue(
        label="Velocity",
        value=0.0,
        expected=0.0,
        observed=0.0,
        threshold=0.0,
        unit="m/s",
    )
    context = SpatialContext(
        map_id="test_map",
        pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
    )

    assert evidence.value == 0.0
    assert evidence.expected == 0.0
    assert evidence.observed == 0.0
    assert evidence.threshold == 0.0
    assert evidence.description is None
    assert context.pose.x == 0.0


@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_numbers_are_rejected(value: float) -> None:
    with pytest.raises(ValidationError):
        EvidenceValue(label="Invalid", value=value)


def test_nested_non_finite_number_is_rejected() -> None:
    with pytest.raises(ValidationError):
        EvidenceValue(label="Invalid", value={"samples": [0.1, float("nan")]})


@pytest.mark.parametrize(
    ("model", "kwargs"),
    [
        (SpatialContext, {"map_id": " ", "pose": {"x": 0, "y": 0, "yaw": 0}}),
        (
            DecisionEvidence,
            {
                "occurred_at_unix_ns": 1,
                "source_module": "\t",
                "action": "stop",
                "rule": "safety_rule",
                "evidence": {},
            },
        ),
        (EvidenceValue, {"label": "", "value": 0.0}),
    ],
)
def test_blank_text_is_rejected(
    model: type[SpatialContext] | type[DecisionEvidence] | type[EvidenceValue],
    kwargs: dict[str, object],
) -> None:
    with pytest.raises(ValidationError):
        model.model_validate(kwargs)


def test_blank_evidence_key_is_rejected() -> None:
    with pytest.raises(ValidationError):
        DecisionEvidence(
            occurred_at_unix_ns=1,
            source_module="planner",
            action="stop",
            rule="safety_rule",
            evidence={" ": EvidenceValue(label="Evidence", value=0.0)},
        )


def test_legacy_record_without_version_is_migrated_without_data_loss() -> None:
    decision = load_decision_evidence(
        {
            "decision_id": "legacy_001",
            "occurred_at": "2026-07-18T15:42:18.392123456Z",
            "source_module": "old_planner",
            "action": "stop",
            "rule": "legacy_rule",
            "evidence": {
                "confidence": 0.65,
                "planner": {"state": "blocked", "retries": 0},
                "available": False,
                "note": None,
            },
        }
    )

    assert decision.schema_version == "0.2"
    assert decision.occurred_at_unix_ns == 1_784_389_338_392_123_456
    assert decision.spatial_context is None
    assert decision.evidence["confidence"] == EvidenceValue(
        label="confidence", value=0.65
    )
    assert decision.evidence["planner"].value == {
        "state": "blocked",
        "retries": 0,
    }
    assert decision.evidence["available"].value is False
    assert decision.evidence["note"].value is None


def test_v02_json_round_trip_preserves_model_and_zero_values() -> None:
    original = _complete_decision().model_copy(
        update={
            "evidence": {
                "velocity": EvidenceValue(
                    label="Velocity",
                    value=0.0,
                    observed=0.0,
                    threshold=0.0,
                    unit="m/s",
                )
            }
        }
    )

    restored = DecisionEvidence.model_validate_json(original.model_dump_json())

    assert restored == original
    assert restored.evidence["velocity"].value == 0.0
    assert restored.evidence["velocity"].observed == 0.0
    assert restored.evidence["velocity"].threshold == 0.0


def test_iso_timestamp_preserves_nanoseconds_and_timezone_offset() -> None:
    assert iso8601_to_unix_ns("1970-01-01T00:00:00.000000001Z") == 1
    assert iso8601_to_unix_ns("1970-01-01T01:00:00.000000001+01:00") == 1


def test_datetime_property_is_utc_and_truncates_to_microseconds() -> None:
    decision = _complete_decision()

    assert decision.occurred_at_datetime == datetime(
        2025, 7, 18, 15, 15, 38, 392123, tzinfo=UTC
    )


@pytest.mark.parametrize(
    "timestamp",
    [
        "1969-12-31T23:59:59.999999999Z",
        "2026-07-18T15:42:18",
        "2026-07-18 15:42:18Z",
        "2026-07-18T15:42:18.1234567890Z",
    ],
)
def test_invalid_or_pre_epoch_timestamp_is_rejected(timestamp: str) -> None:
    with pytest.raises(ValueError):
        iso8601_to_unix_ns(timestamp)


def test_missing_legacy_field_is_reported() -> None:
    with pytest.raises(ValueError, match="Legacy record missing: rule"):
        load_decision_evidence(
            {
                "occurred_at": "2026-07-18T15:42:18Z",
                "source_module": "old_planner",
                "action": "stop",
            }
        )


def test_unsupported_schema_version_is_rejected() -> None:
    with pytest.raises(ValueError, match="Unsupported schema version"):
        load_decision_evidence({"schema_version": "0.3"})


def test_contracts_are_immutable() -> None:
    decision = _complete_decision()

    with pytest.raises(ValidationError):
        decision.action = "continue"
