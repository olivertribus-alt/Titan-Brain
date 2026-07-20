"""Deterministic safety decision loop for TB-PoC-001."""

from __future__ import annotations

import hashlib
import json

from pydantic import Field

from core.incident_store import FileIncidentStore
from core.types.incident import (
    DecisionEvidence,
    EvidenceValue,
    Pose2D,
    SpatialContext,
    StrictFrozenModel,
)


class SafetyObservation(StrictFrozenModel):
    """One validated obstacle-clearance observation."""

    timestamp_ns: int = Field(ge=0)
    map_id: str = Field(min_length=1)
    frame_id: str = Field(min_length=1)
    pose: Pose2D
    clearance_m: float = Field(ge=0.0)
    confidence: float = Field(ge=0.0, le=1.0)
    sensor_id: str = Field(min_length=1)


class SafetyRuleConfig(StrictFrozenModel):
    """Versioned thresholds used by the deterministic safety policy."""

    policy_version: str = Field(default="TB-SAFE-0.1.0", min_length=1)
    clearance_threshold_m: float = Field(default=0.50, gt=0.0)
    confidence_threshold: float = Field(default=0.70, ge=0.0, le=1.0)


class SafetyDecisionResult(StrictFrozenModel):
    """Pure policy result plus its persistence classification."""

    decision: DecisionEvidence
    is_incident: bool


DEFAULT_SAFETY_RULES = SafetyRuleConfig()


def _decision_id(
    observation: SafetyObservation,
    rules: SafetyRuleConfig,
    *,
    is_incident: bool,
) -> str:
    canonical_input = json.dumps(
        {
            "observation": observation.model_dump(mode="json"),
            "rules": rules.model_dump(mode="json"),
        },
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    digest = hashlib.sha256(canonical_input).hexdigest()[:16]
    prefix = "incident" if is_incident else "decision"
    return f"{prefix}_{digest}"


def evaluate_safety(
    observation: SafetyObservation,
    rules: SafetyRuleConfig = DEFAULT_SAFETY_RULES,
) -> SafetyDecisionResult:
    """Evaluate one observation as a deterministic side-effect-free function."""
    clearance_violated = observation.clearance_m < rules.clearance_threshold_m
    confidence_sufficient = observation.confidence >= rules.confidence_threshold

    if clearance_violated and confidence_sufficient:
        action = "emergency_stop"
        rule = "EV-SAFE-01"
        is_incident = True
    elif clearance_violated:
        action = "protective_stop"
        rule = "EV-SAFE-02"
        is_incident = True
    else:
        action = "proceed"
        rule = "EV-SAFE-00"
        is_incident = False

    decision = DecisionEvidence(
        decision_id=_decision_id(observation, rules, is_incident=is_incident),
        occurred_at_unix_ns=observation.timestamp_ns,
        source_module="safety_decision_loop",
        action=action,
        rule=rule,
        evidence={
            "clearance": EvidenceValue(
                label="Frontal Clearance",
                description="Distance to the nearest obstacle in the projected path.",
                value=observation.clearance_m,
                observed=observation.clearance_m,
                threshold=rules.clearance_threshold_m,
                unit="m",
            ),
            "confidence": EvidenceValue(
                label="Obstacle Confidence",
                description="Confidence assigned to the obstacle measurement.",
                value=observation.confidence,
                observed=observation.confidence,
                threshold=rules.confidence_threshold,
                unit="probability",
            ),
            "sensor": EvidenceValue(
                label="Source Sensor",
                value=observation.sensor_id,
            ),
            "policy_version": EvidenceValue(
                label="Safety Policy Version",
                value=rules.policy_version,
            ),
        },
        spatial_context=SpatialContext(
            map_id=observation.map_id,
            frame_id=observation.frame_id,
            pose=observation.pose,
        ),
    )
    return SafetyDecisionResult(decision=decision, is_incident=is_incident)


def run_safety_decision_loop(
    observation: SafetyObservation,
    store: FileIncidentStore,
    rules: SafetyRuleConfig = DEFAULT_SAFETY_RULES,
) -> SafetyDecisionResult:
    """Evaluate one observation and persist only safety incidents."""
    result = evaluate_safety(observation, rules)
    if result.is_incident:
        store.save(result.decision)
    return result
