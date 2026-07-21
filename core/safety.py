"""Deterministic safety decision loop for TB-PoC-001."""

from __future__ import annotations

import hashlib
import json
from typing import Self, cast

from pydantic import Field, model_validator

from core.braking import (
    BrakingEnvelopeAssessment,
    BrakingEnvelopeConfig,
    DirectionalClearances,
    DirectionalClosingSpeeds,
    DirectionalSector,
    StoppingDistanceBreakdown,
    assess_braking_envelope,
)
from core.incident_store import FileIncidentStore
from core.types.incident import (
    DecisionEvidence,
    EvidenceValue,
    Pose2D,
    SpatialContext,
    StrictFrozenModel,
)


class PlanarVelocity(StrictFrozenModel):
    """Robot-relative planar velocity carried by a directional observation."""

    linear_x_mps: float
    linear_y_mps: float
    angular_z_radps: float


class DirectionalSafetyData(StrictFrozenModel):
    """Complete opt-in input required by the dynamic braking policy."""

    clearances: DirectionalClearances
    velocity: PlanarVelocity


class SafetyObservation(StrictFrozenModel):
    """One validated obstacle-clearance observation."""

    timestamp_ns: int = Field(ge=0)
    map_id: str = Field(min_length=1)
    frame_id: str = Field(min_length=1)
    pose: Pose2D
    clearance_m: float = Field(ge=0.0)
    confidence: float = Field(ge=0.0, le=1.0)
    sensor_id: str = Field(min_length=1)
    directional_data: DirectionalSafetyData | None = None

    @model_validator(mode="after")
    def validate_forward_clearance_alias(self) -> Self:
        """Reject contradictory legacy and directional frontal clearance."""
        if (
            self.directional_data is not None
            and self.clearance_m
            != self.directional_data.clearances.forward_m
        ):
            raise ValueError(
                "clearance_m must equal directional forward clearance"
            )
        return self


class SafetyRuleConfig(StrictFrozenModel):
    """Versioned thresholds used by the deterministic safety policy."""

    policy_version: str = Field(default="TB-SAFE-0.1.0", min_length=1)
    clearance_threshold_m: float = Field(default=0.50, gt=0.0)
    confidence_threshold: float = Field(default=0.70, ge=0.0, le=1.0)
    braking_envelope: BrakingEnvelopeConfig | None = None


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
            "observation": observation.model_dump(
                mode="json",
                exclude_none=True,
            ),
            "rules": rules.model_dump(mode="json", exclude_none=True),
        },
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    digest = hashlib.sha256(canonical_input).hexdigest()[:16]
    prefix = "incident" if is_incident else "decision"
    return f"{prefix}_{digest}"


def _base_evidence(
    observation: SafetyObservation,
    rules: SafetyRuleConfig,
) -> dict[str, EvidenceValue]:
    return {
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
    }


def _legacy_policy(
    observation: SafetyObservation,
    rules: SafetyRuleConfig,
) -> tuple[str, str, bool, dict[str, EvidenceValue]]:
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

    evidence = _base_evidence(observation, rules)
    evidence["clearance"] = EvidenceValue(
        label="Frontal Clearance",
        description="Distance to the nearest obstacle in the projected path.",
        value=observation.clearance_m,
        observed=observation.clearance_m,
        threshold=rules.clearance_threshold_m,
        unit="m",
    )
    return action, rule, is_incident, evidence


def _closing_speeds(velocity: PlanarVelocity) -> DirectionalClosingSpeeds:
    return DirectionalClosingSpeeds(
        forward_mps=max(velocity.linear_x_mps, 0.0),
        reverse_mps=max(-velocity.linear_x_mps, 0.0),
        left_mps=max(velocity.linear_y_mps, 0.0),
        right_mps=max(-velocity.linear_y_mps, 0.0),
    )


def _dynamic_evidence(
    observation: SafetyObservation,
    rules: SafetyRuleConfig,
    assessment: BrakingEnvelopeAssessment,
) -> dict[str, EvidenceValue]:
    evidence = _base_evidence(observation, rules)
    evidence["braking_policy_version"] = EvidenceValue(
        label="Braking Policy Version",
        value=assessment.policy_version,
    )
    limiting_sector = cast(DirectionalSector, assessment.limiting_sector)
    limiting = next(
        item
        for item in assessment.assessments
        if item.sector is limiting_sector
    )
    stopping = cast(StoppingDistanceBreakdown, limiting.stopping_distance)
    evidence["clearance"] = EvidenceValue(
        label=f"{limiting.sector.value.title()} Clearance",
        description="Clearance in the limiting active motion sector.",
        value=limiting.observed_clearance_m,
        observed=limiting.observed_clearance_m,
        threshold=stopping.required_clearance_m,
        unit="m",
    )
    evidence["closing_speed"] = EvidenceValue(
        label="Directional Closing Speed",
        value=stopping.closing_speed_mps,
        observed=stopping.closing_speed_mps,
        unit="m/s",
    )
    evidence["reaction_distance"] = EvidenceValue(
        label="Reaction Distance",
        value=stopping.reaction_distance_m,
        unit="m",
    )
    evidence["braking_distance"] = EvidenceValue(
        label="Braking Distance",
        value=stopping.braking_distance_m,
        unit="m",
    )
    evidence["clearance_margin"] = EvidenceValue(
        label="Configured Clearance Margin",
        value=stopping.clearance_margin_m,
        unit="m",
    )
    evidence["limiting_sector"] = EvidenceValue(
        label="Limiting Motion Sector",
        value=limiting.sector.value,
    )
    return evidence


def _dynamic_policy(
    observation: SafetyObservation,
    rules: SafetyRuleConfig,
) -> tuple[str, str, bool, dict[str, EvidenceValue]]:
    config = cast(BrakingEnvelopeConfig, rules.braking_envelope)
    directional = observation.directional_data
    if directional is None:
        evidence = _base_evidence(observation, rules)
        evidence["dynamic_input"] = EvidenceValue(
            label="Directional Braking Input",
            description="Dynamic mode requires clearances and planar velocity.",
            value="missing",
            expected="complete",
        )
        return "protective_stop", "EV-SAFE-DYN-03", True, evidence

    if directional.velocity.angular_z_radps != 0.0:
        evidence = _base_evidence(observation, rules)
        evidence["angular_velocity"] = EvidenceValue(
            label="Unsupported Angular Velocity",
            description=(
                "TB-EVAL-002A has no angular swept-footprint braking model."
            ),
            value=directional.velocity.angular_z_radps,
            observed=directional.velocity.angular_z_radps,
            threshold=0.0,
            unit="rad/s",
        )
        return "protective_stop", "EV-SAFE-DYN-04", True, evidence

    assessment = assess_braking_envelope(
        directional.clearances,
        _closing_speeds(directional.velocity),
        config,
    )
    if assessment.limiting_sector is None:
        return _legacy_policy(observation, rules)

    evidence = _dynamic_evidence(observation, rules, assessment)
    if assessment.safe_to_proceed:
        return "proceed", "EV-SAFE-DYN-00", False, evidence
    if observation.confidence >= rules.confidence_threshold:
        return "emergency_stop", "EV-SAFE-DYN-01", True, evidence
    return "protective_stop", "EV-SAFE-DYN-02", True, evidence


def evaluate_safety(
    observation: SafetyObservation,
    rules: SafetyRuleConfig = DEFAULT_SAFETY_RULES,
) -> SafetyDecisionResult:
    """Evaluate one observation as a deterministic side-effect-free function."""
    if rules.braking_envelope is None:
        action, rule, is_incident, evidence = _legacy_policy(observation, rules)
    else:
        action, rule, is_incident, evidence = _dynamic_policy(observation, rules)

    decision = DecisionEvidence(
        decision_id=_decision_id(observation, rules, is_incident=is_incident),
        occurred_at_unix_ns=observation.timestamp_ns,
        source_module="safety_decision_loop",
        action=action,
        rule=rule,
        evidence=evidence,
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
