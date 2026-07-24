"""Versioned decision-evidence contracts and legacy migration."""

from __future__ import annotations

import re
from datetime import UTC, datetime
from typing import Annotated, Literal, TypeAlias

from pydantic import BaseModel, ConfigDict, Field, field_validator

from core.types.json_value import JsonValue

NonEmptyText: TypeAlias = Annotated[str, Field(min_length=1)]


class StrictFrozenModel(BaseModel):
    """Strict immutable base that rejects non-finite floating-point values."""

    model_config = ConfigDict(
        frozen=True,
        extra="forbid",
        strict=True,
        allow_inf_nan=False,
    )

    @field_validator("*", mode="before")
    @classmethod
    def validate_non_blank_text(cls, value: object) -> object:
        """Reject whitespace-only values in every direct text field."""
        if isinstance(value, str) and not value.strip():
            raise ValueError("Value must not be blank.")
        return value


class Pose2D(StrictFrozenModel):
    """Planar robot pose."""

    x: float
    y: float
    yaw: float


class SpatialContext(StrictFrozenModel):
    """Map and coordinate frame in which a decision occurred."""

    map_id: NonEmptyText
    frame_id: NonEmptyText = "map"
    pose: Pose2D


class EvidenceValue(StrictFrozenModel):
    """Diagnostic evidence together with human-readable comparison metadata."""

    label: NonEmptyText
    description: str | None = None
    value: JsonValue
    expected: JsonValue | None = None
    observed: JsonValue | None = None
    threshold: JsonValue | None = None
    unit: NonEmptyText | None = None


class DecisionEvidence(StrictFrozenModel):
    """Spatially and temporally anchored history of one robot decision."""

    schema_version: Literal["0.2"] = "0.2"
    decision_id: NonEmptyText | None = None
    occurred_at_unix_ns: int = Field(ge=0)
    source_module: NonEmptyText
    action: NonEmptyText
    rule: NonEmptyText
    evidence: dict[str, EvidenceValue]
    spatial_context: SpatialContext | None = None

    @property
    def occurred_at_datetime(self) -> datetime:
        """Return the UTC occurrence time, truncated to microsecond precision."""
        seconds, nanoseconds = divmod(self.occurred_at_unix_ns, 1_000_000_000)
        return datetime.fromtimestamp(seconds, tz=UTC).replace(
            microsecond=nanoseconds // 1_000
        )

    @field_validator("evidence")
    @classmethod
    def validate_evidence_keys(
        cls, evidence: dict[str, EvidenceValue]
    ) -> dict[str, EvidenceValue]:
        """Reject empty and whitespace-only evidence identifiers."""
        if any(not key.strip() for key in evidence):
            raise ValueError("Evidence keys must not be empty.")
        return evidence


ISO_NS_PATTERN = re.compile(
    r"^(?P<base>\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})"
    r"(?:\.(?P<fraction>\d{1,9}))?"
    r"(?P<zone>Z|[+-]\d{2}:\d{2})$"
)
EPOCH = datetime(1970, 1, 1, tzinfo=UTC)


def iso8601_to_unix_ns(value: str) -> int:
    """Convert a timezone-aware ISO-8601 timestamp to Unix nanoseconds."""
    match = ISO_NS_PATTERN.fullmatch(value)
    if not match:
        raise ValueError(f"Invalid timestamp: {value!r}")

    zone = match.group("zone")
    datetime_value = datetime.fromisoformat(
        match.group("base") + ("+00:00" if zone == "Z" else zone)
    )
    delta = datetime_value.astimezone(UTC) - EPOCH
    whole_seconds = delta.days * 86_400 + delta.seconds
    fraction = (match.group("fraction") or "").ljust(9, "0")
    unix_ns = whole_seconds * 1_000_000_000 + int(fraction)
    if unix_ns < 0:
        raise ValueError("Timestamps before the Unix epoch are not supported.")
    return unix_ns


def migrate_v01_to_v02(payload: dict[str, JsonValue]) -> dict[str, JsonValue]:
    """Migrate one legacy decision record without inventing evidence metadata."""
    for field_name in ("occurred_at", "source_module", "action", "rule"):
        if field_name not in payload:
            raise ValueError(f"Legacy record missing: {field_name}")

    occurred_at = payload["occurred_at"]
    if not isinstance(occurred_at, str):
        raise ValueError("Legacy field 'occurred_at' must be a string.")

    legacy_evidence = payload.get("evidence", {})
    if not isinstance(legacy_evidence, dict):
        raise ValueError("Legacy field 'evidence' must be an object.")

    migrated_evidence: dict[str, JsonValue] = {
        key: {"label": key, "value": value} for key, value in legacy_evidence.items()
    }
    return {
        "schema_version": "0.2",
        "decision_id": payload.get("decision_id"),
        "occurred_at_unix_ns": iso8601_to_unix_ns(occurred_at),
        "source_module": payload["source_module"],
        "action": payload["action"],
        "rule": payload["rule"],
        "evidence": migrated_evidence,
        "spatial_context": payload.get("spatial_context"),
    }


def load_decision_evidence(payload: dict[str, JsonValue]) -> DecisionEvidence:
    """Load current evidence or migrate an unversioned/v0.1 record first."""
    version = payload.get("schema_version", "0.1")
    if version == "0.1":
        payload = migrate_v01_to_v02(payload)
    elif version != "0.2":
        raise ValueError(f"Unsupported schema version: {version!r}")
    return DecisionEvidence.model_validate(payload)
