"""Atomic filesystem persistence for Titan Brain decision evidence."""

from __future__ import annotations

import os
import re
import tempfile
from pathlib import Path

from pydantic import ConfigDict, TypeAdapter, ValidationError

from core.types.incident import DecisionEvidence, load_decision_evidence
from core.types.json_value import JsonValue

_INCIDENT_ID_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")
_JSON_OBJECT_ADAPTER = TypeAdapter(
    dict[str, JsonValue],
    config=ConfigDict(strict=True, allow_inf_nan=False),
)


class IncidentStoreError(ValueError):
    """A decision could not be safely loaded from or written to a store."""


class IncidentConflictError(IncidentStoreError):
    """An incident ID already exists with different immutable content."""


def validate_incident_id(incident_id: str) -> str:
    """Validate an incident identifier before using it as a filename."""
    if not _INCIDENT_ID_PATTERN.fullmatch(incident_id):
        raise IncidentStoreError(f"Invalid incident ID: {incident_id!r}")
    return incident_id


class FileIncidentStore:
    """Store immutable incidents using atomic no-overwrite publication."""

    def __init__(self, root: Path) -> None:
        self._root = root

    @property
    def root(self) -> Path:
        """Return the store directory."""
        return self._root

    def _path_for(self, incident_id: str) -> Path:
        return self._root / f"{validate_incident_id(incident_id)}.json"

    def load(self, incident_id: str) -> DecisionEvidence:
        """Load and contract-validate one incident from this store."""
        incident_path = self._path_for(incident_id)
        try:
            serialized = incident_path.read_text(encoding="utf-8")
        except FileNotFoundError as error:
            raise IncidentStoreError(f"Incident not found: {incident_id}") from error
        except OSError as error:
            raise IncidentStoreError(
                f"Unable to read incident {incident_id}: {error}"
            ) from error

        try:
            payload = _JSON_OBJECT_ADAPTER.validate_json(serialized)
            decision = load_decision_evidence(payload)
        except (ValueError, ValidationError) as error:
            message = f"Invalid incident data for {incident_id}: {error}"
            raise IncidentStoreError(message) from error

        if decision.decision_id != incident_id:
            raise IncidentStoreError(
                "Incident ID mismatch: "
                f"requested {incident_id!r}, data contains {decision.decision_id!r}"
            )
        return decision

    def save(self, decision: DecisionEvidence) -> Path:
        """Atomically publish an incident without silently overwriting content."""
        if decision.decision_id is None:
            raise IncidentStoreError("A persisted incident requires a decision ID.")

        incident_path = self._path_for(decision.decision_id)
        self._root.mkdir(parents=True, exist_ok=True)
        serialized = decision.model_dump_json(indent=2) + "\n"

        descriptor, temporary_name = tempfile.mkstemp(
            dir=self._root,
            prefix=f".{decision.decision_id}-",
            suffix=".tmp",
            text=True,
        )
        temporary_path = Path(temporary_name)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as temporary_file:
                temporary_file.write(serialized)
                temporary_file.flush()
                os.fsync(temporary_file.fileno())

            try:
                os.link(temporary_path, incident_path)
            except FileExistsError:
                existing = self.load(decision.decision_id)
                if existing != decision:
                    raise IncidentConflictError(
                        f"Incident already exists with different content: "
                        f"{decision.decision_id}"
                    ) from None
        finally:
            temporary_path.unlink(missing_ok=True)

        return incident_path
