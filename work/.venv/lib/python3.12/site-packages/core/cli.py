"""Command-line interface for Titan Brain diagnostic replay."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path

from pydantic import ValidationError

from core.incident_store import FileIncidentStore, IncidentStoreError
from core.safety import SafetyObservation, run_safety_decision_loop
from core.types.incident import DecisionEvidence
from core.types.json_value import JsonValue

_INCIDENT_DIRECTORY = Path(__file__).parent / "data" / "incidents"
_SEPARATOR = "-" * 50


def load_bundled_decision(incident_id: str) -> DecisionEvidence:
    """Load and validate one decision record bundled with Titan Brain."""
    return FileIncidentStore(_INCIDENT_DIRECTORY).load(incident_id)


def _format_json_value(value: JsonValue) -> str:
    """Format a JSON value compactly for diagnostic output."""
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, (int, float)):
        return f"{value:.2f}"
    if isinstance(value, str):
        return value
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def _format_evidence(decision: DecisionEvidence) -> list[str]:
    """Format evidence in insertion order using observed values when available."""
    lines: list[str] = []
    for evidence in decision.evidence.values():
        displayed_value = evidence.observed
        if displayed_value is None:
            displayed_value = evidence.value

        unit = f" {evidence.unit}" if evidence.unit is not None else ""
        line = f"- {evidence.label}: {_format_json_value(displayed_value)}{unit}"
        if evidence.threshold is not None:
            threshold = _format_json_value(evidence.threshold)
            line += f" (Threshold: {threshold}{unit})"
        lines.append(line)
    return lines


def format_decision_replay(decision: DecisionEvidence) -> str:
    """Render one decision using the stable v0.2 human diagnostic layout."""
    timestamp = decision.occurred_at_datetime.strftime(
        "%Y-%m-%d %H:%M:%S.%f UTC"
    )
    if decision.spatial_context is None:
        location = "unavailable"
    else:
        spatial = decision.spatial_context
        location = (
            f"{spatial.map_id} "
            f"(x: {spatial.pose.x:.2f}, y: {spatial.pose.y:.2f}, "
            f"yaw: {spatial.pose.yaw:.2f})"
        )

    lines = [
        f"[INCIDENT] {decision.decision_id or 'unidentified'}",
        _SEPARATOR,
        f"TIME:     {timestamp}",
        f"LOCATION: {location}",
        "",
        "DECISION:",
        f"Module:   {decision.source_module}",
        f"Action:   {decision.action}",
        f"Rule:     {decision.rule}",
        "",
        "EVIDENCE:",
        *_format_evidence(decision),
        _SEPARATOR,
    ]
    return "\n".join(lines)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="tb",
        description="Replay Titan Brain diagnostic decisions.",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    replay = commands.add_parser("replay", help="replay one diagnostic incident")
    replay.add_argument("incident_id", help="incident identifier, e.g. incident_042")
    replay.add_argument(
        "--store",
        type=Path,
        help="read the incident from this directory instead of bundled data",
    )
    safety = commands.add_parser(
        "safety-evaluate",
        help="evaluate one SafetyObservation JSON file",
    )
    safety.add_argument("observation", type=Path, help="observation JSON file")
    safety.add_argument(
        "--store",
        type=Path,
        required=True,
        help="directory for atomically persisted safety incidents",
    )
    return parser


def _run_replay(arguments: argparse.Namespace) -> int:
    store_path = arguments.store or _INCIDENT_DIRECTORY
    try:
        decision = FileIncidentStore(store_path).load(arguments.incident_id)
    except IncidentStoreError as error:
        print(f"tb replay: error: {error}", file=sys.stderr)
        return 2

    print(format_decision_replay(decision))
    return 0


def _run_safety_evaluation(arguments: argparse.Namespace) -> int:
    try:
        serialized = arguments.observation.read_text(encoding="utf-8")
        observation = SafetyObservation.model_validate_json(serialized)
        store = FileIncidentStore(arguments.store)
        result = run_safety_decision_loop(observation, store)
    except (OSError, ValidationError, IncidentStoreError) as error:
        print(f"tb safety-evaluate: error: {error}", file=sys.stderr)
        return 2

    print(f"[DECISION] {result.decision.decision_id}")
    print(f"ACTION:   {result.decision.action}")
    print(f"RULE:     {result.decision.rule}")
    if result.is_incident:
        incident_path = store.root / f"{result.decision.decision_id}.json"
        print(f"INCIDENT: saved {incident_path}")
    else:
        print("INCIDENT: not stored")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    """Run the Titan Brain command-line interface."""
    arguments = _build_parser().parse_args(argv)
    if arguments.command == "replay":
        return _run_replay(arguments)
    if arguments.command == "safety-evaluate":
        return _run_safety_evaluation(arguments)
    raise AssertionError(f"Unhandled command: {arguments.command}")


if __name__ == "__main__":
    raise SystemExit(main())
