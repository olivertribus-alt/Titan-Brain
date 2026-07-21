"""Tests for TB-ROS-PoC-001A dependency-free observation adapter."""

from __future__ import annotations

from pathlib import Path

import pytest
from pydantic import ValidationError

from core.adapters.ros_observation import (
    ObservationAdaptation,
    ObservationAdaptationStatus,
    RosObservationAdapter,
    RosObservationAdapterConfig,
    RosObservationProcessingResult,
    WatchdogStatus,
    adapt_ros_observation,
)
from core.incident_store import FileIncidentStore
from core.safety import SafetyRuleConfig, evaluate_safety

NOW_NS = 12_000_000_000


def _message(
    *,
    timestamp_ns: int = NOW_NS,
    frame_id: str = "map",
    clearance_m: float = 0.32,
    confidence: float = 0.95,
) -> dict[str, object]:
    seconds, nanoseconds = divmod(timestamp_ns, 1_000_000_000)
    return {
        "header": {
            "stamp": {"sec": seconds, "nanosec": nanoseconds},
            "frame_id": frame_id,
        },
        "map_id": "warehouse_zone_c",
        "pose": {"x": 42.15, "y": -12.33, "yaw": 1.5708},
        "clearance_m": clearance_m,
        "confidence": confidence,
        "sensor_id": "front_lidar",
    }


def test_valid_message_is_adapted_evaluated_and_persisted(tmp_path: Path) -> None:
    store = FileIncidentStore(tmp_path)
    adapter = RosObservationAdapter(store)

    result = adapter.process(_message(), now_ns=NOW_NS)

    assert result.adaptation.status is ObservationAdaptationStatus.ACCEPTED
    assert result.adaptation.observation is not None
    assert result.adaptation.observation.timestamp_ns == NOW_NS
    assert result.adaptation.observation.frame_id == "map"
    assert result.decision is not None
    assert result.decision.decision.action == "emergency_stop"
    assert result.decision.decision.decision_id is not None
    assert store.load(result.decision.decision.decision_id) == result.decision.decision
    assert adapter.last_valid_received_at_ns == NOW_NS


def test_safe_message_is_evaluated_without_creating_incident(tmp_path: Path) -> None:
    adapter = RosObservationAdapter(FileIncidentStore(tmp_path))

    result = adapter.process(
        _message(clearance_m=1.20),
        now_ns=NOW_NS,
    )

    assert result.decision is not None
    assert result.decision.decision.action == "proceed"
    assert not list(tmp_path.iterdir())


def test_explicit_safety_rules_are_forwarded_to_existing_evaluator(
    tmp_path: Path,
) -> None:
    rules = SafetyRuleConfig(clearance_threshold_m=0.25)
    adapter = RosObservationAdapter(
        FileIncidentStore(tmp_path),
        safety_rules=rules,
    )

    result = adapter.process(_message(clearance_m=0.32), now_ns=NOW_NS)

    assert result.decision is not None
    assert result.decision.decision.action == "proceed"
    assert result.decision.decision.evidence["clearance"].threshold == 0.25


@pytest.mark.parametrize(
    ("message", "now_ns", "expected_status"),
    [
        (
            _message(timestamp_ns=NOW_NS - 250_000_001),
            NOW_NS,
            ObservationAdaptationStatus.STALE,
        ),
        (
            _message(timestamp_ns=NOW_NS + 1),
            NOW_NS,
            ObservationAdaptationStatus.FUTURE_TIMESTAMP,
        ),
        (
            _message(frame_id="base_link"),
            NOW_NS,
            ObservationAdaptationStatus.FRAME_MISMATCH,
        ),
    ],
)
def test_clock_and_frame_policy_rejects_unusable_messages(
    message: dict[str, object],
    now_ns: int,
    expected_status: ObservationAdaptationStatus,
) -> None:
    result = adapt_ros_observation(message, now_ns=now_ns)

    assert result.status is expected_status
    assert result.accepted is False
    assert result.observation is None
    assert result.detail is not None


def test_age_and_watchdog_boundaries_are_inclusive(tmp_path: Path) -> None:
    config = RosObservationAdapterConfig(
        max_observation_age_ns=100,
        watchdog_timeout_ns=200,
    )
    adapter = RosObservationAdapter(FileIncidentStore(tmp_path), config=config)

    result = adapter.process(
        _message(timestamp_ns=NOW_NS - 100),
        now_ns=NOW_NS,
    )

    assert result.adaptation.accepted is True
    assert adapter.watchdog(now_ns=NOW_NS + 200).status is WatchdogStatus.HEALTHY
    assert adapter.watchdog(now_ns=NOW_NS + 201).status is WatchdogStatus.TIMED_OUT


@pytest.mark.parametrize(
    "mutate",
    [
        "missing_header",
        "invalid_nanosecond",
        "non_finite_clearance",
        "extra_field",
    ],
)
def test_invalid_transport_messages_are_controlled_rejections(
    mutate: str,
) -> None:
    message = _message()
    if mutate == "missing_header":
        del message["header"]
    elif mutate == "invalid_nanosecond":
        header = message["header"]
        assert isinstance(header, dict)
        stamp = header["stamp"]
        assert isinstance(stamp, dict)
        stamp["nanosec"] = 1_000_000_000
    elif mutate == "non_finite_clearance":
        message["clearance_m"] = float("nan")
    else:
        message["unexpected"] = True

    result = adapt_ros_observation(message, now_ns=NOW_NS)

    assert result.status is ObservationAdaptationStatus.INVALID_MESSAGE
    assert result.observation is None
    assert result.detail == "Message failed strict RosObservationMessage validation."


def test_rejected_messages_do_not_refresh_watchdog_or_create_decisions(
    tmp_path: Path,
) -> None:
    config = RosObservationAdapterConfig(
        max_observation_age_ns=100,
        watchdog_timeout_ns=200,
    )
    adapter = RosObservationAdapter(FileIncidentStore(tmp_path), config=config)
    accepted = adapter.process(_message(clearance_m=1.2), now_ns=NOW_NS)

    rejected = adapter.process(
        _message(frame_id="base_link"),
        now_ns=NOW_NS + 150,
    )

    assert accepted.decision is not None
    assert rejected.decision is None
    assert adapter.last_valid_received_at_ns == NOW_NS
    assert adapter.watchdog(now_ns=NOW_NS + 201).status is WatchdogStatus.TIMED_OUT
    assert not list(tmp_path.iterdir())


def test_watchdog_reports_no_data_and_clock_regression(tmp_path: Path) -> None:
    adapter = RosObservationAdapter(FileIncidentStore(tmp_path))

    initial = adapter.watchdog(now_ns=NOW_NS)
    adapter.process(_message(clearance_m=1.2), now_ns=NOW_NS)
    regression = adapter.watchdog(now_ns=NOW_NS - 1)

    assert initial.status is WatchdogStatus.NO_VALID_OBSERVATION
    assert initial.healthy is False
    assert initial.elapsed_ns is None
    assert regression.status is WatchdogStatus.CLOCK_REGRESSION
    assert regression.healthy is False


@pytest.mark.parametrize("now_ns", [-1, True, 1.5])
def test_invalid_adapter_clock_is_programming_error(now_ns: object) -> None:
    with pytest.raises(ValueError, match="now_ns"):
        adapt_ros_observation(_message(), now_ns=now_ns)  # type: ignore[arg-type]


def test_adapter_config_rejects_incoherent_timeout_ordering() -> None:
    with pytest.raises(ValidationError, match="watchdog_timeout_ns"):
        RosObservationAdapterConfig(
            max_observation_age_ns=200,
            watchdog_timeout_ns=199,
        )


def test_result_models_reject_internally_inconsistent_states() -> None:
    accepted = adapt_ros_observation(_message(), now_ns=NOW_NS)
    assert accepted.observation is not None
    rejected = adapt_ros_observation(
        _message(frame_id="base_link"),
        now_ns=NOW_NS,
    )
    decision = evaluate_safety(accepted.observation)

    with pytest.raises(ValidationError, match="Accepted adaptation"):
        ObservationAdaptation(status=ObservationAdaptationStatus.ACCEPTED)
    with pytest.raises(ValidationError, match="Rejected adaptation"):
        ObservationAdaptation(
            status=ObservationAdaptationStatus.FRAME_MISMATCH,
            observation=accepted.observation,
            detail="wrong frame",
        )
    with pytest.raises(ValidationError, match="decision must exist"):
        RosObservationProcessingResult(adaptation=accepted)
    with pytest.raises(ValidationError, match="decision must exist"):
        RosObservationProcessingResult(adaptation=rejected, decision=decision)
