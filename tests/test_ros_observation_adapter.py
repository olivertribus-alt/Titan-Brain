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
from core.braking import BrakingEnvelopeConfig
from core.incident_store import FileIncidentStore
from core.safety import SafetyRuleConfig, evaluate_safety
from core.stability import EvaluatorState, SafetyStabilityFilter, StabilityConfig

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


def _dynamic_message(
    *,
    clearance_m: float,
    linear_x_mps: float,
) -> dict[str, object]:
    message = _message(clearance_m=clearance_m)
    message["directional_data"] = {
        "clearances": {
            "forward_m": clearance_m,
            "reverse_m": 10.0,
            "left_m": 10.0,
            "right_m": 10.0,
        },
        "velocity": {
            "linear_x_mps": linear_x_mps,
            "linear_y_mps": 0.0,
            "angular_z_radps": 0.0,
        },
    }
    return message


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
    assert result.instantaneous_decision is None
    assert result.stability_transition is None
    assert adapter.last_stability_transition is None
    assert not list(tmp_path.iterdir())


def test_adapter_applies_hysteresis_and_hold_before_releasing_stop(
    tmp_path: Path,
) -> None:
    adapter = RosObservationAdapter(
        FileIncidentStore(tmp_path),
        stability_config=StabilityConfig(
            policy_version="TB-STABILITY-0.1.0",
            clearance_hysteresis_m=0.1,
            recovery_hold_time_ns=200,
        ),
    )

    danger = adapter.process(_message(), now_ns=NOW_NS)
    holding = adapter.process(
        _message(timestamp_ns=NOW_NS + 1, clearance_m=0.7),
        now_ns=NOW_NS + 1,
    )
    released = adapter.process(
        _message(timestamp_ns=NOW_NS + 201, clearance_m=0.7),
        now_ns=NOW_NS + 201,
    )

    assert danger.stability_transition is not None
    assert danger.stability_transition.state is EvaluatorState.E_STOP
    assert holding.instantaneous_decision is not None
    assert holding.instantaneous_decision.decision.action == "proceed"
    assert holding.decision is not None
    assert holding.decision.decision.action == "emergency_stop"
    assert holding.stability_transition is not None
    assert holding.stability_transition.state is EvaluatorState.RECOVERY_HOLDING
    assert released.decision is not None
    assert released.decision.decision.action == "proceed"
    assert released.stability_transition is not None
    assert released.stability_transition.state is EvaluatorState.OK
    assert adapter.last_stability_transition == released.stability_transition
    assert len(list(tmp_path.glob("*.json"))) == 2


def test_adapter_noise_cancels_recovery_hold_without_refreshing_start(
    tmp_path: Path,
) -> None:
    adapter = RosObservationAdapter(
        FileIncidentStore(tmp_path),
        stability_config=StabilityConfig(
            policy_version="TB-STABILITY-0.1.0",
            clearance_hysteresis_m=0.1,
            recovery_hold_time_ns=200,
        ),
    )
    adapter.process(_message(), now_ns=NOW_NS)
    started = adapter.process(
        _message(timestamp_ns=NOW_NS + 1, clearance_m=0.7),
        now_ns=NOW_NS + 1,
    )
    cancelled = adapter.process(
        _message(timestamp_ns=NOW_NS + 100, clearance_m=0.59),
        now_ns=NOW_NS + 100,
    )
    restarted = adapter.process(
        _message(timestamp_ns=NOW_NS + 150, clearance_m=0.7),
        now_ns=NOW_NS + 150,
    )

    assert started.stability_transition is not None
    assert started.stability_transition.recovery_started_at_ns == NOW_NS + 1
    assert cancelled.stability_transition is not None
    assert cancelled.stability_transition.state is EvaluatorState.E_STOP
    assert cancelled.stability_transition.recovery_started_at_ns is None
    assert restarted.stability_transition is not None
    assert restarted.stability_transition.recovery_started_at_ns == NOW_NS + 150


def test_adapter_watchdog_gap_restarts_full_recovery_window(
    tmp_path: Path,
) -> None:
    adapter = RosObservationAdapter(
        FileIncidentStore(tmp_path),
        config=RosObservationAdapterConfig(
            max_observation_age_ns=100,
            watchdog_timeout_ns=100,
        ),
        stability_config=StabilityConfig(
            policy_version="TB-STABILITY-0.1.0",
            clearance_hysteresis_m=0.1,
            recovery_hold_time_ns=100,
        ),
    )
    adapter.process(_message(), now_ns=NOW_NS)
    adapter.process(
        _message(timestamp_ns=NOW_NS + 1, clearance_m=0.7),
        now_ns=NOW_NS + 1,
    )

    restarted = adapter.process(
        _message(timestamp_ns=NOW_NS + 102, clearance_m=0.7),
        now_ns=NOW_NS + 102,
    )
    released = adapter.process(
        _message(timestamp_ns=NOW_NS + 202, clearance_m=0.7),
        now_ns=NOW_NS + 202,
    )

    assert restarted.stability_transition is not None
    assert restarted.stability_transition.state is EvaluatorState.RECOVERY_HOLDING
    assert restarted.stability_transition.recovery_started_at_ns == NOW_NS + 102
    assert restarted.decision is not None
    assert restarted.decision.decision.action == "emergency_stop"
    assert released.stability_transition is not None
    assert released.stability_transition.state is EvaluatorState.OK


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


def test_dynamic_braking_rules_are_forwarded_to_evaluator(
    tmp_path: Path,
) -> None:
    rules = SafetyRuleConfig(
        policy_version="TB-SAFE-0.2.0",
        braking_envelope=BrakingEnvelopeConfig(
            policy_version="TB-BRAKE-0.2.0",
            reaction_time_ns=250_000_000,
            assured_deceleration_mps2=1.0,
            clearance_margin_m=0.5,
        ),
    )
    adapter = RosObservationAdapter(
        FileIncidentStore(tmp_path),
        safety_rules=rules,
    )

    result = adapter.process(
        _dynamic_message(clearance_m=0.8, linear_x_mps=0.8),
        now_ns=NOW_NS,
    )

    assert result.decision is not None
    assert result.decision.decision.action == "emergency_stop"
    assert result.decision.decision.rule == "EV-SAFE-DYN-01"


def test_dynamic_mode_fails_closed_for_legacy_message(tmp_path: Path) -> None:
    rules = SafetyRuleConfig(
        braking_envelope=BrakingEnvelopeConfig(
            policy_version="TB-BRAKE-0.2.0",
            reaction_time_ns=250_000_000,
            assured_deceleration_mps2=1.0,
            clearance_margin_m=0.5,
        )
    )
    adapter = RosObservationAdapter(
        FileIncidentStore(tmp_path),
        safety_rules=rules,
    )

    result = adapter.process(_message(clearance_m=1.2), now_ns=NOW_NS)

    assert result.decision is not None
    assert result.decision.decision.action == "protective_stop"
    assert result.decision.decision.rule == "EV-SAFE-DYN-03"


def test_contradictory_dynamic_clearance_is_rejected_before_evaluation(
    tmp_path: Path,
) -> None:
    adapter = RosObservationAdapter(FileIncidentStore(tmp_path))
    message = _dynamic_message(clearance_m=0.8, linear_x_mps=0.1)
    directional = message["directional_data"]
    assert isinstance(directional, dict)
    clearances = directional["clearances"]
    assert isinstance(clearances, dict)
    clearances["forward_m"] = 0.9

    result = adapter.process(message, now_ns=NOW_NS)

    assert result.adaptation.status is ObservationAdaptationStatus.INVALID_MESSAGE
    assert result.adaptation.detail == "Message contains contradictory safety data."
    assert result.decision is None
    assert adapter.last_valid_received_at_ns is None


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
    stabilized = SafetyStabilityFilter(
        StabilityConfig(
            policy_version="TB-STABILITY-0.1.0",
            clearance_hysteresis_m=0.1,
            recovery_hold_time_ns=200,
        )
    ).process(decision, now_ns=NOW_NS)

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
    with pytest.raises(ValidationError, match="evidence must be complete"):
        RosObservationProcessingResult(
            adaptation=accepted,
            decision=decision,
            instantaneous_decision=decision,
        )
    with pytest.raises(ValidationError, match="Rejected input"):
        RosObservationProcessingResult(
            adaptation=rejected,
            instantaneous_decision=decision,
            stability_transition=stabilized.transition,
        )
