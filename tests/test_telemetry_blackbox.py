"""TB-EVAL-009B bounded ring, trigger, and snapshot tests."""

from __future__ import annotations

import hashlib
import json

import pytest
from pydantic import ValidationError

from core.telemetry_blackbox import (
    ArbitrationTelemetry,
    BlackboxState,
    CommandTelemetry,
    EnvelopeTelemetry,
    LifecycleTelemetry,
    SnapshotTrigger,
    TelemetryBlackbox,
    TelemetryBlackboxConfig,
    TelemetryBlackboxFrame,
)


def _config(
    *,
    capacity: int = 3,
    post_trigger: int = 2,
) -> TelemetryBlackboxConfig:
    return TelemetryBlackboxConfig(
        policy_version="TB-EVAL-009B-TEST",
        capacity_frames=capacity,
        post_trigger_frames=post_trigger,
    )


def _command(timestamp_ns: int, value: float) -> CommandTelemetry:
    return CommandTelemetry(
        source_timestamp_ns=timestamp_ns,
        linear_x_mps=value,
        angular_z_radps=-value,
    )


def _frame(
    sequence_id: int,
    *,
    timestamp_ns: int | None = None,
    complete: bool = True,
) -> TelemetryBlackboxFrame:
    effective_timestamp = (
        sequence_id * 20_000_000
        if timestamp_ns is None
        else timestamp_ns
    )
    if not complete:
        return TelemetryBlackboxFrame(
            sequence_id=sequence_id,
            recorded_at_ns=effective_timestamp,
        )
    return TelemetryBlackboxFrame(
        sequence_id=sequence_id,
        recorded_at_ns=effective_timestamp,
        teleoperation_command=_command(effective_timestamp, 0.5),
        autonomy_command=_command(effective_timestamp, 0.3),
        authoritative_command=_command(effective_timestamp, 0.2),
        arbitration=ArbitrationTelemetry(
            source_timestamp_ns=effective_timestamp,
            mode=1,
            reason="MOTION_ENVELOPE_CLAMPED",
            active_source="teleoperation",
            system_fault_state=0,
            correlation_id=f"frame-{sequence_id}",
        ),
        envelope=EnvelopeTelemetry(
            source_timestamp_ns=effective_timestamp,
            state=2,
            reason="CLEARANCE_LIMITED",
            scan_valid=True,
            distance_forward_m=0.8,
            distance_lateral_m=1.2,
            max_linear_velocity_mps=0.4,
            max_angular_velocity_radps=0.5,
        ),
        lifecycle=LifecycleTelemetry(
            source_timestamp_ns=effective_timestamp,
            state=1,
            reason="warning_zone",
            is_faulted=False,
            recovery_active=False,
            max_linear_velocity_mps=0.4,
            max_angular_velocity_radps=0.5,
        ),
    )


def test_default_contract_is_ten_seconds_at_fifty_hertz() -> None:
    config = TelemetryBlackboxConfig()

    assert config.policy_version == "TB-EVAL-009B-0.1.0"
    assert config.capacity_frames == 500
    assert config.post_trigger_frames == 50
    assert config.maximum_snapshot_frames == 550


def test_rolling_ring_evicts_oldest_frame_at_fixed_capacity() -> None:
    blackbox = TelemetryBlackbox(_config())
    for sequence_id in range(1, 6):
        blackbox.record(_frame(sequence_id))

    assert blackbox.frame_count == 3
    assert [
        frame.sequence_id for frame in blackbox.rolling_frames()
    ] == [3, 4, 5]
    assert blackbox.state.value == BlackboxState.ARMED.value


def test_trigger_freezes_exact_pre_and_post_window() -> None:
    blackbox = TelemetryBlackbox(_config())
    for sequence_id in range(1, 5):
        blackbox.record(_frame(sequence_id))

    accepted = blackbox.trigger(
        SnapshotTrigger.EMERGENCY_STOP,
        "lifecycle emergency transition",
    )
    blackbox.record(_frame(5))
    assert blackbox.state is BlackboxState.CAPTURING_POST_TRIGGER
    assert blackbox.remaining_post_trigger_frames == 1
    blackbox.record(_frame(6))

    snapshot = blackbox.last_snapshot
    assert accepted is True
    assert snapshot is not None
    assert snapshot.trigger is SnapshotTrigger.EMERGENCY_STOP
    assert snapshot.trigger_frame_index == 2
    assert snapshot.trigger_timestamp_ns == _frame(4).recorded_at_ns
    assert snapshot.frozen_at_ns == _frame(6).recorded_at_ns
    assert [frame.sequence_id for frame in snapshot.frames] == [2, 3, 4, 5, 6]
    assert len(snapshot.frames) == blackbox.config.maximum_snapshot_frames
    assert blackbox.state.value == BlackboxState.ARMED.value


def test_trigger_is_ignored_while_post_capture_is_active() -> None:
    blackbox = TelemetryBlackbox(_config())
    blackbox.record(_frame(1))
    assert blackbox.trigger(SnapshotTrigger.HARD_FAULT, "hardware") is True

    assert blackbox.trigger(SnapshotTrigger.MANUAL, "operator") is False
    blackbox.record(_frame(2))
    blackbox.record(_frame(3))

    snapshot = blackbox.last_snapshot
    assert snapshot is not None
    assert snapshot.trigger is SnapshotTrigger.HARD_FAULT
    assert snapshot.trigger_reason == "hardware"


def test_zero_post_window_freezes_synchronously() -> None:
    blackbox = TelemetryBlackbox(_config(post_trigger=0))
    blackbox.record(_frame(1))

    assert blackbox.trigger(SnapshotTrigger.MANUAL, "service request") is True

    snapshot = blackbox.last_snapshot
    assert snapshot is not None
    assert [frame.sequence_id for frame in snapshot.frames] == [1]
    assert snapshot.frozen_at_ns == snapshot.trigger_timestamp_ns
    assert blackbox.state is BlackboxState.ARMED


def test_empty_ring_cannot_create_an_ambiguous_snapshot() -> None:
    blackbox = TelemetryBlackbox(_config())

    assert blackbox.trigger(SnapshotTrigger.MANUAL, "empty") is False
    assert blackbox.last_snapshot is None


def test_later_snapshot_replaces_previous_with_increasing_id() -> None:
    blackbox = TelemetryBlackbox(_config(post_trigger=0))
    blackbox.record(_frame(1))
    blackbox.trigger(SnapshotTrigger.MANUAL, "first")
    first = blackbox.last_snapshot
    blackbox.record(_frame(2))
    blackbox.trigger(SnapshotTrigger.HARD_FAULT, "second")
    second = blackbox.last_snapshot

    assert first is not None
    assert second is not None
    assert first.snapshot_id == 1
    assert second.snapshot_id == 2
    assert second.trigger is SnapshotTrigger.HARD_FAULT


@pytest.mark.parametrize(
    "frame",
    [
        _frame(1, timestamp_ns=1),
        _frame(1, timestamp_ns=2),
    ],
)
def test_duplicate_or_regressed_sequence_is_rejected(
    frame: TelemetryBlackboxFrame,
) -> None:
    blackbox = TelemetryBlackbox(_config())
    blackbox.record(_frame(1, timestamp_ns=1))

    with pytest.raises(ValueError, match="sequence"):
        blackbox.record(frame)


def test_regressed_recording_clock_is_rejected() -> None:
    blackbox = TelemetryBlackbox(_config())
    blackbox.record(_frame(1, timestamp_ns=100))

    with pytest.raises(ValueError, match="timestamp"):
        blackbox.record(_frame(2, timestamp_ns=99))


def test_partial_frames_preserve_missing_transport_evidence() -> None:
    blackbox = TelemetryBlackbox(_config())
    blackbox.record(_frame(1, complete=False))

    frame = blackbox.rolling_frames()[0]
    assert frame.teleoperation_command is None
    assert frame.authoritative_command is None
    assert frame.arbitration is None
    assert frame.envelope is None
    assert frame.lifecycle is None


@pytest.mark.parametrize(
    "factory",
    [
        lambda: CommandTelemetry(
            source_timestamp_ns=0,
            linear_x_mps=float("nan"),
            angular_z_radps=0.0,
        ),
        lambda: EnvelopeTelemetry(
            source_timestamp_ns=0,
            state=0,
            reason="invalid",
            scan_valid=False,
            max_linear_velocity_mps=float("inf"),
            max_angular_velocity_radps=0.0,
        ),
        lambda: LifecycleTelemetry(
            source_timestamp_ns=0,
            state=0,
            reason="invalid",
            is_faulted=False,
            recovery_active=False,
            max_linear_velocity_mps=0.0,
            max_angular_velocity_radps=float("nan"),
        ),
    ],
)
def test_nonfinite_telemetry_is_rejected(factory: object) -> None:
    with pytest.raises(ValidationError):
        factory()  # type: ignore[operator]


@pytest.mark.parametrize(
    "values",
    [
        {"capacity_frames": 0},
        {"capacity_frames": 100_001},
        {"post_trigger_frames": -1},
        {"capacity_frames": 3, "post_trigger_frames": 4},
    ],
)
def test_invalid_memory_bounds_are_rejected(
    values: dict[str, int],
) -> None:
    with pytest.raises(ValidationError):
        TelemetryBlackboxConfig.model_validate(values)


def test_blank_trigger_reason_is_rejected() -> None:
    blackbox = TelemetryBlackbox(_config())
    blackbox.record(_frame(1))

    with pytest.raises(ValueError, match="reason"):
        blackbox.trigger(SnapshotTrigger.MANUAL, " ")


def test_snapshot_json_is_stable_structured_and_immutable() -> None:
    payloads: list[str] = []
    for _ in range(20):
        blackbox = TelemetryBlackbox(_config(post_trigger=0))
        blackbox.record(_frame(1))
        blackbox.trigger(SnapshotTrigger.MANUAL, "determinism")
        payloads.append(blackbox.snapshot_json(indent=2))

    hashes = {
        hashlib.sha256(payload.encode("utf-8")).hexdigest()
        for payload in payloads
    }
    decoded = json.loads(payloads[0])
    assert decoded["trigger"] == "manual"
    assert decoded["frames"][0]["sequence_id"] == 1
    assert len(set(payloads)) == 1
    assert len(hashes) == 1

    snapshot = blackbox.last_snapshot
    assert snapshot is not None
    with pytest.raises(ValidationError):
        snapshot.snapshot_id = 99


def test_snapshot_json_requires_a_frozen_window() -> None:
    with pytest.raises(ValueError, match="no frozen"):
        TelemetryBlackbox(_config()).snapshot_json()
