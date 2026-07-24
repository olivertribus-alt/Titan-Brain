"""Deterministic bounded telemetry blackbox for TB-EVAL-009B."""

from __future__ import annotations

import math
from collections import deque
from enum import StrEnum
from typing import Literal, Self

from pydantic import Field, model_validator

from core.types.incident import StrictFrozenModel


class BlackboxState(StrEnum):
    """Capture state of the bounded recorder."""

    ARMED = "armed"
    CAPTURING_POST_TRIGGER = "capturing_post_trigger"


class SnapshotTrigger(StrEnum):
    """Auditable causes that can freeze one incident window."""

    EMERGENCY_STOP = "emergency_stop"
    HARD_FAULT = "hard_fault"
    MANUAL = "manual"


class TelemetryBlackboxConfig(StrictFrozenModel):
    """Fixed memory and post-trigger bounds for one recorder."""

    schema_version: Literal["0.1"] = "0.1"
    policy_version: str = Field(
        default="TB-EVAL-009B-0.1.0",
        min_length=1,
    )
    capacity_frames: int = Field(default=500, gt=0, le=100_000)
    post_trigger_frames: int = Field(default=50, ge=0, le=100_000)

    @model_validator(mode="after")
    def validate_bounds(self) -> Self:
        """Keep every snapshot within an explicit deployment bound."""
        if self.post_trigger_frames > self.capacity_frames:
            raise ValueError("post-trigger frames must not exceed ring capacity")
        return self

    @property
    def maximum_snapshot_frames(self) -> int:
        """Return the strict upper bound for one frozen incident."""
        return self.capacity_frames + self.post_trigger_frames


class CommandTelemetry(StrictFrozenModel):
    """One finite planar command sampled from a ROS topic."""

    source_timestamp_ns: int = Field(ge=0)
    linear_x_mps: float
    angular_z_radps: float

    @model_validator(mode="after")
    def validate_finite_command(self) -> Self:
        """Reject non-finite actuator intent or output."""
        if not all(
            math.isfinite(value) for value in (self.linear_x_mps, self.angular_z_radps)
        ):
            raise ValueError("command telemetry must be finite")
        return self


class ArbitrationTelemetry(StrictFrozenModel):
    """Bounded fields from one arbitration status."""

    source_timestamp_ns: int = Field(ge=0)
    mode: int = Field(ge=0)
    reason: str = Field(min_length=1)
    active_source: str = Field(min_length=1)
    system_fault_state: int = Field(ge=0)
    correlation_id: str


class EnvelopeTelemetry(StrictFrozenModel):
    """Bounded fields from one dynamic envelope diagnostic."""

    source_timestamp_ns: int = Field(ge=0)
    state: int = Field(ge=0)
    reason: str = Field(min_length=1)
    scan_valid: bool
    distance_forward_m: float | None = Field(default=None, ge=0.0)
    distance_lateral_m: float | None = Field(default=None, ge=0.0)
    max_linear_velocity_mps: float = Field(ge=0.0)
    max_angular_velocity_radps: float = Field(ge=0.0)

    @model_validator(mode="after")
    def validate_finite_envelope(self) -> Self:
        """Reject numeric corruption while allowing absent distances."""
        values = [
            self.max_linear_velocity_mps,
            self.max_angular_velocity_radps,
        ]
        if self.distance_forward_m is not None:
            values.append(self.distance_forward_m)
        if self.distance_lateral_m is not None:
            values.append(self.distance_lateral_m)
        if any(not math.isfinite(value) for value in values):
            raise ValueError("envelope telemetry must be finite")
        return self


class LifecycleTelemetry(StrictFrozenModel):
    """Bounded fields from one lifecycle status."""

    source_timestamp_ns: int = Field(ge=0)
    state: int = Field(ge=0)
    reason: str = Field(min_length=1)
    is_faulted: bool
    recovery_active: bool
    max_linear_velocity_mps: float = Field(ge=0.0)
    max_angular_velocity_radps: float = Field(ge=0.0)

    @model_validator(mode="after")
    def validate_finite_authority(self) -> Self:
        """Require finite lifecycle authority."""
        if not all(
            math.isfinite(value)
            for value in (
                self.max_linear_velocity_mps,
                self.max_angular_velocity_radps,
            )
        ):
            raise ValueError("lifecycle telemetry must be finite")
        return self


class TelemetryBlackboxFrame(StrictFrozenModel):
    """One immutable 50 Hz correlated blackbox frame."""

    schema_version: Literal["0.1"] = "0.1"
    sequence_id: int = Field(gt=0)
    recorded_at_ns: int = Field(ge=0)
    teleoperation_command: CommandTelemetry | None = None
    autonomy_command: CommandTelemetry | None = None
    authoritative_command: CommandTelemetry | None = None
    arbitration: ArbitrationTelemetry | None = None
    envelope: EnvelopeTelemetry | None = None
    lifecycle: LifecycleTelemetry | None = None


class TelemetryBlackboxSnapshot(StrictFrozenModel):
    """Frozen pre/post incident window ready for structured export."""

    schema_version: Literal["0.1"] = "0.1"
    policy_version: str = Field(min_length=1)
    snapshot_id: int = Field(gt=0)
    trigger: SnapshotTrigger
    trigger_reason: str = Field(min_length=1)
    trigger_timestamp_ns: int = Field(ge=0)
    trigger_frame_index: int = Field(ge=0)
    frozen_at_ns: int = Field(ge=0)
    frames: tuple[TelemetryBlackboxFrame, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_window(self) -> Self:
        """Keep trigger location, ordering, and bounds internally coherent."""
        if self.trigger_frame_index >= len(self.frames):
            raise ValueError("trigger frame index is outside the snapshot")
        trigger_frame = self.frames[self.trigger_frame_index]
        if trigger_frame.recorded_at_ns != self.trigger_timestamp_ns:
            raise ValueError("trigger timestamp must match the trigger frame")
        if self.frozen_at_ns < self.trigger_timestamp_ns:
            raise ValueError("snapshot cannot freeze before its trigger")
        sequence_ids = [frame.sequence_id for frame in self.frames]
        if any(
            current >= following
            for current, following in zip(
                sequence_ids,
                sequence_ids[1:],
                strict=False,
            )
        ):
            raise ValueError("snapshot frame sequences must increase")
        timestamps = [frame.recorded_at_ns for frame in self.frames]
        if any(
            current > following
            for current, following in zip(
                timestamps,
                timestamps[1:],
                strict=False,
            )
        ):
            raise ValueError("snapshot frame timestamps must be monotonic")
        return self


class TelemetryBlackbox:
    """O(1)-per-tick bounded ring with deterministic incident freezing."""

    def __init__(
        self,
        config: TelemetryBlackboxConfig | None = None,
    ) -> None:
        self._config = config or TelemetryBlackboxConfig()
        self._ring: deque[TelemetryBlackboxFrame] = deque(
            maxlen=self._config.capacity_frames
        )
        self._state = BlackboxState.ARMED
        self._capture_frames: list[TelemetryBlackboxFrame] | None = None
        self._capture_trigger: SnapshotTrigger | None = None
        self._capture_reason: str | None = None
        self._capture_trigger_index: int | None = None
        self._remaining_post_trigger_frames = 0
        self._last_sequence_id: int | None = None
        self._last_recorded_at_ns: int | None = None
        self._snapshot_sequence_id = 0
        self._last_snapshot: TelemetryBlackboxSnapshot | None = None

    @property
    def config(self) -> TelemetryBlackboxConfig:
        """Return immutable capacity and policy configuration."""
        return self._config

    @property
    def state(self) -> BlackboxState:
        """Return whether the recorder is armed or capturing post-trigger."""
        return self._state

    @property
    def frame_count(self) -> int:
        """Return the number of retained rolling frames."""
        return len(self._ring)

    @property
    def remaining_post_trigger_frames(self) -> int:
        """Return the bounded post-trigger work still required."""
        return self._remaining_post_trigger_frames

    @property
    def last_snapshot(self) -> TelemetryBlackboxSnapshot | None:
        """Return the latest immutable frozen snapshot."""
        return self._last_snapshot

    def rolling_frames(self) -> tuple[TelemetryBlackboxFrame, ...]:
        """Return a stable copy of the currently retained pre-window."""
        return tuple(self._ring)

    def record(self, frame: TelemetryBlackboxFrame) -> None:
        """Append one frame with O(1) rolling-buffer work."""
        if (
            self._last_sequence_id is not None
            and frame.sequence_id <= self._last_sequence_id
        ):
            raise ValueError("blackbox frame sequence must strictly increase")
        if (
            self._last_recorded_at_ns is not None
            and frame.recorded_at_ns < self._last_recorded_at_ns
        ):
            raise ValueError("blackbox frame timestamp must not regress")

        self._ring.append(frame)
        self._last_sequence_id = frame.sequence_id
        self._last_recorded_at_ns = frame.recorded_at_ns

        if self._state is BlackboxState.CAPTURING_POST_TRIGGER:
            assert self._capture_frames is not None
            self._capture_frames.append(frame)
            self._remaining_post_trigger_frames -= 1
            if self._remaining_post_trigger_frames == 0:
                self._freeze(frame.recorded_at_ns)

    def trigger(
        self,
        trigger: SnapshotTrigger,
        reason: str,
    ) -> bool:
        """Start one bounded post-trigger capture from the latest frame."""
        checked_reason = str(reason).strip()
        if not checked_reason:
            raise ValueError("snapshot trigger reason must not be blank")
        if self._state is BlackboxState.CAPTURING_POST_TRIGGER:
            return False
        if not self._ring:
            return False

        self._state = BlackboxState.CAPTURING_POST_TRIGGER
        self._capture_frames = list(self._ring)
        self._capture_trigger = trigger
        self._capture_reason = checked_reason
        self._capture_trigger_index = len(self._capture_frames) - 1
        self._remaining_post_trigger_frames = self._config.post_trigger_frames
        if self._remaining_post_trigger_frames == 0:
            self._freeze(self._ring[-1].recorded_at_ns)
        return True

    def _freeze(self, frozen_at_ns: int) -> None:
        frames = self._capture_frames
        trigger = self._capture_trigger
        reason = self._capture_reason
        trigger_index = self._capture_trigger_index
        assert frames is not None
        assert trigger is not None
        assert reason is not None
        assert trigger_index is not None
        if len(frames) > self._config.maximum_snapshot_frames:
            raise RuntimeError("blackbox snapshot exceeded configured bound")

        self._snapshot_sequence_id += 1
        self._last_snapshot = TelemetryBlackboxSnapshot(
            policy_version=self._config.policy_version,
            snapshot_id=self._snapshot_sequence_id,
            trigger=trigger,
            trigger_reason=reason,
            trigger_timestamp_ns=frames[trigger_index].recorded_at_ns,
            trigger_frame_index=trigger_index,
            frozen_at_ns=frozen_at_ns,
            frames=tuple(frames),
        )
        self._state = BlackboxState.ARMED
        self._capture_frames = None
        self._capture_trigger = None
        self._capture_reason = None
        self._capture_trigger_index = None
        self._remaining_post_trigger_frames = 0

    def snapshot_json(self, *, indent: int | None = None) -> str:
        """Serialize the latest snapshot as deterministic structured JSON."""
        if self._last_snapshot is None:
            raise ValueError("no frozen blackbox snapshot is available")
        return self._last_snapshot.model_dump_json(indent=indent)


__all__ = [
    "ArbitrationTelemetry",
    "BlackboxState",
    "CommandTelemetry",
    "EnvelopeTelemetry",
    "LifecycleTelemetry",
    "SnapshotTrigger",
    "TelemetryBlackbox",
    "TelemetryBlackboxConfig",
    "TelemetryBlackboxFrame",
    "TelemetryBlackboxSnapshot",
]
