"""Dependency-free ROS observation boundary for TB-ROS-PoC-001A.

The module deliberately contains no ``rclpy`` dependency. A future ROS node is
responsible for QoS configuration and message extraction; this boundary owns
strict validation, frame policy, clock checks, watchdog state, and forwarding
accepted observations to the existing deterministic safety loop.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Self

from pydantic import Field, ValidationError, model_validator

from core.incident_store import FileIncidentStore
from core.safety import (
    SafetyDecisionResult,
    SafetyObservation,
    SafetyRuleConfig,
    run_safety_decision_loop,
)
from core.types.incident import Pose2D, StrictFrozenModel


class RosTime(StrictFrozenModel):
    """ROS-compatible timestamp split into seconds and nanoseconds."""

    sec: int = Field(ge=0)
    nanosec: int = Field(ge=0, lt=1_000_000_000)

    @property
    def timestamp_ns(self) -> int:
        """Return the timestamp as one non-negative nanosecond value."""
        return self.sec * 1_000_000_000 + self.nanosec


class RosHeader(StrictFrozenModel):
    """Minimal normalized subset of ``std_msgs/Header`` used by the core."""

    stamp: RosTime
    frame_id: str = Field(min_length=1)


class RosObservationMessage(StrictFrozenModel):
    """Normalized ROS-side message before conversion to ``SafetyObservation``."""

    header: RosHeader
    map_id: str = Field(min_length=1)
    pose: Pose2D
    clearance_m: float = Field(ge=0.0)
    confidence: float = Field(ge=0.0, le=1.0)
    sensor_id: str = Field(min_length=1)


class RosObservationAdapterConfig(StrictFrozenModel):
    """Versioned frame, freshness, and watchdog policy for one adapter."""

    policy_version: str = Field(default="TB-ROS-OBS-0.1.0", min_length=1)
    expected_frame_id: str = Field(default="map", min_length=1)
    max_observation_age_ns: int = Field(default=250_000_000, gt=0)
    max_future_skew_ns: int = Field(default=0, ge=0)
    watchdog_timeout_ns: int = Field(default=500_000_000, gt=0)

    @model_validator(mode="after")
    def validate_timeout_ordering(self) -> Self:
        """Keep the watchdog window at least as large as accepted data age."""
        if self.watchdog_timeout_ns < self.max_observation_age_ns:
            raise ValueError(
                "watchdog_timeout_ns must be greater than or equal to "
                "max_observation_age_ns"
            )
        return self


DEFAULT_ROS_OBSERVATION_ADAPTER_CONFIG = RosObservationAdapterConfig()


class ObservationAdaptationStatus(StrEnum):
    """Outcome of validating and adapting one transport message."""

    ACCEPTED = "accepted"
    INVALID_MESSAGE = "invalid_message"
    FRAME_MISMATCH = "frame_mismatch"
    STALE = "stale"
    FUTURE_TIMESTAMP = "future_timestamp"


class ObservationAdaptation(StrictFrozenModel):
    """Structured result that never invents data for rejected messages."""

    status: ObservationAdaptationStatus
    observation: SafetyObservation | None = None
    detail: str | None = None

    @property
    def accepted(self) -> bool:
        """Return whether a validated internal observation is available."""
        return self.status is ObservationAdaptationStatus.ACCEPTED

    @model_validator(mode="after")
    def validate_state(self) -> Self:
        """Enforce mutually exclusive accepted and rejected result shapes."""
        if self.accepted:
            if self.observation is None or self.detail is not None:
                raise ValueError(
                    "Accepted adaptation requires an observation and no detail."
                )
        elif self.observation is not None or self.detail is None:
            raise ValueError(
                "Rejected adaptation requires detail and no observation."
            )
        return self


class RosObservationProcessingResult(StrictFrozenModel):
    """Adapter result plus a safety decision only for accepted input."""

    adaptation: ObservationAdaptation
    decision: SafetyDecisionResult | None = None

    @model_validator(mode="after")
    def validate_state(self) -> Self:
        """Prevent evaluation results from being attached to rejected input."""
        if self.adaptation.accepted != (self.decision is not None):
            raise ValueError(
                "A decision must exist exactly when adaptation is accepted."
            )
        return self


class WatchdogStatus(StrEnum):
    """Health state derived from the last accepted observation."""

    NO_VALID_OBSERVATION = "no_valid_observation"
    HEALTHY = "healthy"
    TIMED_OUT = "timed_out"
    CLOCK_REGRESSION = "clock_regression"


class WatchdogReport(StrictFrozenModel):
    """Publishable watchdog state with no actuator authority."""

    status: WatchdogStatus
    elapsed_ns: int | None = Field(default=None, ge=0)
    timeout_ns: int = Field(gt=0)

    @property
    def healthy(self) -> bool:
        """Return whether valid observations are arriving within the timeout."""
        return self.status is WatchdogStatus.HEALTHY


def _validate_clock_ns(value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError("now_ns must be a non-negative integer")
    return value


def _rejected(
    status: ObservationAdaptationStatus,
    detail: str,
) -> ObservationAdaptation:
    return ObservationAdaptation(status=status, detail=detail)


def adapt_ros_observation(
    message: object,
    *,
    now_ns: int,
    config: RosObservationAdapterConfig = DEFAULT_ROS_OBSERVATION_ADAPTER_CONFIG,
) -> ObservationAdaptation:
    """Strictly validate and convert one normalized ROS-side message."""
    checked_now_ns = _validate_clock_ns(now_ns)
    try:
        ros_message = RosObservationMessage.model_validate(message)
    except ValidationError:
        return _rejected(
            ObservationAdaptationStatus.INVALID_MESSAGE,
            "Message failed strict RosObservationMessage validation.",
        )

    if ros_message.header.frame_id != config.expected_frame_id:
        return _rejected(
            ObservationAdaptationStatus.FRAME_MISMATCH,
            f"Expected frame {config.expected_frame_id!r}, received "
            f"{ros_message.header.frame_id!r}.",
        )

    timestamp_ns = ros_message.header.stamp.timestamp_ns
    age_ns = checked_now_ns - timestamp_ns
    if age_ns < -config.max_future_skew_ns:
        return _rejected(
            ObservationAdaptationStatus.FUTURE_TIMESTAMP,
            f"Observation timestamp is {-age_ns} ns in the future.",
        )
    if age_ns > config.max_observation_age_ns:
        return _rejected(
            ObservationAdaptationStatus.STALE,
            f"Observation age {age_ns} ns exceeds the "
            f"{config.max_observation_age_ns} ns limit.",
        )

    observation = SafetyObservation(
        timestamp_ns=timestamp_ns,
        map_id=ros_message.map_id,
        frame_id=ros_message.header.frame_id,
        pose=ros_message.pose,
        clearance_m=ros_message.clearance_m,
        confidence=ros_message.confidence,
        sensor_id=ros_message.sensor_id,
    )
    return ObservationAdaptation(
        status=ObservationAdaptationStatus.ACCEPTED,
        observation=observation,
    )


class RosObservationAdapter:
    """Stateful ROS boundary with watchdog and existing safety-loop integration."""

    def __init__(
        self,
        store: FileIncidentStore,
        *,
        config: RosObservationAdapterConfig = DEFAULT_ROS_OBSERVATION_ADAPTER_CONFIG,
        safety_rules: SafetyRuleConfig | None = None,
    ) -> None:
        self._store = store
        self._config = config
        self._safety_rules = safety_rules
        self._last_valid_received_at_ns: int | None = None

    @property
    def last_valid_received_at_ns(self) -> int | None:
        """Return receipt time of the last message accepted by the adapter."""
        return self._last_valid_received_at_ns

    def process(
        self,
        message: object,
        *,
        now_ns: int,
    ) -> RosObservationProcessingResult:
        """Validate one message and evaluate it only when adaptation succeeds."""
        checked_now_ns = _validate_clock_ns(now_ns)
        adaptation = adapt_ros_observation(
            message,
            now_ns=checked_now_ns,
            config=self._config,
        )
        if adaptation.observation is None:
            return RosObservationProcessingResult(adaptation=adaptation)

        if self._safety_rules is None:
            decision = run_safety_decision_loop(adaptation.observation, self._store)
        else:
            decision = run_safety_decision_loop(
                adaptation.observation,
                self._store,
                self._safety_rules,
            )
        self._last_valid_received_at_ns = checked_now_ns
        return RosObservationProcessingResult(
            adaptation=adaptation,
            decision=decision,
        )

    def watchdog(self, *, now_ns: int) -> WatchdogReport:
        """Report transport health without publishing an actuator command."""
        checked_now_ns = _validate_clock_ns(now_ns)
        last_valid = self._last_valid_received_at_ns
        if last_valid is None:
            return WatchdogReport(
                status=WatchdogStatus.NO_VALID_OBSERVATION,
                timeout_ns=self._config.watchdog_timeout_ns,
            )
        if checked_now_ns < last_valid:
            return WatchdogReport(
                status=WatchdogStatus.CLOCK_REGRESSION,
                timeout_ns=self._config.watchdog_timeout_ns,
            )

        elapsed_ns = checked_now_ns - last_valid
        status = (
            WatchdogStatus.HEALTHY
            if elapsed_ns <= self._config.watchdog_timeout_ns
            else WatchdogStatus.TIMED_OUT
        )
        return WatchdogReport(
            status=status,
            elapsed_ns=elapsed_ns,
            timeout_ns=self._config.watchdog_timeout_ns,
        )
