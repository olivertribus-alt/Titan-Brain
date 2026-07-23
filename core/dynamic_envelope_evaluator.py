"""Deterministic TB-EVAL-008A dynamic motion-envelope evaluator."""

from __future__ import annotations

import math
from enum import StrEnum
from typing import Literal, Self

from pydantic import Field, model_validator

from core.braking import BrakingEnvelopeConfig, calculate_stopping_distance
from core.motion_envelope import calculate_permitted_speed_limit
from core.types.incident import StrictFrozenModel

LINEAR_ZERO_CLAMP_MPS = 1e-5
ANGULAR_ZERO_CLAMP_RADPS = 1e-5


class EnvelopeSource(StrEnum):
    """Bounded sensor sources accepted by the evaluator."""

    NONE = "none"
    LIDAR = "lidar"


class EnvelopeState(StrEnum):
    """Motion-authority state emitted with every evaluation."""

    FAIL_CLOSED = "fail_closed"
    PROTECTIVE_STOP = "protective_stop"
    LIMITED = "limited"
    NOMINAL = "nominal"


class LimitingZone(StrEnum):
    """Geometric or control-plane input currently limiting authority."""

    NONE = "none"
    FORWARD = "forward"
    LATERAL = "lateral"
    SENSOR = "sensor"
    SYSTEM_FAULT = "system_fault"
    TIMING = "timing"


class EnvelopeConfig(StrictFrozenModel):
    """Physical assumptions and policy limits for one evaluator."""

    schema_version: Literal["0.1"] = "0.1"
    policy_version: str = Field(
        default="TB-EVAL-008A-0.1.0",
        min_length=1,
    )
    reaction_time_ns: int = Field(default=100_000_000, gt=0)
    assured_deceleration_mps2: float = Field(default=1.5, gt=0.0)
    clearance_margin_m: float = Field(default=0.30, ge=0.0)
    nominal_linear_velocity_mps: float = Field(default=1.0, gt=0.0)
    nominal_angular_velocity_radps: float = Field(default=1.0, gt=0.0)
    angular_swept_radius_m: float = Field(default=0.45, gt=0.0)
    confidence_threshold: float = Field(default=0.5, ge=0.0, le=1.0)
    max_sensor_age_s: float = Field(default=0.20, gt=0.0)

    @model_validator(mode="after")
    def validate_finite_values(self) -> Self:
        """Reject infinities and NaNs that satisfy ordinary range checks."""
        values = (
            self.assured_deceleration_mps2,
            self.clearance_margin_m,
            self.nominal_linear_velocity_mps,
            self.nominal_angular_velocity_radps,
            self.angular_swept_radius_m,
            self.confidence_threshold,
            self.max_sensor_age_s,
        )
        if any(not math.isfinite(value) for value in values):
            raise ValueError("envelope configuration must be finite")
        return self


class SensorFrame(StrictFrozenModel):
    """Constant-size evidence extracted from one bounded scan."""

    distance_forward_m: float = Field(ge=0.0)
    distance_lateral_m: float = Field(ge=0.0)
    confidence: float = Field(ge=0.0, le=1.0)
    source: EnvelopeSource
    age_s: float = Field(ge=0.0)

    @model_validator(mode="after")
    def validate_finite_values(self) -> Self:
        """Keep invalid sensor arithmetic out of the safety calculation."""
        values = (
            self.distance_forward_m,
            self.distance_lateral_m,
            self.confidence,
            self.age_s,
        )
        if any(not math.isfinite(value) for value in values):
            raise ValueError("sensor frame must contain finite values")
        if self.source is EnvelopeSource.NONE:
            raise ValueError("sensor frame requires an explicit sensor source")
        return self


class EnvelopeResult(StrictFrozenModel):
    """Immutable dynamic authority and auditable braking evidence."""

    schema_version: Literal["0.1"] = "0.1"
    policy_version: str = Field(min_length=1)
    max_linear_velocity_mps: float = Field(ge=0.0)
    max_angular_velocity_radps: float = Field(ge=0.0)
    linear_stopping_distance_m: float = Field(ge=0.0)
    angular_stopping_distance_m: float = Field(ge=0.0)
    distance_forward_m: float | None = Field(default=None, ge=0.0)
    distance_lateral_m: float | None = Field(default=None, ge=0.0)
    state: EnvelopeState
    source: EnvelopeSource
    limiting_zone: LimitingZone
    reason: str = Field(min_length=1)

    @property
    def stop_only(self) -> bool:
        """Return whether this result grants no motion authority."""
        return (
            self.max_linear_velocity_mps == 0.0
            and self.max_angular_velocity_radps == 0.0
        )


class DynamicEnvelopeEvaluator:
    """Evaluate constant-size distance evidence with bounded O(1) work."""

    def __init__(self, config: EnvelopeConfig | None = None) -> None:
        self._config = config or EnvelopeConfig()
        self._braking_config = BrakingEnvelopeConfig(
            policy_version=self._config.policy_version,
            reaction_time_ns=self._config.reaction_time_ns,
            assured_deceleration_mps2=(
                self._config.assured_deceleration_mps2
            ),
            clearance_margin_m=self._config.clearance_margin_m,
        )

    @property
    def config(self) -> EnvelopeConfig:
        """Expose immutable configuration for adapters and diagnostics."""
        return self._config

    def stopping_distance(self, speed_mps: float) -> float:
        """Return required linear clearance for a finite non-negative speed."""
        if (
            isinstance(speed_mps, bool)
            or not isinstance(speed_mps, (int, float))
            or not math.isfinite(float(speed_mps))
            or speed_mps < 0.0
        ):
            raise ValueError("speed_mps must be finite and non-negative")
        if speed_mps == 0.0:
            return 0.0
        return calculate_stopping_distance(
            float(speed_mps),
            self._braking_config,
        ).required_clearance_m

    def fail_closed(
        self,
        reason: str,
        *,
        limiting_zone: LimitingZone = LimitingZone.SENSOR,
    ) -> EnvelopeResult:
        """Create a deterministic zero-authority result."""
        checked_reason = str(reason).strip()
        if not checked_reason:
            raise ValueError("fail-closed reason must not be blank")
        return EnvelopeResult(
            policy_version=self._config.policy_version,
            max_linear_velocity_mps=0.0,
            max_angular_velocity_radps=0.0,
            linear_stopping_distance_m=0.0,
            angular_stopping_distance_m=0.0,
            state=EnvelopeState.FAIL_CLOSED,
            source=EnvelopeSource.NONE,
            limiting_zone=limiting_zone,
            reason=checked_reason,
        )

    def evaluate(self, frame: SensorFrame | None) -> EnvelopeResult:
        """Calculate safe linear and angular limits from one sensor frame."""
        if frame is None:
            return self.fail_closed("SENSOR_FRAME_MISSING")
        if frame.age_s > self._config.max_sensor_age_s:
            return self.fail_closed("SENSOR_FRAME_STALE")
        if frame.confidence < self._config.confidence_threshold:
            return self.fail_closed("SENSOR_CONFIDENCE_LOW")

        linear_limit = min(
            self._config.nominal_linear_velocity_mps,
            calculate_permitted_speed_limit(
                frame.distance_forward_m,
                self._braking_config,
            ).max_closing_speed_mps,
        )
        swept_clearance = min(
            frame.distance_forward_m,
            frame.distance_lateral_m,
        )
        tangential_limit = calculate_permitted_speed_limit(
            swept_clearance,
            self._braking_config,
        ).max_closing_speed_mps
        angular_limit = min(
            self._config.nominal_angular_velocity_radps,
            tangential_limit / self._config.angular_swept_radius_m,
        )
        if linear_limit < LINEAR_ZERO_CLAMP_MPS:
            linear_limit = 0.0
        if angular_limit < ANGULAR_ZERO_CLAMP_RADPS:
            angular_limit = 0.0

        if linear_limit == 0.0 and angular_limit == 0.0:
            state = EnvelopeState.PROTECTIVE_STOP
            reason = "CLEARANCE_REQUIRES_STOP"
        elif (
            linear_limit < self._config.nominal_linear_velocity_mps
            or angular_limit < self._config.nominal_angular_velocity_radps
        ):
            state = EnvelopeState.LIMITED
            reason = "CLEARANCE_LIMITED"
        else:
            state = EnvelopeState.NOMINAL
            reason = "NOMINAL_AUTHORITY"

        linear_ratio = (
            linear_limit / self._config.nominal_linear_velocity_mps
        )
        angular_ratio = (
            angular_limit / self._config.nominal_angular_velocity_radps
        )
        if state is EnvelopeState.NOMINAL:
            limiting_zone = LimitingZone.NONE
        elif linear_ratio <= angular_ratio:
            limiting_zone = LimitingZone.FORWARD
        else:
            limiting_zone = LimitingZone.LATERAL

        tangential_speed = (
            angular_limit * self._config.angular_swept_radius_m
        )
        return EnvelopeResult(
            policy_version=self._config.policy_version,
            max_linear_velocity_mps=linear_limit,
            max_angular_velocity_radps=angular_limit,
            linear_stopping_distance_m=self.stopping_distance(linear_limit),
            angular_stopping_distance_m=self.stopping_distance(
                tangential_speed
            ),
            distance_forward_m=frame.distance_forward_m,
            distance_lateral_m=frame.distance_lateral_m,
            state=state,
            source=frame.source,
            limiting_zone=limiting_zone,
            reason=reason,
        )


__all__ = [
    "DynamicEnvelopeEvaluator",
    "EnvelopeConfig",
    "EnvelopeResult",
    "EnvelopeSource",
    "EnvelopeState",
    "LimitingZone",
    "SensorFrame",
]
