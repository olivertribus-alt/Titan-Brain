"""Inverse braking and directional motion-authority mathematics."""

from __future__ import annotations

import math
from decimal import Decimal, localcontext
from enum import StrEnum
from typing import Literal, Self

from pydantic import Field, model_validator

from core.braking import (
    NANOSECONDS_PER_SECOND,
    BrakingEnvelopeConfig,
    DirectionalClearances,
    DirectionalSector,
    calculate_stopping_distance,
)
from core.types.incident import StrictFrozenModel

_DECIMAL_PRECISION = 100
_MAX_ADJUSTMENT_STEPS = 4


class PermittedSpeedStatus(StrEnum):
    """Whether the scalar envelope grants translational motion authority."""

    MOVEMENT_PERMITTED = "movement_permitted"
    STOP_ONLY = "stop_only"


class AngularMotionPolicyStatus(StrEnum):
    """Why TB-EVAL-005B grants no angular motion authority."""

    BLOCKED_INSUFFICIENT_CLEARANCE = "blocked_insufficient_clearance"
    BLOCKED_SWEPT_FOOTPRINT_UNAVAILABLE = "blocked_swept_footprint_unavailable"


def _checked_clearance(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value < 0:
        raise ValueError("observed_clearance_m must be a non-negative finite number")
    try:
        checked = float(value)
    except OverflowError as error:
        raise ValueError(
            "observed_clearance_m must be a non-negative finite number"
        ) from error
    if not math.isfinite(checked):
        raise ValueError("observed_clearance_m must be a non-negative finite number")
    return 0.0 if checked == 0.0 else checked


def _available_clearance(
    observed_clearance_m: float,
    clearance_margin_m: float,
) -> float:
    available = observed_clearance_m - clearance_margin_m
    return available if available > 0.0 else 0.0


def _forward_model_accepts(
    speed_mps: float,
    observed_clearance_m: float,
    config: BrakingEnvelopeConfig,
) -> bool:
    try:
        required = calculate_stopping_distance(
            speed_mps,
            config,
        ).required_clearance_m
    except ValueError:
        return False
    return required <= observed_clearance_m


def _solve_max_closing_speed(
    observed_clearance_m: float,
    config: BrakingEnvelopeConfig,
) -> float:
    available_clearance_m = _available_clearance(
        observed_clearance_m,
        config.clearance_margin_m,
    )
    if available_clearance_m == 0.0:
        return 0.0

    try:
        reaction_time_s = config.reaction_time_ns / NANOSECONDS_PER_SECOND
    except OverflowError as error:
        raise ValueError("reaction-time conversion overflowed") from error
    with localcontext() as context:
        context.prec = _DECIMAL_PRECISION
        deceleration = Decimal(str(config.assured_deceleration_mps2))
        reaction_time = Decimal(str(reaction_time_s))
        available = Decimal(str(observed_clearance_m)) - Decimal(
            str(config.clearance_margin_m)
        )
        reaction_term = deceleration * reaction_time
        discriminant = (
            reaction_term * reaction_term + Decimal(2) * deceleration * available
        )
        root = discriminant.sqrt()
        exact_limit = (Decimal(2) * deceleration * available) / (root + reaction_term)

    candidate = float(exact_limit)
    if not math.isfinite(candidate):
        raise ValueError("permitted-speed result must be finite")
    if candidate == 0.0:
        return 0.0

    for _ in range(_MAX_ADJUSTMENT_STEPS):
        if Decimal(str(candidate)) <= exact_limit and _forward_model_accepts(
            candidate, observed_clearance_m, config
        ):
            break
        candidate = math.nextafter(candidate, 0.0)
    else:
        raise ValueError("could not establish a conservative speed limit")
    return candidate


class PermittedSpeedLimit(StrictFrozenModel):
    """Auditable scalar speed authority derived from one clearance sample."""

    schema_version: Literal["0.1"] = "0.1"
    policy_version: str = Field(min_length=1)
    observed_clearance_m: float = Field(ge=0.0)
    clearance_margin_m: float = Field(ge=0.0)
    available_clearance_m: float = Field(ge=0.0)
    reaction_time_ns: int = Field(gt=0)
    assured_deceleration_mps2: float = Field(gt=0.0)
    max_closing_speed_mps: float = Field(ge=0.0)
    status: PermittedSpeedStatus

    @property
    def stop_only(self) -> bool:
        """Return whether no representable positive speed is authorized."""
        return self.status is PermittedSpeedStatus.STOP_ONLY

    @model_validator(mode="after")
    def validate_evidence(self) -> Self:
        """Keep the immutable evidence identical to the inverse model."""
        expected_available = _available_clearance(
            self.observed_clearance_m,
            self.clearance_margin_m,
        )
        if self.available_clearance_m != expected_available:
            raise ValueError("available clearance does not match evidence")

        config = BrakingEnvelopeConfig(
            policy_version=self.policy_version,
            reaction_time_ns=self.reaction_time_ns,
            assured_deceleration_mps2=self.assured_deceleration_mps2,
            clearance_margin_m=self.clearance_margin_m,
        )
        expected_limit = _solve_max_closing_speed(
            self.observed_clearance_m,
            config,
        )
        if self.max_closing_speed_mps != expected_limit:
            raise ValueError("maximum speed does not match inverse braking evidence")

        expected_status = (
            PermittedSpeedStatus.MOVEMENT_PERMITTED
            if expected_limit > 0.0
            else PermittedSpeedStatus.STOP_ONLY
        )
        if self.status is not expected_status:
            raise ValueError("speed status does not match the calculated limit")
        return self


class DirectionalSpeedLimit(StrictFrozenModel):
    """Scalar authority bound to one robot-relative sector."""

    sector: DirectionalSector
    speed_limit: PermittedSpeedLimit


class PlanarVelocityLimits(StrictFrozenModel):
    """Signed body-frame limits prepared for deterministic command clamping."""

    min_linear_x_mps: float = Field(le=0.0)
    max_linear_x_mps: float = Field(ge=0.0)
    min_linear_y_mps: float = Field(le=0.0)
    max_linear_y_mps: float = Field(ge=0.0)
    max_abs_angular_z_radps: float = Field(ge=0.0)


class DirectionalMotionEnvelope(StrictFrozenModel):
    """Four-sector translation authority and fail-closed angular policy."""

    schema_version: Literal["0.1"] = "0.1"
    policy_version: str = Field(min_length=1)
    sector_limits: tuple[DirectionalSpeedLimit, ...] = Field(
        min_length=4,
        max_length=4,
    )
    velocity_limits: PlanarVelocityLimits
    translation_permitted_in_all_directions: bool
    angular_policy_status: AngularMotionPolicyStatus

    @model_validator(mode="after")
    def validate_evidence(self) -> Self:
        """Prevent directional, physical, or angular evidence forgery."""
        actual_order = tuple(item.sector for item in self.sector_limits)
        if actual_order != tuple(DirectionalSector):
            raise ValueError("sector limits must contain every sector in order")

        speed_limits = tuple(item.speed_limit for item in self.sector_limits)
        if any(item.policy_version != self.policy_version for item in speed_limits):
            raise ValueError("sector policy versions must match the envelope")
        physical_assumptions = {
            (
                item.reaction_time_ns,
                item.assured_deceleration_mps2,
                item.clearance_margin_m,
            )
            for item in speed_limits
        }
        if len(physical_assumptions) != 1:
            raise ValueError("sector physical assumptions must be identical")

        forward, reverse, left, right = (
            item.max_closing_speed_mps for item in speed_limits
        )
        expected_limits = PlanarVelocityLimits(
            min_linear_x_mps=_negative_or_zero(reverse),
            max_linear_x_mps=forward,
            min_linear_y_mps=_negative_or_zero(right),
            max_linear_y_mps=left,
            max_abs_angular_z_radps=0.0,
        )
        if self.velocity_limits != expected_limits:
            raise ValueError("velocity limits do not match sector evidence")

        expected_all_directions = all(not item.stop_only for item in speed_limits)
        if self.translation_permitted_in_all_directions is not expected_all_directions:
            raise ValueError("omnidirectional translation status does not match limits")
        expected_angular_status = (
            AngularMotionPolicyStatus.BLOCKED_SWEPT_FOOTPRINT_UNAVAILABLE
            if expected_all_directions
            else AngularMotionPolicyStatus.BLOCKED_INSUFFICIENT_CLEARANCE
        )
        if self.angular_policy_status is not expected_angular_status:
            raise ValueError("angular policy status does not match evidence")
        return self


def _negative_or_zero(limit_mps: float) -> float:
    return -limit_mps if limit_mps > 0.0 else 0.0


def calculate_permitted_speed_limit(
    observed_clearance_m: float,
    config: BrakingEnvelopeConfig,
) -> PermittedSpeedLimit:
    """Invert the stopping model into a conservative scalar speed limit."""
    checked_clearance = _checked_clearance(observed_clearance_m)
    available_clearance_m = _available_clearance(
        checked_clearance,
        config.clearance_margin_m,
    )
    max_closing_speed_mps = _solve_max_closing_speed(
        checked_clearance,
        config,
    )
    return PermittedSpeedLimit(
        policy_version=config.policy_version,
        observed_clearance_m=checked_clearance,
        clearance_margin_m=config.clearance_margin_m,
        available_clearance_m=available_clearance_m,
        reaction_time_ns=config.reaction_time_ns,
        assured_deceleration_mps2=config.assured_deceleration_mps2,
        max_closing_speed_mps=max_closing_speed_mps,
        status=(
            PermittedSpeedStatus.MOVEMENT_PERMITTED
            if max_closing_speed_mps > 0.0
            else PermittedSpeedStatus.STOP_ONLY
        ),
    )


def calculate_directional_motion_envelope(
    clearances: DirectionalClearances,
    config: BrakingEnvelopeConfig,
) -> DirectionalMotionEnvelope:
    """Invert four directional clearances into signed planar limits."""
    raw_clearances = (
        (DirectionalSector.FORWARD, clearances.forward_m),
        (DirectionalSector.REVERSE, clearances.reverse_m),
        (DirectionalSector.LEFT, clearances.left_m),
        (DirectionalSector.RIGHT, clearances.right_m),
    )
    sector_limits = tuple(
        DirectionalSpeedLimit(
            sector=sector,
            speed_limit=calculate_permitted_speed_limit(clearance, config),
        )
        for sector, clearance in raw_clearances
    )
    forward, reverse, left, right = (
        item.speed_limit.max_closing_speed_mps for item in sector_limits
    )
    translation_permitted_in_all_directions = all(
        not item.speed_limit.stop_only for item in sector_limits
    )
    return DirectionalMotionEnvelope(
        policy_version=config.policy_version,
        sector_limits=sector_limits,
        velocity_limits=PlanarVelocityLimits(
            min_linear_x_mps=_negative_or_zero(reverse),
            max_linear_x_mps=forward,
            min_linear_y_mps=_negative_or_zero(right),
            max_linear_y_mps=left,
            max_abs_angular_z_radps=0.0,
        ),
        translation_permitted_in_all_directions=(
            translation_permitted_in_all_directions
        ),
        angular_policy_status=(
            AngularMotionPolicyStatus.BLOCKED_SWEPT_FOOTPRINT_UNAVAILABLE
            if translation_permitted_in_all_directions
            else AngularMotionPolicyStatus.BLOCKED_INSUFFICIENT_CLEARANCE
        ),
    )
