"""Inverse braking mathematics for TB-EVAL-005A motion authority."""

from __future__ import annotations

import math
from decimal import Decimal, localcontext
from enum import StrEnum
from typing import Literal, Self

from pydantic import Field, model_validator

from core.braking import (
    NANOSECONDS_PER_SECOND,
    BrakingEnvelopeConfig,
    calculate_stopping_distance,
)
from core.types.incident import StrictFrozenModel

_DECIMAL_PRECISION = 100
_MAX_ADJUSTMENT_STEPS = 4


class PermittedSpeedStatus(StrEnum):
    """Whether the scalar envelope grants translational motion authority."""

    MOVEMENT_PERMITTED = "movement_permitted"
    STOP_ONLY = "stop_only"


def _checked_clearance(value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or value < 0
    ):
        raise ValueError(
            "observed_clearance_m must be a non-negative finite number"
        )
    try:
        checked = float(value)
    except OverflowError as error:
        raise ValueError(
            "observed_clearance_m must be a non-negative finite number"
        ) from error
    if not math.isfinite(checked):
        raise ValueError(
            "observed_clearance_m must be a non-negative finite number"
        )
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
        available = (
            Decimal(str(observed_clearance_m))
            - Decimal(str(config.clearance_margin_m))
        )
        reaction_term = deceleration * reaction_time
        discriminant = (
            reaction_term * reaction_term
            + Decimal(2) * deceleration * available
        )
        root = discriminant.sqrt()
        exact_limit = (
            Decimal(2) * deceleration * available
        ) / (root + reaction_term)

    candidate = float(exact_limit)
    if not math.isfinite(candidate):
        raise ValueError("permitted-speed result must be finite")
    if candidate == 0.0:
        return 0.0

    for _ in range(_MAX_ADJUSTMENT_STEPS):
        if (
            Decimal(str(candidate)) <= exact_limit
            and _forward_model_accepts(candidate, observed_clearance_m, config)
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
