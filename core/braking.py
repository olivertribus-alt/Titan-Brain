"""Deterministic directional braking-envelope mathematics for TB-EVAL-002A."""

from __future__ import annotations

import math
from enum import StrEnum
from typing import Self

from pydantic import Field, model_validator

from core.types.incident import StrictFrozenModel

NANOSECONDS_PER_SECOND = 1_000_000_000


class DirectionalSector(StrEnum):
    """Robot-relative sectors used by the translational braking model."""

    FORWARD = "forward"
    REVERSE = "reverse"
    LEFT = "left"
    RIGHT = "right"


class DirectionalClearances(StrictFrozenModel):
    """Nearest obstacle clearance in each robot-relative sector."""

    forward_m: float = Field(ge=0.0)
    reverse_m: float = Field(ge=0.0)
    left_m: float = Field(ge=0.0)
    right_m: float = Field(ge=0.0)


class DirectionalClosingSpeeds(StrictFrozenModel):
    """Non-negative translational speed toward each directional sector."""

    forward_mps: float = Field(ge=0.0)
    reverse_mps: float = Field(ge=0.0)
    left_mps: float = Field(ge=0.0)
    right_mps: float = Field(ge=0.0)

    @model_validator(mode="after")
    def reject_opposing_motion(self) -> Self:
        """Reject physically contradictory closing speeds on one body axis."""
        if self.forward_mps > 0.0 and self.reverse_mps > 0.0:
            raise ValueError("forward and reverse closing speeds are exclusive")
        if self.left_mps > 0.0 and self.right_mps > 0.0:
            raise ValueError("left and right closing speeds are exclusive")
        return self


class BrakingEnvelopeConfig(StrictFrozenModel):
    """Explicit deployment assumptions used by the stopping-distance model."""

    policy_version: str = Field(min_length=1)
    reaction_time_ns: int = Field(gt=0)
    assured_deceleration_mps2: float = Field(gt=0.0)
    clearance_margin_m: float = Field(ge=0.0)


class StoppingDistanceBreakdown(StrictFrozenModel):
    """Auditable terms of one stopping-distance calculation."""

    closing_speed_mps: float = Field(gt=0.0)
    reaction_distance_m: float = Field(ge=0.0)
    braking_distance_m: float = Field(ge=0.0)
    clearance_margin_m: float = Field(ge=0.0)
    required_clearance_m: float = Field(gt=0.0)

    @model_validator(mode="after")
    def validate_sum(self) -> Self:
        """Keep the reported total identical to its evidence terms."""
        expected = (
            self.reaction_distance_m + self.braking_distance_m + self.clearance_margin_m
        )
        if self.required_clearance_m != expected:
            raise ValueError("required clearance must equal its evidence terms")
        return self


class SectorBrakingAssessment(StrictFrozenModel):
    """Stopping-distance comparison for one active or inactive sector."""

    sector: DirectionalSector
    observed_clearance_m: float = Field(ge=0.0)
    active: bool
    stopping_distance: StoppingDistanceBreakdown | None = None
    clearance_surplus_m: float | None = None
    clearance_sufficient: bool | None = None

    @model_validator(mode="after")
    def validate_shape(self) -> Self:
        """Distinguish ignored directions from assessed directions."""
        evidence = (
            self.stopping_distance,
            self.clearance_surplus_m,
            self.clearance_sufficient,
        )
        if self.active and any(value is None for value in evidence):
            raise ValueError("active sector requires complete braking evidence")
        if not self.active and any(value is not None for value in evidence):
            raise ValueError("inactive sector must not contain braking evidence")
        return self


class BrakingEnvelopeAssessment(StrictFrozenModel):
    """Deterministic result across all robot-relative directional sectors."""

    policy_version: str = Field(min_length=1)
    assessments: tuple[SectorBrakingAssessment, ...] = Field(
        min_length=4,
        max_length=4,
    )
    safe_to_proceed: bool
    limiting_sector: DirectionalSector | None = None

    @property
    def violated_sectors(self) -> tuple[DirectionalSector, ...]:
        """Return active sectors whose clearance is below the requirement."""
        return tuple(
            assessment.sector
            for assessment in self.assessments
            if assessment.clearance_sufficient is False
        )

    @model_validator(mode="after")
    def validate_summary(self) -> Self:
        """Keep the summary consistent with the four sector assessments."""
        expected_order = tuple(DirectionalSector)
        actual_order = tuple(item.sector for item in self.assessments)
        if actual_order != expected_order:
            raise ValueError("assessments must contain every sector in order")

        active = [item for item in self.assessments if item.active]
        expected_safe = all(item.clearance_sufficient is True for item in active)
        if self.safe_to_proceed is not expected_safe:
            raise ValueError("safe_to_proceed does not match sector evidence")

        expected_limiting = (
            min(
                active,
                key=lambda item: (
                    item.clearance_surplus_m
                    if item.clearance_surplus_m is not None
                    else math.inf
                ),
            ).sector
            if active
            else None
        )
        if self.limiting_sector is not expected_limiting:
            raise ValueError("limiting_sector does not match sector evidence")
        return self


def calculate_stopping_distance(
    closing_speed_mps: float,
    config: BrakingEnvelopeConfig,
) -> StoppingDistanceBreakdown:
    """Calculate reaction, braking, and configured clearance-margin terms."""
    if not math.isfinite(closing_speed_mps) or closing_speed_mps <= 0.0:
        raise ValueError("closing_speed_mps must be finite and greater than zero")

    try:
        reaction_time_s = config.reaction_time_ns / NANOSECONDS_PER_SECOND
        reaction_distance_m = closing_speed_mps * reaction_time_s
        braking_distance_m = (closing_speed_mps * closing_speed_mps) / (
            2.0 * config.assured_deceleration_mps2
        )
        required_clearance_m = (
            reaction_distance_m + braking_distance_m + config.clearance_margin_m
        )
    except OverflowError as error:
        raise ValueError("stopping-distance calculation overflowed") from error

    if not all(
        math.isfinite(value)
        for value in (
            reaction_distance_m,
            braking_distance_m,
            required_clearance_m,
        )
    ):
        raise ValueError("stopping-distance result must be finite")

    return StoppingDistanceBreakdown(
        closing_speed_mps=closing_speed_mps,
        reaction_distance_m=reaction_distance_m,
        braking_distance_m=braking_distance_m,
        clearance_margin_m=config.clearance_margin_m,
        required_clearance_m=required_clearance_m,
    )


def _assess_sector(
    sector: DirectionalSector,
    *,
    closing_speed_mps: float,
    observed_clearance_m: float,
    config: BrakingEnvelopeConfig,
) -> SectorBrakingAssessment:
    if closing_speed_mps == 0.0:
        return SectorBrakingAssessment(
            sector=sector,
            observed_clearance_m=observed_clearance_m,
            active=False,
        )

    stopping_distance = calculate_stopping_distance(
        closing_speed_mps,
        config,
    )
    clearance_surplus_m = observed_clearance_m - stopping_distance.required_clearance_m
    if clearance_surplus_m == 0.0:
        clearance_surplus_m = 0.0
    return SectorBrakingAssessment(
        sector=sector,
        observed_clearance_m=observed_clearance_m,
        active=True,
        stopping_distance=stopping_distance,
        clearance_surplus_m=clearance_surplus_m,
        clearance_sufficient=observed_clearance_m
        >= stopping_distance.required_clearance_m,
    )


def assess_braking_envelope(
    clearances: DirectionalClearances,
    closing_speeds: DirectionalClosingSpeeds,
    config: BrakingEnvelopeConfig,
) -> BrakingEnvelopeAssessment:
    """Compare directional clearance with a deterministic braking envelope."""
    raw_sectors = (
        (
            DirectionalSector.FORWARD,
            closing_speeds.forward_mps,
            clearances.forward_m,
        ),
        (
            DirectionalSector.REVERSE,
            closing_speeds.reverse_mps,
            clearances.reverse_m,
        ),
        (
            DirectionalSector.LEFT,
            closing_speeds.left_mps,
            clearances.left_m,
        ),
        (
            DirectionalSector.RIGHT,
            closing_speeds.right_mps,
            clearances.right_m,
        ),
    )
    assessments = tuple(
        _assess_sector(
            sector,
            closing_speed_mps=speed,
            observed_clearance_m=clearance,
            config=config,
        )
        for sector, speed, clearance in raw_sectors
    )
    active = [item for item in assessments if item.active]
    limiting_sector = (
        min(
            active,
            key=lambda item: (
                item.clearance_surplus_m
                if item.clearance_surplus_m is not None
                else math.inf
            ),
        ).sector
        if active
        else None
    )
    return BrakingEnvelopeAssessment(
        policy_version=config.policy_version,
        assessments=assessments,
        safe_to_proceed=all(item.clearance_sufficient is True for item in active),
        limiting_sector=limiting_sector,
    )
