"""Pure planar geometry used by the ROS 2 TF transport boundary."""

from __future__ import annotations

import math
from typing import Self

from pydantic import model_validator

from core.types.incident import Pose2D, StrictFrozenModel

_MIN_QUATERNION_NORM_SQUARED = 1e-24


class Quaternion(StrictFrozenModel):
    """Finite quaternion received from a normalized TF2 transform."""

    x: float
    y: float
    z: float
    w: float

    @model_validator(mode="after")
    def validate_nonzero_norm(self) -> Self:
        """Reject rotations that cannot be normalized deterministically."""
        norm_squared = self.norm_squared
        if (
            not math.isfinite(norm_squared)
            or norm_squared <= _MIN_QUATERNION_NORM_SQUARED
        ):
            raise ValueError("Quaternion norm must be finite and non-zero.")
        return self

    @property
    def norm_squared(self) -> float:
        """Return the squared Euclidean norm."""
        norm = math.hypot(self.x, self.y, self.z, self.w)
        return norm * norm


class PlanarTransform(StrictFrozenModel):
    """Translation and quaternion rotation projected into a 2D plane."""

    translation_x: float
    translation_y: float
    rotation: Quaternion


def normalize_yaw(yaw: float) -> float:
    """Normalize a finite angle to the canonical interval ``[-pi, pi)``."""
    if not math.isfinite(yaw):
        raise ValueError("Yaw must be finite.")
    normalized = (yaw + math.pi) % math.tau - math.pi
    return 0.0 if normalized == 0.0 else normalized


def yaw_from_quaternion(quaternion: Quaternion) -> float:
    """Extract canonical yaw after normalizing the input quaternion."""
    inverse_norm = 1.0 / math.sqrt(quaternion.norm_squared)
    x = quaternion.x * inverse_norm
    y = quaternion.y * inverse_norm
    z = quaternion.z * inverse_norm
    w = quaternion.w * inverse_norm
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return normalize_yaw(math.atan2(sin_yaw, cos_yaw))


def apply_planar_transform(pose: Pose2D, transform: PlanarTransform) -> Pose2D:
    """Apply a TF2-style target-from-source transform to a planar pose."""
    transform_yaw = yaw_from_quaternion(transform.rotation)
    cos_yaw = math.cos(transform_yaw)
    sin_yaw = math.sin(transform_yaw)
    return Pose2D(
        x=transform.translation_x + cos_yaw * pose.x - sin_yaw * pose.y,
        y=transform.translation_y + sin_yaw * pose.x + cos_yaw * pose.y,
        yaw=normalize_yaw(transform_yaw + pose.yaw),
    )
