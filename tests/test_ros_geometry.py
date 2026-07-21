"""Tests for dependency-free planar TF2 geometry."""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from core.adapters.ros_geometry import (
    PlanarTransform,
    Quaternion,
    apply_planar_transform,
    normalize_yaw,
    yaw_from_quaternion,
)
from core.types.incident import Pose2D


def _yaw_quaternion(yaw: float, *, scale: float = 1.0) -> Quaternion:
    half_yaw = yaw / 2.0
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw) * scale,
        w=math.cos(half_yaw) * scale,
    )


@pytest.mark.parametrize(
    ("yaw", "expected"),
    [
        (0.0, 0.0),
        (math.pi, -math.pi),
        (-math.pi, -math.pi),
        (3.0 * math.pi, -math.pi),
        (2.0 * math.pi, 0.0),
    ],
)
def test_normalize_yaw_uses_one_canonical_interval(
    yaw: float,
    expected: float,
) -> None:
    assert normalize_yaw(yaw) == pytest.approx(expected)


@pytest.mark.parametrize("yaw", [float("nan"), float("inf"), float("-inf")])
def test_normalize_yaw_rejects_non_finite_values(yaw: float) -> None:
    with pytest.raises(ValueError, match="finite"):
        normalize_yaw(yaw)


def test_yaw_extraction_normalizes_non_unit_quaternion() -> None:
    quaternion = _yaw_quaternion(math.pi / 2.0, scale=4.0)

    assert quaternion.norm_squared == pytest.approx(16.0)
    assert yaw_from_quaternion(quaternion) == pytest.approx(math.pi / 2.0)


def test_planar_transform_rotates_translates_and_composes_yaw() -> None:
    pose = Pose2D(x=1.0, y=0.0, yaw=3.0 * math.pi / 4.0)
    transform = PlanarTransform(
        translation_x=10.0,
        translation_y=20.0,
        rotation=_yaw_quaternion(math.pi / 2.0),
    )

    transformed = apply_planar_transform(pose, transform)

    assert transformed.x == pytest.approx(10.0)
    assert transformed.y == pytest.approx(21.0)
    assert transformed.yaw == pytest.approx(-3.0 * math.pi / 4.0)


def test_identity_transform_preserves_pose() -> None:
    pose = Pose2D(x=-4.0, y=3.5, yaw=-0.75)
    transform = PlanarTransform(
        translation_x=0.0,
        translation_y=0.0,
        rotation=_yaw_quaternion(0.0),
    )

    assert apply_planar_transform(pose, transform) == pose


def test_quaternion_rejects_zero_non_finite_and_overflowing_norms() -> None:
    with pytest.raises(ValidationError, match="non-zero"):
        Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
    with pytest.raises(ValidationError):
        Quaternion(x=0.0, y=0.0, z=float("nan"), w=1.0)
    with pytest.raises(ValidationError, match="finite"):
        Quaternion(x=1e308, y=1e308, z=1e308, w=1e308)


def test_planar_transform_rejects_non_finite_translation() -> None:
    with pytest.raises(ValidationError):
        PlanarTransform(
            translation_x=float("inf"),
            translation_y=0.0,
            rotation=_yaw_quaternion(0.0),
        )
