"""TB-EVAL-006A tests for the dependency-free command governor."""

from __future__ import annotations

import math
from typing import Any, cast

import pytest
from pydantic import ValidationError

from core.command_governor import (
    CommandGovernor,
    GovernorCommand,
    GovernorConfig,
    GovernorReason,
    GovernorState,
    govern_command,
)


def _config(**overrides: float) -> GovernorConfig:
    values: dict[str, object] = {
        "max_linear_velocity_mps": 10.0,
        "max_angular_velocity_radps": 10.0,
        "max_linear_acceleration_mps2": 2.0,
        "max_linear_deceleration_mps2": 4.0,
        "max_angular_acceleration_radps2": 2.0,
        "max_angular_deceleration_radps2": 4.0,
        "max_linear_jerk_mps3": 100.0,
        "max_angular_jerk_radps3": 100.0,
    }
    values.update(overrides)
    config_type: Any = GovernorConfig
    return cast(GovernorConfig, config_type(**values))


def test_default_profile_and_short_aliases() -> None:
    config = GovernorConfig()

    assert config.v_max == 1.0
    assert config.omega_max == 1.0
    assert config.a_accel_max == 1.0
    assert config.a_decel_max == 2.0
    assert config.j_max == 5.0
    config_type: Any = GovernorConfig
    aliases = config_type(
        v_max=2,
        omega_max=3,
        a_accel_max=4,
        a_decel_max=5,
        alpha_accel_max=6,
        alpha_decel_max=7,
        j_max=8,
        j_angular_max=9,
    )
    assert aliases.max_linear_velocity_mps == 2.0
    assert aliases.max_angular_velocity_radps == 3.0
    assert aliases.max_angular_jerk_radps3 == 9.0


@pytest.mark.parametrize(
    "field",
    (
        "max_linear_velocity_mps",
        "max_angular_velocity_radps",
        "max_linear_acceleration_mps2",
        "max_linear_deceleration_mps2",
        "max_angular_acceleration_radps2",
        "max_angular_deceleration_radps2",
        "max_linear_jerk_mps3",
        "max_angular_jerk_radps3",
    ),
)
def test_limits_must_be_positive_and_finite(field: str) -> None:
    config_type: Any = GovernorConfig
    with pytest.raises(ValidationError):
        config_type(**{field: 0})
    with pytest.raises(ValidationError):
        config_type(**{field: -1})
    with pytest.raises(ValidationError):
        config_type(**{field: math.inf})
    with pytest.raises(ValidationError):
        config_type(**{field: True})


def test_command_wire_aliases_and_frozen_contracts() -> None:
    command_type: Any = GovernorCommand
    command = command_type(
        linear_x_mps=0.5,
        angular_z=0.25,
        stop=False,
        correlation_id="corr-1",
        timestamp_ns=10,
    )
    assert command.linear_velocity_mps == 0.5
    assert command.angular_velocity_radps == 0.25
    assert command.emergency_stop is False
    integer_alias = command_type(linear_x=1, angular_z=2)
    assert integer_alias.linear_velocity_mps == 1.0
    assert integer_alias.angular_velocity_radps == 2.0
    with pytest.raises(ValidationError):
        GovernorCommand.model_validate(object())
    with pytest.raises(ValidationError):
        GovernorCommand(linear_velocity_mps=math.nan)
    with pytest.raises(ValidationError):
        GovernorCommand(correlation_id=" ")
    with pytest.raises(ValidationError):
        GovernorCommand(emergency_stop=1)  # type: ignore[arg-type]


def test_first_step_and_nominal_passthrough() -> None:
    governor = CommandGovernor(_config())
    result = governor.step(
        GovernorCommand(linear_velocity_mps=1.0, angular_velocity_radps=0.5),
        timestamp_ns=1_000_000_000,
    )

    assert result.reason is GovernorReason.NOMINAL
    assert result.is_safe is True
    assert result.linear_velocity_mps == 1.0
    assert result.angular_velocity_radps == 0.5
    assert result.dt_ns == 1_000_000_000


def test_speed_saturation_is_symmetric() -> None:
    governor = CommandGovernor(_config())
    result = governor.step(
        {"linear_x": 50.0, "angular_z_radps": -50.0},
        timestamp_ns=5_000_000_000,
    )

    assert result.reason is GovernorReason.SPEED_LIMITED
    assert result.linear_velocity_mps == 10.0
    assert result.angular_velocity_radps == -10.0


def test_acceleration_limit_applies_to_both_signs() -> None:
    governor = CommandGovernor(_config())
    governor.step(GovernorCommand(linear_velocity_mps=0.0), timestamp_ns=1_000_000_000)
    positive = governor.step(
        GovernorCommand(linear_velocity_mps=10.0),
        timestamp_ns=2_000_000_000,
    )
    negative = governor.step(
        GovernorCommand(linear_velocity_mps=-10.0),
        timestamp_ns=3_000_000_000,
    )

    assert positive.reason is GovernorReason.ACCELERATION_LIMITED
    assert positive.linear_velocity_mps == 2.0
    assert negative.reason is GovernorReason.DECELERATION_LIMITED
    assert negative.linear_velocity_mps == -2.0


def test_stronger_deceleration_limit_is_used_for_slowing() -> None:
    governor = CommandGovernor(_config())
    governor.step(
        GovernorCommand(linear_velocity_mps=8.0),
        timestamp_ns=4_000_000_000,
    )
    result = governor.step(
        GovernorCommand(linear_velocity_mps=0.0),
        timestamp_ns=5_000_000_000,
    )

    assert result.reason is GovernorReason.DECELERATION_LIMITED
    assert result.linear_velocity_mps == 4.0


def test_direction_reversal_never_overshoots_target() -> None:
    governor = CommandGovernor(_config())
    governor.step(
        GovernorCommand(linear_velocity_mps=5.0),
        timestamp_ns=3_000_000_000,
    )
    result = governor.step(
        GovernorCommand(linear_velocity_mps=-5.0),
        timestamp_ns=5_000_000_000,
    )

    assert result.linear_velocity_mps == -3.0
    assert abs(result.linear_velocity_mps) <= 5.0


def test_angular_acceleration_and_deceleration_are_independent() -> None:
    governor = CommandGovernor(_config())
    governor.step(
        GovernorCommand(angular_velocity_radps=8.0),
        timestamp_ns=4_000_000_000,
    )
    result = governor.step(
        GovernorCommand(angular_velocity_radps=0.0),
        timestamp_ns=5_000_000_000,
    )

    assert result.angular_velocity_radps == 4.0
    assert result.reason is GovernorReason.DECELERATION_LIMITED


def test_jerk_limit_smooths_acceleration_change() -> None:
    governor = CommandGovernor(
        _config(max_linear_acceleration_mps2=10.0, max_linear_jerk_mps3=1.0)
    )
    result = governor.step(
        GovernorCommand(linear_velocity_mps=10.0),
        timestamp_ns=1_000_000_000,
    )

    assert result.reason is GovernorReason.JERK_LIMITED
    assert result.linear_acceleration_mps2 == 1.0
    assert result.linear_velocity_mps == 1.0


def test_jerk_guard_does_not_accelerate_away_from_a_stop() -> None:
    governor = CommandGovernor(
        _config(max_linear_acceleration_mps2=10.0, max_linear_jerk_mps3=1.0)
    )
    governor.step(GovernorCommand(linear_velocity_mps=10.0), timestamp_ns=1_000_000_000)
    result = governor.step(
        GovernorCommand(linear_velocity_mps=0.0),
        timestamp_ns=2_000_000_000,
    )

    assert result.linear_velocity_mps <= 1.0
    assert result.linear_acceleration_mps2 <= 0.0


def test_emergency_stop_bypasses_ramp_and_resets_baseline() -> None:
    governor = CommandGovernor(_config())
    governor.step(GovernorCommand(linear_velocity_mps=5.0), timestamp_ns=1_000_000_000)
    result = governor.step(
        GovernorCommand(emergency_stop=True, correlation_id="stop-1"),
        timestamp_ns=1_100_000_000,
    )

    assert result.reason is GovernorReason.EMERGENCY_STOP
    assert result.emergency_override is True
    assert result.is_safe is True
    assert result.linear_velocity_mps == 0.0
    assert governor.state.linear_velocity_mps == 0.0


def test_ordinary_zero_command_is_not_emergency_override() -> None:
    governor = CommandGovernor(_config())
    governor.step(GovernorCommand(linear_velocity_mps=5.0), timestamp_ns=1_000_000_000)
    result = governor.step(
        GovernorCommand(linear_velocity_mps=0.0),
        timestamp_ns=1_100_000_000,
    )

    assert result.emergency_override is False
    assert result.linear_velocity_mps < 5.0


@pytest.mark.parametrize(
    ("command", "timestamp", "reason"),
    (
        (GovernorCommand(), 0, GovernorReason.NON_POSITIVE_DT),
        (GovernorCommand(), -1, GovernorReason.INVALID_TIMESTAMP),
        (GovernorCommand(), 1.0, GovernorReason.INVALID_TIMESTAMP),
        ({"linear_velocity_mps": 1.0}, None, GovernorReason.INVALID_TIMESTAMP),
        (object(), 1_000_000_000, GovernorReason.INVALID_COMMAND),
    ),
)
def test_invalid_time_and_command_inputs_fail_closed(
    command: object,
    timestamp: object,
    reason: GovernorReason,
) -> None:
    governor = CommandGovernor(_config())
    result = governor.step(command, timestamp)  # type: ignore[arg-type]

    assert result.reason is reason
    assert result.is_safe is False
    assert result.linear_velocity_mps == 0.0
    assert result.angular_velocity_radps == 0.0


def test_clock_regression_fails_closed_without_advancing_clock() -> None:
    governor = CommandGovernor(_config(), initial_timestamp_ns=100)
    result = governor.step(GovernorCommand(), timestamp_ns=99)

    assert result.reason is GovernorReason.CLOCK_REGRESSION
    assert governor.last_timestamp_ns == 100
    assert result.is_safe is False


def test_duplicate_timestamp_fails_closed() -> None:
    governor = CommandGovernor(_config(), initial_timestamp_ns=100)
    result = governor.step(GovernorCommand(), timestamp_ns=100)

    assert result.reason is GovernorReason.NON_POSITIVE_DT


def test_invalid_command_resets_baseline_and_future_command_recovers() -> None:
    governor = CommandGovernor(_config())
    governor.step(GovernorCommand(linear_velocity_mps=2.0), timestamp_ns=100)
    invalid = governor.step({"linear_velocity_mps": math.inf}, timestamp_ns=200)
    recovered = governor.step(
        GovernorCommand(linear_velocity_mps=1.0),
        timestamp_ns=300,
    )

    assert invalid.reason is GovernorReason.INVALID_COMMAND
    assert recovered.linear_velocity_mps > 0.0
    assert governor.last_timestamp_ns == 300


def test_nonfinite_constructed_command_fails_closed() -> None:
    governor = CommandGovernor(_config())
    invalid = GovernorCommand.model_construct(
        linear_velocity_mps=math.inf,
        angular_velocity_radps=0.0,
        emergency_stop=False,
        correlation_id="constructed",
        timestamp_ns=None,
    )

    result = governor.step(invalid, timestamp_ns=100)

    assert result.reason is GovernorReason.INVALID_COMMAND
    assert result.is_safe is False


def test_explicit_and_keyword_timestamps_cannot_both_be_used() -> None:
    governor = CommandGovernor(_config())
    result = governor.step(GovernorCommand(), 1, now_ns=2)

    assert result.reason is GovernorReason.INVALID_TIMESTAMP


def test_alias_methods_and_one_shot_function() -> None:
    governor = CommandGovernor(_config())
    result = governor.evaluate(
        GovernorCommand(linear_velocity_mps=1.0),
        now_ns=1_000_000_000,
    )
    assert result.correlation_id == "unknown"
    result = governor.govern(
        GovernorCommand(linear_velocity_mps=1.0),
        now_ns=2_000_000_000,
    )
    assert result.linear_x_mps == result.linear_velocity_mps
    assert result.angular_z_radps == result.angular_velocity_radps
    one_shot = govern_command(
        {"linear_x": 0.5, "correlation_id": "one"},
        timestamp_ns=1_000_000_000,
        config=_config(),
    )
    assert one_shot.correlation_id == "one"


def test_invalid_constructor_inputs_and_state_snapshot() -> None:
    with pytest.raises(ValueError):
        CommandGovernor(_config(), initial_timestamp_ns=-1)
    with pytest.raises(ValueError):
        CommandGovernor(_config(), initial_timestamp_ns=True)
    with pytest.raises(ValueError):
        CommandGovernor(object())  # type: ignore[arg-type]

    governor = CommandGovernor(_config())
    assert governor.config is governor.config
    state: GovernorState = governor.state
    assert state.timestamp_ns == 0
    with pytest.raises(ValidationError):
        GovernorConfig.model_validate(object())
