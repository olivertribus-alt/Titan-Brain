"""TB-SAFE-001A acceptance tests for the dependency-free supervisor."""

from __future__ import annotations

from typing import Any

import pytest

from core.safety_supervisor import (
    DEFAULT_HEARTBEAT_TIMEOUT_NS,
    Heartbeat,
    HeartbeatChannel,
    RelayRequest,
    SafetyReason,
    SafetyState,
    SafetySupervisor,
    SafetySupervisorConfig,
    SafetySupervisorResult,
)


CHANNELS = tuple(HeartbeatChannel)


def _config(
    *,
    timeout_ns: int = 100,
    initialization_timeout_ns: int = 100,
) -> SafetySupervisorConfig:
    return SafetySupervisorConfig(
        control_arbiter_timeout_ns=timeout_ns,
        actuator_monitor_timeout_ns=timeout_ns,
        odometry_timeout_ns=timeout_ns,
        initialization_timeout_ns=initialization_timeout_ns,
    )


def _supervisor(
    *,
    timeout_ns: int = 100,
    initialization_timeout_ns: int = 100,
) -> SafetySupervisor:
    return SafetySupervisor(
        _config(
            timeout_ns=timeout_ns,
            initialization_timeout_ns=initialization_timeout_ns,
        ),
        started_at_ns=0,
    )


def _make_ready(supervisor: SafetySupervisor, *, timestamp_ns: int = 1) -> None:
    result: SafetySupervisorResult | None = None
    for offset, channel in enumerate(CHANNELS):
        result = supervisor.receive_heartbeat(
            channel,
            timestamp_ns=timestamp_ns + offset,
        )
    assert result is not None
    assert result.state is SafetyState.OK


def test_defaults_and_channel_budget_snapshot() -> None:
    config = SafetySupervisorConfig()

    assert DEFAULT_HEARTBEAT_TIMEOUT_NS == 200_000_000
    assert config.timeout_for(HeartbeatChannel.CONTROL_ARBITER) == 200_000_000
    assert config.timeout_for("actuator_monitor") == 200_000_000
    assert dict(config.channel_timeouts) == {
        HeartbeatChannel.CONTROL_ARBITER: 200_000_000,
        HeartbeatChannel.ACTUATOR_MONITOR: 200_000_000,
        HeartbeatChannel.ODOMETRY: 200_000_000,
    }


@pytest.mark.parametrize(
    "field",
    (
        "control_arbiter_timeout_ns",
        "actuator_monitor_timeout_ns",
        "odometry_timeout_ns",
        "initialization_timeout_ns",
    ),
)
def test_config_rejects_non_positive_or_non_integral_budget(field: str) -> None:
    config_type: Any = SafetySupervisorConfig
    with pytest.raises(ValueError):
        config_type(**{field: 0})
    with pytest.raises(ValueError):
        config_type(**{field: -1})
    with pytest.raises(ValueError):
        config_type(**{field: 0.2})
    with pytest.raises(ValueError):
        config_type(**{field: True})


def test_unknown_budget_channel_and_invalid_start_time_are_rejected() -> None:
    config = SafetySupervisorConfig()
    with pytest.raises(ValueError):
        config.timeout_for("unknown")
    with pytest.raises(ValueError):
        SafetySupervisor(config, started_at_ns=-1)
    with pytest.raises(ValueError):
        SafetySupervisor(config, started_at_ns=True)


def test_initializing_is_fail_closed_until_all_channels_register() -> None:
    supervisor = _supervisor()

    initial = supervisor.last_result
    assert initial.state is SafetyState.INITIALIZING
    assert initial.relay_request is RelayRequest.REQUEST_SAFETY_OPEN
    assert initial.is_safe is False
    assert initial.missing_channels == CHANNELS

    first = supervisor.receive_heartbeat(
        HeartbeatChannel.CONTROL_ARBITER,
        timestamp_ns=1,
    )
    assert first.state is SafetyState.INITIALIZING
    assert first.reason is SafetyReason.HEARTBEAT_RECEIVED
    assert HeartbeatChannel.CONTROL_ARBITER in first.missing_channels


def test_all_fresh_heartbeats_close_the_requested_relay() -> None:
    supervisor = _supervisor()
    _make_ready(supervisor)

    result = supervisor.last_result
    assert result.state is SafetyState.OK
    assert result.relay_request is RelayRequest.REQUEST_SAFETY_CLOSED
    assert result.is_safe is True
    assert result.reason is SafetyReason.ALL_HEARTBEATS_HEALTHY
    assert supervisor.last_heartbeat_ns("odometry") == 3


def test_initialization_timeout_trips_missing_channels() -> None:
    supervisor = _supervisor(initialization_timeout_ns=10)

    at_boundary = supervisor.evaluate(now_ns=10)
    assert at_boundary.state is SafetyState.INITIALIZING
    tripped = supervisor.evaluate(now_ns=11)
    assert tripped.state is SafetyState.TRIPPED
    assert tripped.reason is SafetyReason.INITIALIZATION_TIMEOUT
    assert tripped.relay_request is RelayRequest.REQUEST_SAFETY_OPEN
    assert tripped.failed_channels == CHANNELS


def test_stale_channel_trips_after_ready_state() -> None:
    supervisor = _supervisor(timeout_ns=10)
    _make_ready(supervisor)

    result = supervisor.evaluate(now_ns=14)
    assert result.state is SafetyState.TRIPPED
    assert result.reason is SafetyReason.HEARTBEAT_TIMEOUT
    assert HeartbeatChannel.CONTROL_ARBITER in result.failed_channels


def test_heartbeat_error_trips_and_cannot_auto_recover() -> None:
    supervisor = _supervisor()
    _make_ready(supervisor)

    error = supervisor.receive_heartbeat(
        HeartbeatChannel.ACTUATOR_MONITOR,
        timestamp_ns=5,
        healthy=False,
        error="driver offline",
    )
    assert error.state is SafetyState.TRIPPED
    assert error.reason is SafetyReason.HEARTBEAT_ERROR
    recovered = supervisor.receive_heartbeat(
        HeartbeatChannel.ACTUATOR_MONITOR,
        timestamp_ns=6,
    )
    assert recovered.state is SafetyState.TRIPPED
    assert recovered.reason is SafetyReason.HEARTBEAT_ERROR
    assert recovered.relay_request is RelayRequest.REQUEST_SAFETY_OPEN


def test_invalid_channel_timestamp_and_health_are_fail_closed() -> None:
    invalid_channel = _supervisor().receive_heartbeat(
        "not-registered",
        timestamp_ns=1,
    )
    assert invalid_channel.reason is SafetyReason.INVALID_HEARTBEAT
    assert invalid_channel.state is SafetyState.TRIPPED

    invalid_timestamp = _supervisor().receive_heartbeat(
        HeartbeatChannel.ODOMETRY,
        timestamp_ns=float("nan"),
    )
    assert invalid_timestamp.reason is SafetyReason.INVALID_TIMESTAMP
    assert invalid_timestamp.state is SafetyState.TRIPPED

    invalid_health = _supervisor().receive_heartbeat(
        HeartbeatChannel.ODOMETRY,
        timestamp_ns=1,
        healthy=1,
    )
    assert invalid_health.reason is SafetyReason.INVALID_HEARTBEAT
    assert invalid_health.state is SafetyState.TRIPPED

    invalid_error = _supervisor().receive_heartbeat(
        HeartbeatChannel.ODOMETRY,
        timestamp_ns=1,
        error=" ",
    )
    assert invalid_error.reason is SafetyReason.INVALID_HEARTBEAT


def test_clock_regression_is_fail_closed() -> None:
    supervisor = _supervisor()
    supervisor.receive_heartbeat(
        HeartbeatChannel.CONTROL_ARBITER,
        timestamp_ns=10,
    )

    regression = supervisor.evaluate(now_ns=9)
    assert regression.state is SafetyState.TRIPPED
    assert regression.reason is SafetyReason.CLOCK_REGRESSION

    invalid_now = _supervisor().evaluate(now_ns=-1)
    assert invalid_now.state is SafetyState.TRIPPED
    assert invalid_now.reason is SafetyReason.INVALID_TIMESTAMP


def test_hardware_fault_latch_is_sticky_for_all_future_inputs() -> None:
    supervisor = _supervisor()

    latched = supervisor.latch_hardware_fault(
        now_ns=4,
        detail="relay welded",
    )
    assert latched.state is SafetyState.HARDWARE_FAULT_LATCH
    assert latched.reason is SafetyReason.HARDWARE_FAULT
    assert latched.relay_request is RelayRequest.REQUEST_SAFETY_OPEN

    after_tick = supervisor.tick(now_ns=5)
    after_heartbeat = supervisor.heartbeat(
        HeartbeatChannel.ODOMETRY,
        timestamp_ns=6,
    )
    assert after_tick.reason is SafetyReason.HARDWARE_FAULT_LATCHED
    assert after_heartbeat.state is SafetyState.HARDWARE_FAULT_LATCH
    assert after_heartbeat.relay_request is RelayRequest.REQUEST_SAFETY_OPEN


def test_invalid_latch_time_still_latches_hardware_fault() -> None:
    supervisor = _supervisor()
    result = supervisor.latch_hardware_fault(now_ns=float("inf"))

    assert result.state is SafetyState.HARDWARE_FAULT_LATCH
    assert result.evaluated_at_ns == 0
    assert result.detail is not None


def test_heartbeat_contract_and_immutable_snapshots() -> None:
    heartbeat = Heartbeat(
        channel=HeartbeatChannel.ODOMETRY,
        timestamp_ns=3,
    )
    assert heartbeat.healthy is True
    supervisor = _supervisor()
    snapshot = supervisor.last_heartbeats
    with pytest.raises(TypeError):
        snapshot[HeartbeatChannel.ODOMETRY] = 4  # type: ignore[index]

    with pytest.raises(ValueError):
        Heartbeat(channel="odometry", timestamp_ns=3)  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        Heartbeat(channel=HeartbeatChannel.ODOMETRY, timestamp_ns=-1)
    with pytest.raises(ValueError):
        Heartbeat(
            channel=HeartbeatChannel.ODOMETRY,
            timestamp_ns=3,
            healthy=1,  # type: ignore[arg-type]
        )


def test_aliases_and_positional_record_api_are_usable() -> None:
    supervisor = _supervisor()
    first = supervisor.record_heartbeat(HeartbeatChannel.CONTROL_ARBITER, 1)
    assert first.reason is SafetyReason.HEARTBEAT_RECEIVED
    second = supervisor.receive_heartbeat(
        HeartbeatChannel.ACTUATOR_MONITOR,
        timestamp_ns=2,
    )
    assert second.state is SafetyState.INITIALIZING
