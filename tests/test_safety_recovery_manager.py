"""TB-EVAL-009A lifecycle, hysteresis, and recovery dwell tests."""

from __future__ import annotations

import hashlib

import pytest
from pydantic import ValidationError

from core.safety_recovery_manager import (
    SafetyLifecycleEvidence,
    SafetyLifecycleReason,
    SafetyLifecycleState,
    SafetyRecoveryConfig,
    SafetyRecoveryManager,
    transition_safety_lifecycle,
)

DWELL_NS = 1_000_000_000


def _config(**updates: object) -> SafetyRecoveryConfig:
    values: dict[str, object] = {
        "policy_version": "TB-EVAL-009A-TEST",
        "stop_margin_m": 0.30,
        "warning_distance_m": 1.00,
        "distance_hysteresis_m": 0.10,
        "recovery_dwell_time_ns": DWELL_NS,
        "degraded_linear_speed_limit_mps": 0.50,
        "degraded_angular_speed_limit_radps": 0.50,
        "recovery_linear_speed_limit_mps": 0.20,
        "recovery_angular_speed_limit_radps": 0.40,
    }
    values.update(updates)
    return SafetyRecoveryConfig.model_validate(values)


def _evidence(**updates: object) -> SafetyLifecycleEvidence:
    values: dict[str, object] = {
        "fault_status_valid": True,
        "is_faulted": False,
        "sensor_valid": True,
        "sensor_fresh": True,
        "time_valid": True,
        "noncritical_warning": False,
        "distance_min_m": 5.0,
        "max_linear_velocity_mps": 1.0,
        "max_angular_velocity_radps": 1.0,
    }
    values.update(updates)
    return SafetyLifecycleEvidence.model_validate(values)


def _complete_initial_recovery(
    manager: SafetyRecoveryManager,
    *,
    started_at_ns: int = 0,
) -> None:
    started = manager.update(_evidence(), now_ns=started_at_ns)
    released = manager.update(
        _evidence(),
        now_ns=started_at_ns + DWELL_NS,
    )
    assert started.state is SafetyLifecycleState.RECOVERY
    assert released.state is SafetyLifecycleState.NORMAL


def test_startup_is_fail_closed_until_full_recovery_dwell() -> None:
    manager = SafetyRecoveryManager(_config())

    started = manager.update(_evidence(), now_ns=10)
    holding = manager.update(_evidence(), now_ns=10 + DWELL_NS - 1)
    released = manager.update(_evidence(), now_ns=10 + DWELL_NS)

    assert started.state is SafetyLifecycleState.RECOVERY
    assert started.reason is SafetyLifecycleReason.RECOVERY_STARTED
    assert started.recovery_started_at_ns == 10
    assert started.recovery_elapsed_ns == 0
    assert started.max_linear_velocity_mps == 0.20
    assert started.max_angular_velocity_radps == 0.40
    assert holding.state is SafetyLifecycleState.RECOVERY
    assert holding.reason is SafetyLifecycleReason.RECOVERY_HOLDING
    assert holding.recovery_elapsed_ns == DWELL_NS - 1
    assert released.state is SafetyLifecycleState.NORMAL
    assert released.reason is SafetyLifecycleReason.RECOVERY_COMPLETE_NORMAL
    assert released.recovery_started_at_ns is None
    assert released.max_linear_velocity_mps == 1.0


@pytest.mark.parametrize(
    ("update", "reason"),
    [
        ({"is_faulted": True}, SafetyLifecycleReason.SYSTEM_FAULT),
        (
            {"fault_status_valid": False},
            SafetyLifecycleReason.FAULT_STATUS_INVALID,
        ),
        ({"time_valid": False}, SafetyLifecycleReason.TIME_INVALID),
        ({"sensor_valid": False}, SafetyLifecycleReason.SENSOR_INVALID),
        ({"sensor_fresh": False}, SafetyLifecycleReason.SENSOR_STALE),
        (
            {"distance_min_m": 0.30},
            SafetyLifecycleReason.STOP_MARGIN_BREACH,
        ),
    ],
)
def test_critical_inputs_enter_exact_zero_stop_in_one_update(
    update: dict[str, object],
    reason: SafetyLifecycleReason,
) -> None:
    manager = SafetyRecoveryManager(_config())
    _complete_initial_recovery(manager)

    transition = manager.update(_evidence(**update), now_ns=DWELL_NS + 1)

    assert transition.state is SafetyLifecycleState.EMERGENCY_STOP
    assert transition.reason is reason
    assert transition.stop_only is True
    assert transition.max_linear_velocity_mps == 0.0
    assert transition.max_angular_velocity_radps == 0.0
    assert transition.recovery_started_at_ns is None


def test_stop_margin_boundary_is_closed_and_next_float_is_degraded() -> None:
    manager = SafetyRecoveryManager(_config())
    _complete_initial_recovery(manager)

    stopped = manager.update(
        _evidence(distance_min_m=0.30),
        now_ns=DWELL_NS + 1,
    )
    recovering = manager.update(
        _evidence(distance_min_m=0.300000001),
        now_ns=DWELL_NS + 2,
    )
    degraded = manager.update(
        _evidence(distance_min_m=0.300000001),
        now_ns=2 * DWELL_NS + 2,
    )

    assert stopped.state is SafetyLifecycleState.EMERGENCY_STOP
    assert recovering.state is SafetyLifecycleState.RECOVERY
    assert degraded.state is SafetyLifecycleState.DEGRADED
    assert degraded.reason is SafetyLifecycleReason.RECOVERY_COMPLETE_DEGRADED


def test_recovery_cannot_reach_normal_inside_hysteresis_band() -> None:
    manager = SafetyRecoveryManager(_config())

    started = manager.update(
        _evidence(distance_min_m=1.05),
        now_ns=0,
    )
    completed = manager.update(
        _evidence(distance_min_m=1.05),
        now_ns=DWELL_NS,
    )

    assert started.state is SafetyLifecycleState.RECOVERY
    assert completed.state is SafetyLifecycleState.DEGRADED
    assert completed.reason is SafetyLifecycleReason.RECOVERY_COMPLETE_DEGRADED


def test_warning_zone_enters_degraded_without_stopping() -> None:
    manager = SafetyRecoveryManager(_config())
    _complete_initial_recovery(manager)

    transition = manager.update(
        _evidence(distance_min_m=0.75),
        now_ns=DWELL_NS + 1,
    )

    assert transition.state is SafetyLifecycleState.DEGRADED
    assert transition.reason is SafetyLifecycleReason.WARNING_ZONE
    assert transition.stop_only is False
    assert transition.max_linear_velocity_mps == 0.50
    assert transition.max_angular_velocity_radps == 0.50


def test_noncritical_warning_reduces_nominal_authority() -> None:
    manager = SafetyRecoveryManager(_config())
    _complete_initial_recovery(manager)

    transition = manager.update(
        _evidence(noncritical_warning=True),
        now_ns=DWELL_NS + 1,
    )

    assert transition.state is SafetyLifecycleState.DEGRADED
    assert transition.reason is SafetyLifecycleReason.NONCRITICAL_WARNING
    assert transition.max_linear_velocity_mps == 0.50


def test_degraded_release_requires_strict_hysteresis_threshold() -> None:
    manager = SafetyRecoveryManager(_config())
    _complete_initial_recovery(manager)
    manager.update(
        _evidence(distance_min_m=0.90),
        now_ns=DWELL_NS + 1,
    )

    at_warning = manager.update(
        _evidence(distance_min_m=1.00),
        now_ns=DWELL_NS + 2,
    )
    inside_hysteresis = manager.update(
        _evidence(distance_min_m=1.10),
        now_ns=DWELL_NS + 3,
    )
    released = manager.update(
        _evidence(distance_min_m=1.100000001),
        now_ns=DWELL_NS + 4,
    )

    assert at_warning.state is SafetyLifecycleState.DEGRADED
    assert inside_hysteresis.state is SafetyLifecycleState.DEGRADED
    assert released.state is SafetyLifecycleState.NORMAL
    assert released.reason is SafetyLifecycleReason.NORMAL_CLEARANCE


def test_new_fault_cancels_recovery_and_restarts_full_dwell() -> None:
    manager = SafetyRecoveryManager(_config())
    _complete_initial_recovery(manager)
    manager.update(_evidence(is_faulted=True), now_ns=DWELL_NS + 10)
    first_recovery = manager.update(_evidence(), now_ns=DWELL_NS + 20)
    interrupted = manager.update(
        _evidence(sensor_fresh=False),
        now_ns=DWELL_NS + 100,
    )
    restarted = manager.update(_evidence(), now_ns=DWELL_NS + 200)
    still_holding = manager.update(
        _evidence(),
        now_ns=2 * DWELL_NS + 199,
    )
    released = manager.update(
        _evidence(),
        now_ns=2 * DWELL_NS + 200,
    )

    assert first_recovery.recovery_started_at_ns == DWELL_NS + 20
    assert interrupted.state is SafetyLifecycleState.EMERGENCY_STOP
    assert restarted.recovery_started_at_ns == DWELL_NS + 200
    assert still_holding.state is SafetyLifecycleState.RECOVERY
    assert released.state is SafetyLifecycleState.NORMAL


def test_dynamic_envelope_limits_are_never_expanded() -> None:
    config = _config()
    manager = SafetyRecoveryManager(config)
    evidence = _evidence(
        distance_min_m=0.80,
        max_linear_velocity_mps=0.12,
        max_angular_velocity_radps=0.15,
    )

    recovering = manager.update(evidence, now_ns=0)
    degraded = manager.update(evidence, now_ns=DWELL_NS)

    assert recovering.max_linear_velocity_mps == 0.12
    assert recovering.max_angular_velocity_radps == 0.15
    assert degraded.max_linear_velocity_mps == 0.12
    assert degraded.max_angular_velocity_radps == 0.15


def test_clock_regression_latches_immediate_emergency_stop() -> None:
    manager = SafetyRecoveryManager(_config())
    manager.update(_evidence(), now_ns=100)
    transition = manager.update(_evidence(), now_ns=99)

    assert transition.state is SafetyLifecycleState.EMERGENCY_STOP
    assert transition.reason is SafetyLifecycleReason.CLOCK_REGRESSION
    assert transition.evaluated_at_ns == 99
    assert transition.monotonic_time_ns == 100
    assert transition.stop_only is True


@pytest.mark.parametrize("now_ns", [-1, 1.5, True, None])
def test_invalid_time_fails_closed(now_ns: object) -> None:
    transition = transition_safety_lifecycle(
        None,
        _evidence(),
        _config(),
        now_ns=now_ns,
    )

    assert transition.state is SafetyLifecycleState.EMERGENCY_STOP
    assert transition.reason is SafetyLifecycleReason.TIME_INVALID
    assert transition.stop_only is True


@pytest.mark.parametrize(
    "update",
    [
        {"stop_margin_m": float("nan")},
        {"warning_distance_m": float("inf")},
        {"distance_hysteresis_m": float("inf")},
        {"warning_distance_m": 0.30},
        {"recovery_linear_speed_limit_mps": 0.60},
        {"recovery_angular_speed_limit_radps": 0.60},
    ],
)
def test_invalid_or_contradictory_config_is_rejected(
    update: dict[str, object],
) -> None:
    with pytest.raises(ValidationError):
        _config(**update)


@pytest.mark.parametrize(
    "update",
    [
        {"distance_min_m": float("nan")},
        {"max_linear_velocity_mps": float("inf")},
        {"max_angular_velocity_radps": float("nan")},
        {"distance_min_m": None},
    ],
)
def test_invalid_healthy_evidence_is_rejected(
    update: dict[str, object],
) -> None:
    with pytest.raises(ValidationError):
        _evidence(**update)


def test_fault_evidence_may_omit_distance_and_still_fails_closed() -> None:
    transition = transition_safety_lifecycle(
        None,
        _evidence(is_faulted=True, distance_min_m=None),
        _config(),
        now_ns=0,
    )

    assert transition.reason is SafetyLifecycleReason.SYSTEM_FAULT
    assert transition.stop_only is True


def test_results_are_immutable_and_bit_deterministic() -> None:
    config = _config()
    evidence = _evidence(distance_min_m=0.75)
    results = [
        transition_safety_lifecycle(None, evidence, config, now_ns=123)
        for _ in range(100)
    ]
    serialized = [result.model_dump_json() for result in results]
    hashes = {
        hashlib.sha256(payload.encode("utf-8")).hexdigest() for payload in serialized
    }

    assert all(result == results[0] for result in results)
    assert len(set(serialized)) == 1
    assert len(hashes) == 1
    with pytest.raises(ValidationError):
        results[0].state = SafetyLifecycleState.NORMAL
