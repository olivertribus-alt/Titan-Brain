"""Acceptance tests for TB-ACT-001B stop acknowledgement monitoring."""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from core.actuator_feedback import (
    ActuatorFeedback,
    ActuatorFeedbackConfig,
)
from core.stop_ack_monitor import (
    HardwareResetRequest,
    StopAckMonitor,
    StopAcknowledgement,
    StopAckReason,
    StopMonitorConfig,
    StopMonitorResult,
    StopMonitorState,
    StopRequest,
)


@pytest.fixture
def config() -> StopMonitorConfig:
    return StopMonitorConfig(
        stop_budget_ns=100,
        feedback_config=ActuatorFeedbackConfig(
            epsilon_stop_linear=0.01,
            epsilon_stop_angular=0.02,
            stale_threshold_ns=50,
        ),
    )


@pytest.fixture
def monitor(config: StopMonitorConfig) -> StopAckMonitor:
    return StopAckMonitor(config)


def _request(**overrides: object) -> StopRequest:
    payload: dict[str, object] = {
        "correlation_id": "stop-001",
        "sequence_id": 10,
        "requested_timestamp_ns": 100,
    }
    payload.update(overrides)
    return StopRequest.model_validate(payload)


def _feedback(**overrides: object) -> ActuatorFeedback:
    payload: dict[str, object] = {
        "measured_linear_x": 0.0,
        "measured_linear_y": 0.0,
        "measured_angular_z": 0.0,
        "correlation_id": "stop-001",
        "sequence_id": 1,
        "timestamp_ns": 110,
    }
    payload.update(overrides)
    return ActuatorFeedback.model_validate(payload)


def test_monitor_starts_idle_and_does_not_ack_without_request(
    monitor: StopAckMonitor,
) -> None:
    assert monitor.state is StopMonitorState.IDLE
    assert monitor.is_latched is False

    tick = monitor.tick(now_ns=100)
    observation = monitor.observe_feedback(_feedback(), now_ns=100)

    assert tick.state is StopMonitorState.IDLE
    assert tick.reason is StopAckReason.NO_STOP_REQUEST
    assert observation.reason is StopAckReason.NO_STOP_REQUEST
    assert monitor.acknowledgement is None
    assert monitor.config is monitor.config
    assert monitor._budget_expired(100) is False
    assert (
        monitor.reset_fault(None, now_ns=100).reason
        is StopAckReason.RESET_NOT_REQUIRED
    )


def test_request_starts_pending_stop_window(monitor: StopAckMonitor) -> None:
    result = monitor.request_stop(_request(), now_ns=100)

    assert result.state is StopMonitorState.STOP_PENDING
    assert result.reason is StopAckReason.STOP_REQUESTED
    assert result.is_latched is False
    assert result.correlation_id == "stop-001"
    assert result.request_sequence_id == 10


def test_fresh_stopped_feedback_emits_auditable_acknowledgement(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)

    result = monitor.observe_feedback(
        _feedback(sequence_id=4, timestamp_ns=120),
        now_ns=120,
    )

    assert result.state is StopMonitorState.STOP_ACKNOWLEDGED
    assert result.reason is StopAckReason.STOP_ACKNOWLEDGED
    assert result.is_latched is False
    assert result.acknowledgement is not None
    assert result.acknowledgement.correlation_id == "stop-001"
    assert result.acknowledgement.request_sequence_id == 10
    assert result.acknowledgement.feedback_sequence_id == 4
    assert result.acknowledgement.sequence_id == 4
    assert result.acknowledgement.stop_latency_ns == 20
    assert monitor.acknowledgement == result.acknowledgement


def test_moving_feedback_remains_pending_until_budget_expires(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)

    missing = monitor.observe_feedback(None, now_ns=110)
    moving = monitor.observe_feedback(
        _feedback(measured_linear_x=0.5, timestamp_ns=120),
        now_ns=120,
    )
    still_pending = monitor.tick(now_ns=199)

    assert missing.reason is StopAckReason.FEEDBACK_MISSING
    assert moving.state is StopMonitorState.STOP_PENDING
    assert moving.reason is StopAckReason.FEEDBACK_MOVING
    assert still_pending.state is StopMonitorState.STOP_PENDING
    assert still_pending.reason is StopAckReason.FEEDBACK_MISSING


def test_missing_feedback_at_budget_boundary_latches_timeout(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)

    result = monitor.tick(now_ns=200)

    assert result.state is StopMonitorState.HARDWARE_FAULT_LATCH
    assert result.reason is StopAckReason.STOP_TIMEOUT
    assert result.is_latched is True
    assert monitor.is_latched is True


def test_invalid_feedback_latches_immediately(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)

    result = monitor.observe_feedback(
        {
            "measured_linear_x": float("nan"),
            "measured_linear_y": 0.0,
            "measured_angular_z": 0.0,
            "correlation_id": "stop-001",
            "sequence_id": 1,
            "timestamp_ns": 110,
        },
        now_ns=110,
    )

    assert result.state is StopMonitorState.HARDWARE_FAULT_LATCH
    assert result.reason is StopAckReason.INVALID_FEEDBACK
    assert monitor.is_latched is True


def test_stale_feedback_latches_immediately(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)

    result = monitor.observe_feedback(
        _feedback(timestamp_ns=110),
        now_ns=160,
    )

    assert result.state is StopMonitorState.HARDWARE_FAULT_LATCH
    assert result.reason is StopAckReason.STALE_FEEDBACK


def test_correlation_mismatch_has_specific_internal_fault_reason(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)

    result = monitor.observe_feedback(
        _feedback(correlation_id="other-stop"),
        now_ns=110,
    )

    assert monitor.is_latched is True
    assert monitor.fault_reason is StopAckReason.FEEDBACK_CORRELATION_MISMATCH
    assert result.reason is StopAckReason.FEEDBACK_CORRELATION_MISMATCH


def test_sequence_regression_latches_replay_fault(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)
    monitor.observe_feedback(
        _feedback(sequence_id=2, measured_linear_x=0.5),
        now_ns=110,
    )

    result = monitor.observe_feedback(
        _feedback(sequence_id=2, measured_linear_x=0.5, timestamp_ns=120),
        now_ns=120,
    )

    assert monitor.is_latched is True
    assert monitor.fault_reason is StopAckReason.FEEDBACK_SEQUENCE_REGRESSION
    assert result.reason is StopAckReason.FEEDBACK_SEQUENCE_REGRESSION


def test_latch_is_sticky_against_feedback_time_and_new_request(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)
    monitor.tick(now_ns=200)

    feedback_result = monitor.observe_feedback(_feedback(), now_ns=300)
    request_result = monitor.request_stop(_request(sequence_id=11), now_ns=301)
    tick_result = monitor.tick(now_ns=302)

    assert feedback_result.reason is StopAckReason.STOP_TIMEOUT
    assert request_result.reason is StopAckReason.STOP_TIMEOUT
    assert tick_result.reason is StopAckReason.STOP_TIMEOUT
    assert monitor.is_latched is True


def test_explicit_valid_reset_clears_latch_but_not_automatically(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)
    monitor.tick(now_ns=200)

    invalid_reset = monitor.reset_fault(None, now_ns=201)
    valid_reset = monitor.reset_fault(
        HardwareResetRequest(reset_id="hw-reset-1", timestamp_ns=210),
        now_ns=210,
    )

    assert invalid_reset.state is StopMonitorState.HARDWARE_FAULT_LATCH
    assert invalid_reset.reason is StopAckReason.RESET_INVALID
    assert valid_reset.state is StopMonitorState.IDLE
    assert valid_reset.reason is StopAckReason.LATCH_RESET
    assert monitor.is_latched is False


@pytest.mark.parametrize(
    "reset, now_ns",
    [
        ({"reset_id": "", "timestamp_ns": 210}, 210),
        ({"reset_id": "r1", "timestamp_ns": 211}, 210),
        ({"reset_id": "r1", "timestamp_ns": 199}, 199),
        ({"reset_id": "r1", "timestamp_ns": 210}, -1),
    ],
)
def test_invalid_reset_protocol_keeps_latch(
    monitor: StopAckMonitor,
    reset: object,
    now_ns: int,
) -> None:
    monitor.request_stop(_request(), now_ns=100)
    monitor.tick(now_ns=200)

    result = monitor.reset_fault(reset, now_ns=now_ns)  # type: ignore[arg-type]

    assert result.state is StopMonitorState.HARDWARE_FAULT_LATCH
    assert result.reason is StopAckReason.RESET_INVALID
    assert monitor.is_latched is True


def test_clock_regression_during_pending_stop_latches(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)

    result = monitor.tick(now_ns=99)

    assert result.state is StopMonitorState.HARDWARE_FAULT_LATCH
    assert monitor.fault_reason is StopAckReason.CLOCK_REGRESSION


def test_invalid_clock_during_pending_stop_latches(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)

    result = monitor.observe_feedback(_feedback(), now_ns=-1)

    assert result.state is StopMonitorState.HARDWARE_FAULT_LATCH
    assert result.reason is StopAckReason.CLOCK_REGRESSION


def test_invalid_stop_request_does_not_start_window(
    monitor: StopAckMonitor,
) -> None:
    missing = monitor.request_stop(None, now_ns=100)
    malformed = monitor.request_stop(
        {"correlation_id": "stop-001", "sequence_id": 10},
        now_ns=100,
    )
    future = monitor.request_stop(
        _request(requested_timestamp_ns=101),
        now_ns=100,
    )

    assert missing.reason is StopAckReason.STOP_REQUEST_INVALID
    assert malformed.state is StopMonitorState.IDLE
    assert malformed.reason is StopAckReason.STOP_REQUEST_INVALID
    assert future.state is StopMonitorState.IDLE
    assert future.reason is StopAckReason.STOP_REQUEST_INVALID


def test_invalid_clock_on_initial_request_is_fail_closed(
    monitor: StopAckMonitor,
) -> None:
    result = monitor.request_stop(_request(), now_ns=-1)

    assert result.state is StopMonitorState.IDLE
    assert result.reason is StopAckReason.CLOCK_REGRESSION


def test_acknowledged_state_is_sticky_until_new_request(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)
    acknowledgement = monitor.observe_feedback(_feedback(), now_ns=110)

    after_budget = monitor.tick(now_ns=500)
    repeated_feedback = monitor.observe_feedback(
        _feedback(sequence_id=2, measured_linear_x=1.0, timestamp_ns=500),
        now_ns=500,
    )

    assert acknowledgement.state is StopMonitorState.STOP_ACKNOWLEDGED
    assert after_budget.reason is StopAckReason.STOP_ACKNOWLEDGED
    assert repeated_feedback.reason is StopAckReason.STOP_ACKNOWLEDGED
    assert monitor.is_latched is False


def test_new_request_resets_acknowledgement_and_can_ack_again(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)
    monitor.observe_feedback(_feedback(), now_ns=110)

    pending = monitor.request_stop(
        _request(correlation_id="stop-002", sequence_id=11),
        now_ns=120,
    )
    acknowledged = monitor.observe_feedback(
        _feedback(correlation_id="stop-002", sequence_id=1, timestamp_ns=125),
        now_ns=125,
    )

    assert pending.state is StopMonitorState.STOP_PENDING
    assert acknowledged.state is StopMonitorState.STOP_ACKNOWLEDGED
    assert acknowledged.correlation_id == "stop-002"


def test_acknowledgement_contract_rejects_impossible_timing() -> None:
    with pytest.raises(ValidationError, match="cannot precede"):
        StopAcknowledgement(
            correlation_id="stop-001",
            request_sequence_id=1,
            feedback_sequence_id=1,
            requested_timestamp_ns=20,
            acknowledged_timestamp_ns=10,
            measured_linear_x=0.0,
            measured_linear_y=0.0,
            measured_angular_z=0.0,
        )


def test_request_and_reset_contracts_are_immutable() -> None:
    request = _request()
    reset = HardwareResetRequest(reset_id="reset-1", timestamp_ns=100)

    with pytest.raises(ValidationError):
        request.sequence_id = 20
    with pytest.raises(ValidationError):
        reset.reset_id = "other"


def test_request_wire_aliases_are_supported() -> None:
    request = StopRequest.model_validate(
        {
            "correlation_id": "stop-001",
            "sequence": 10,
            "requested_timestamp": 100,
        }
    )

    assert request.sequence == 10
    assert request.requested_timestamp == 100

    timestamp_alias_request = StopRequest.model_validate(
        {
            "correlation_id": "stop-001",
            "sequence": 10,
            "timestamp": 100,
        }
    )
    assert timestamp_alias_request.requested_timestamp_ns == 100


def test_request_validation_rejects_non_mapping_payload() -> None:
    with pytest.raises(ValidationError):
        StopRequest.model_validate(1)


def test_result_contract_rejects_inconsistent_latch_and_ack_shapes() -> None:
    with pytest.raises(ValidationError, match="remain latched"):
        StopMonitorResult(
            state=StopMonitorState.HARDWARE_FAULT_LATCH,
            reason=StopAckReason.STOP_TIMEOUT,
            is_latched=False,
            evaluated_timestamp_ns=1,
        )
    with pytest.raises(ValidationError, match="only hardware fault"):
        StopMonitorResult(
            state=StopMonitorState.IDLE,
            reason=StopAckReason.NO_STOP_REQUEST,
            is_latched=True,
            evaluated_timestamp_ns=1,
        )
    with pytest.raises(ValidationError, match="match monitor state"):
        StopMonitorResult(
            state=StopMonitorState.STOP_PENDING,
            reason=StopAckReason.FEEDBACK_MOVING,
            is_latched=False,
            evaluated_timestamp_ns=1,
            acknowledgement=StopAcknowledgement(
                correlation_id="stop-001",
                request_sequence_id=1,
                feedback_sequence_id=1,
                requested_timestamp_ns=1,
                acknowledged_timestamp_ns=1,
                measured_linear_x=0.0,
                measured_linear_y=0.0,
                measured_angular_z=0.0,
            ),
        )


def test_result_wire_enums_are_supported() -> None:
    result = StopMonitorResult.model_validate(
        {
            "state": "idle",
            "reason": "no_stop_request",
            "is_latched": False,
            "evaluated_timestamp_ns": 1,
        }
    )

    assert result.state is StopMonitorState.IDLE
    assert result.reason is StopAckReason.NO_STOP_REQUEST


def test_non_mapping_feedback_is_invalid_and_latches(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)

    result = monitor.observe_feedback(object(), now_ns=110)  # type: ignore[arg-type]

    assert result.state is StopMonitorState.HARDWARE_FAULT_LATCH
    assert result.reason is StopAckReason.INVALID_FEEDBACK


def test_invalid_future_feedback_latches_invalid_data(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)

    result = monitor.observe_feedback(
        _feedback(timestamp_ns=111),
        now_ns=110,
    )

    assert result.state is StopMonitorState.HARDWARE_FAULT_LATCH
    assert monitor.fault_reason is StopAckReason.INVALID_FEEDBACK


def test_reset_wrapper_clears_latch(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)
    monitor.tick(now_ns=200)

    result = monitor.reset(reset_id="hw-reset-2", now_ns=210)

    assert result.state is StopMonitorState.IDLE
    assert result.reason is StopAckReason.LATCH_RESET


def test_non_finite_model_constructed_feedback_latches_invalid_data(
    monitor: StopAckMonitor,
) -> None:
    monitor.request_stop(_request(), now_ns=100)
    feedback = ActuatorFeedback.model_construct(
        measured_linear_x=math.inf,
        measured_linear_y=0.0,
        measured_angular_z=0.0,
        correlation_id="stop-001",
        sequence_id=1,
        timestamp_ns=110,
    )

    result = monitor.observe_feedback(feedback, now_ns=110)

    assert result.state is StopMonitorState.HARDWARE_FAULT_LATCH
    assert monitor.fault_reason is StopAckReason.INVALID_FEEDBACK
