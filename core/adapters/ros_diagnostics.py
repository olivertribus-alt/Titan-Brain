"""Dependency-free ROS diagnostic projection for TB-OBS-004."""

from pydantic import Field

from core.observability import EvaluationObservabilityReport
from core.types.incident import StrictFrozenModel


class RosEvaluatorDiagnostic(StrictFrozenModel):
    """ROS-message-shaped primitives derived from an observability report."""

    schema_version: str = Field(min_length=1)
    policy_version: str = Field(min_length=1)
    correlation_id: str = Field(min_length=1)
    decision_id: str
    outcome: str
    latency_status: str
    timing_valid: bool
    within_budget: bool
    observation_to_receive_ns: int = Field(ge=0)
    receive_to_decision_ns: int = Field(ge=0)
    decision_to_publish_ns: int = Field(ge=0)
    end_to_end_ns: int = Field(ge=0)
    exceeded_budgets: tuple[str, ...]
    detail: str
    total_count: int = Field(ge=0)
    normal_count: int = Field(ge=0)
    warning_count: int = Field(ge=0)
    e_stop_count: int = Field(ge=0)
    rejected_count: int = Field(ge=0)
    budget_exceeded_count: int = Field(ge=0)
    invalid_timing_count: int = Field(ge=0)


def to_ros_evaluator_diagnostic(
    report: EvaluationObservabilityReport,
) -> RosEvaluatorDiagnostic:
    """Project a strict core report onto ROS-safe scalar and array fields."""
    counters = report.counters
    return RosEvaluatorDiagnostic(
        schema_version=report.schema_version,
        policy_version=report.policy_version,
        correlation_id=report.correlation_id,
        decision_id=report.decision_id or "none",
        outcome=report.outcome.value,
        latency_status=report.latency_status.value,
        timing_valid=report.timing_valid,
        within_budget=report.within_budget,
        observation_to_receive_ns=report.observation_to_receive_ns or 0,
        receive_to_decision_ns=report.receive_to_decision_ns or 0,
        decision_to_publish_ns=report.decision_to_publish_ns or 0,
        end_to_end_ns=report.end_to_end_ns or 0,
        exceeded_budgets=report.exceeded_budgets,
        detail=report.detail or "none",
        total_count=counters.total,
        normal_count=counters.normal,
        warning_count=counters.warning,
        e_stop_count=counters.e_stop,
        rejected_count=counters.rejected,
        budget_exceeded_count=counters.budget_exceeded,
        invalid_timing_count=counters.invalid_timing,
    )
