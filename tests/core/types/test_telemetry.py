"""Tests for telemetry validation."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from core.types.identifiers import create_module_id
from core.types.telemetry import CognitiveTelemetry, RuntimeTelemetry, Telemetry


def _runtime() -> RuntimeTelemetry:
    return RuntimeTelemetry(
        latency_ms=1.0,
        execution_time_ms=2.0,
        queue_depth=0,
        cpu_usage_percent=50.0,
        memory_usage_mb=128.0,
    )


def _cognitive() -> CognitiveTelemetry:
    return CognitiveTelemetry(
        confidence=0.9,
        entropy=0.1,
        prediction_error=0.1,
        novelty=0.2,
        uncertainty=0.1,
    )


def test_telemetry_rejects_invalid_confidence_and_is_immutable() -> None:
    with pytest.raises(ValidationError):
        CognitiveTelemetry(
            confidence=1.1,
            entropy=0.1,
            prediction_error=0.1,
            novelty=0.2,
            uncertainty=0.1,
        )

    telemetry = Telemetry(
        module_id=create_module_id("Analyzer"),
        timestamp_ns=1,
        runtime=_runtime(),
        cognitive=_cognitive(),
    )
    with pytest.raises(ValidationError):
        telemetry.timestamp_ns = 2
