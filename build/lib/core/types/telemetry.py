"""Strict telemetry contracts for Titan Brain modules."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from core.types.identifiers import ModuleId


class RuntimeTelemetry(BaseModel):
    """Runtime measurements captured by a module."""

    model_config = ConfigDict(frozen=True, extra="forbid", strict=True)

    latency_ms: float = Field(ge=0.0)
    execution_time_ms: float = Field(ge=0.0)
    queue_depth: int = Field(ge=0)
    cpu_usage_percent: float = Field(ge=0.0, le=100.0)
    memory_usage_mb: float = Field(ge=0.0)


class CognitiveTelemetry(BaseModel):
    """Cognitive quality measurements captured by a module."""

    model_config = ConfigDict(frozen=True, extra="forbid", strict=True)

    confidence: float = Field(ge=0.0, le=1.0)
    entropy: float = Field(ge=0.0)
    prediction_error: float = Field(ge=0.0)
    novelty: float = Field(ge=0.0)
    uncertainty: float = Field(ge=0.0)


class Telemetry(BaseModel):
    """Versioned telemetry snapshot attached to a message."""

    model_config = ConfigDict(frozen=True, extra="forbid", strict=True)

    schema_version: Literal[1] = 1
    module_id: ModuleId
    timestamp_ns: int = Field(ge=0)
    runtime: RuntimeTelemetry
    cognitive: CognitiveTelemetry
