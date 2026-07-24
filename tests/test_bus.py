"""Tests for CognitiveBus registration and delivery behavior."""

from __future__ import annotations

import asyncio
from typing import Literal

import pytest
from pydantic import ConfigDict

from core.bus import CognitiveBus, InvalidDestinationError, UnknownPayloadError
from core.types.identifiers import (
    ModuleId,
    create_capability_id,
    create_message_id,
    create_module_id,
    create_trace_id,
)
from core.types.infrastructure import Capability
from core.types.message import (
    Broadcast,
    CognitiveMessage,
    Destination,
    IncomingMessageEnvelope,
)
from core.types.payload import BasePayload
from core.types.priority import Priority
from core.types.telemetry import CognitiveTelemetry, RuntimeTelemetry, Telemetry


class AnalysisPayload(BasePayload):
    """Concrete payload used to verify bus behavior."""

    model_config = ConfigDict(frozen=True, extra="forbid", strict=True)

    schema_version: Literal[1] = 1
    payload_type: Literal["analysis"] = "analysis"
    score: float


def _module_id(name: str) -> ModuleId:
    return create_module_id(name)


def _telemetry(module_id: str) -> Telemetry:
    return Telemetry(
        module_id=_module_id(module_id),
        timestamp_ns=1,
        runtime=RuntimeTelemetry(
            latency_ms=1.0,
            execution_time_ms=1.0,
            queue_depth=0,
            cpu_usage_percent=1.0,
            memory_usage_mb=1.0,
        ),
        cognitive=CognitiveTelemetry(
            confidence=0.5,
            entropy=0.1,
            prediction_error=0.1,
            novelty=0.1,
            uncertainty=0.1,
        ),
    )


def _incoming(destination: Destination) -> IncomingMessageEnvelope:
    return IncomingMessageEnvelope(
        message_id=create_message_id(),
        trace_id=create_trace_id(),
        source=_module_id("Source"),
        destination=destination,
        timestamp_ns=1,
        priority=Priority.NORMAL,
        telemetry=_telemetry("Source"),
        payload={"schema_version": 1, "payload_type": "analysis", "score": 0.8},
    )


def _capability() -> Capability:
    return Capability(
        capability_id=create_capability_id("analysis_consumer"),
        consumes=(AnalysisPayload,),
        produces=(),
        version="1",
    )


def test_broadcast_delivers_only_to_subscribers() -> None:
    bus = CognitiveBus()
    received: list[float] = []

    async def handler(message: CognitiveMessage[BasePayload]) -> None:
        assert isinstance(message.payload, AnalysisPayload)
        received.append(message.payload.score)

    bus.register_module(_module_id("Consumer"), _capability(), handler)

    report = asyncio.run(bus.send(_incoming(Broadcast())))

    assert report.delivered_to == (_module_id("Consumer"),)
    assert report.failures == ()
    assert received == [0.8]


def test_targeted_delivery_success() -> None:
    bus = CognitiveBus()
    received: list[float] = []

    async def handler(message: CognitiveMessage[BasePayload]) -> None:
        assert isinstance(message.payload, AnalysisPayload)
        received.append(message.payload.score)

    target_id = _module_id("TargetModule")
    bus.register_module(target_id, _capability(), handler)

    report = asyncio.run(bus.send(_incoming(target_id)))

    assert report.delivered_to == (target_id,)
    assert report.failures == ()
    assert received == [0.8]


def test_targeted_delivery_rejects_non_consumer() -> None:
    bus = CognitiveBus()

    async def handler(message: CognitiveMessage[BasePayload]) -> None:
        return None

    bus.register_module(_module_id("Consumer"), _capability(), handler)

    with pytest.raises(InvalidDestinationError):
        asyncio.run(bus.send(_incoming(_module_id("OtherModule"))))


def test_targeted_delivery_rejects_unregistered_module() -> None:
    bus = CognitiveBus()
    async def handler(message: CognitiveMessage[BasePayload]) -> None:
        pass
    bus.register_module(_module_id("Dummy"), _capability(), handler)
    with pytest.raises(InvalidDestinationError):
        asyncio.run(bus.send(_incoming(_module_id("NonExistent"))))


def test_unknown_payload_type_is_rejected() -> None:
    bus = CognitiveBus()
    raw = _incoming(Broadcast()).model_copy(
        update={
            "payload": {
                "schema_version": 1,
                "payload_type": "unknown",
                "score": 0.8,
            }
        }
    )

    with pytest.raises(UnknownPayloadError):
        asyncio.run(bus.send(raw))


def test_broadcast_reports_handler_failure_without_stopping_delivery() -> None:
    bus = CognitiveBus()
    received: list[float] = []

    async def successful_handler(message: CognitiveMessage[BasePayload]) -> None:
        assert isinstance(message.payload, AnalysisPayload)
        received.append(message.payload.score)

    async def failing_handler(message: CognitiveMessage[BasePayload]) -> None:
        raise RuntimeError("handler failed")

    bus.register_module(_module_id("Alpha"), _capability(), failing_handler)
    bus.register_module(_module_id("Bravo"), _capability(), successful_handler)

    report = asyncio.run(bus.send(_incoming(Broadcast())))

    assert report.delivered_to == (_module_id("Bravo"),)
    assert report.failures[0].module_id == _module_id("Alpha")
    assert received == [0.8]


def test_module_unregister() -> None:
    bus = CognitiveBus()

    async def handler(message: CognitiveMessage[BasePayload]) -> None:
        pass

    mod_id = _module_id("TempModule")
    bus.register_module(mod_id, _capability(), handler)
    bus.unregister_module(mod_id)
