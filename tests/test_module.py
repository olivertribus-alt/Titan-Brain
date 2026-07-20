"""Tests for BaseModule lifecycle behavior."""

from __future__ import annotations

import asyncio
from typing import Literal

import pytest
from pydantic import ConfigDict

from core.bus import CognitiveBus, UnknownPayloadError
from core.module import BaseModule
from core.types.identifiers import (
    ModuleId,
    create_capability_id,
    create_message_id,
    create_module_id,
    create_trace_id,
)
from core.types.infrastructure import Capability, ModuleState
from core.types.message import Broadcast, CognitiveMessage, IncomingMessageEnvelope
from core.types.payload import BasePayload
from core.types.priority import Priority
from core.types.telemetry import CognitiveTelemetry, RuntimeTelemetry, Telemetry


class LifecyclePayload(BasePayload):
    """Concrete payload used by lifecycle tests."""

    model_config = ConfigDict(frozen=True, extra="forbid", strict=True)

    schema_version: Literal[1] = 1
    payload_type: Literal["lifecycle"] = "lifecycle"
    value: int


class TestModule(BaseModule):
    """Minimal concrete module used to exercise the base lifecycle."""

    __test__ = False

    def __init__(self, module_id: ModuleId, bus: CognitiveBus) -> None:
        super().__init__(module_id, bus)
        self.events: list[str] = []

    @property
    def capability(self) -> Capability:
        return Capability(
            capability_id=create_capability_id("lifecycle_consumer"),
            consumes=(LifecyclePayload,),
            produces=(),
            version="1",
        )

    async def on_start(self) -> None:
        self.events.append("started")

    async def on_stop(self) -> None:
        self.events.append("stopped")

    async def handle(self, message: CognitiveMessage[BasePayload]) -> None:
        assert isinstance(message.payload, LifecyclePayload)
        self.events.append(f"handled:{message.payload.value}")


def _telemetry(module_id: ModuleId) -> Telemetry:
    return Telemetry(
        module_id=module_id,
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


def _incoming(source: ModuleId) -> IncomingMessageEnvelope:
    return IncomingMessageEnvelope(
        message_id=create_message_id(),
        trace_id=create_trace_id(),
        source=source,
        destination=Broadcast(),
        timestamp_ns=1,
        priority=Priority.NORMAL,
        telemetry=_telemetry(source),
        payload={"schema_version": 1, "payload_type": "lifecycle", "value": 7},
    )


def test_module_start_stop_and_restart_manage_bus_registration() -> None:
    bus = CognitiveBus()
    module_id = create_module_id("LifecycleModule")
    module = TestModule(module_id, bus)

    asyncio.run(module.start())

    assert module.state is ModuleState.READY
    asyncio.run(bus.send(_incoming(create_module_id("Source"))))
    assert module.events == ["started", "handled:7"]

    asyncio.run(module.stop())

    assert module.state.value == "STOPPED"
    assert module.events[-1] == "stopped"
    with pytest.raises(UnknownPayloadError):
        asyncio.run(bus.send(_incoming(create_module_id("Source"))))

    asyncio.run(module.start())

    assert module.state is ModuleState.READY
    assert module.events[-1] == "started"
