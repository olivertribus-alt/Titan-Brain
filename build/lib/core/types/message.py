"""Core message envelopes for Titan Brain communication."""

from __future__ import annotations

from typing import Generic, Literal, TypeVar

from pydantic import BaseModel, ConfigDict, Field

from core.types.identifiers import MessageId, ModuleId, TraceId
from core.types.json_value import JsonValue
from core.types.priority import Priority
from core.types.telemetry import Telemetry

PayloadT = TypeVar("PayloadT")


class Broadcast(BaseModel):
    """Destination for messages targeted at every module."""

    model_config = ConfigDict(frozen=True, extra="forbid", strict=True)

    type: Literal["BROADCAST"] = "BROADCAST"


Destination = ModuleId | Broadcast


class MessageEnvelope(BaseModel, Generic[PayloadT]):
    """Common message envelope with a configurable payload representation."""

    model_config = ConfigDict(frozen=True, extra="forbid", strict=True)

    message_id: MessageId
    trace_id: TraceId
    correlation_id: MessageId | None = None
    source: ModuleId
    destination: Destination
    timestamp_ns: int = Field(ge=0)
    priority: Priority
    telemetry: Telemetry
    payload: PayloadT


class IncomingMessageEnvelope(MessageEnvelope[dict[str, JsonValue]]):
    """Phase-one envelope with an unvalidated JSON payload object."""


class CognitiveMessage(MessageEnvelope[PayloadT], Generic[PayloadT]):
    """Phase-two envelope with a validated concrete payload."""
