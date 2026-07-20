"""Immutable delivery result contracts."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict

from core.types.identifiers import MessageId, ModuleId


class DeliveryFailure(BaseModel):
    """A handler failure observed during one dispatch operation."""

    model_config = ConfigDict(frozen=True, extra="forbid", strict=True)

    module_id: ModuleId
    error: str


class DeliveryReport(BaseModel):
    """Immutable outcome of one message dispatch operation."""

    model_config = ConfigDict(frozen=True, extra="forbid", strict=True)

    message_id: MessageId
    delivered_to: tuple[ModuleId, ...]
    failures: tuple[DeliveryFailure, ...]
