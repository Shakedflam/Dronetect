"""Common drone SDK connection boundary used by the realtime server."""

from __future__ import annotations

from collections.abc import AsyncIterator
from typing import Any, Protocol

from websockets.asyncio.server import ServerConnection

from realtime_server.protocol import (
    MESSAGE_TYPE_OPERATING_CONFIG_RESULT,
    ProtocolError,
    encode_message,
    parse_json_message,
    validate_telemetry_message,
    validate_operating_config_result_message,
)


class DroneSdkConnection(Protocol):
    """Source-agnostic drone connection consumed by the realtime server."""

    def iter_telemetry(self) -> AsyncIterator[dict[str, Any]]:
        """Yield normalized telemetry and controller-result messages."""
        ...

    async def send_command(self, command: dict[str, Any]) -> None:
        """Send a normalized command message to the connected drone source."""
        ...

    async def send_error(self, error: ProtocolError) -> None:
        """Send a protocol error to the connected drone source."""
        ...

    async def close(self, *, code: int, reason: str) -> None:
        """Close the connection to the connected drone source."""
        ...


class WebSocketDroneSdkAdapter:
    """Adapts a drone-side WebSocket client to the common SDK connection."""

    def __init__(self, websocket: ServerConnection) -> None:
        self._websocket = websocket

    async def iter_telemetry(self) -> AsyncIterator[dict[str, Any]]:
        async for raw in self._websocket:
            try:
                message = parse_json_message(str(raw))
                if message.get("type") == MESSAGE_TYPE_OPERATING_CONFIG_RESULT:
                    yield validate_operating_config_result_message(message)
                else:
                    yield validate_telemetry_message(message)
            except ProtocolError as exc:
                await self._websocket.send(encode_message(exc.to_message()))

    async def send_command(self, command: dict[str, Any]) -> None:
        await self._websocket.send(encode_message(command))

    async def send_error(self, error: ProtocolError) -> None:
        await self._websocket.send(encode_message(error.to_message()))

    async def close(self, *, code: int, reason: str) -> None:
        await self._websocket.close(code=code, reason=reason)
