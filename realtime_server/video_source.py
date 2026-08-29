"""Common video source boundary for the realtime server."""

from __future__ import annotations

from collections.abc import AsyncIterator
from typing import Protocol

from websockets.asyncio.server import ServerConnection

from services.video import VideoFrameBundle, decode_video_frame_bundle


class VideoSourceConnection(Protocol):
    """Source-agnostic video connection consumed by the realtime server."""

    def iter_frames(self) -> AsyncIterator[VideoFrameBundle]:
        """Yield normalized video frames from a drone camera source."""
        ...

    async def close(self, *, code: int, reason: str) -> None:
        """Close the video source connection."""
        ...


class WebSocketVideoSourceAdapter:
    """Adapts a drone-side video WebSocket to the video source interface."""

    def __init__(self, websocket: ServerConnection) -> None:
        self._websocket = websocket

    async def iter_frames(self) -> AsyncIterator[VideoFrameBundle]:
        async for packet in self._websocket:
            if isinstance(packet, str):
                continue
            yield decode_video_frame_bundle(bytes(packet))

    async def close(self, *, code: int, reason: str) -> None:
        await self._websocket.close(code=code, reason=reason)
