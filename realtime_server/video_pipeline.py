"""Realtime video routing for UI WebRTC viewers and vision forwarding."""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from io import BytesIO
from fractions import Fraction
from queue import Empty, Full, Queue

from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame as AvVideoFrame
from PIL import Image
from websockets.exceptions import ConnectionClosed

from config.video import VideoConfig
from realtime_server.protocol import (
    ProtocolError,
    encode_message,
    parse_json_message,
    validate_detection_message,
    validate_role,
)
from services.video import (
    JPEG_ENCODING,
    LatestFrameBuffer,
    VideoFrame,
    VideoFrameBundle,
    encode_video_frame,
)

try:
    from websockets.asyncio.client import connect
except ImportError:
    connect = None


VIDEO_INGEST_PATH = "/video/ingest"
VIDEO_WEBRTC_PATH = "/video/webrtc"
ROLE_VIEWER = "viewer"
RECONNECT_DELAY_S = 1.0
QUEUE_TIMEOUT_S = 0.1


class VideoHub:
    """Latest-frame hub for browser WebRTC viewers."""

    def __init__(self) -> None:
        self._frames = LatestFrameBuffer()

    async def publish(self, frame: VideoFrame) -> None:
        await self._frames.publish(frame)

    def create_track(self) -> VideoStreamTrack:
        return LatestFrameTrack(self._frames)


class LatestFrameTrack(VideoStreamTrack):
    """WebRTC video track that skips stale frames."""

    def __init__(self, frames: LatestFrameBuffer) -> None:
        super().__init__()
        self._frames = frames
        self._last_sequence = 0
        self._pts = 0

    async def recv(self) -> AvVideoFrame:
        self._last_sequence, frame = await self._frames.wait_next(self._last_sequence)
        av_frame = _jpeg_to_av_frame(frame)
        av_frame.pts = self._pts
        av_frame.time_base = Fraction(1, 90000)
        self._pts += 3000
        return av_frame


class VisionFrameForwarder:
    """Forwards latest eligible frames to vision_server without backlog."""

    def __init__(
        self,
        frames_url: str,
        connect_factory: Callable[[str], Awaitable] | None = None,
        detection_handler: Callable[[dict], Awaitable[None]] | None = None,
    ) -> None:
        self.frames_url = frames_url
        self._connect_factory = connect_factory
        self._detection_handler = detection_handler
        self._queue: Queue[VideoFrame] = Queue(maxsize=1)
        self._task: asyncio.Task | None = None
        self._stop = asyncio.Event()

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._connect_forever())

    async def stop(self) -> None:
        self._stop.set()
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    def publish(self, frame: VideoFrame) -> None:
        if self._queue.full():
            try:
                self._queue.get_nowait()
            except Empty:
                pass
        try:
            self._queue.put_nowait(frame)
        except Full:
            pass

    async def _connect_forever(self) -> None:
        while not self._stop.is_set():
            self._drain_stale()
            try:
                connector = self._connect_factory or connect
                if connector is None:
                    await asyncio.sleep(RECONNECT_DELAY_S)
                    continue
                async with connector(self.frames_url) as websocket:
                    await self._exchange_frames_and_detections(websocket)
            except asyncio.CancelledError:
                raise
            except Exception:
                if not self._stop.is_set():
                    await asyncio.sleep(RECONNECT_DELAY_S)

    async def _exchange_frames_and_detections(self, websocket) -> None:
        sender = asyncio.create_task(self._send_frames(websocket))
        receiver = asyncio.create_task(self._receive_detections(websocket))
        done, pending = await asyncio.wait(
            {sender, receiver},
            return_when=asyncio.FIRST_COMPLETED,
        )
        for task in pending:
            task.cancel()
        if pending:
            await asyncio.gather(*pending, return_exceptions=True)
        for task in done:
            task.result()

    async def _send_frames(self, websocket) -> None:
        while not self._stop.is_set():
            try:
                frame = await asyncio.to_thread(
                    self._queue.get,
                    True,
                    QUEUE_TIMEOUT_S,
                )
            except Empty:
                continue
            await websocket.send(encode_video_frame(frame))

    async def _receive_detections(self, websocket) -> None:
        async for raw in websocket:
            if not isinstance(raw, str):
                continue
            try:
                message = validate_detection_message(parse_json_message(raw))
            except ProtocolError:
                continue
            if self._detection_handler is not None:
                await self._detection_handler(message)

    def _drain_stale(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except Empty:
                return


class VideoPipeline:
    """Coordinates video ingest, WebRTC viewers, and vision forwarding."""

    def __init__(
        self,
        config: VideoConfig,
        detection_handler: Callable[[dict], Awaitable[None]] | None = None,
    ) -> None:
        self.config = config
        self.hub = VideoHub()
        self.vision = VisionFrameForwarder(
            config.vision_server_frames_url,
            detection_handler=detection_handler,
        )

    def start(self) -> None:
        self.vision.start()

    async def publish(self, bundle: VideoFrameBundle) -> None:
        await self.hub.publish(bundle.ui)
        if bundle.vision.frame_id % self.config.vision_frame_interval == 0:
            self.vision.publish(bundle.vision)

    async def handle_webrtc_viewer(self, websocket) -> None:
        peer = RTCPeerConnection()
        peer.addTrack(self.hub.create_track())

        try:
            raw = await websocket.recv()
            offer = _validate_offer(parse_json_message(str(raw)))
            await peer.setRemoteDescription(
                RTCSessionDescription(sdp=offer["sdp"], type=offer["type"])
            )
            answer = await peer.createAnswer()
            await peer.setLocalDescription(answer)
            assert peer.localDescription is not None
            await websocket.send(
                encode_message(
                    {
                        "type": peer.localDescription.type,
                        "sdp": peer.localDescription.sdp,
                    }
                )
            )
            async for _ in websocket:
                pass
        except ProtocolError:
            await websocket.send(
                encode_message(
                    ProtocolError(
                        "invalid_webrtc_signal",
                        "WebRTC viewer must send an SDP offer.",
                    ).to_message()
                )
            )
        except ConnectionClosed:
            pass
        finally:
            await peer.close()


def validate_video_role(role: str | None, *, expected: str) -> str:
    if expected == ROLE_VIEWER:
        if role == ROLE_VIEWER:
            return role
        raise ProtocolError(
            "invalid_role",
            "WebSocket role must be 'viewer'.",
            details={"role": role},
        )
    return validate_role(role)


def _jpeg_to_av_frame(frame: VideoFrame) -> AvVideoFrame:
    if frame.encoding != JPEG_ENCODING:
        raise ValueError(f"Unsupported video frame encoding: {frame.encoding}")
    image = Image.open(BytesIO(frame.data)).convert("RGB")
    return AvVideoFrame.from_image(image)


def _validate_offer(message: dict) -> dict[str, str]:
    signal_type = message.get("type")
    sdp = message.get("sdp")
    if signal_type != "offer" or not isinstance(sdp, str) or not sdp:
        raise ProtocolError("invalid_webrtc_signal", "WebRTC viewer must send an SDP offer.")
    return {"type": signal_type, "sdp": sdp}
