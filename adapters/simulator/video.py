"""Video client for the Webots simulator camera."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from io import BytesIO
import os
from queue import Empty, Full, Queue
from threading import Event, Thread
import time
import traceback
from typing import Any

from config.video import VideoConfig, load_video_config
from services.video import (
    JPEG_ENCODING,
    VideoFrame,
    VideoFrameBundle,
    encode_video_frame_bundle,
)

try:
    from PIL import Image
except ImportError:  # The simulator should still fly if video deps are absent.
    Image = None

try:
    from websockets.asyncio.client import connect
except ImportError:  # The simulator should still fly if server deps are absent.
    connect = None


DEFAULT_VIDEO_INGEST_URL = "ws://127.0.0.1:8765/video/ingest?role=drone"
RECONNECT_DELAY_S = 1.0
QUEUE_TIMEOUT_S = 0.1


@dataclass(frozen=True, slots=True)
class RawCameraFrame:
    frame_id: int
    timestamp: float
    width: int
    height: int
    bgra: bytes


class SimulatorVideoClient:
    """Non-blocking simulator video adapter with latest-frame queues."""

    def __init__(self, url: str | None = None, config: VideoConfig | None = None) -> None:
        self.config = config or load_video_config()
        self.url = url or os.getenv("DRONTECT_SIMULATOR_VIDEO_WS_URL", DEFAULT_VIDEO_INGEST_URL)
        self._raw_frames: Queue[RawCameraFrame] = Queue(maxsize=1)
        self._outgoing: Queue[bytes] = Queue(maxsize=1)
        self._stop = Event()
        self._thread: Thread | None = None
        self._frame_id = 0
        self._last_capture_monotonic = 0.0

    def start(self) -> None:
        if connect is None or Image is None or self._thread is not None:
            return

        self._thread = Thread(
            target=self._run_loop,
            name="drontect-simulator-video-client",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def publish_camera_frame(self, camera: Any) -> None:
        if connect is None or Image is None:
            return

        now = time.monotonic()
        min_period = 1.0 / self.config.source_target_fps
        if now - self._last_capture_monotonic < min_period:
            return

        self._last_capture_monotonic = now
        self._frame_id += 1
        raw_frame = RawCameraFrame(
            frame_id=self._frame_id,
            timestamp=time.time(),
            width=int(camera.getWidth()),
            height=int(camera.getHeight()),
            bgra=bytes(camera.getImage()),
        )
        _put_latest(self._raw_frames, raw_frame)

    def _run_loop(self) -> None:
        asyncio.run(self._run())

    async def _run(self) -> None:
        encoder = asyncio.create_task(self._encode_frames())
        sender = asyncio.create_task(self._connect_forever())
        try:
            await asyncio.wait({encoder, sender}, return_when=asyncio.FIRST_COMPLETED)
        finally:
            encoder.cancel()
            sender.cancel()

    async def _encode_frames(self) -> None:
        while not self._stop.is_set():
            try:
                raw_frame = await asyncio.to_thread(
                    self._raw_frames.get,
                    True,
                    QUEUE_TIMEOUT_S,
                )
            except Empty:
                continue
            try:
                packet = await asyncio.to_thread(
                    _encode_camera_frame_bundle,
                    raw_frame,
                    self.config,
                )
            except Exception:
                print("Simulator video encoding failed:")
                traceback.print_exc()
                continue
            _put_latest(self._outgoing, packet)

    async def _connect_forever(self) -> None:
        while not self._stop.is_set():
            try:
                assert connect is not None
                async with connect(self.url) as websocket:
                    await self._send_frames(websocket)
            except Exception:
                if not self._stop.is_set():
                    print(f"Simulator video connection failed; retrying {self.url}")
                    traceback.print_exc()
                    await asyncio.sleep(RECONNECT_DELAY_S)

    async def _send_frames(self, websocket) -> None:
        while not self._stop.is_set():
            try:
                packet = await asyncio.to_thread(
                    self._outgoing.get,
                    True,
                    QUEUE_TIMEOUT_S,
                )
            except Empty:
                continue
            await websocket.send(packet)


def _encode_camera_frame_bundle(raw_frame: RawCameraFrame, config: VideoConfig) -> bytes:
    assert Image is not None
    image = Image.frombytes(
        "RGBA",
        (raw_frame.width, raw_frame.height),
        raw_frame.bgra,
        "raw",
        "BGRA",
    ).convert("RGB")

    ui_frame = _encode_lane_frame(
        image,
        frame_id=raw_frame.frame_id,
        timestamp=raw_frame.timestamp,
        max_width=config.ui_max_width,
        max_height=config.ui_max_height,
        quality=config.ui_jpeg_quality,
    )
    vision_frame = _encode_lane_frame(
        image,
        frame_id=raw_frame.frame_id,
        timestamp=raw_frame.timestamp,
        max_width=config.vision_max_width,
        max_height=config.vision_max_height,
        quality=config.vision_jpeg_quality,
    )
    return encode_video_frame_bundle(VideoFrameBundle(ui=ui_frame, vision=vision_frame))


def _encode_lane_frame(
    image,
    *,
    frame_id: int,
    timestamp: float,
    max_width: int,
    max_height: int,
    quality: int,
) -> VideoFrame:
    encoded = _resize_to_fit(image, max_width, max_height)
    buffer = BytesIO()
    encoded.save(
        buffer,
        format="JPEG",
        quality=quality,
        optimize=False,
    )
    return VideoFrame(
        frame_id=frame_id,
        timestamp=timestamp,
        width=encoded.width,
        height=encoded.height,
        encoding=JPEG_ENCODING,
        data=buffer.getvalue(),
    )


def _resize_to_fit(image, max_width: int, max_height: int):
    if image.width <= max_width and image.height <= max_height:
        return image

    scale = min(max_width / image.width, max_height / image.height)
    width = max(1, int(round(image.width * scale)))
    height = max(1, int(round(image.height * scale)))
    # LANCZOS gives a visibly sharper downscale than bilinear for a modest,
    # one-time CPU cost per frame — worth it since this runs once per frame,
    # not per pixel-neighborhood lookup during playback.
    return image.resize((width, height), Image.Resampling.LANCZOS)


def _put_latest(queue: Queue, item: object) -> None:
    if queue.full():
        try:
            queue.get_nowait()
        except Empty:
            pass
    try:
        queue.put_nowait(item)
    except Full:
        pass
