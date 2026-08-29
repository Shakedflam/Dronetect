"""Shared video frame primitives and bounded latest-frame helpers."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import json
import struct
from typing import Any


VIDEO_MESSAGE_TYPE_FRAME = "video_frame"
VIDEO_MESSAGE_TYPE_FRAME_BUNDLE = "video_frame_bundle"
JPEG_ENCODING = "image/jpeg"
VIDEO_MAX_PACKET_BYTES = 8 * 1024 * 1024
_HEADER_LENGTH = struct.Struct(">I")


@dataclass(frozen=True, slots=True)
class VideoFrame:
    frame_id: int
    timestamp: float
    width: int
    height: int
    encoding: str
    data: bytes


@dataclass(frozen=True, slots=True)
class VideoFrameBundle:
    ui: VideoFrame
    vision: VideoFrame


class VideoPacketError(ValueError):
    """Raised when a binary video packet cannot be decoded."""


class LatestFrameBuffer:
    """Single-slot async frame handoff that drops stale frames."""

    def __init__(self) -> None:
        self._condition = asyncio.Condition()
        self._frame: VideoFrame | None = None
        self._sequence = 0

    async def publish(self, frame: VideoFrame) -> None:
        async with self._condition:
            self._frame = frame
            self._sequence += 1
            self._condition.notify_all()

    async def wait_next(self, last_sequence: int = 0) -> tuple[int, VideoFrame]:
        async with self._condition:
            await self._condition.wait_for(
                lambda: self._frame is not None and self._sequence > last_sequence
            )
            assert self._frame is not None
            return self._sequence, self._frame

    @property
    def sequence(self) -> int:
        return self._sequence


def encode_video_frame(frame: VideoFrame) -> bytes:
    header = {
        "type": VIDEO_MESSAGE_TYPE_FRAME,
        "frame_id": frame.frame_id,
        "timestamp": frame.timestamp,
        "width": frame.width,
        "height": frame.height,
        "encoding": frame.encoding,
    }
    header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    return _HEADER_LENGTH.pack(len(header_bytes)) + header_bytes + frame.data


def encode_video_frame_bundle(bundle: VideoFrameBundle) -> bytes:
    if bundle.ui.frame_id != bundle.vision.frame_id:
        raise VideoPacketError("Bundled UI and vision frames must share a frame_id.")
    if bundle.ui.timestamp != bundle.vision.timestamp:
        raise VideoPacketError("Bundled UI and vision frames must share a timestamp.")

    header = {
        "type": VIDEO_MESSAGE_TYPE_FRAME_BUNDLE,
        "frame_id": bundle.ui.frame_id,
        "timestamp": bundle.ui.timestamp,
        "frames": {
            "ui": _frame_header(bundle.ui),
            "vision": _frame_header(bundle.vision),
        },
    }
    header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    return (
        _HEADER_LENGTH.pack(len(header_bytes))
        + header_bytes
        + bundle.ui.data
        + bundle.vision.data
    )


def decode_video_frame(packet: bytes) -> VideoFrame:
    return decode_video_frame_bundle(packet).ui


def decode_video_frame_bundle(packet: bytes) -> VideoFrameBundle:
    if len(packet) < _HEADER_LENGTH.size:
        raise VideoPacketError("Video packet is missing its header length.")

    header_length = _HEADER_LENGTH.unpack(packet[: _HEADER_LENGTH.size])[0]
    header_start = _HEADER_LENGTH.size
    header_end = header_start + header_length
    if len(packet) < header_end:
        raise VideoPacketError("Video packet header is truncated.")

    try:
        header = json.loads(packet[header_start:header_end].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise VideoPacketError("Video packet header must be valid JSON.") from exc

    if not isinstance(header, dict):
        raise VideoPacketError("Video packet header must be a JSON object.")
    packet_type = header.get("type")
    if packet_type == VIDEO_MESSAGE_TYPE_FRAME:
        frame = _decode_legacy_frame(header, packet[header_end:])
        return VideoFrameBundle(ui=frame, vision=frame)

    if packet_type != VIDEO_MESSAGE_TYPE_FRAME_BUNDLE:
        raise VideoPacketError("Video packet type must be 'video_frame'.")

    return _decode_frame_bundle(header, packet[header_end:])


def _frame_header(frame: VideoFrame) -> dict[str, Any]:
    return {
        "width": frame.width,
        "height": frame.height,
        "encoding": frame.encoding,
        "byte_length": len(frame.data),
    }


def _decode_legacy_frame(header: dict[str, Any], data: bytes) -> VideoFrame:
    return VideoFrame(
        frame_id=_required_int(header, "frame_id"),
        timestamp=_required_number(header, "timestamp"),
        width=_required_int(header, "width"),
        height=_required_int(header, "height"),
        encoding=_required_string(header, "encoding"),
        data=data,
    )


def _decode_frame_bundle(header: dict[str, Any], data: bytes) -> VideoFrameBundle:
    frames = header.get("frames")
    if not isinstance(frames, dict):
        raise VideoPacketError("Video frame bundle must include frame metadata.")

    frame_id = _required_int(header, "frame_id")
    timestamp = _required_number(header, "timestamp")
    offset = 0
    decoded: dict[str, VideoFrame] = {}
    for lane in ("ui", "vision"):
        metadata = frames.get(lane)
        if not isinstance(metadata, dict):
            raise VideoPacketError(f"Video frame bundle lane '{lane}' is missing.")
        byte_length = _required_int(metadata, "byte_length")
        if byte_length < 0:
            raise VideoPacketError("Video frame byte_length must be non-negative.")
        next_offset = offset + byte_length
        if len(data) < next_offset:
            raise VideoPacketError("Video frame bundle data is truncated.")
        decoded[lane] = VideoFrame(
            frame_id=frame_id,
            timestamp=timestamp,
            width=_required_int(metadata, "width"),
            height=_required_int(metadata, "height"),
            encoding=_required_string(metadata, "encoding"),
            data=data[offset:next_offset],
        )
        offset = next_offset

    if offset != len(data):
        raise VideoPacketError("Video frame bundle has trailing data.")
    return VideoFrameBundle(ui=decoded["ui"], vision=decoded["vision"])


def _required_int(header: dict[str, Any], key: str) -> int:
    value = header.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise VideoPacketError(f"Video packet header field '{key}' must be an integer.")
    return value


def _required_number(header: dict[str, Any], key: str) -> float:
    value = header.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise VideoPacketError(f"Video packet header field '{key}' must be numeric.")
    return float(value)


def _required_string(header: dict[str, Any], key: str) -> str:
    value = header.get(key)
    if not isinstance(value, str) or not value:
        raise VideoPacketError(f"Video packet header field '{key}' must be a string.")
    return value
