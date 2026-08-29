import asyncio

import pytest

from services.video import (
    JPEG_ENCODING,
    LatestFrameBuffer,
    VideoFrame,
    VideoFrameBundle,
    VideoPacketError,
    decode_video_frame,
    decode_video_frame_bundle,
    encode_video_frame,
    encode_video_frame_bundle,
)


def test_video_packet_round_trips_metadata_and_jpeg_bytes():
    frame = VideoFrame(
        frame_id=42,
        timestamp=1720000000.5,
        width=640,
        height=360,
        encoding=JPEG_ENCODING,
        data=b"fake-jpeg",
    )

    decoded = decode_video_frame(encode_video_frame(frame))

    assert decoded == frame


def test_video_packet_bundle_round_trips_ui_and_vision_lanes():
    bundle = VideoFrameBundle(
        ui=VideoFrame(42, 1720000000.5, 960, 540, JPEG_ENCODING, b"ui-jpeg"),
        vision=VideoFrame(42, 1720000000.5, 1280, 720, JPEG_ENCODING, b"vision-jpeg"),
    )

    decoded = decode_video_frame_bundle(encode_video_frame_bundle(bundle))

    assert decoded == bundle


def test_legacy_video_packet_decodes_as_both_lanes():
    frame = VideoFrame(
        frame_id=42,
        timestamp=1720000000.5,
        width=640,
        height=360,
        encoding=JPEG_ENCODING,
        data=b"fake-jpeg",
    )

    decoded = decode_video_frame_bundle(encode_video_frame(frame))

    assert decoded.ui == frame
    assert decoded.vision == frame


def test_video_packet_rejects_invalid_header():
    with pytest.raises(VideoPacketError):
        decode_video_frame(b"\x00\x00\x00\x20{}")


def test_latest_frame_buffer_drops_stale_frames():
    async def scenario():
        frames = LatestFrameBuffer()
        await frames.publish(
            VideoFrame(1, 1.0, 640, 360, JPEG_ENCODING, b"one")
        )
        await frames.publish(
            VideoFrame(2, 2.0, 640, 360, JPEG_ENCODING, b"two")
        )
        await frames.publish(
            VideoFrame(3, 3.0, 640, 360, JPEG_ENCODING, b"three")
        )

        sequence, frame = await asyncio.wait_for(frames.wait_next(0), 1.0)

        assert sequence == 3
        assert frame.frame_id == 3
        assert frame.data == b"three"

    asyncio.run(scenario())
