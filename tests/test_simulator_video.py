from adapters.simulator.video import RawCameraFrame, _encode_camera_frame_bundle
from config.video import VideoConfig
from services.video import decode_video_frame_bundle


def test_vision_lane_does_not_upscale_beyond_source_resolution():
    raw_frame = RawCameraFrame(
        frame_id=1,
        timestamp=1.0,
        width=960,
        height=540,
        bgra=bytes([30, 80, 120, 255]) * (960 * 540),
    )

    packet = _encode_camera_frame_bundle(
        raw_frame,
        VideoConfig(
            vision_max_width=1280,
            vision_max_height=720,
        ),
    )
    bundle = decode_video_frame_bundle(packet)

    assert bundle.vision.width == 960
    assert bundle.vision.height == 540


def test_1280_source_downsamples_ui_and_keeps_native_vision():
    raw_frame = RawCameraFrame(
        frame_id=1,
        timestamp=1.0,
        width=1280,
        height=720,
        bgra=bytes([30, 80, 120, 255]) * (1280 * 720),
    )

    packet = _encode_camera_frame_bundle(
        raw_frame,
        VideoConfig(
            vision_max_width=1280,
            vision_max_height=720,
        ),
    )
    bundle = decode_video_frame_bundle(packet)

    assert bundle.ui.width == 960
    assert bundle.ui.height == 540
    assert bundle.vision.width == 1280
    assert bundle.vision.height == 720
