import json

from config.video import VideoConfig, load_video_config


def test_video_config_uses_defaults_when_file_is_missing(tmp_path):
    config = load_video_config(tmp_path / "missing.json")

    assert config == VideoConfig()


def test_video_config_reads_frame_rates_and_vision_urls(tmp_path):
    path = tmp_path / "video.json"
    path.write_text(
        json.dumps(
            {
                "source_target_fps": 25,
                "ui_max_width": 960,
                "ui_max_height": 540,
                "ui_jpeg_quality": 74,
                "vision_frame_interval": 3,
                "vision_max_width": 1280,
                "vision_max_height": 720,
                "vision_jpeg_quality": 91,
                "vision_tracker_lost_after_seconds": 0.4,
                "vision_tracker_remove_after_seconds": 1.2,
                "vision_tracker_max_velocity_px_per_sec": 3200.0,
                "vision_tracker_min_center_distance_frame_diagonals": 0.18,
                "vision_server_frames_url": "ws://127.0.0.1:9999/frames",
                "vision_server_diagnostics_host": "0.0.0.0",
                "vision_server_diagnostics_port": 9998,
            }
        ),
        encoding="utf-8",
    )

    config = load_video_config(path)

    assert config.source_target_fps == 25
    assert config.ui_max_width == 960
    assert config.ui_max_height == 540
    assert config.ui_jpeg_quality == 74
    assert config.vision_frame_interval == 3
    assert config.vision_max_width == 1280
    assert config.vision_max_height == 720
    assert config.vision_jpeg_quality == 91
    assert config.vision_tracker_lost_after_seconds == 0.4
    assert config.vision_tracker_remove_after_seconds == 1.2
    assert config.vision_tracker_max_velocity_px_per_sec == 3200.0
    assert config.vision_tracker_min_center_distance_frame_diagonals == 0.18
    assert config.vision_server_frames_url == "ws://127.0.0.1:9999/frames"
    assert config.vision_server_diagnostics_host == "0.0.0.0"
    assert config.vision_server_diagnostics_port == 9998


def test_video_config_rejects_invalid_frame_counts(tmp_path):
    path = tmp_path / "video.json"
    path.write_text(
        json.dumps(
            {
                "source_target_fps": 0,
                "ui_max_width": -1,
                "ui_max_height": False,
                "ui_jpeg_quality": 100,
                "vision_frame_interval": -1,
                "vision_max_width": 0,
                "vision_max_height": "720",
                "vision_jpeg_quality": 0,
                "vision_tracker_lost_after_seconds": 0,
                "vision_tracker_remove_after_seconds": "1.2",
                "vision_tracker_max_velocity_px_per_sec": False,
                "vision_tracker_min_center_distance_frame_diagonals": -0.1,
                "vision_server_diagnostics_port": True,
            }
        ),
        encoding="utf-8",
    )

    config = load_video_config(path)
    defaults = VideoConfig()

    assert config.source_target_fps == defaults.source_target_fps
    assert config.ui_max_width == defaults.ui_max_width
    assert config.ui_max_height == defaults.ui_max_height
    assert config.ui_jpeg_quality == defaults.ui_jpeg_quality
    assert config.vision_frame_interval == defaults.vision_frame_interval
    assert config.vision_max_width == defaults.vision_max_width
    assert config.vision_max_height == defaults.vision_max_height
    assert config.vision_jpeg_quality == defaults.vision_jpeg_quality
    assert config.vision_tracker_lost_after_seconds == defaults.vision_tracker_lost_after_seconds
    assert config.vision_tracker_remove_after_seconds == defaults.vision_tracker_remove_after_seconds
    assert config.vision_tracker_max_velocity_px_per_sec == defaults.vision_tracker_max_velocity_px_per_sec
    assert (
        config.vision_tracker_min_center_distance_frame_diagonals
        == defaults.vision_tracker_min_center_distance_frame_diagonals
    )
    assert config.vision_server_diagnostics_port == defaults.vision_server_diagnostics_port
