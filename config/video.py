"""Configuration for the local video pipeline."""

from __future__ import annotations

from dataclasses import dataclass, replace
import json
import os
from pathlib import Path
from typing import Any


DEFAULT_VIDEO_CONFIG_PATH = Path(__file__).with_name("video.json")


@dataclass(frozen=True, slots=True)
class VideoConfig:
    source_target_fps: int = 30
    ui_max_width: int = 960
    ui_max_height: int = 540
    ui_jpeg_quality: int = 85
    vision_frame_interval: int = 2
    vision_max_width: int = 960
    vision_max_height: int = 540
    vision_jpeg_quality: int = 92
    vision_tracker_lost_after_seconds: float = 0.25
    vision_tracker_remove_after_seconds: float = 0.80
    vision_tracker_max_velocity_px_per_sec: float = 5000.0
    vision_tracker_min_center_distance_frame_diagonals: float = 0.20
    vision_server_frames_url: str = "ws://127.0.0.1:8870/frames"
    vision_server_diagnostics_host: str = "127.0.0.1"
    vision_server_diagnostics_port: int = 8871


def load_video_config(path: str | Path | None = None) -> VideoConfig:
    config_path = Path(path) if path is not None else DEFAULT_VIDEO_CONFIG_PATH
    if not config_path.exists():
        return _apply_deployment_env_overrides(VideoConfig())

    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return _apply_deployment_env_overrides(VideoConfig())

    if not isinstance(raw, dict):
        return _apply_deployment_env_overrides(VideoConfig())

    defaults = VideoConfig()
    return _apply_deployment_env_overrides(VideoConfig(
        source_target_fps=_positive_int(
            raw.get("source_target_fps"),
            defaults.source_target_fps,
        ),
        ui_max_width=_positive_int(
            raw.get("ui_max_width"),
            defaults.ui_max_width,
        ),
        ui_max_height=_positive_int(
            raw.get("ui_max_height"),
            defaults.ui_max_height,
        ),
        ui_jpeg_quality=_jpeg_quality(
            raw.get("ui_jpeg_quality"),
            defaults.ui_jpeg_quality,
        ),
        vision_frame_interval=_positive_int(
            raw.get("vision_frame_interval"),
            defaults.vision_frame_interval,
        ),
        vision_max_width=_positive_int(
            raw.get("vision_max_width"),
            defaults.vision_max_width,
        ),
        vision_max_height=_positive_int(
            raw.get("vision_max_height"),
            defaults.vision_max_height,
        ),
        vision_jpeg_quality=_jpeg_quality(
            raw.get("vision_jpeg_quality"),
            defaults.vision_jpeg_quality,
        ),
        vision_tracker_lost_after_seconds=_positive_float(
            raw.get("vision_tracker_lost_after_seconds"),
            defaults.vision_tracker_lost_after_seconds,
        ),
        vision_tracker_remove_after_seconds=_positive_float(
            raw.get("vision_tracker_remove_after_seconds"),
            defaults.vision_tracker_remove_after_seconds,
        ),
        vision_tracker_max_velocity_px_per_sec=_positive_float(
            raw.get("vision_tracker_max_velocity_px_per_sec"),
            defaults.vision_tracker_max_velocity_px_per_sec,
        ),
        vision_tracker_min_center_distance_frame_diagonals=_positive_float(
            raw.get("vision_tracker_min_center_distance_frame_diagonals"),
            defaults.vision_tracker_min_center_distance_frame_diagonals,
        ),
        vision_server_frames_url=_non_empty_string(
            raw.get("vision_server_frames_url"),
            defaults.vision_server_frames_url,
        ),
        vision_server_diagnostics_host=_non_empty_string(
            raw.get("vision_server_diagnostics_host"),
            defaults.vision_server_diagnostics_host,
        ),
        vision_server_diagnostics_port=_positive_int(
            raw.get("vision_server_diagnostics_port"),
            defaults.vision_server_diagnostics_port,
        ),
    ))


def _apply_deployment_env_overrides(config: VideoConfig) -> VideoConfig:
    """Cross-host overrides for pointing realtime_server at a non-local
    vision_server (e.g. a separate cloud container). Unset by default, so
    behavior is unchanged unless these env vars are explicitly set."""
    updates: dict[str, Any] = {}

    frames_url = os.getenv("DRONTECT_VISION_FRAMES_URL")
    if frames_url:
        updates["vision_server_frames_url"] = frames_url

    diagnostics_host = os.getenv("DRONTECT_VISION_DIAGNOSTICS_HOST")
    if diagnostics_host:
        updates["vision_server_diagnostics_host"] = diagnostics_host

    diagnostics_port = os.getenv("DRONTECT_VISION_DIAGNOSTICS_PORT")
    if diagnostics_port:
        parsed_port = _positive_int(int(diagnostics_port), 0) if diagnostics_port.isdigit() else 0
        if parsed_port:
            updates["vision_server_diagnostics_port"] = parsed_port

    return replace(config, **updates) if updates else config


def _positive_int(value: Any, fallback: int) -> int:
    if isinstance(value, bool):
        return fallback
    if isinstance(value, int) and value > 0:
        return value
    return fallback


def _jpeg_quality(value: Any, fallback: int) -> int:
    if isinstance(value, bool):
        return fallback
    if isinstance(value, int) and 1 <= value <= 95:
        return value
    return fallback


def _positive_float(value: Any, fallback: float) -> float:
    if isinstance(value, bool):
        return fallback
    if isinstance(value, (int, float)) and value > 0:
        return float(value)
    return fallback


def _non_empty_string(value: Any, fallback: str) -> str:
    if isinstance(value, str) and value:
        return value
    return fallback
