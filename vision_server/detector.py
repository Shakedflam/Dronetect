"""Drone detection adapters for the local vision server."""

from __future__ import annotations

from dataclasses import dataclass
from io import BytesIO
import os
from pathlib import Path
from typing import Any, Protocol

from PIL import Image

from services.video import JPEG_ENCODING, VideoFrame


DEFAULT_YOLO_MODEL_PATH = "models/drone.pt"
DEFAULT_YOLO_CONFIDENCE = 0.38
DEFAULT_YOLO_IMAGE_SIZE: int | None = 768
DRONE_CLASS_NAMES = {"drone", "uav", "quadcopter"}


Box = tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class Detection:
    class_name: str
    confidence: float
    box: Box


class DroneDetector(Protocol):
    def detect(self, frame: VideoFrame) -> list[Detection]:
        """Return drone detections for a single video frame."""
        ...


@dataclass(frozen=True, slots=True)
class NoopDroneDetector:
    reason: str = "YOLO detection disabled."

    def detect(self, frame: VideoFrame) -> list[Detection]:
        return []


class YoloDroneDetector:
    def __init__(
        self,
        model: Any,
        *,
        confidence_threshold: float = DEFAULT_YOLO_CONFIDENCE,
        image_size: int | None = DEFAULT_YOLO_IMAGE_SIZE,
    ) -> None:
        self.model = model
        self.confidence_threshold = confidence_threshold
        self.image_size = image_size

    def detect(self, frame: VideoFrame) -> list[Detection]:
        if frame.encoding != JPEG_ENCODING:
            raise ValueError(f"Unsupported vision frame encoding: {frame.encoding}")

        image = Image.open(BytesIO(frame.data)).convert("RGB")
        predict_kwargs: dict[str, Any] = {
            "conf": self.confidence_threshold,
            "verbose": False,
        }
        if self.image_size is not None:
            predict_kwargs["imgsz"] = self.image_size

        results = self.model.predict(image, **predict_kwargs)
        return detections_from_yolo_results(
            results,
            confidence_threshold=self.confidence_threshold,
        )


def create_drone_detector(
    *,
    model_path: str | Path | None = None,
    confidence_threshold: float | None = None,
    image_size: int | None = None,
    logger=print,
) -> DroneDetector:
    resolved_model_path = _resolve_model_path(
        model_path or os.getenv("DRONTECT_YOLO_MODEL_PATH", DEFAULT_YOLO_MODEL_PATH)
    )
    threshold = _confidence_from_env(confidence_threshold)
    resolved_image_size = _image_size_from_env(image_size)

    if not resolved_model_path.exists():
        reason = f"YOLO detection disabled: model not found at {resolved_model_path}"
        logger(reason)
        return NoopDroneDetector(reason)

    try:
        from ultralytics import YOLO  # pyright: ignore[reportMissingImports]
    except Exception as exc:
        reason = f"YOLO detection disabled: failed to import ultralytics ({exc})"
        logger(reason)
        return NoopDroneDetector(reason)

    try:
        model = YOLO(str(resolved_model_path))
    except Exception as exc:
        reason = f"YOLO detection disabled: failed to load {resolved_model_path} ({exc})"
        logger(reason)
        return NoopDroneDetector(reason)

    logger(
        "YOLO drone detector loaded "
        f"model={resolved_model_path} confidence={threshold:.2f} "
        f"imgsz={resolved_image_size or 'default'}"
    )
    return YoloDroneDetector(
        model,
        confidence_threshold=threshold,
        image_size=resolved_image_size,
    )


def detections_from_yolo_results(
    results: Any,
    *,
    confidence_threshold: float = DEFAULT_YOLO_CONFIDENCE,
) -> list[Detection]:
    detections: list[Detection] = []
    for result in results:
        names = getattr(result, "names", {})
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            continue

        xyxy_values = _to_list(getattr(boxes, "xyxy", []))
        confidence_values = _to_list(getattr(boxes, "conf", []))
        class_values = _to_list(getattr(boxes, "cls", []))

        for xyxy, confidence, class_id in zip(
            xyxy_values,
            confidence_values,
            class_values,
        ):
            confidence_float = float(confidence)
            if confidence_float < confidence_threshold:
                continue

            class_index = int(class_id)
            class_name = _class_name(names, class_index)
            if not _is_drone_class(class_name, class_index):
                continue

            x1, y1, x2, y2 = (float(value) for value in xyxy)
            detections.append(
                Detection(
                    class_name=class_name,
                    confidence=confidence_float,
                    box=(x1, y1, x2, y2),
                )
            )
    return detections


def _resolve_model_path(path: str | Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return Path(__file__).resolve().parents[1] / candidate


def _confidence_from_env(explicit: float | None) -> float:
    if explicit is not None:
        return explicit
    raw = os.getenv("DRONTECT_YOLO_CONFIDENCE")
    if raw is None:
        return DEFAULT_YOLO_CONFIDENCE
    try:
        value = float(raw)
    except ValueError:
        return DEFAULT_YOLO_CONFIDENCE
    if 0.0 <= value <= 1.0:
        return value
    return DEFAULT_YOLO_CONFIDENCE


def _image_size_from_env(explicit: int | None) -> int | None:
    if explicit is not None:
        return explicit if explicit > 0 else DEFAULT_YOLO_IMAGE_SIZE
    raw = os.getenv("DRONTECT_YOLO_IMGSZ")
    if raw is None:
        return DEFAULT_YOLO_IMAGE_SIZE
    try:
        value = int(raw)
    except ValueError:
        return DEFAULT_YOLO_IMAGE_SIZE
    if value > 0:
        return value
    return DEFAULT_YOLO_IMAGE_SIZE


def _to_list(value: Any) -> list:
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "tolist"):
        return value.tolist()
    return list(value)


def _class_name(names: Any, class_index: int) -> str:
    if isinstance(names, dict):
        raw = names.get(class_index, str(class_index))
    elif isinstance(names, list) and 0 <= class_index < len(names):
        raw = names[class_index]
    else:
        raw = str(class_index)
    return str(raw)


def _is_drone_class(class_name: str, class_index: int) -> bool:
    return class_index == 0 or class_name.strip().lower() in DRONE_CLASS_NAMES
