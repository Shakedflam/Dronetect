"""Opt-in model regression test for simulator drone detection fixtures."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
from time import perf_counter
from typing import Any

import pytest


TESTS_ROOT = Path(__file__).resolve().parent
REPO_ROOT = TESTS_ROOT.parent
FIXTURE_ROOT = TESTS_ROOT / "fixtures" / "choose_model"
IMAGES_ROOT = FIXTURE_ROOT / "images"
MANIFEST_PATH = FIXTURE_ROOT / "manifest.json"
REPORT_PATH = REPO_ROOT / "artifacts" / "choose_model_last_report.json"


@dataclass(frozen=True, slots=True)
class ModelConfig:
    name: str
    path: Path
    image_size: int
    confidence: float


MODEL_CONFIGS = [
    ModelConfig("drone_960_conf025", REPO_ROOT / "models" / "drone.pt", 960, 0.25),
]


def test_drone_model_has_no_false_positives_on_negative_fixtures():
    if os.getenv("DRONTECT_RUN_MODEL_SELECTION_TESTS") != "1":
        pytest.skip("Set DRONTECT_RUN_MODEL_SELECTION_TESTS=1 to run YOLO model regression.")

    pytest.importorskip("ultralytics")
    from ultralytics import YOLO

    manifest = _load_manifest()
    for config in MODEL_CONFIGS:
        if not config.path.exists():
            pytest.skip(f"Missing local model weights: {config.path}")

    report: dict[str, Any] = {}
    models = {config.name: YOLO(str(config.path)) for config in MODEL_CONFIGS}
    for config in MODEL_CONFIGS:
        rows = []
        total_error = 0
        missed_images = 0
        false_positive_images = 0
        elapsed_ms = 0.0

        for item in manifest:
            start = perf_counter()
            detections = _predict(
                models[config.name],
                IMAGES_ROOT / item["file"],
                image_size=config.image_size,
                confidence=config.confidence,
            )
            elapsed_ms += (perf_counter() - start) * 1000

            count = len(detections)
            min_expected = item["expected_drones_min"]
            max_expected = item["expected_drones_max"]
            if count < min_expected:
                missed_images += 1
                total_error += min_expected - count
            elif count > max_expected:
                total_error += count - max_expected

            if max_expected == 0 and count > 0:
                false_positive_images += 1

            rows.append(
                {
                    "file": item["file"],
                    "expected_drones_min": min_expected,
                    "expected_drones_max": max_expected,
                    "expected_birds": item["expected_birds"],
                    "detections": detections,
                }
            )

        report[config.name] = {
            "model_path": str(config.path.relative_to(REPO_ROOT)),
            "image_size": config.image_size,
            "confidence": config.confidence,
            "total_error": total_error,
            "missed_images": missed_images,
            "false_positive_images": false_positive_images,
            "average_ms": round(elapsed_ms / len(manifest), 1),
            "rows": rows,
        }

    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, indent=2), encoding="utf-8")

    chosen = report["drone_960_conf025"]
    assert chosen["false_positive_images"] == 0


def _load_manifest() -> list[dict[str, Any]]:
    raw = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    assert isinstance(raw, list)
    for item in raw:
        image_path = IMAGES_ROOT / item["file"]
        assert image_path.exists(), f"Missing model comparison fixture: {image_path}"
    return raw


def _predict(
    model: Any,
    image_path: Path,
    *,
    image_size: int,
    confidence: float,
) -> list[dict[str, Any]]:
    results = model.predict(
        str(image_path),
        imgsz=image_size,
        conf=confidence,
        verbose=False,
    )
    boxes = getattr(results[0], "boxes", None)
    if boxes is None:
        return []

    detections = []
    for xyxy, score, class_id in zip(
        boxes.xyxy.tolist(),
        boxes.conf.tolist(),
        boxes.cls.tolist(),
    ):
        detections.append(
            {
                "class_id": int(class_id),
                "confidence": round(float(score), 4),
                "box": [round(float(value), 1) for value in xyxy],
            }
        )
    detections.sort(key=lambda detection: detection["confidence"], reverse=True)
    return detections
