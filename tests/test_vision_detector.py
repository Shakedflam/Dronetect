from services.video import JPEG_ENCODING, VideoFrame
from vision_server.detector import (
    Detection,
    NoopDroneDetector,
    YoloDroneDetector,
    create_drone_detector,
    detections_from_yolo_results,
)


class FakeBoxes:
    def __init__(self):
        self.xyxy = [
            [10.0, 20.0, 110.0, 120.0],
            [1.0, 2.0, 3.0, 4.0],
            [30.0, 40.0, 130.0, 140.0],
            [50.0, 60.0, 150.0, 160.0],
        ]
        self.conf = [0.90, 0.95, 0.20, 0.80]
        self.cls = [2, 3, 0, 0]


class FakeResult:
    names = {
        0: "target",
        2: "drone",
        3: "person",
    }
    boxes = FakeBoxes()


def test_yolo_result_parser_returns_only_drone_detections():
    detections = detections_from_yolo_results([FakeResult()], confidence_threshold=0.35)

    assert detections == [
        Detection(
            class_name="drone",
            confidence=0.90,
            box=(10.0, 20.0, 110.0, 120.0),
        ),
        Detection(
            class_name="target",
            confidence=0.80,
            box=(50.0, 60.0, 150.0, 160.0),
        ),
    ]


def test_yolo_result_parser_accepts_drone_aliases():
    class AliasBoxes:
        xyxy = [[10.0, 20.0, 30.0, 40.0], [50.0, 60.0, 70.0, 80.0]]
        conf = [0.7, 0.8]
        cls = [5, 6]

    class AliasResult:
        names = {5: "UAV", 6: "quadcopter"}
        boxes = AliasBoxes()

    detections = detections_from_yolo_results([AliasResult()], confidence_threshold=0.35)

    assert [detection.class_name for detection in detections] == ["UAV", "quadcopter"]
    assert detections[0].box == (10.0, 20.0, 30.0, 40.0)


def test_missing_yolo_model_creates_noop_detector(tmp_path):
    messages: list[str] = []

    detector = create_drone_detector(
        model_path=tmp_path / "missing.pt",
        logger=messages.append,
    )

    assert isinstance(detector, NoopDroneDetector)
    assert "model not found" in messages[0]
    assert detector.detect(_frame()) == []


def test_yolo_detector_passes_image_size_to_model():
    model = FakeYoloModel()
    detector = YoloDroneDetector(model, confidence_threshold=0.25, image_size=960)

    detections = detector.detect(_real_jpeg_frame())

    assert model.predict_kwargs["conf"] == 0.25
    assert model.predict_kwargs["imgsz"] == 960
    assert detections == [
        Detection(
            class_name="drone",
            confidence=0.5,
            box=(1.0, 2.0, 3.0, 4.0),
        )
    ]


def test_create_drone_detector_reads_image_size_env(monkeypatch, tmp_path):
    import sys
    import types

    yolo_module = types.ModuleType("ultralytics")
    setattr(yolo_module, "YOLO", FakeYoloModel)
    monkeypatch.setitem(sys.modules, "ultralytics", yolo_module)
    monkeypatch.setenv("DRONTECT_YOLO_IMGSZ", "960")

    model_path = tmp_path / "drone.pt"
    model_path.write_bytes(b"weights")
    detector = create_drone_detector(model_path=model_path, logger=lambda _: None)

    assert isinstance(detector, YoloDroneDetector)
    assert detector.image_size == 960


class FakeYoloModel:
    names = {0: "drone"}

    def __init__(self, path: str | None = None) -> None:
        self.path = path
        self.predict_kwargs = {}

    def predict(self, image, **kwargs):
        self.predict_kwargs = kwargs

        class Boxes:
            xyxy = [[1.0, 2.0, 3.0, 4.0]]
            conf = [0.5]
            cls = [0]

        class Result:
            names = {0: "drone"}
            boxes = Boxes()

        return [Result()]


def _frame() -> VideoFrame:
    return VideoFrame(
        frame_id=1,
        timestamp=1.0,
        width=640,
        height=360,
        encoding=JPEG_ENCODING,
        data=b"jpeg",
    )


def _real_jpeg_frame() -> VideoFrame:
    from io import BytesIO

    from PIL import Image

    buffer = BytesIO()
    Image.new("RGB", (4, 4), color="white").save(buffer, format="JPEG")
    return VideoFrame(
        frame_id=1,
        timestamp=1.0,
        width=4,
        height=4,
        encoding=JPEG_ENCODING,
        data=buffer.getvalue(),
    )
