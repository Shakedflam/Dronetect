import pytest

from realtime_server.protocol import (
    ROLE_DRONE,
    ProtocolError,
    build_detection_message,
    command_ack,
    encode_message,
    parse_json_message,
    validate_detection_message,
    validate_command_message,
    validate_role,
    validate_telemetry_message,
)
from services.gps import LocalPosition
from services.telemetry import build_simulator_telemetry
from services.video import JPEG_ENCODING, VideoFrame
from vision_server.detector import Detection


def test_valid_telemetry_validates_and_serializes():
    message = build_simulator_telemetry(
        position_local=LocalPosition(-12.3, 6.1, 14.8),
        heading_deg=91.4,
        speed_mps=4.2,
        state="FLYING_RANDOMLY",
        timestamp=1720000000.123,
    )

    validated = validate_telemetry_message(parse_json_message(encode_message(message)))

    assert validated["type"] == "telemetry"
    assert validated["source"] == "simulator"
    assert "positionLocal" not in validated["data"]
    assert validated["data"]["altitudeM"] == pytest.approx(14.8)
    assert validated["data"]["gps"]["lat"] > 32.085300


def test_valid_command_validates_and_serializes():
    raw = encode_message(
        {
            "type": "command",
            "commandId": "cmd-1",
            "command": "return_home",
            "params": {},
        }
    )

    command = validate_command_message(parse_json_message(raw))

    assert command == {
        "type": "command",
        "commandId": "cmd-1",
        "command": "return_home",
        "params": {},
    }


def test_valid_detection_message_validates_and_serializes():
    raw = build_detection_message(
        VideoFrame(
            frame_id=150,
            timestamp=1720000000.123,
            width=960,
            height=540,
            encoding=JPEG_ENCODING,
            data=b"jpeg",
        ),
        [
            Detection(
                class_name="drone",
                confidence=0.87,
                box=(420.0, 180.0, 580.0, 310.0),
            )
        ],
    )

    message = validate_detection_message(parse_json_message(encode_message(raw)))

    assert message == {
        "type": "detections",
        "frameId": 150,
        "detections": [
            {
                "className": "drone",
                "confidence": 0.87,
                "box": [420.0, 180.0, 580.0, 310.0],
            }
        ],
    }


def test_empty_detection_message_validates():
    message = validate_detection_message(
        {
            "type": "detections",
            "frameId": 150,
            "detections": [],
        }
    )

    assert message == {
        "type": "detections",
        "frameId": 150,
        "detections": [],
    }


@pytest.mark.parametrize(
    "message,field",
    [
        ({"type": "telemetry", "frameId": 150, "detections": []}, "type"),
        ({"type": "detections", "detections": []}, "frameId"),
        ({"type": "detections", "frameId": 150, "detections": {}}, "detections"),
        (
            {
                "type": "detections",
                "frameId": 150,
                "detections": [{"className": "drone", "confidence": 1.5, "box": [1, 2, 3, 4]}],
            },
            "confidence",
        ),
        (
            {
                "type": "detections",
                "frameId": 150,
                "detections": [{"className": "drone", "confidence": 0.8, "box": [1, 2, 3]}],
            },
            "box",
        ),
    ],
)
def test_invalid_detection_messages_are_rejected(message, field):
    with pytest.raises(ProtocolError) as exc_info:
        validate_detection_message(message)

    if field != "type":
        assert exc_info.value.details["field"] == field


def test_unknown_command_is_rejected():
    with pytest.raises(ProtocolError) as exc_info:
        validate_command_message(
            {
                "type": "command",
                "commandId": "cmd-1",
                "command": "follow",
                "params": {},
            }
        )

    assert exc_info.value.code == "unsupported_command"
    assert exc_info.value.command_id == "cmd-1"


def test_missing_required_fields_produce_structured_errors():
    with pytest.raises(ProtocolError) as exc_info:
        validate_telemetry_message({"type": "telemetry"})

    error = exc_info.value.to_message()

    assert error["type"] == "error"
    assert error["code"] == "validation_error"
    assert error["details"]["field"] == "timestamp"


def test_command_ack_uses_same_command_id():
    assert command_ack("cmd-1") == {
        "type": "command_ack",
        "commandId": "cmd-1",
        "status": "accepted",
    }


def test_drone_role_is_canonical_and_simulator_role_is_legacy_alias():
    assert validate_role("drone") == ROLE_DRONE
    assert validate_role("simulator") == ROLE_DRONE
