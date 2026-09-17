import pytest

from realtime_server.protocol import (
    ROLE_DRONE,
    ProtocolError,
    build_detection_message,
    command_ack,
    encode_message,
    mission_state_message,
    mission_status_message,
    parse_json_message,
    validate_detection_message,
    validate_command_message,
    validate_mission_state_message,
    validate_mission_status_message,
    validate_role,
    validate_telemetry_message,
)
from services.mission_state import MissionState
from services.gps import LocalPosition
from services.telemetry import build_simulator_telemetry
from services.video import JPEG_ENCODING, VideoFrame
from vision_server.tracker import TrackedDetection


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


def test_simulator_telemetry_accepts_configurable_battery_percent():
    message = build_simulator_telemetry(
        position_local=LocalPosition(0.0, 0.0, 15.0),
        heading_deg=0.0,
        speed_mps=0.0,
        state="PATROL",
        battery_percent=12.5,
    )

    assert message["data"]["batteryPercent"] == 12.5


def test_telemetry_round_trips_aircraft_safety_rth_state():
    message = build_simulator_telemetry(
        position_local=LocalPosition(0.0, 0.0, 15.0),
        heading_deg=0.0,
        speed_mps=0.0,
        state="RETURNING_HOME",
        aircraft_safety_rth_active=True,
        aircraft_safety_rth_reason="connection_lost",
    )

    validated = validate_telemetry_message(message)

    assert validated["data"]["aircraftSafetyRthActive"] is True
    assert validated["data"]["aircraftSafetyRthReason"] == "connection_lost"


def test_telemetry_round_trips_target_follow_diagnostics():
    diagnostics = {
        "flightState": "TARGET_LOST_SEARCH",
        "trackerId": "drone-7",
        "detectionConfidence": 0.91,
        "imageCenterError": {"x": 0.25, "y": -0.05},
        "confirmation": {"hits": 2, "requiredHits": 2, "progress": 1.0},
        "estimatedImageVelocity": {
            "x": 0.07,
            "y": -0.02,
            "magnitude": 0.073,
            "directionDeg": -15.9,
        },
        "boundingBoxScale": 0.1,
        "boundingBoxAreaRatio": 0.01,
        "standoffError": 0.015,
        "targetLostDurationS": 0.4,
        "patrolInterrupted": True,
        "patrolResumeState": "PATROL",
        "patrolResumeStatus": "interrupted",
    }
    message = build_simulator_telemetry(
        position_local=LocalPosition(0.0, 0.0, 15.0),
        heading_deg=0.0,
        speed_mps=0.0,
        state="TARGET_FOLLOW",
        target_follow_diagnostics=diagnostics,
    )

    validated = validate_telemetry_message(message)

    assert validated["data"]["targetFollowDiagnostics"] == diagnostics


def test_mission_status_round_trips_authoritative_backend_state():
    message = mission_status_message(
        state=MissionState.ROI_PATROL,
        active_roi={
            "center": {"lat": 32.1, "lon": 34.8},
            "radiusM": 250.0,
        },
        patrol_priorities=[{"regionId": "N-4", "rank": 1}],
        current_patrol_region_id="N-3",
        target_patrol_region_id="N-4",
        battery_percent=18.0,
        low_battery=True,
        safety_rth_active=False,
        rth_reason=None,
    )

    validated = validate_mission_status_message(
        parse_json_message(encode_message(message))
    )

    assert validated == message
    assert validated["state"] == "ROI_PATROL"
    assert validated["activeRoi"]["radiusM"] == 250.0
    assert validated["patrol"]["targetRegionId"] == "N-4"
    assert validated["batteryRth"]["lowBattery"] is True


def test_mission_status_missing_area_types_defaults_to_patrol():
    message = mission_status_message(
        state=MissionState.ON_GROUND,
        active_roi=None,
        patrol_priorities=[],
        current_patrol_region_id=None,
        target_patrol_region_id=None,
        battery_percent=100.0,
        low_battery=False,
        safety_rth_active=False,
        rth_reason=None,
    )
    del message["patrol"]["areaTypes"]

    validated = validate_mission_status_message(message)

    assert validated["patrol"]["areaTypes"] == []


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


@pytest.mark.parametrize("command", ["start_patrol", "clear_roi", "return_home"])
def test_valid_parameterless_mission_commands(command):
    message = validate_command_message(
        {
            "type": "command",
            "commandId": "mission-1",
            "command": command,
            "params": {},
        }
    )

    assert message["command"] == command
    assert message["params"] == {}


def test_set_roi_accepts_zero_coordinate_and_radius_payload():
    message = validate_command_message(
        {
            "type": "command",
            "commandId": "roi-1",
            "command": "set_roi",
            "params": {
                "center": {"lat": 0, "lon": 0},
                "radiusM": 0,
            },
        }
    )

    assert message["params"] == {
        "center": {"lat": 0.0, "lon": 0.0},
        "radiusM": 0.0,
    }


def test_set_patrol_priorities_accepts_ranked_and_empty_sets():
    ranked = validate_command_message(
        {
            "type": "command",
            "commandId": "priorities-1",
            "command": "set_patrol_priorities",
            "params": {
                "priorities": [
                    {"regionId": "N-4", "rank": 1},
                    {"regionId": "NE-4", "rank": 2},
                ]
            },
        }
    )
    empty = validate_command_message(
        {
            "type": "command",
            "commandId": "priorities-2",
            "command": "set_patrol_priorities",
            "params": {"priorities": []},
        }
    )

    assert ranked["params"]["priorities"] == [
        {"regionId": "N-4", "rank": 1},
        {"regionId": "NE-4", "rank": 2},
    ]
    assert empty["params"] == {"priorities": []}


def test_set_patrol_area_types_accepts_enum_values_and_empty_overrides():
    typed = validate_command_message(
        {
            "type": "command",
            "commandId": "area-types-1",
            "command": "set_patrol_area_types",
            "params": {
                "areaTypes": [
                    {"regionId": "N-1", "areaType": "IGNORE"},
                    {"regionId": "N-2", "areaType": "NO_FLY"},
                ]
            },
        }
    )
    empty = validate_command_message(
        {
            "type": "command",
            "commandId": "area-types-2",
            "command": "set_patrol_area_types",
            "params": {"areaTypes": []},
        }
    )

    assert typed["params"]["areaTypes"] == [
        {"regionId": "N-1", "areaType": "IGNORE"},
        {"regionId": "N-2", "areaType": "NO_FLY"},
    ]
    assert empty["params"] == {"areaTypes": []}


@pytest.mark.parametrize("area_type", ["OTHER", "ignore", None, True])
def test_set_patrol_area_types_rejects_unknown_type(area_type):
    with pytest.raises(ProtocolError) as exc_info:
        validate_command_message(
            {
                "type": "command",
                "commandId": "area-types-bad",
                "command": "set_patrol_area_types",
                "params": {
                    "areaTypes": [
                        {"regionId": "N-1", "areaType": area_type}
                    ]
                },
            }
        )

    assert exc_info.value.code == "invalid_params"


@pytest.mark.parametrize(
    "priorities",
    [
        None,
        {},
        ["N-4"],
        [{"regionId": "BAD-1", "rank": 1}],
        [{"regionId": "N-4", "rank": 0}],
        [{"regionId": "N-4", "rank": True}],
        [
            {"regionId": "N-4", "rank": 1},
            {"regionId": "N-4", "rank": 2},
        ],
        [
            {"regionId": "N-4", "rank": 1},
            {"regionId": "NE-4", "rank": 1},
        ],
        [
            {"regionId": "N-4", "rank": 1},
            {"regionId": "NE-4", "rank": 3},
        ],
    ],
)
def test_set_patrol_priorities_rejects_invalid_sets(priorities):
    with pytest.raises(ProtocolError) as exc_info:
        validate_command_message(
            {
                "type": "command",
                "commandId": "priorities-bad",
                "command": "set_patrol_priorities",
                "params": {"priorities": priorities},
            }
        )

    assert exc_info.value.code == "invalid_params"
    assert exc_info.value.command_id == "priorities-bad"


@pytest.mark.parametrize(
    ("params", "field"),
    [
        ({}, "center"),
        ({"center": {"lat": 0, "lon": 0}}, "radiusM"),
        ({"center": {"lat": 91, "lon": 0}, "radiusM": 10}, "center.lat"),
        ({"center": {"lat": 0, "lon": -181}, "radiusM": 10}, "center.lon"),
        ({"center": {"lat": 0, "lon": 0}, "radiusM": -1}, "radiusM"),
        ({"center": {"lat": True, "lon": 0}, "radiusM": 10}, "lat"),
    ],
)
def test_set_roi_rejects_invalid_params(params, field):
    with pytest.raises(ProtocolError) as exc_info:
        validate_command_message(
            {
                "type": "command",
                "commandId": "roi-bad",
                "command": "set_roi",
                "params": params,
            }
        )

    assert exc_info.value.code == "invalid_params"
    assert exc_info.value.command_id == "roi-bad"
    assert exc_info.value.details["field"] == field


def test_parameterless_mission_command_rejects_params():
    with pytest.raises(ProtocolError) as exc_info:
        validate_command_message(
            {
                "type": "command",
                "commandId": "patrol-bad",
                "command": "start_patrol",
                "params": {"unexpected": True},
            }
        )

    assert exc_info.value.code == "invalid_params"
    assert exc_info.value.command_id == "patrol-bad"


def test_mission_state_message_validates_and_serializes():
    message = mission_state_message(MissionState.ON_GROUND)

    assert validate_mission_state_message(
        parse_json_message(encode_message(message))
    ) == {
        "type": "mission_state",
        "state": "ON_GROUND",
    }


def test_unknown_mission_state_is_rejected():
    with pytest.raises(ProtocolError) as exc_info:
        validate_mission_state_message(
            {"type": "mission_state", "state": "FLYING_RANDOMLY"}
        )

    assert exc_info.value.code == "validation_error"
    assert exc_info.value.details["field"] == "state"


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
            TrackedDetection(
                class_name="drone",
                confidence=0.87,
                box=(420.0, 180.0, 580.0, 310.0),
                track_id="drone-1",
                track_state="confirmed",
                last_seen_timestamp=1720000000.123,
                velocity_px_per_sec=(12.0, -3.0),
            )
        ],
    )

    message = validate_detection_message(parse_json_message(encode_message(raw)))

    assert message == {
        "type": "detections",
        "frameId": 150,
        "timestampMs": 1720000000123,
        "frameWidth": 960,
        "frameHeight": 540,
        "detections": [
            {
                "className": "drone",
                "confidence": 0.87,
                "box": [420.0, 180.0, 580.0, 310.0],
                "trackId": "drone-1",
                "trackState": "confirmed",
                "lastSeenTimestamp": 1720000000.123,
                "velocityPxPerSec": {"x": 12.0, "y": -3.0},
            }
        ],
    }


def test_empty_detection_message_validates():
    message = validate_detection_message(
        {
            "type": "detections",
            "frameId": 150,
            "timestampMs": 150000,
            "frameWidth": 960,
            "frameHeight": 540,
            "detections": [],
        }
    )

    assert message == {
        "type": "detections",
        "frameId": 150,
        "timestampMs": 150000,
        "frameWidth": 960,
        "frameHeight": 540,
        "detections": [],
    }


@pytest.mark.parametrize(
    "message,field",
    [
        ({"type": "telemetry", "frameId": 150, "detections": []}, "type"),
        ({"type": "detections", "detections": []}, "frameId"),
        ({"type": "detections", "frameId": 150, "frameWidth": 960, "frameHeight": 540, "detections": []}, "timestampMs"),
        ({"type": "detections", "frameId": 150, "timestampMs": 150000, "frameHeight": 540, "detections": []}, "frameWidth"),
        ({"type": "detections", "frameId": 150, "timestampMs": 150000, "frameWidth": 960, "detections": []}, "frameHeight"),
        (
            {
                "type": "detections",
                "frameId": 150,
                "timestampMs": 150000,
                "frameWidth": 960,
                "frameHeight": 540,
                "detections": {},
            },
            "detections",
        ),
        (
            {
                "type": "detections",
                "frameId": 150,
                "timestampMs": 150000,
                "frameWidth": 960,
                "frameHeight": 540,
                "detections": [
                    {
                        "className": "drone",
                        "confidence": 1.5,
                        "box": [1, 2, 3, 4],
                        "trackId": "drone-1",
                        "trackState": "confirmed",
                        "lastSeenTimestamp": 1.0,
                        "velocityPxPerSec": {"x": 0.0, "y": 0.0},
                    }
                ],
            },
            "confidence",
        ),
        (
            {
                "type": "detections",
                "frameId": 150,
                "timestampMs": 150000,
                "frameWidth": 960,
                "frameHeight": 540,
                "detections": [
                    {
                        "className": "drone",
                        "confidence": 0.8,
                        "box": [1, 2, 3],
                        "trackId": "drone-1",
                        "trackState": "confirmed",
                        "lastSeenTimestamp": 1.0,
                        "velocityPxPerSec": {"x": 0.0, "y": 0.0},
                    }
                ],
            },
            "box",
        ),
        (
            {
                "type": "detections",
                "frameId": 150,
                "timestampMs": 150000,
                "frameWidth": 960,
                "frameHeight": 540,
                "detections": [
                    {
                        "className": "drone",
                        "confidence": 0.8,
                        "box": [1, 2, 3, 4],
                        "trackState": "confirmed",
                        "lastSeenTimestamp": 1.0,
                        "velocityPxPerSec": {"x": 0.0, "y": 0.0},
                    }
                ],
            },
            "trackId",
        ),
        (
            {
                "type": "detections",
                "frameId": 150,
                "timestampMs": 150000,
                "frameWidth": 960,
                "frameHeight": 540,
                "detections": [
                    {
                        "className": "drone",
                        "confidence": 0.8,
                        "box": [1, 2, 3, 4],
                        "trackId": "drone-1",
                        "trackState": "bad",
                        "lastSeenTimestamp": 1.0,
                        "velocityPxPerSec": {"x": 0.0, "y": 0.0},
                    }
                ],
            },
            "trackState",
        ),
        (
            {
                "type": "detections",
                "frameId": 150,
                "timestampMs": 150000,
                "frameWidth": 960,
                "frameHeight": 540,
                "detections": [
                    {
                        "className": "drone",
                        "confidence": 0.8,
                        "box": [1, 2, 3, 4],
                        "trackId": "drone-1",
                        "trackState": "confirmed",
                        "lastSeenTimestamp": 1.0,
                        "velocityPxPerSec": None,
                    }
                ],
            },
            "velocityPxPerSec",
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


def test_safety_reason_round_trips_only_on_return_home():
    command = validate_command_message(
        {
            "type": "command",
            "commandId": "safety-rth-1",
            "command": "return_home",
            "params": {},
            "safetyReason": "low_battery",
        }
    )
    assert command["safetyReason"] == "low_battery"

    with pytest.raises(ProtocolError):
        validate_command_message(
            {
                "type": "command",
                "commandId": "cmd-2",
                "command": "start_patrol",
                "params": {},
                "safetyReason": "low_battery",
            }
        )

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
