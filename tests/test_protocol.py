import pytest

from realtime_server.protocol import (
    ROLE_DRONE,
    ProtocolError,
    command_ack,
    encode_message,
    parse_json_message,
    validate_command_message,
    validate_role,
    validate_telemetry_message,
)
from services.gps import LocalPosition
from services.telemetry import build_simulator_telemetry


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
