"""JSON message protocol for the Drontect realtime server."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from typing import Any

from services.commands import SUPPORTED_COMMANDS


MESSAGE_TYPE_TELEMETRY = "telemetry"
MESSAGE_TYPE_COMMAND = "command"
MESSAGE_TYPE_COMMAND_ACK = "command_ack"
MESSAGE_TYPE_DETECTIONS = "detections"
MESSAGE_TYPE_ERROR = "error"

ROLE_DRONE = "drone"
ROLE_SIMULATOR = "simulator"
ROLE_TABLET = "tablet"
SUPPORTED_ROLES = frozenset({ROLE_DRONE, ROLE_SIMULATOR, ROLE_TABLET})


@dataclass(frozen=True, slots=True)
class ProtocolError(Exception):
    code: str
    message: str
    command_id: str | None = None
    details: dict[str, Any] = field(default_factory=dict)

    def to_message(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "type": MESSAGE_TYPE_ERROR,
            "code": self.code,
            "message": self.message,
        }
        if self.command_id is not None:
            payload["commandId"] = self.command_id
        if self.details:
            payload["details"] = self.details
        return payload


def parse_json_message(raw: str) -> dict[str, Any]:
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ProtocolError("invalid_json", "Message must be valid JSON.") from exc
    if not isinstance(parsed, dict):
        raise ProtocolError("invalid_message", "Message must be a JSON object.")
    return parsed


def encode_message(message: dict[str, Any]) -> str:
    return json.dumps(message, separators=(",", ":"))


def validate_role(role: str | None) -> str:
    if role == ROLE_SIMULATOR:
        return ROLE_DRONE
    if role in SUPPORTED_ROLES:
        return role
    raise ProtocolError(
        "invalid_role",
        "WebSocket role must be 'drone' or 'tablet'.",
        details={"role": role},
    )


def validate_command_message(message: dict[str, Any]) -> dict[str, Any]:
    command_id = _required_string(message, "commandId")
    try:
        message_type = _required_string(message, "type")
        command = _required_string(message, "command")
        if message_type != MESSAGE_TYPE_COMMAND:
            raise ProtocolError(
                "invalid_type",
                "Tablet messages sent as commands must use type 'command'.",
                command_id=command_id,
                details={"type": message_type},
            )
        if command not in SUPPORTED_COMMANDS:
            raise ProtocolError(
                "unsupported_command",
                f"Unsupported command '{command}'.",
                command_id=command_id,
                details={"supported": sorted(SUPPORTED_COMMANDS)},
            )
        params = message.get("params", {})
        if not isinstance(params, dict):
            raise ProtocolError(
                "invalid_params",
                "Command params must be a JSON object.",
                command_id=command_id,
            )
    except ProtocolError as exc:
        if exc.command_id is None:
            raise ProtocolError(exc.code, exc.message, command_id=command_id, details=exc.details) from exc
        raise

    return {
        "type": MESSAGE_TYPE_COMMAND,
        "commandId": command_id,
        "command": command,
        "params": params,
    }


def validate_telemetry_message(message: dict[str, Any]) -> dict[str, Any]:
    message_type = _required_string(message, "type")
    if message_type != MESSAGE_TYPE_TELEMETRY:
        raise ProtocolError(
            "invalid_type",
            "Simulator messages sent as telemetry must use type 'telemetry'.",
            details={"type": message_type},
        )

    timestamp = _required_number(message, "timestamp")
    source = _required_string(message, "source")
    data = _required_object(message, "data")
    gps = _required_object(data, "gps")

    return {
        "type": MESSAGE_TYPE_TELEMETRY,
        "timestamp": timestamp,
        "source": source,
        "data": {
            "gps": {
                "lat": _required_number(gps, "lat"),
                "lng": _required_number(gps, "lng"),
            },
            "altitudeM": _required_number(data, "altitudeM"),
            "headingDeg": _required_number(data, "headingDeg"),
            "speedMps": _required_number(data, "speedMps"),
            "batteryPercent": _required_number(data, "batteryPercent"),
            "state": _required_string(data, "state"),
        },
    }


def build_detection_message(frame: Any, detections: list[Any]) -> dict[str, Any]:
    return {
        "type": MESSAGE_TYPE_DETECTIONS,
        "frameId": frame.frame_id,
        "timestampMs": _timestamp_ms(frame.timestamp),
        "frameWidth": frame.width,
        "frameHeight": frame.height,
        "detections": [
            {
                "className": detection.class_name,
                "confidence": detection.confidence,
                "box": list(detection.box),
                "trackId": detection.track_id,
                "trackState": detection.track_state,
                "lastSeenTimestamp": detection.last_seen_timestamp,
                "velocityPxPerSec": {
                    "x": detection.velocity_px_per_sec[0],
                    "y": detection.velocity_px_per_sec[1],
                },
            }
            for detection in detections
        ],
    }


def validate_detection_message(message: dict[str, Any]) -> dict[str, Any]:
    message_type = _required_string(message, "type")
    if message_type != MESSAGE_TYPE_DETECTIONS:
        raise ProtocolError(
            "invalid_type",
            "Vision messages sent as detections must use type 'detections'.",
            details={"type": message_type},
        )

    detections = message.get("detections")
    if not isinstance(detections, list):
        raise ProtocolError(
            "validation_error",
            "Required list field 'detections' is missing or invalid.",
            details={"field": "detections"},
        )

    return {
        "type": MESSAGE_TYPE_DETECTIONS,
        "frameId": _required_int(message, "frameId"),
        "timestampMs": _required_number(message, "timestampMs"),
        "frameWidth": _required_int(message, "frameWidth"),
        "frameHeight": _required_int(message, "frameHeight"),
        "detections": [_validate_detection(detection) for detection in detections],
    }


def command_ack(command_id: str, status: str = "accepted") -> dict[str, str]:
    return {
        "type": MESSAGE_TYPE_COMMAND_ACK,
        "commandId": command_id,
        "status": status,
    }


def _validate_detection(value: Any) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ProtocolError(
            "validation_error",
            "Detection entries must be JSON objects.",
            details={"field": "detections"},
        )

    confidence = _required_number(value, "confidence")
    if confidence < 0.0 or confidence > 1.0:
        raise ProtocolError(
            "validation_error",
            "Detection confidence must be between 0 and 1.",
            details={"field": "confidence"},
        )

    box = value.get("box")
    if not isinstance(box, list) or len(box) != 4:
        raise ProtocolError(
            "validation_error",
            "Detection box must be a four-number list.",
            details={"field": "box"},
        )

    return {
        "className": _required_string(value, "className"),
        "confidence": confidence,
        "box": [_required_number({"value": coordinate}, "value") for coordinate in box],
        "trackId": _required_string(value, "trackId"),
        "trackState": _required_track_state(value),
        "lastSeenTimestamp": _required_number(value, "lastSeenTimestamp"),
        "velocityPxPerSec": _required_velocity(value),
    }


def _required_track_state(message: dict[str, Any]) -> str:
    value = _required_string(message, "trackState")
    if value not in {"tentative", "confirmed", "lost"}:
        raise ProtocolError(
            "validation_error",
            "Detection trackState must be 'tentative', 'confirmed', or 'lost'.",
            details={"field": "trackState"},
        )
    return value


def _required_velocity(message: dict[str, Any]) -> dict[str, float]:
    velocity = message.get("velocityPxPerSec")
    if not isinstance(velocity, dict):
        raise ProtocolError(
            "validation_error",
            "Detection velocityPxPerSec must be an object.",
            details={"field": "velocityPxPerSec"},
        )
    return {
        "x": _required_number(velocity, "x"),
        "y": _required_number(velocity, "y"),
    }


def _required_int(message: dict[str, Any], key: str) -> int:
    value = message.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError(
            "validation_error",
            f"Required integer field '{key}' is missing or invalid.",
            details={"field": key},
        )
    return value


def _required_string(message: dict[str, Any], key: str) -> str:
    value = message.get(key)
    if isinstance(value, str) and value:
        return value
    raise ProtocolError(
        "validation_error",
        f"Required string field '{key}' is missing or invalid.",
        details={"field": key},
    )


def _required_number(message: dict[str, Any], key: str) -> float:
    value = message.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(
            "validation_error",
            f"Required numeric field '{key}' is missing or invalid.",
            details={"field": key},
        )
    return value


def _required_object(message: dict[str, Any], key: str) -> dict[str, Any]:
    value = message.get(key)
    if isinstance(value, dict):
        return value
    raise ProtocolError(
        "validation_error",
        f"Required object field '{key}' is missing or invalid.",
        details={"field": key},
    )


def _timestamp_ms(timestamp_seconds: float) -> int:
    return int(round(timestamp_seconds * 1000.0))
