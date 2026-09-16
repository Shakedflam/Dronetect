"""JSON message protocol for the Drontect realtime server."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from typing import Any

from services.commands import CommandName, SUPPORTED_COMMANDS
from services.mission_state import MissionState
from services.patrol_regions import PATROL_REGION_IDS, PatrolAreaType


MESSAGE_TYPE_TELEMETRY = "telemetry"
MESSAGE_TYPE_COMMAND = "command"
MESSAGE_TYPE_COMMAND_ACK = "command_ack"
MESSAGE_TYPE_DETECTIONS = "detections"
MESSAGE_TYPE_MISSION_STATE = "mission_state"
MESSAGE_TYPE_MISSION_STATUS = "mission_status"
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
        params = _validate_command_params(command, params, command_id)
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


def mission_state_message(state: MissionState) -> dict[str, str]:
    return {
        "type": MESSAGE_TYPE_MISSION_STATE,
        "state": state.value,
    }


def mission_status_message(
    *,
    state: MissionState,
    active_roi: dict[str, Any] | None,
    patrol_priorities: list[dict[str, Any]],
    current_patrol_region_id: str | None,
    target_patrol_region_id: str | None,
    battery_percent: float | None,
    low_battery: bool,
    safety_rth_active: bool,
    rth_reason: str | None,
    drone_connected: bool = False,
    patrol_area_types: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Build the authoritative mission snapshot sent to tablet clients."""
    return {
        "type": MESSAGE_TYPE_MISSION_STATUS,
        "state": state.value,
        "droneConnected": drone_connected,
        "activeRoi": active_roi,
        "patrol": {
            "priorities": patrol_priorities,
            "areaTypes": patrol_area_types or [],
            "currentRegionId": current_patrol_region_id,
            "targetRegionId": target_patrol_region_id,
        },
        "batteryRth": {
            "batteryPercent": battery_percent,
            "lowBattery": low_battery,
            "rthActive": state in {MissionState.RTH, MissionState.LANDING},
            "safetyRthActive": safety_rth_active,
            "rthReason": rth_reason,
        },
    }


def validate_mission_status_message(message: dict[str, Any]) -> dict[str, Any]:
    message_type = _required_string(message, "type")
    if message_type != MESSAGE_TYPE_MISSION_STATUS:
        raise ProtocolError(
            "invalid_type",
            "Mission-status messages must use type 'mission_status'.",
            details={"type": message_type},
        )
    try:
        state = MissionState(_required_string(message, "state"))
    except ValueError as exc:
        raise ProtocolError(
            "validation_error",
            "Mission status contains an unknown state.",
            details={"field": "state"},
        ) from exc
    drone_connected = message.get("droneConnected")
    if not isinstance(drone_connected, bool):
        raise ProtocolError(
            "validation_error",
            "droneConnected must be a boolean.",
            details={"field": "droneConnected"},
        )

    active_roi = message.get("activeRoi")
    if active_roi is not None:
        active_roi = _required_object(message, "activeRoi")
        center = _required_object(active_roi, "center")
        active_roi = {
            "center": {
                "lat": _required_number(center, "lat"),
                "lon": _required_number(center, "lon"),
            },
            "radiusM": _required_number(active_roi, "radiusM"),
        }

    patrol = _required_object(message, "patrol")
    priorities = patrol.get("priorities")
    if not isinstance(priorities, list):
        raise ProtocolError(
            "validation_error",
            "Mission-status patrol priorities must be an array.",
            details={"field": "patrol.priorities"},
        )
    validated_priorities: list[dict[str, Any]] = []
    for entry in priorities:
        if not isinstance(entry, dict):
            raise ProtocolError(
                "validation_error",
                "Mission-status priority entries must be objects.",
                details={"field": "patrol.priorities"},
            )
        validated_priorities.append(
            {
                "regionId": _required_string(entry, "regionId"),
                "rank": _required_int(entry, "rank"),
            }
        )
    raw_area_types = patrol.get("areaTypes", [])
    if not isinstance(raw_area_types, list):
        raise ProtocolError(
            "validation_error",
            "Mission-status patrol areaTypes must be an array.",
            details={"field": "patrol.areaTypes"},
        )
    validated_area_types: list[dict[str, str]] = []
    for entry in raw_area_types:
        if not isinstance(entry, dict):
            raise ProtocolError(
                "validation_error",
                "Mission-status area type entries must be objects.",
                details={"field": "patrol.areaTypes"},
            )
        region_id = _required_string(entry, "regionId")
        try:
            area_type = PatrolAreaType(_required_string(entry, "areaType"))
        except ValueError as exc:
            raise ProtocolError(
                "validation_error",
                "Unknown patrol area type.",
                details={"field": "patrol.areaTypes.areaType"},
            ) from exc
        validated_area_types.append(
            {"regionId": region_id, "areaType": area_type.value}
        )

    battery_rth = _required_object(message, "batteryRth")
    battery_percent = battery_rth.get("batteryPercent")
    if battery_percent is not None and not isinstance(battery_percent, (int, float)):
        raise ProtocolError(
            "validation_error",
            "batteryPercent must be a number or null.",
            details={"field": "batteryRth.batteryPercent"},
        )
    for field_name in ("lowBattery", "rthActive", "safetyRthActive"):
        if not isinstance(battery_rth.get(field_name), bool):
            raise ProtocolError(
                "validation_error",
                f"{field_name} must be a boolean.",
                details={"field": f"batteryRth.{field_name}"},
            )

    return mission_status_message(
        state=state,
        active_roi=active_roi,
        patrol_priorities=validated_priorities,
        current_patrol_region_id=_optional_string(patrol, "currentRegionId"),
        target_patrol_region_id=_optional_string(patrol, "targetRegionId"),
        battery_percent=None if battery_percent is None else float(battery_percent),
        low_battery=battery_rth["lowBattery"],
        safety_rth_active=battery_rth["safetyRthActive"],
        rth_reason=_optional_string(battery_rth, "rthReason"),
        drone_connected=drone_connected,
        patrol_area_types=validated_area_types,
    )


def validate_mission_state_message(message: dict[str, Any]) -> dict[str, str]:
    message_type = _required_string(message, "type")
    if message_type != MESSAGE_TYPE_MISSION_STATE:
        raise ProtocolError(
            "invalid_type",
            "Mission-state messages must use type 'mission_state'.",
            details={"type": message_type},
        )
    raw_state = _required_string(message, "state")
    try:
        state = MissionState(raw_state)
    except ValueError as exc:
        raise ProtocolError(
            "validation_error",
            f"Unknown mission state '{raw_state}'.",
            details={"field": "state"},
        ) from exc
    return mission_state_message(state)


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
            "targetPatrolRegionId": _optional_string(
                data, "targetPatrolRegionId"
            ),
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


def _validate_command_params(
    command: str,
    params: dict[str, Any],
    command_id: str,
) -> dict[str, Any]:
    if command == CommandName.SET_PATROL_PRIORITIES:
        raw_priorities = params.get("priorities")
        if not isinstance(raw_priorities, list):
            raise ProtocolError(
                "invalid_params",
                "Patrol priorities must be a list.",
                command_id=command_id,
                details={"field": "priorities"},
            )
        priorities: list[dict[str, Any]] = []
        seen_region_ids: set[str] = set()
        seen_ranks: set[int] = set()
        for index, entry in enumerate(raw_priorities):
            if not isinstance(entry, dict):
                raise ProtocolError(
                    "invalid_params",
                    "Each patrol priority must be an object.",
                    command_id=command_id,
                    details={"field": f"priorities[{index}]"},
                )
            region_id = entry.get("regionId")
            rank = entry.get("rank")
            if not isinstance(region_id, str) or region_id not in PATROL_REGION_IDS:
                raise ProtocolError(
                    "invalid_params",
                    f"Unknown patrol region '{region_id}'.",
                    command_id=command_id,
                    details={"field": f"priorities[{index}].regionId"},
                )
            if isinstance(rank, bool) or not isinstance(rank, int) or rank < 1:
                raise ProtocolError(
                    "invalid_params",
                    "Patrol priority rank must be a positive integer.",
                    command_id=command_id,
                    details={"field": f"priorities[{index}].rank"},
                )
            if region_id in seen_region_ids:
                raise ProtocolError(
                    "invalid_params",
                    "Patrol priority region IDs must be unique.",
                    command_id=command_id,
                    details={"field": "priorities"},
                )
            if rank in seen_ranks:
                raise ProtocolError(
                    "invalid_params",
                    "Patrol priority ranks must be unique.",
                    command_id=command_id,
                    details={"field": "priorities"},
                )
            seen_region_ids.add(region_id)
            seen_ranks.add(rank)
            priorities.append({"regionId": region_id, "rank": rank})
        if seen_ranks != set(range(1, len(priorities) + 1)):
            raise ProtocolError(
                "invalid_params",
                "Patrol priority ranks must be contiguous from 1.",
                command_id=command_id,
                details={"field": "priorities"},
            )
        return {"priorities": priorities}

    if command == CommandName.SET_PATROL_AREA_TYPES:
        raw_area_types = params.get("areaTypes")
        if not isinstance(raw_area_types, list):
            raise ProtocolError(
                "invalid_params",
                "Patrol areaTypes must be a list.",
                command_id=command_id,
                details={"field": "areaTypes"},
            )
        area_types: list[dict[str, str]] = []
        seen_region_ids: set[str] = set()
        for index, entry in enumerate(raw_area_types):
            if not isinstance(entry, dict):
                raise ProtocolError(
                    "invalid_params",
                    "Each patrol area type must be an object.",
                    command_id=command_id,
                    details={"field": f"areaTypes[{index}]"},
                )
            region_id = entry.get("regionId")
            if not isinstance(region_id, str) or region_id not in PATROL_REGION_IDS:
                raise ProtocolError(
                    "invalid_params",
                    f"Unknown patrol region '{region_id}'.",
                    command_id=command_id,
                    details={"field": f"areaTypes[{index}].regionId"},
                )
            if region_id in seen_region_ids:
                raise ProtocolError(
                    "invalid_params",
                    "Patrol area type region IDs must be unique.",
                    command_id=command_id,
                    details={"field": "areaTypes"},
                )
            try:
                area_type = PatrolAreaType(entry.get("areaType"))
            except (TypeError, ValueError) as exc:
                raise ProtocolError(
                    "invalid_params",
                    "Patrol areaType must be PATROL, IGNORE, or NO_FLY.",
                    command_id=command_id,
                    details={"field": f"areaTypes[{index}].areaType"},
                ) from exc
            seen_region_ids.add(region_id)
            area_types.append(
                {"regionId": region_id, "areaType": area_type.value}
            )
        return {"areaTypes": area_types}

    if command == CommandName.SET_ROI:
        try:
            center = _required_object(params, "center")
        except ProtocolError as exc:
            raise ProtocolError(
                "invalid_params",
                exc.message,
                command_id=command_id,
                details=exc.details,
            ) from exc
        lat = _finite_number(center, "lat", command_id)
        lon = _finite_number(center, "lon", command_id)
        radius_m = _finite_number(params, "radiusM", command_id)
        if not -90.0 <= lat <= 90.0:
            raise ProtocolError(
                "invalid_params",
                "ROI latitude must be between -90 and 90.",
                command_id=command_id,
                details={"field": "center.lat"},
            )
        if not -180.0 <= lon <= 180.0:
            raise ProtocolError(
                "invalid_params",
                "ROI longitude must be between -180 and 180.",
                command_id=command_id,
                details={"field": "center.lon"},
            )
        if radius_m < 0.0:
            raise ProtocolError(
                "invalid_params",
                "ROI radiusM must be zero or greater.",
                command_id=command_id,
                details={"field": "radiusM"},
            )
        return {
            "center": {"lat": lat, "lon": lon},
            "radiusM": radius_m,
        }

    if command in {
        CommandName.START_PATROL,
        CommandName.CLEAR_ROI,
        CommandName.RETURN_HOME,
    } and params:
        raise ProtocolError(
            "invalid_params",
            f"Command '{command}' does not accept params.",
            command_id=command_id,
        )
    return params


def _finite_number(
    message: dict[str, Any],
    key: str,
    command_id: str,
) -> float:
    try:
        value = _required_number(message, key)
    except ProtocolError as exc:
        raise ProtocolError(
            "invalid_params",
            exc.message,
            command_id=command_id,
            details=exc.details,
        ) from exc
    if not math.isfinite(value):
        raise ProtocolError(
            "invalid_params",
            f"Field '{key}' must be finite.",
            command_id=command_id,
            details={"field": key},
        )
    return float(value)


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


def _optional_string(message: dict[str, Any], key: str) -> str | None:
    value = message.get(key)
    if value is None:
        return None
    if isinstance(value, str) and value:
        return value
    raise ProtocolError(
        "validation_error",
        f"Optional string field '{key}' is invalid.",
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
