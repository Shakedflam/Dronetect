"""Local wire protocol between the DJI flight bridge and the Android DJI layer.

This is deliberately a *separate* protocol from realtime_server's tablet/drone
facing one (`realtime_server/protocol.py`). It never leaves the aircraft's own
local link -- the bridge process and the Android app are expected to be on the
same device or the same LAN/hotspot, not routed through the cloud realtime
server. Keeping it separate means:

- realtime_server, the tablet app, and vision_server never need to know this
  protocol exists.
- DJI Mobile SDK V5 specifics never appear on the Python side: Android is
  responsible for translating every message here to/from actual SDK calls
  (VirtualStickManager, Gimbal, FlightController, ...). If DJI's SDK shapes
  change, only the Android translation layer changes, not this module.

Message shapes, Android -> bridge:
    - MESSAGE_TYPE_TELEMETRY: position/attitude/battery/gimbal snapshot, sent
      every tick. This is what feeds flight_logic.FlightLogic.update() each
      cycle, so it is the message the bridge's fail-safe logic (see
      android_link.py) treats as the "is this link still trustworthy" signal.
    - MESSAGE_TYPE_RC_STATE: whether the RC-N3's physical sticks are live and
      what they're commanding, matching manual_input.RemoteController.
    - MESSAGE_TYPE_GIMBAL_STATE: confirmation of the gimbal's actual pitch,
      independent of the continuous telemetry.gimbalPitchDeg feed FlightLogic
      consumes -- useful for detecting a stalled/limited gimbal.

Message shapes, bridge -> Android:
    - MESSAGE_TYPE_SETPOINT: a velocity + yaw command, local-frame meters/sec
      and degrees, matching flight_logic.advance_kinematic_flight()'s output
      shape. Carries `ttlMs`: Android must refuse to actuate a setpoint older
      than that, independent of whatever the bridge itself decides -- this is
      the second half of "never continue flying a stale movement command."
    - MESSAGE_TYPE_GIMBAL_COMMAND: a target gimbal pitch.
    - MESSAGE_TYPE_RTH: a fire-and-forget request to trigger DJI's own
      go-home/failsafe behavior. Unlike the setpoint/gimbal messages, this is
      never gated on telemetry freshness -- it must still be attempted when
      the link is degraded, since that is exactly when it matters most.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from realtime_server.protocol import encode_message, parse_json_message

__all__ = [
    "MESSAGE_TYPE_TELEMETRY",
    "MESSAGE_TYPE_RC_STATE",
    "MESSAGE_TYPE_GIMBAL_STATE",
    "MESSAGE_TYPE_SETPOINT",
    "MESSAGE_TYPE_GIMBAL_COMMAND",
    "MESSAGE_TYPE_RTH",
    "LocalProtocolError",
    "AndroidTelemetry",
    "AndroidRcState",
    "AndroidGimbalState",
    "VelocitySetpoint",
    "GimbalCommand",
    "RthCommand",
    "encode_message",
    "parse_json_message",
    "validate_android_telemetry_message",
    "build_android_telemetry_message",
    "validate_android_rc_state_message",
    "build_android_rc_state_message",
    "validate_android_gimbal_state_message",
    "build_android_gimbal_state_message",
    "build_setpoint_message",
    "validate_setpoint_message",
    "build_gimbal_command_message",
    "validate_gimbal_command_message",
    "build_rth_message",
    "validate_rth_message",
]

MESSAGE_TYPE_TELEMETRY = "dji_telemetry"
MESSAGE_TYPE_RC_STATE = "dji_rc_state"
MESSAGE_TYPE_GIMBAL_STATE = "dji_gimbal_state"

MESSAGE_TYPE_SETPOINT = "dji_setpoint"
MESSAGE_TYPE_GIMBAL_COMMAND = "dji_gimbal_command"
MESSAGE_TYPE_RTH = "dji_rth"

# Matches realtime_server.protocol.MANUAL_STICK_AXES's naming, but this is a
# distinct wire vocabulary local to this link.
STICK_AXES = ("forward", "right", "up", "yawRate")


class LocalProtocolError(ValueError):
    """A malformed message on the bridge<->Android local link."""

    def __init__(self, code: str, message: str, *, details: dict[str, Any] | None = None) -> None:
        self.code = code
        self.details = details or {}
        super().__init__(message)


@dataclass(frozen=True, slots=True)
class AndroidTelemetry:
    timestamp: float
    lat: float
    lng: float
    altitude_m: float
    heading_deg: float
    speed_mps: float
    battery_percent: float
    gimbal_pitch_deg: float
    flight_mode: str


@dataclass(frozen=True, slots=True)
class AndroidRcState:
    timestamp: float
    connected: bool
    forward: float
    right: float
    up: float
    yaw_rate: float


@dataclass(frozen=True, slots=True)
class AndroidGimbalState:
    timestamp: float
    pitch_deg: float


@dataclass(frozen=True, slots=True)
class VelocitySetpoint:
    """Local-frame velocity + yaw setpoint, bridge -> Android.

    Velocity axes match flight_logic.advance_kinematic_flight()'s local ENU
    meters/second frame (x=east, y=north, z=up). yaw_deg is an absolute
    heading in degrees (0 = north, clockwise) -- flight_logic works in
    radians internally, so the bridge converts at this boundary and nothing
    DJI-flavored (degrees, a particular yaw convention) leaks into the pure
    Python autonomy modules.
    """

    issued_at: float
    ttl_ms: int
    velocity_x_mps: float
    velocity_y_mps: float
    velocity_z_mps: float
    yaw_deg: float


@dataclass(frozen=True, slots=True)
class GimbalCommand:
    issued_at: float
    ttl_ms: int
    pitch_deg: float


@dataclass(frozen=True, slots=True)
class RthCommand:
    issued_at: float
    reason: str


# --------------------------------------------------------------------------
# Android -> bridge
# --------------------------------------------------------------------------


def validate_android_telemetry_message(message: dict[str, Any]) -> AndroidTelemetry:
    _require_type(message, MESSAGE_TYPE_TELEMETRY)
    gps = _required_object(message, "gps")
    return AndroidTelemetry(
        timestamp=_required_number(message, "timestamp"),
        lat=_required_number(gps, "lat"),
        lng=_required_number(gps, "lng"),
        altitude_m=_required_number(message, "altitudeM"),
        heading_deg=_required_number(message, "headingDeg"),
        speed_mps=_required_number(message, "speedMps"),
        battery_percent=_required_number(message, "batteryPercent"),
        gimbal_pitch_deg=_required_number(message, "gimbalPitchDeg"),
        flight_mode=_optional_string(message, "flightMode") or "",
    )


def build_android_telemetry_message(telemetry: AndroidTelemetry) -> dict[str, Any]:
    return {
        "type": MESSAGE_TYPE_TELEMETRY,
        "timestamp": telemetry.timestamp,
        "gps": {"lat": telemetry.lat, "lng": telemetry.lng},
        "altitudeM": telemetry.altitude_m,
        "headingDeg": telemetry.heading_deg,
        "speedMps": telemetry.speed_mps,
        "batteryPercent": telemetry.battery_percent,
        "gimbalPitchDeg": telemetry.gimbal_pitch_deg,
        "flightMode": telemetry.flight_mode,
    }


def validate_android_rc_state_message(message: dict[str, Any]) -> AndroidRcState:
    _require_type(message, MESSAGE_TYPE_RC_STATE)
    connected = message.get("connected")
    if not isinstance(connected, bool):
        raise LocalProtocolError(
            "validation_error", "connected must be a boolean.", details={"field": "connected"}
        )
    stick = _required_object(message, "stick")
    values = {axis: _required_number(stick, axis) for axis in STICK_AXES}
    return AndroidRcState(
        timestamp=_required_number(message, "timestamp"),
        connected=connected,
        forward=values["forward"],
        right=values["right"],
        up=values["up"],
        yaw_rate=values["yawRate"],
    )


def build_android_rc_state_message(state: AndroidRcState) -> dict[str, Any]:
    return {
        "type": MESSAGE_TYPE_RC_STATE,
        "timestamp": state.timestamp,
        "connected": state.connected,
        "stick": {
            "forward": state.forward,
            "right": state.right,
            "up": state.up,
            "yawRate": state.yaw_rate,
        },
    }


def validate_android_gimbal_state_message(message: dict[str, Any]) -> AndroidGimbalState:
    _require_type(message, MESSAGE_TYPE_GIMBAL_STATE)
    return AndroidGimbalState(
        timestamp=_required_number(message, "timestamp"),
        pitch_deg=_required_number(message, "pitchDeg"),
    )


def build_android_gimbal_state_message(state: AndroidGimbalState) -> dict[str, Any]:
    return {
        "type": MESSAGE_TYPE_GIMBAL_STATE,
        "timestamp": state.timestamp,
        "pitchDeg": state.pitch_deg,
    }


# --------------------------------------------------------------------------
# bridge -> Android
# --------------------------------------------------------------------------


def build_setpoint_message(setpoint: VelocitySetpoint) -> dict[str, Any]:
    return {
        "type": MESSAGE_TYPE_SETPOINT,
        "issuedAt": setpoint.issued_at,
        "ttlMs": setpoint.ttl_ms,
        "velocity": {
            "x": setpoint.velocity_x_mps,
            "y": setpoint.velocity_y_mps,
            "z": setpoint.velocity_z_mps,
        },
        "yawDeg": setpoint.yaw_deg,
    }


def validate_setpoint_message(message: dict[str, Any]) -> VelocitySetpoint:
    _require_type(message, MESSAGE_TYPE_SETPOINT)
    velocity = _required_object(message, "velocity")
    return VelocitySetpoint(
        issued_at=_required_number(message, "issuedAt"),
        ttl_ms=_required_int(message, "ttlMs"),
        velocity_x_mps=_required_number(velocity, "x"),
        velocity_y_mps=_required_number(velocity, "y"),
        velocity_z_mps=_required_number(velocity, "z"),
        yaw_deg=_required_number(message, "yawDeg"),
    )


def build_gimbal_command_message(command: GimbalCommand) -> dict[str, Any]:
    return {
        "type": MESSAGE_TYPE_GIMBAL_COMMAND,
        "issuedAt": command.issued_at,
        "ttlMs": command.ttl_ms,
        "pitchDeg": command.pitch_deg,
    }


def validate_gimbal_command_message(message: dict[str, Any]) -> GimbalCommand:
    _require_type(message, MESSAGE_TYPE_GIMBAL_COMMAND)
    return GimbalCommand(
        issued_at=_required_number(message, "issuedAt"),
        ttl_ms=_required_int(message, "ttlMs"),
        pitch_deg=_required_number(message, "pitchDeg"),
    )


def build_rth_message(command: RthCommand) -> dict[str, Any]:
    return {
        "type": MESSAGE_TYPE_RTH,
        "issuedAt": command.issued_at,
        "reason": command.reason,
    }


def validate_rth_message(message: dict[str, Any]) -> RthCommand:
    _require_type(message, MESSAGE_TYPE_RTH)
    return RthCommand(
        issued_at=_required_number(message, "issuedAt"),
        reason=_required_string(message, "reason"),
    )


# --------------------------------------------------------------------------
# validation helpers (local copies, intentionally not shared with
# realtime_server/protocol.py -- this link's message shapes are independent
# of the tablet-facing protocol and should be free to evolve separately)
# --------------------------------------------------------------------------


def _require_type(message: dict[str, Any], expected: str) -> None:
    actual = message.get("type")
    if actual != expected:
        raise LocalProtocolError(
            "invalid_type",
            f"Expected message type '{expected}', got '{actual}'.",
            details={"type": actual},
        )


def _required_number(message: dict[str, Any], key: str) -> float:
    value = message.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise LocalProtocolError(
            "validation_error",
            f"Required numeric field '{key}' is missing or invalid.",
            details={"field": key},
        )
    return float(value)


def _required_int(message: dict[str, Any], key: str) -> int:
    value = message.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise LocalProtocolError(
            "validation_error",
            f"Required integer field '{key}' is missing or invalid.",
            details={"field": key},
        )
    return value


def _required_string(message: dict[str, Any], key: str) -> str:
    value = message.get(key)
    if isinstance(value, str) and value:
        return value
    raise LocalProtocolError(
        "validation_error",
        f"Required string field '{key}' is missing or invalid.",
        details={"field": key},
    )


def _optional_string(message: dict[str, Any], key: str) -> str | None:
    value = message.get(key)
    if value is None:
        return None
    if isinstance(value, str):
        return value
    raise LocalProtocolError(
        "validation_error", f"Optional string field '{key}' is invalid.", details={"field": key}
    )


def _required_object(message: dict[str, Any], key: str) -> dict[str, Any]:
    value = message.get(key)
    if isinstance(value, dict):
        return value
    raise LocalProtocolError(
        "validation_error",
        f"Required object field '{key}' is missing or invalid.",
        details={"field": key},
    )
