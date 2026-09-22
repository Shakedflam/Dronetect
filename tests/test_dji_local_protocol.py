"""Round-trip and validation tests for the bridge<->Android local protocol."""

import pytest

from adapters.dji.local_protocol import (
    AndroidGimbalState,
    AndroidRcState,
    AndroidTelemetry,
    GimbalCommand,
    LocalProtocolError,
    RthCommand,
    VelocitySetpoint,
    build_android_gimbal_state_message,
    build_android_rc_state_message,
    build_android_telemetry_message,
    build_gimbal_command_message,
    build_rth_message,
    build_setpoint_message,
    validate_android_gimbal_state_message,
    validate_android_rc_state_message,
    validate_android_telemetry_message,
    validate_gimbal_command_message,
    validate_rth_message,
    validate_setpoint_message,
)


def test_telemetry_round_trips():
    telemetry = AndroidTelemetry(
        timestamp=12.5,
        lat=32.1,
        lng=34.8,
        altitude_m=42.0,
        heading_deg=91.5,
        speed_mps=3.2,
        battery_percent=76.0,
        gimbal_pitch_deg=-12.0,
        flight_mode="GPS_ATTI",
    )
    message = build_android_telemetry_message(telemetry)
    assert message["type"] == "dji_telemetry"
    assert validate_android_telemetry_message(message) == telemetry


def test_telemetry_rejects_missing_field():
    message = build_android_telemetry_message(
        AndroidTelemetry(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "")
    )
    del message["altitudeM"]
    with pytest.raises(LocalProtocolError):
        validate_android_telemetry_message(message)


def test_telemetry_rejects_wrong_type():
    message = build_android_rc_state_message(AndroidRcState(0.0, True, 0.0, 0.0, 0.0, 0.0))
    with pytest.raises(LocalProtocolError):
        validate_android_telemetry_message(message)


def test_rc_state_round_trips():
    state = AndroidRcState(timestamp=1.0, connected=True, forward=0.5, right=-0.25, up=0.0, yaw_rate=0.1)
    message = build_android_rc_state_message(state)
    assert message["type"] == "dji_rc_state"
    assert validate_android_rc_state_message(message) == state


def test_rc_state_requires_boolean_connected():
    message = build_android_rc_state_message(AndroidRcState(0.0, True, 0.0, 0.0, 0.0, 0.0))
    message["connected"] = "yes"
    with pytest.raises(LocalProtocolError):
        validate_android_rc_state_message(message)


def test_rc_state_stick_out_of_range_is_still_a_valid_message():
    # The local protocol intentionally does not clamp/reject stick magnitude
    # -- that's a flight_logic/manual_velocity_command concern, not a wire
    # concern, matching how realtime_server/protocol.py DOES clamp its own
    # tablet-facing SET_MANUAL_COMMAND (a different, stricter contract for a
    # different, less-trusted client).
    state = AndroidRcState(timestamp=1.0, connected=True, forward=5.0, right=0.0, up=0.0, yaw_rate=0.0)
    message = build_android_rc_state_message(state)
    assert validate_android_rc_state_message(message).forward == 5.0


def test_gimbal_state_round_trips():
    state = AndroidGimbalState(timestamp=3.0, pitch_deg=-45.0)
    message = build_android_gimbal_state_message(state)
    assert message["type"] == "dji_gimbal_state"
    assert validate_android_gimbal_state_message(message) == state


def test_setpoint_round_trips():
    setpoint = VelocitySetpoint(
        issued_at=100.0, ttl_ms=250, velocity_x_mps=1.5, velocity_y_mps=-2.0, velocity_z_mps=0.5, yaw_deg=270.0
    )
    message = build_setpoint_message(setpoint)
    assert message["type"] == "dji_setpoint"
    assert validate_setpoint_message(message) == setpoint


def test_setpoint_rejects_missing_velocity_axis():
    message = build_setpoint_message(VelocitySetpoint(0.0, 100, 0.0, 0.0, 0.0, 0.0))
    del message["velocity"]["z"]
    with pytest.raises(LocalProtocolError):
        validate_setpoint_message(message)


def test_gimbal_command_round_trips():
    command = GimbalCommand(issued_at=5.0, ttl_ms=1000, pitch_deg=12.5)
    message = build_gimbal_command_message(command)
    assert message["type"] == "dji_gimbal_command"
    assert validate_gimbal_command_message(message) == command


def test_rth_round_trips():
    command = RthCommand(issued_at=9.0, reason="connection_lost")
    message = build_rth_message(command)
    assert message["type"] == "dji_rth"
    assert validate_rth_message(message) == command


def test_rth_requires_nonempty_reason():
    message = build_rth_message(RthCommand(0.0, "x"))
    message["reason"] = ""
    with pytest.raises(LocalProtocolError):
        validate_rth_message(message)


@pytest.mark.parametrize(
    "validator,builder,make",
    [
        (validate_setpoint_message, build_setpoint_message, lambda: VelocitySetpoint(0.0, 1, 0.0, 0.0, 0.0, 0.0)),
        (validate_gimbal_command_message, build_gimbal_command_message, lambda: GimbalCommand(0.0, 1, 0.0)),
        (validate_rth_message, build_rth_message, lambda: RthCommand(0.0, "r")),
    ],
)
def test_bridge_to_android_messages_reject_wrong_type(validator, builder, make):
    message = builder(make())
    message["type"] = "not_a_real_type"
    with pytest.raises(LocalProtocolError):
        validator(message)
