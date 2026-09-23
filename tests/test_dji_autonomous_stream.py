"""Regression tests for the real-flight stall: the ROI mission advanced on the
server while the aircraft never moved.

Two independent causes are pinned here:
  * the bridge produces no setpoints at all until a grounded fix establishes
    session_home, which is silent unless it says so;
  * a transiting mission must put real, non-zero velocity on the wire.
"""
import asyncio
import json

from adapters.dji.flight_bridge import FlightState
from services.mission_commands import CommandName
from services.gps import destination_point
from tests.test_dji_flight_bridge import (
    DEFAULT_HOME,
    _feed_ground_rc,
    _feed_telemetry,
    _make_bridge,
    _queue_command,
)


class _Recorder:
    def __init__(self):
        self.messages = []

    async def send(self, raw):
        self.messages.append(json.loads(raw))


def _setpoints(recorder):
    return [m for m in recorder.messages if m.get("type") == "dji_setpoint"]


def _tick(bridge, recorder):
    # _feed_telemetry/_feed_ground_rc each swap in a placeholder connection, so
    # the recorder has to be reinstalled immediately before every tick.
    bridge.android_link._connection = recorder
    return asyncio.run(bridge.run_tick_once())


def _ground_then_fly(bridge, now, recorder, *, grounded_first: bool, altitude_m=15.0):
    if grounded_first:
        _feed_telemetry(bridge, now, is_flying=False, altitude_m=0.0, flight_mode="GPS_NORMAL")
        _feed_ground_rc(bridge, now, is_flying=False)
        _tick(bridge, recorder)
    for _ in range(3):
        now[0] += 0.05
        _feed_telemetry(bridge, now, is_flying=True, altitude_m=altitude_m,
                        speed_mps=0.0, battery_percent=80.0, flight_mode="GPS_NORMAL")
        _feed_ground_rc(bridge, now, is_flying=True)
        _tick(bridge, recorder)


def test_bridge_never_streams_setpoints_without_a_grounded_session_home(capsys):
    """Linking the bridge only after takeoff leaves session_home unset, so no
    setpoint is ever produced. That must be stated, not silent."""
    now = [0.0]
    bridge = _make_bridge(now)
    recorder = _Recorder()

    _ground_then_fly(bridge, now, recorder, grounded_first=False)

    assert bridge.session_home is None
    assert _setpoints(recorder) == []
    assert "session_home=waiting reason=aircraft_not_grounded" in capsys.readouterr().out


def test_bridge_streams_setpoints_once_a_grounded_fix_established_home():
    now = [0.0]
    bridge = _make_bridge(now)
    recorder = _Recorder()

    _ground_then_fly(bridge, now, recorder, grounded_first=True)

    assert bridge.session_home is not None
    assert _setpoints(recorder), "a configured, airborne session must stream setpoints"


def test_idle_mission_streams_zero_velocity_setpoints():
    """These are the commands that must not arm DJI authority on Android."""
    now = [0.0]
    bridge = _make_bridge(now)
    recorder = _Recorder()

    _ground_then_fly(bridge, now, recorder, grounded_first=True)

    assert bridge.logic.state is FlightState.IDLE
    velocity = _setpoints(recorder)[-1]["velocity"]
    assert (velocity["x"], velocity["y"], velocity["z"]) == (0.0, 0.0, 0.0)


def test_transiting_to_roi_puts_non_zero_velocity_on_the_wire():
    """The core of the real failure: a transiting mission must serialize real
    motion, with the TTL and yaw the Android side expects."""
    now = [0.0]
    bridge = _make_bridge(now)
    recorder = _Recorder()
    _ground_then_fly(bridge, now, recorder, grounded_first=True)

    roi_point = destination_point(DEFAULT_HOME, 60.0, 90.0)
    _queue_command(
        bridge,
        CommandName.SET_ROI.value,
        {"center": {"lat": roi_point.lat, "lon": roi_point.lng}, "radiusM": 20.0},
    )

    moving = None
    for _ in range(40):
        now[0] += 0.05
        _feed_telemetry(bridge, now, is_flying=True, altitude_m=15.0, speed_mps=0.0,
                        battery_percent=80.0, flight_mode="GPS_NORMAL")
        _feed_ground_rc(bridge, now, is_flying=True)
        outcome = _tick(bridge, recorder)
        latest = _setpoints(recorder)[-1]
        velocity = latest["velocity"]
        if any(velocity[axis] != 0.0 for axis in ("x", "y", "z")):
            moving = latest
            break

    assert moving is not None, "transiting toward the ROI produced only zero velocity"
    assert outcome.state in {
        FlightState.TRANSITING_TO_ROI.value,
        FlightState.TAKING_OFF.value,
    }
    assert moving["ttlMs"] == bridge.config.setpoint_ttl_ms
    assert 0.0 <= moving["yawDeg"] <= 360.0
    assert all(isinstance(moving["velocity"][axis], float) for axis in ("x", "y", "z"))
