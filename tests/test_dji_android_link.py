"""Tests for AndroidLink's fail-safe guarantees and wire round-trip.

Most tests drive AndroidLink's internal state directly (matching how
tests/test_simulator_client.py tests SimulatorRealtimeClient's connection
-loss watchdog) so the fail-safe logic is verified without needing a real
socket. One end-to-end test exercises the actual local WebSocket server.
"""

import asyncio
import json
import time
from dataclasses import replace

import pytest
from websockets.asyncio.client import connect

from adapters.dji.android_link import AndroidLink
from adapters.dji.local_protocol import (
    build_android_gimbal_state_message,
    build_android_rc_state_message,
    build_android_telemetry_message,
    AndroidGimbalState,
    AndroidRcState,
    AndroidTelemetry,
)


class _FakeConnection:
    """Stand-in for a ServerConnection: just enough to be "open" and
    recording what AndroidLink tries to send."""

    def __init__(self) -> None:
        self.sent_raw: list[str] = []

    async def send(self, raw: str) -> None:
        self.sent_raw.append(raw)


def _connected_link(now: list[float]) -> AndroidLink:
    link = AndroidLink(clock=lambda: now[0])
    link._connection = _FakeConnection()
    link._mark_connected()
    return link


def test_send_setpoint_refuses_without_a_connection():
    now = [0.0]
    link = AndroidLink(clock=lambda: now[0])

    sent = asyncio.run(
        link.send_setpoint(
            velocity_x_mps=1.0, velocity_y_mps=0.0, velocity_z_mps=0.0, yaw_deg=0.0, ttl_ms=250, telemetry_timeout_s=1.0
        )
    )

    assert sent is False


def test_send_setpoint_refuses_with_stale_telemetry():
    now = [0.0]
    link = _connected_link(now)
    link._handle_message(json.dumps(build_android_telemetry_message(
        AndroidTelemetry(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 100.0, 0.0, "")
    )))
    link._telemetry_received_monotonic = 0.0

    now[0] = 5.0  # far past any reasonable timeout

    sent = asyncio.run(
        link.send_setpoint(
            velocity_x_mps=1.0, velocity_y_mps=0.0, velocity_z_mps=0.0, yaw_deg=0.0, ttl_ms=250, telemetry_timeout_s=1.0
        )
    )

    assert sent is False


def test_send_setpoint_succeeds_with_fresh_telemetry():
    now = [0.0]
    link = _connected_link(now)
    link._handle_message(json.dumps(build_android_telemetry_message(
        AndroidTelemetry(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 100.0, 0.0, "")
    )))
    link._telemetry_received_monotonic = 0.0
    now[0] = 0.3  # within a 1.0s timeout

    sent = asyncio.run(
        link.send_setpoint(
            velocity_x_mps=1.0, velocity_y_mps=2.0, velocity_z_mps=-0.5, yaw_deg=90.0, ttl_ms=250, telemetry_timeout_s=1.0
        )
    )

    assert sent is True
    sent_messages = [json.loads(raw) for raw in link._connection.sent_raw]
    assert len(sent_messages) == 1
    assert sent_messages[0]["type"] == "dji_setpoint"
    assert sent_messages[0]["velocity"] == {"x": 1.0, "y": 2.0, "z": -0.5}
    assert sent_messages[0]["sessionId"] == link._session_id
    assert sent_messages[0]["seq"] > 0


def test_zero_ground_setpoint_uses_fresh_rc_safety_not_gps():
    now = [10.0]
    link = _connected_link(now)
    safe = AndroidRcState(timestamp=time.time(), connected=True,
                          forward=0.0, right=0.0, up=0.0, yaw_rate=0.0,
                          aircraft_connected=True, is_flying=False)
    link._handle_message(json.dumps(build_android_rc_state_message(safe)))
    assert link.latest_telemetry is None
    assert asyncio.run(link.send_zero_ground_test_setpoint(
        ttl_ms=250, safety_timeout_s=1.0))
    assert json.loads(link._connection.sent_raw[0])["velocity"] == {
        "x": 0.0, "y": 0.0, "z": 0.0}
    assert not asyncio.run(link.send_setpoint(
        velocity_x_mps=1.0, velocity_y_mps=0.0, velocity_z_mps=0.0,
        yaw_deg=0.0, ttl_ms=250, telemetry_timeout_s=1.0))

    for unsafe in (replace(safe, right=0.1), replace(safe, connected=False),
                   replace(safe, aircraft_connected=False),
                   replace(safe, is_flying=True), replace(safe, is_flying=None)):
        link._handle_message(json.dumps(build_android_rc_state_message(unsafe)))
        assert not asyncio.run(link.send_zero_ground_test_setpoint(
            ttl_ms=250, safety_timeout_s=1.0))
    link._handle_message(json.dumps(build_android_rc_state_message(safe)))
    now[0] += 1.1
    assert not asyncio.run(link.send_zero_ground_test_setpoint(
        ttl_ms=250, safety_timeout_s=1.0))


def test_phase4_pulse_requires_fresh_stable_airborne_telemetry_and_centered_rc():
    now = [10.0]
    link = _connected_link(now)
    telemetry = AndroidTelemetry(
        timestamp=time.time(), lat=32.0, lng=34.0, altitude_m=3.0,
        heading_deg=0.0, speed_mps=0.0, battery_percent=80.0,
        gimbal_pitch_deg=0.0, flight_mode="GPS_NORMAL", is_flying=True,
    )
    rc = AndroidRcState(
        timestamp=time.time(), connected=True, forward=0.0, right=0.0,
        up=0.0, yaw_rate=0.0, aircraft_connected=True, is_flying=True,
    )
    link._handle_message(json.dumps(build_android_telemetry_message(telemetry)))
    link._handle_message(json.dumps(build_android_rc_state_message(rc)))
    assert link.phase4_airborne_safety_rejection(1.0) is None
    assert asyncio.run(link.send_phase4_vertical_pulse_setpoint(
        vertical_mps=0.30, ttl_ms=250, safety_timeout_s=1.0))
    assert json.loads(link._connection.sent_raw[0])["velocity"] == {
        "x": 0.0, "y": 0.0, "z": 0.30}
    assert not asyncio.run(link.send_phase4_vertical_pulse_setpoint(
        vertical_mps=-0.30, ttl_ms=250, safety_timeout_s=1.0))
    link._handle_message(json.dumps(build_android_rc_state_message(replace(rc, up=0.1))))
    assert link.phase4_airborne_safety_rejection(1.0) == "rc_stick_moved_up_0.100"
    assert not asyncio.run(link.send_phase4_vertical_pulse_setpoint(
        vertical_mps=0.30, ttl_ms=250, safety_timeout_s=1.0))
    now[0] += 1.1
    assert link.phase4_airborne_safety_rejection(1.0) == "telemetry_stale"


def test_authenticated_clock_probe_echoes_tablet_timestamp_and_server_receive_time():
    link = AndroidLink(auth_token="a" * 64)
    link._connection = _FakeConnection()
    link._mark_connected()
    probe = time.time()
    response = link._handle_message(json.dumps({
        "type": "dji_clock_probe", "sessionId": link._session_id,
        "seq": 1, "timestamp": probe,
    }))
    assert response is not None
    assert response["type"] == "dji_clock_sync"
    assert response["sessionId"] == link._session_id
    assert response["probeTimestamp"] == probe
    assert abs(response["serverReceivedAt"] - time.time()) < 1.0
    assert link._last_sequence == 1
    assert link._handle_message(json.dumps({
        "type": "dji_clock_probe", "sessionId": "b" * 32,
        "seq": 2, "timestamp": time.time(),
    })) is None


def test_send_rth_ignores_telemetry_freshness():
    """RTH must be attempted even when telemetry is stale or absent --
    that's exactly when it matters most. Only "not connected at all" stops it."""
    now = [0.0]
    link = _connected_link(now)
    # No telemetry has ever been received -- send_setpoint would refuse, but
    # send_rth must not.
    sent = asyncio.run(link.send_rth(reason="connection_lost"))

    assert sent is True
    sent_messages = [json.loads(raw) for raw in link._connection.sent_raw]
    assert sent_messages[0]["type"] == "dji_rth"
    assert sent_messages[0]["reason"] == "connection_lost"


def test_send_release_ignores_telemetry_freshness():
    """Handing the aircraft to the pilot must never wait on telemetry."""
    now = [0.0]
    link = _connected_link(now)
    assert asyncio.run(link.send_release(reason="rc_stick_up_-0.30")) is True
    sent = [json.loads(raw) for raw in link._connection.sent_raw]
    assert sent[0]["type"] == "dji_release"
    assert sent[0]["reason"] == "rc_stick_up_-0.30"
    assert sent[0]["sessionId"] and sent[0]["seq"] == 1


def test_native_takeoff_and_land_require_fresh_telemetry():
    """Unlike release/RTH these start motion, so they are gated like setpoints."""
    now = [0.0]
    link = _connected_link(now)
    assert asyncio.run(link.send_native_takeoff(control_epoch=3, telemetry_timeout_s=1.0)) is False
    assert asyncio.run(link.send_native_land(control_epoch=3, telemetry_timeout_s=1.0)) is False
    assert link._connection.sent_raw == []

    link._handle_message(json.dumps(build_android_telemetry_message(
        AndroidTelemetry(0.0, 32.0, 34.0, 10.0, 0.0, 0.0, 100.0, 0.0, "GPS_NORMAL", True)
    )))
    assert asyncio.run(link.send_native_takeoff(control_epoch=3, telemetry_timeout_s=1.0)) is True
    assert asyncio.run(link.send_native_land(control_epoch=3, telemetry_timeout_s=1.0)) is True
    sent = [json.loads(raw) for raw in link._connection.sent_raw]
    assert [m["type"] for m in sent] == ["dji_takeoff", "dji_land"]
    assert all(m["controlEpoch"] == 3 and m["ttlMs"] == 1500 for m in sent)
    assert [m["seq"] for m in sent] == [1, 2]


def test_rc_stick_deflection_uses_androids_centre_band_and_freshness():
    now = [0.0]
    link = _connected_link(now)

    def feed(**sticks):
        values = dict(forward=0.0, right=0.0, up=0.0, yaw_rate=0.0)
        values.update(sticks)
        link._handle_message(json.dumps(build_android_rc_state_message(AndroidRcState(
            timestamp=now[0], connected=True, aircraft_connected=True, is_flying=True, **values))))

    feed(up=33 / 660)  # inside the centre band Android treats as "not moved"
    assert link.rc_stick_deflection(1.0) is None
    feed(up=-0.30)
    assert link.rc_stick_deflection(1.0) == ("up", -0.30)
    now[0] += 2.0  # stale RC state never reads as a takeover
    assert link.rc_stick_deflection(1.0) is None


def test_send_gimbal_command_requires_connection_but_not_fresh_telemetry():
    now = [0.0]
    link = _connected_link(now)

    sent = asyncio.run(link.send_gimbal_command(pitch_deg=10.0, ttl_ms=1000))

    assert sent is True
    sent_messages = [json.loads(raw) for raw in link._connection.sent_raw]
    assert sent_messages[0]["pitchDeg"] == 10.0


def test_connection_lost_for_mirrors_simulator_client_semantics():
    now = [100.0]
    link = AndroidLink(clock=lambda: now[0])

    assert link.connection_lost_for(3.0) is False

    link._mark_connected()
    link._connection = object()
    assert link.connected is True

    link._connection = None
    link._mark_disconnected()
    assert link.connected is False
    assert link.connection_lost_for(3.0) is False

    now[0] = 103.0
    assert link.connection_lost_for(3.0) is True


def test_connection_lost_for_requires_positive_timeout():
    link = AndroidLink()
    with pytest.raises(ValueError, match="positive"):
        link.connection_lost_for(0.0)


def test_reconnection_clears_connection_loss_timer():
    now = [100.0]
    link = AndroidLink(clock=lambda: now[0])
    link._mark_connected()
    link._connection = None
    link._mark_disconnected()
    now[0] = 110.0
    assert link.connection_lost_for(3.0) is True

    link._connection = object()
    link._mark_connected()

    assert link.connected is True
    assert link.connection_lost_for(3.0) is False


def test_handle_message_drops_malformed_input_without_raising():
    link = AndroidLink()
    link._handle_message("not json at all")
    link._handle_message(json.dumps({"type": "dji_telemetry"}))  # missing required fields
    assert link.latest_telemetry is None


def test_handle_message_ignores_unknown_type():
    link = AndroidLink()
    link._handle_message(json.dumps({"type": "future_message_type", "foo": "bar"}))
    assert link.latest_telemetry is None
    assert link.latest_rc_state is None


def test_handle_message_updates_rc_and_gimbal_state():
    link = AndroidLink()
    link._handle_message(json.dumps(build_android_rc_state_message(AndroidRcState(1.0, True, 0.1, 0.2, 0.3, 0.4))))
    link._handle_message(json.dumps(build_android_gimbal_state_message(AndroidGimbalState(2.0, -30.0))))

    assert link.latest_rc_state == AndroidRcState(1.0, True, 0.1, 0.2, 0.3, 0.4)
    assert link.latest_gimbal_state == AndroidGimbalState(2.0, -30.0)


def test_authenticated_session_rejects_stale_and_wrong_session_and_clears_on_disconnect():
    link = AndroidLink(auth_token="a" * 64)
    link._connection = _FakeConnection()
    link._mark_connected()
    message = build_android_telemetry_message(
        AndroidTelemetry(time.time(), 32.0, 34.8, 15.0, 90.0, 0.0, 64.0, None, "GPS_ATTI")
    )
    message.update(sessionId=link._session_id, seq=1)
    link._handle_message(json.dumps(message))
    assert link.latest_telemetry is not None
    old_session = link._session_id
    link._connection = None
    link._mark_disconnected()
    assert link.latest_telemetry is None
    link._connection = _FakeConnection()
    link._mark_connected()
    assert link._session_id != old_session
    link._handle_message(json.dumps(message))
    assert link.latest_telemetry is None
    message.update(sessionId=link._session_id, timestamp=time.time() - 10)
    link._handle_message(json.dumps(message))
    assert link.latest_telemetry is None
    message["timestamp"] = time.time()
    link._handle_message(json.dumps(message))
    assert link.latest_telemetry is not None
    message["gps"] = {"lat": 33.0, "lng": 35.0}
    link._handle_message(json.dumps(message))
    assert link.latest_telemetry.lat == 32.0  # duplicate sequence refused


def test_remote_socket_requires_bearer_token_and_exact_path():
    async def scenario():
        link = AndroidLink(host="127.0.0.1", port=0, auth_token="a" * 64)
        task = asyncio.create_task(link.serve_forever())
        try:
            for _ in range(50):
                if link._server is not None:
                    break
                await asyncio.sleep(0.01)
            port = link._server.sockets[0].getsockname()[1]
            async with connect(f"ws://127.0.0.1:{port}/dji/bridge") as unauthenticated:
                await unauthenticated.wait_closed()
                assert not link.connected
            async with connect(f"ws://127.0.0.1:{port}/wrong",
                               additional_headers={"Authorization": "Bearer " + "a" * 64}) as wrong_path:
                await wrong_path.wait_closed()
                assert not link.connected
            async with connect(f"ws://127.0.0.1:{port}/dji/bridge",
                               additional_headers={"Authorization": "Bearer " + "a" * 64}) as android:
                hello = json.loads(await android.recv())
                assert hello["type"] == "dji_hello"
                assert len(hello["sessionId"]) == 32
                assert link.connected
        finally:
            task.cancel()
    asyncio.run(scenario())


# -- end-to-end over a real local socket -------------------------------------


def test_android_telemetry_reaches_the_link_over_a_real_socket():
    async def scenario():
        link = AndroidLink(host="127.0.0.1", port=0)
        server_task = asyncio.create_task(link.serve_forever())
        try:
            # Let the server bind before connecting.
            for _ in range(50):
                if link._server is not None:
                    break
                await asyncio.sleep(0.01)
            assert link._server is not None
            port = link._server.sockets[0].getsockname()[1]

            async with connect(f"ws://127.0.0.1:{port}") as android:
                telemetry = AndroidTelemetry(1.0, 32.0, 34.8, 15.0, 90.0, 2.0, 88.0, -5.0, "GPS_ATTI")
                await android.send(json.dumps(build_android_telemetry_message(telemetry)))

                for _ in range(50):
                    if link.latest_telemetry is not None:
                        break
                    await asyncio.sleep(0.01)

                assert link.latest_telemetry == telemetry
                assert link.connected is True
        finally:
            server_task.cancel()

    asyncio.run(scenario())


def test_zero_setpoints_keep_50ms_cadence_with_concurrent_uplink():
    """Telemetry and clock replies must not queue local downlink for seconds."""
    async def scenario():
        link = AndroidLink(host="127.0.0.1", port=0, auth_token="a" * 64)
        server_task = asyncio.create_task(link.serve_forever())
        try:
            for _ in range(50):
                if link._server is not None:
                    break
                await asyncio.sleep(0.01)
            port = link._server.sockets[0].getsockname()[1]
            async with connect(f"ws://127.0.0.1:{port}/dji/bridge",
                               additional_headers={"Authorization": "Bearer " + "a" * 64}) as android:
                session = json.loads(await android.recv())["sessionId"]
                telemetry = build_android_telemetry_message(
                    AndroidTelemetry(time.time(), 32.0, 34.8, 0.0, 90.0, 0.0, 64.0, 0.0, "GPS"))
                telemetry.update(sessionId=session, seq=1, timestamp=time.time())
                await android.send(json.dumps(telemetry))
                for _ in range(50):
                    if link.latest_telemetry is not None:
                        break
                    await asyncio.sleep(0.01)
                assert link.latest_telemetry is not None

                async def uplink():
                    for seq in range(2, 12):
                        message = {"type": "dji_clock_probe", "sessionId": session,
                                   "seq": seq, "timestamp": time.time()}
                        await android.send(json.dumps(message))
                        await asyncio.sleep(0.05)

                async def downlink():
                    for _ in range(10):
                        assert await link.send_setpoint(
                            velocity_x_mps=0.0, velocity_y_mps=0.0,
                            velocity_z_mps=0.0, yaw_deg=90.0, ttl_ms=250,
                            telemetry_timeout_s=1.0)
                        await asyncio.sleep(0.05)

                sender = asyncio.create_task(downlink())
                producer = asyncio.create_task(uplink())
                ages_ms = []
                while len(ages_ms) < 10:
                    message = json.loads(await asyncio.wait_for(android.recv(), 2.0))
                    if message["type"] == "dji_setpoint":
                        ages_ms.append((time.time() - message["issuedAt"]) * 1000)
                await sender
                await producer
                assert max(ages_ms) < 250
        finally:
            server_task.cancel()

    asyncio.run(scenario())


def test_second_android_connection_is_refused():
    async def scenario():
        link = AndroidLink(host="127.0.0.1", port=0)
        server_task = asyncio.create_task(link.serve_forever())
        try:
            for _ in range(50):
                if link._server is not None:
                    break
                await asyncio.sleep(0.01)
            port = link._server.sockets[0].getsockname()[1]

            async with connect(f"ws://127.0.0.1:{port}") as first:
                for _ in range(50):
                    if link.connected:
                        break
                    await asyncio.sleep(0.01)
                assert link.connected is True

                with pytest.raises(Exception):
                    async with connect(f"ws://127.0.0.1:{port}") as second:
                        await second.recv()
        finally:
            server_task.cancel()

    asyncio.run(scenario())
