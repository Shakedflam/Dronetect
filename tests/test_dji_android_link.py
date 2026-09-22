"""Tests for AndroidLink's fail-safe guarantees and wire round-trip.

Most tests drive AndroidLink's internal state directly (matching how
tests/test_simulator_client.py tests SimulatorRealtimeClient's connection
-loss watchdog) so the fail-safe logic is verified without needing a real
socket. One end-to-end test exercises the actual local WebSocket server.
"""

import asyncio
import json

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
