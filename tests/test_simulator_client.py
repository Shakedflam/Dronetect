import asyncio

import pytest

from adapters.simulator.client import SimulatorRealtimeClient
from realtime_server.protocol import encode_message


def test_connection_loss_watchdog_starts_only_after_an_established_link():
    now = [100.0]
    client = SimulatorRealtimeClient(clock=lambda: now[0])

    assert client.connection_lost_for(3.0) is False

    client._mark_connected()
    assert client.connected is True
    client._mark_disconnected()
    assert client.connected is False
    assert client.connection_lost_for(3.0) is False

    now[0] = 103.0
    assert client.connection_lost_for(3.0) is True


def test_reconnection_clears_connection_loss_timer():
    now = [100.0]
    client = SimulatorRealtimeClient(clock=lambda: now[0])
    client._mark_connected()
    client._mark_disconnected()
    now[0] = 110.0
    assert client.connection_lost_for(3.0) is True

    client._mark_connected()

    assert client.connected is True
    assert client.connection_lost_for(3.0) is False


def test_connection_loss_timeout_must_be_positive():
    client = SimulatorRealtimeClient()

    with pytest.raises(ValueError, match="positive"):
        client.connection_lost_for(0.0)


def test_detection_messages_are_received_separately_from_commands():
    message = {
        "type": "detections",
        "frameId": 12,
        "timestampMs": 1200,
        "frameWidth": 1000,
        "frameHeight": 500,
        "detections": [
            {
                "className": "drone",
                "confidence": 0.9,
                "box": [100.0, 100.0, 200.0, 200.0],
                "trackId": "drone-1",
                "trackState": "tentative",
                "lastSeenTimestamp": 1.2,
                "velocityPxPerSec": {"x": 0.0, "y": 0.0},
            }
        ],
    }

    class OneMessageWebSocket:
        def __aiter__(self):
            async def messages():
                yield encode_message(message)

            return messages()

    client = SimulatorRealtimeClient()
    asyncio.run(client._receive_commands(OneMessageWebSocket()))

    assert client.drain_commands() == []
    assert client.drain_detections() == [message]


def test_safety_return_home_reason_reaches_simulator_command_queue():
    message = {
        "type": "command",
        "commandId": "safety-rth-1",
        "command": "return_home",
        "params": {},
        "safetyReason": "gps_fault",
    }

    class OneMessageWebSocket:
        def __aiter__(self):
            async def messages():
                yield encode_message(message)

            return messages()

    client = SimulatorRealtimeClient()
    asyncio.run(client._receive_commands(OneMessageWebSocket()))

    assert client.drain_commands() == [message]
