import pytest

from adapters.simulator.client import SimulatorRealtimeClient


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
