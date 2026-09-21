"""Wire-level validation for the manual takeover and its stick stream."""

import pytest

from realtime_server.protocol import ProtocolError, validate_command_message


def command(name, params):
    return validate_command_message(
        {"type": "command", "commandId": "c-1", "command": name, "params": params}
    )


def test_manual_takeover_carries_a_boolean():
    assert command("set_manual_control", {"engaged": True})["params"] == {"engaged": True}


def test_a_manual_takeover_without_a_boolean_is_refused():
    with pytest.raises(ProtocolError) as error:
        command("set_manual_control", {"engaged": "yes"})
    assert error.value.code == "invalid_params"


def test_a_stick_is_four_normalized_axes():
    stick = command(
        "set_manual_command",
        {"forward": 1.0, "right": -0.5, "up": 0.0, "yawRate": 0.25},
    )["params"]
    assert stick == {"forward": 1.0, "right": -0.5, "up": 0.0, "yawRate": 0.25}


def test_a_missing_axis_rests_at_centre():
    """A pad that only drives one axis must not have to spell out the rest."""
    assert command("set_manual_command", {"forward": 1.0})["params"] == {
        "forward": 1.0,
        "right": 0.0,
        "up": 0.0,
        "yawRate": 0.0,
    }


@pytest.mark.parametrize("value", [1.5, -2.0, float("nan"), float("inf")])
def test_an_out_of_range_axis_is_refused_rather_than_clamped(value):
    """A client sending 50 is a bug, and flying its nearest legal value hides it."""
    with pytest.raises(ProtocolError) as error:
        command("set_manual_command", {"forward": value})
    assert error.value.code == "invalid_params"
    assert error.value.details == {"field": "forward"}


def test_a_non_numeric_axis_is_refused():
    with pytest.raises(ProtocolError) as error:
        command("set_manual_command", {"yawRate": True})
    assert error.value.code == "invalid_params"
