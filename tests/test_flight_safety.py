import pytest

from services.flight_safety import (
    AIRCRAFT_RTH_REASON,
    LOW_BATTERY_REASON,
    DefaultSafetyRthPolicy,
    FlightSafetySnapshot,
)


def test_low_battery_at_threshold_requests_rth():
    policy = DefaultSafetyRthPolicy(low_battery_threshold_percent=25.0)

    signal = policy.evaluate(FlightSafetySnapshot(battery_percent=25.0))

    assert signal.required is True
    assert signal.reason == LOW_BATTERY_REASON


def test_battery_above_threshold_does_not_request_rth():
    policy = DefaultSafetyRthPolicy(low_battery_threshold_percent=25.0)

    signal = policy.evaluate(FlightSafetySnapshot(battery_percent=25.1))

    assert signal.required is False
    assert signal.reason is None


def test_normalized_aircraft_safety_rth_takes_precedence_over_battery():
    policy = DefaultSafetyRthPolicy(low_battery_threshold_percent=20.0)

    signal = policy.evaluate(
        FlightSafetySnapshot(
            battery_percent=90.0,
            aircraft_rth_active=True,
            aircraft_rth_reason="dji_failsafe_rth",
        )
    )

    assert signal.required is True
    assert signal.reason == "dji_failsafe_rth"


def test_aircraft_safety_rth_has_adapter_neutral_default_reason():
    signal = DefaultSafetyRthPolicy().evaluate(
        FlightSafetySnapshot(aircraft_rth_active=True)
    )

    assert signal.reason == AIRCRAFT_RTH_REASON


@pytest.mark.parametrize("threshold", [-0.1, 100.1, float("nan")])
def test_invalid_low_battery_threshold_is_rejected(threshold):
    with pytest.raises(ValueError):
        DefaultSafetyRthPolicy(low_battery_threshold_percent=threshold)
