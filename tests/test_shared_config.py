import pytest

from config.shared_config import OperatingConfig, shared_config


def test_operating_config_accepts_values_at_the_legal_boundary():
    config = OperatingConfig(
        max_flight_radius_m=shared_config.legal_max_flight_radius_m,
        cruise_altitude_m=shared_config.legal_max_cruise_altitude_m,
    )

    assert config.max_flight_radius_m == shared_config.legal_max_flight_radius_m
    assert config.cruise_altitude_m == shared_config.legal_max_cruise_altitude_m


def test_operating_config_rejects_radius_above_legal_maximum():
    with pytest.raises(ValueError, match="legal maximum"):
        OperatingConfig(
            max_flight_radius_m=shared_config.legal_max_flight_radius_m + 1.0,
            cruise_altitude_m=10.0,
        )


def test_operating_config_rejects_altitude_above_legal_maximum():
    with pytest.raises(ValueError, match="legal maximum"):
        OperatingConfig(
            max_flight_radius_m=300.0,
            cruise_altitude_m=shared_config.legal_max_cruise_altitude_m + 1.0,
        )
