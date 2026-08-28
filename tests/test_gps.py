import pytest

from services.gps import DEFAULT_HOME, LocalPosition, local_meters_to_wgs84


def test_home_local_position_maps_to_default_home():
    gps = local_meters_to_wgs84(LocalPosition(0.0, 0.0, 15.0))

    assert gps.lat == pytest.approx(DEFAULT_HOME.lat)
    assert gps.lng == pytest.approx(DEFAULT_HOME.lng)


def test_positive_x_moves_longitude_east():
    gps = local_meters_to_wgs84(LocalPosition(100.0, 0.0, 15.0))

    assert gps.lat == pytest.approx(DEFAULT_HOME.lat)
    assert gps.lng > DEFAULT_HOME.lng


def test_positive_y_moves_latitude_north():
    gps = local_meters_to_wgs84(LocalPosition(0.0, 100.0, 15.0))

    assert gps.lat > DEFAULT_HOME.lat
    assert gps.lng == pytest.approx(DEFAULT_HOME.lng)


def test_altitude_is_not_folded_into_wgs84_position():
    low = local_meters_to_wgs84(LocalPosition(10.0, 20.0, 1.0))
    high = local_meters_to_wgs84(LocalPosition(10.0, 20.0, 100.0))

    assert high == low
