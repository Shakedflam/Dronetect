import pytest

from services.gps import (
    DEFAULT_HOME,
    LocalPosition,
    destination_point,
    local_meters_to_wgs84,
    wgs84_to_local_meters,
    wgs84_distance_m,
)


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


def test_destination_point_zero_distance_returns_home():
    point = destination_point(DEFAULT_HOME, 0.0, 90.0)

    assert point == DEFAULT_HOME


def test_destination_point_bearing_east_moves_longitude_only():
    point = destination_point(DEFAULT_HOME, 100.0, 90.0)

    assert point.lat == pytest.approx(DEFAULT_HOME.lat)
    assert point.lng > DEFAULT_HOME.lng


def test_destination_point_bearing_north_moves_latitude_only():
    point = destination_point(DEFAULT_HOME, 100.0, 0.0)

    assert point.lat > DEFAULT_HOME.lat
    assert point.lng == pytest.approx(DEFAULT_HOME.lng)


def test_wgs84_distance_matches_short_range_destination_distance():
    point = destination_point(DEFAULT_HOME, 2500.0, 135.0)

    assert wgs84_distance_m(DEFAULT_HOME, point) == pytest.approx(
        2500.0, rel=1e-4
    )


def test_wgs84_distance_is_symmetric_and_zero_at_same_point():
    point = destination_point(DEFAULT_HOME, 1000.0, 45.0)

    assert wgs84_distance_m(DEFAULT_HOME, DEFAULT_HOME) == 0.0
    assert wgs84_distance_m(DEFAULT_HOME, point) == pytest.approx(
        wgs84_distance_m(point, DEFAULT_HOME)
    )


def test_wgs84_to_local_meters_reverses_local_conversion():
    original = LocalPosition(x=1200.0, y=-750.0, z=0.0)
    point = local_meters_to_wgs84(original)

    restored = wgs84_to_local_meters(point, altitude_m=15.0)

    assert restored.x == pytest.approx(original.x, rel=1e-4)
    assert restored.y == pytest.approx(original.y)
    assert restored.z == pytest.approx(15.0)
