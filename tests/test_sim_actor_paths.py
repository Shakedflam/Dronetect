import math

import pytest

from controllers.sim_actor_paths import (
    BirdPath,
    CircularPath,
    WaypointPath,
    bird_position,
    circular_position,
    waypoint_position,
    yaw_toward,
)


def test_waypoint_path_moves_at_constant_speed_and_loops():
    path = WaypointPath(
        waypoints=((0.0, 0.0, 10.0), (10.0, 0.0, 10.0), (10.0, 10.0, 10.0)),
        speed_mps=5.0,
    )

    assert waypoint_position(path, 1.0) == pytest.approx((5.0, 0.0, 10.0))
    assert waypoint_position(path, 2.0) == pytest.approx((10.0, 0.0, 10.0))
    assert waypoint_position(path, 4.0) == pytest.approx((10.0, 10.0, 10.0))


def test_circular_path_is_reproducible_at_known_times():
    path = CircularPath(
        center=(-45.0, 13.0, 19.0),
        radius_m=9.0,
        altitude_amplitude_m=1.0,
        period_s=20.0,
    )

    assert circular_position(path, 0.0) == pytest.approx((-36.0, 13.0, 19.0))
    assert circular_position(path, 5.0) == pytest.approx((-45.0, 22.0, 19.0))


def test_bird_path_varies_position_and_altitude_deterministically():
    path = BirdPath(
        center=(-50.0, 4.0, 12.0),
        radius_x_m=5.0,
        radius_y_m=3.0,
        altitude_amplitude_m=0.5,
        period_s=12.0,
        phase_rad=0.3,
    )

    first = bird_position(path, 1.25)
    second = bird_position(path, 1.25)

    assert first == pytest.approx(second)
    assert first[2] != pytest.approx(path.center[2])


def test_yaw_toward_returns_fallback_without_motion():
    assert yaw_toward((1.0, 2.0, 3.0), (1.0, 2.0, 3.0), fallback=0.7) == 0.7
    assert yaw_toward((0.0, 0.0, 0.0), (0.0, 2.0, 0.0)) == pytest.approx(
        math.pi / 2.0
    )
