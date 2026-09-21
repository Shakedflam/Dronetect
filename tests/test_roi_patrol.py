import math

import pytest

from controllers.autonomous_drone.roi_patrol import (
    LawnMowerRoiPatrolPlanner,
    RoiPatrolConfig,
)


def test_lawnmower_waypoints_are_inset_inside_roi():
    planner = LawnMowerRoiPatrolPlanner(
        RoiPatrolConfig(lane_count=5, boundary_margin_m=3.0)
    )

    planner.reset((10.0, -5.0), 20.0, 12.0, (0.0, 0.0, 12.0))

    assert len(planner.waypoints) == 10
    for x, y, z in planner.waypoints:
        assert math.hypot(x - 10.0, y + 5.0) <= 17.0 + 1e-9
        assert z == 12.0


def test_lawnmower_plan_cycles_forever():
    planner = LawnMowerRoiPatrolPlanner(RoiPatrolConfig(lane_count=2))
    first = planner.reset((0.0, 0.0), 20.0, 10.0, (-20.0, 0.0, 10.0))

    following = [planner.next_waypoint() for _ in range(len(planner.waypoints))]

    assert following[-1] == first
    assert len(set(planner.waypoints)) == 4


def test_roi_progress_snapshot_restores_exact_next_waypoint():
    planner = LawnMowerRoiPatrolPlanner(RoiPatrolConfig(lane_count=3))
    planner.reset((0.0, 0.0), 20.0, 10.0, (-20.0, 0.0, 10.0))
    planner.next_waypoint()
    progress = planner.snapshot()
    expected = planner.next_waypoint()

    restored = LawnMowerRoiPatrolPlanner(RoiPatrolConfig(lane_count=3))
    restored.restore(progress)

    assert restored.next_waypoint() == expected
    assert progress.remaining_path[0] == expected


def test_tiny_roi_degrades_safely_to_center_hold():
    planner = LawnMowerRoiPatrolPlanner(RoiPatrolConfig(boundary_margin_m=2.0))

    first = planner.reset((4.0, 5.0), 1.0, 8.0, (4.0, 5.0, 8.0))

    assert first == (4.0, 5.0, 8.0)
    assert planner.next_waypoint() == first


def test_partial_roi_is_clipped_to_maximum_flight_radius():
    planner = LawnMowerRoiPatrolPlanner(
        RoiPatrolConfig(
            lane_count=5,
            boundary_margin_m=0.5,
            max_flight_radius_m=30.0,
        )
    )

    planner.reset((28.0, 0.0), 10.0, 15.0, (0.0, 0.0, 15.0))

    assert planner.waypoints
    for x, y, altitude in planner.waypoints:
        assert math.hypot(x, y) <= 30.0 + 1e-6
        assert math.hypot(x - 28.0, y) <= 10.0 + 1e-6
        assert altitude == 15.0


def test_non_overlapping_roi_is_rejected_without_waypoints():
    planner = LawnMowerRoiPatrolPlanner(
        RoiPatrolConfig(max_flight_radius_m=30.0)
    )

    with pytest.raises(ValueError, match="does not overlap"):
        planner.reset((50.0, 0.0), 5.0, 15.0, (0.0, 0.0, 15.0))

    assert planner.waypoints == ()


def test_retarget_altitude_rewrites_z_without_changing_order_or_next_index():
    planner = LawnMowerRoiPatrolPlanner(RoiPatrolConfig(lane_count=3))
    planner.reset((0.0, 0.0), 20.0, 10.0, (-20.0, 0.0, 10.0))
    planner.next_waypoint()
    xy_before = [(x, y) for x, y, _ in planner.waypoints]
    next_index_before = planner._next_index

    planner.retarget_altitude(20.0)

    assert [(x, y) for x, y, _ in planner.waypoints] == xy_before
    assert all(z == 20.0 for _, _, z in planner.waypoints)
    assert planner._next_index == next_index_before
    next_x, next_y, next_z = planner.next_waypoint()
    assert next_z == 20.0


@pytest.mark.parametrize(
    "config",
    [
        RoiPatrolConfig(lane_count=2, boundary_margin_m=0.0),
        RoiPatrolConfig(lane_count=9, boundary_margin_m=10.0),
    ],
)
def test_valid_planner_config_is_accepted(config):
    assert LawnMowerRoiPatrolPlanner(config).config == config


@pytest.mark.parametrize(
    "kwargs",
    [
        {"lane_count": 1},
        {"boundary_margin_m": -1.0},
        {"boundary_margin_m": float("inf")},
    ],
)
def test_invalid_planner_config_is_rejected(kwargs):
    with pytest.raises(ValueError):
        RoiPatrolConfig(**kwargs)
