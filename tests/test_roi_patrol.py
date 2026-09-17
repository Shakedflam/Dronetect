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
