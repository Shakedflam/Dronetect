import math

import pytest

from services.gps import DEFAULT_HOME, wgs84_to_local_meters
from services.no_fly_router import NoFlyRouter, NoFlyRoutingConfig
from services.patrol_planner import PatrolPlanner
from services.patrol_regions import PatrolAreaType, PatrolRegionStore


def _path_length(start, route):
    points = [start, *route]
    return sum(math.dist(left, right) for left, right in zip(points, points[1:]))


def test_direct_route_is_unchanged_when_it_does_not_cross_no_fly():
    store = PatrolRegionStore(home_radius_m=300.0)
    store.set_area_type("N-1", PatrolAreaType.NO_FLY)
    router = NoFlyRouter(NoFlyRoutingConfig(clearance_m=5.0))
    start = (-200.0, -100.0)
    destination = (200.0, -100.0)

    assert router.route(start, destination, store.regions) == [destination]


def test_home_can_depart_when_first_band_contains_no_fly_area():
    store = PatrolRegionStore(home_radius_m=300.0)
    store.set_area_type("N-1", PatrolAreaType.NO_FLY)
    planner = PatrolPlanner(store=store, started_at=0.0)
    decision = planner.choose_next(DEFAULT_HOME, now=100.0)
    local_target = wgs84_to_local_meters(decision.region.center)
    router = NoFlyRouter(NoFlyRoutingConfig(clearance_m=5.0))

    route = router.route(
        (0.0, 0.0),
        (local_target.x, local_target.y),
        store.regions,
    )

    assert route[-1] == pytest.approx((local_target.x, local_target.y))
    assert decision.region.area_type is PatrolAreaType.PATROL


def test_crossing_route_uses_safe_boundary_waypoints():
    store = PatrolRegionStore(home_radius_m=300.0)
    store.set_area_type("N-1", PatrolAreaType.NO_FLY)
    router = NoFlyRouter(NoFlyRoutingConfig(clearance_m=5.0))
    start = (-200.0, 100.0)
    destination = (200.0, 100.0)

    route = router.route(start, destination, store.regions)

    assert len(route) > 1
    assert route[-1] == destination
    assert _path_length(start, route) > math.dist(start, destination)
    for left, right in zip([start, *route], route):
        assert not router.segment_crosses_no_fly(left, right, store.regions)


def test_router_chooses_shorter_side_of_asymmetric_no_fly_crossing():
    store = PatrolRegionStore(home_radius_m=300.0)
    store.set_area_type("N-1", PatrolAreaType.NO_FLY)
    router = NoFlyRouter(NoFlyRoutingConfig(clearance_m=5.0))
    start = (-180.0, 80.0)
    destination = (80.0, 180.0)

    route = router.route(start, destination, store.regions)

    # The west boundary is much closer to the start than routing around the
    # east side, so the first detour remains on the west side.
    assert route[0][0] < 0.0
    for left, right in zip([start, *route], route):
        assert not router.segment_crosses_no_fly(left, right, store.regions)


def test_egress_route_leaves_containing_zone_then_enforces_all_no_fly():
    store = PatrolRegionStore(home_radius_m=300.0)
    store.set_area_type("N-1", PatrolAreaType.NO_FLY)
    store.set_area_type("NE-1", PatrolAreaType.NO_FLY)
    router = NoFlyRouter(NoFlyRoutingConfig(clearance_m=5.0, collision_sample_m=1.0))
    start = (0.0, 60.0)
    destination = (-100.0, -150.0)

    route = router.egress_route(start, destination, store.regions)

    assert route[-1] == destination
    assert len(route) >= 2
    # The first leg is the sole egress exception. Normal enforcement applies
    # from its endpoint onward, including the adjacent NE restricted cell.
    for left, right in zip(route, route[1:]):
        assert not router.segment_crosses_no_fly(left, right, store.regions)


def test_egress_route_uses_normal_router_when_start_is_outside_no_fly():
    store = PatrolRegionStore(home_radius_m=300.0)
    store.set_area_type("N-1", PatrolAreaType.NO_FLY)
    router = NoFlyRouter(NoFlyRoutingConfig(clearance_m=5.0))
    start = (-200.0, -100.0)
    destination = (200.0, -100.0)

    assert router.egress_route(start, destination, store.regions) == router.route(
        start, destination, store.regions
    )


def test_nearest_exit_is_none_when_start_is_already_outside_no_fly():
    store = PatrolRegionStore(home_radius_m=300.0)
    store.set_area_type("N-1", PatrolAreaType.NO_FLY)
    router = NoFlyRouter(NoFlyRoutingConfig(clearance_m=5.0))

    assert router.nearest_exit((-200.0, -100.0), store.regions) is None
    assert router.nearest_exit((0.0, 60.0), []) is None


def test_nearest_exit_leaves_the_containing_cell_and_none_other():
    store = PatrolRegionStore(home_radius_m=300.0)
    store.set_area_type("N-1", PatrolAreaType.NO_FLY)
    store.set_area_type("NE-1", PatrolAreaType.NO_FLY)
    router = NoFlyRouter(NoFlyRoutingConfig(clearance_m=5.0, collision_sample_m=1.0))
    start = (0.0, 60.0)

    exit_point = router.nearest_exit(start, store.regions)

    assert exit_point is not None
    assert exit_point != start
    # The exit point itself must be clear of every NO_FLY cell, not just the
    # one it started inside -- a degenerate zero-length segment is flagged as
    # crossing iff its single point lies inside some obstacle.
    assert not router.segment_crosses_no_fly(exit_point, exit_point, store.regions)


def test_nearest_exit_then_route_reaches_destination_without_crossing_no_fly():
    # nearest_exit() picks the closest way out, not the one that minimizes
    # total distance to a specific destination the way egress_route() does,
    # so its combined path with route() need not match egress_route()
    # exactly -- but it must still be a complete, legal route.
    store = PatrolRegionStore(home_radius_m=300.0)
    store.set_area_type("N-1", PatrolAreaType.NO_FLY)
    router = NoFlyRouter(NoFlyRoutingConfig(clearance_m=5.0, collision_sample_m=1.0))
    start = (0.0, 60.0)
    destination = (-100.0, -150.0)

    exit_point = router.nearest_exit(start, store.regions)
    onward = router.route(exit_point, destination, store.regions)
    path = [exit_point, *onward]

    assert path[-1] == destination
    # Like egress_route(), only the initial start -> exit_point leg is
    # exempt: it necessarily starts inside the cell it is escaping.
    for left, right in zip(path, path[1:]):
        assert not router.segment_crosses_no_fly(left, right, store.regions)


def test_route_goes_around_a_full_depth_multi_direction_no_fly_span():
    # Every band of two adjacent directions restricted end-to-end (hub to the
    # operational boundary) is a realistic operator action (blocking off a
    # whole compass direction), not a contrived edge case. The only way
    # around such a span is to change bearing first, outside where it
    # reaches, then approach the destination -- there is no valid radius at
    # a bearing the span fully occupies.
    store = PatrolRegionStore(home_radius_m=300.0)
    for direction in ("S", "SE"):
        for band in range(1, 5):
            store.set_area_type(f"{direction}-{band}", PatrolAreaType.NO_FLY)
    router = NoFlyRouter(NoFlyRoutingConfig(clearance_m=10.0))
    # Start on the NE side of the restricted span; a region just past the
    # span's far (SW) edge, reachable only by going the long way around.
    start = (150.0, 150.0)
    sw_2 = store.get("SW-2")
    destination = wgs84_to_local_meters(sw_2.center)
    destination = (destination.x, destination.y)

    route = router.route(start, destination, store.regions)

    assert route[-1] == destination
    for left, right in zip([start, *route], route):
        assert not router.segment_crosses_no_fly(left, right, store.regions)
    # A safe route must exist, but a needlessly wide one -- e.g. always
    # swinging out to the operational boundary even when a much closer
    # detour around the obstacle's actual edge would do -- is its own bug:
    # the shortest safe path should stay within shouting distance of the
    # unobstructed straight-line distance.
    direct_distance = math.dist(start, destination)
    assert _path_length(start, route) < direct_distance * 1.2


@pytest.mark.parametrize(
    "kwargs",
    [
        {"clearance_m": 0.0},
        {"arc_step_deg": math.inf},
        {"collision_sample_m": -1.0},
    ],
)
def test_invalid_routing_config_is_rejected(kwargs):
    with pytest.raises(ValueError):
        NoFlyRoutingConfig(**kwargs)
