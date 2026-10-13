"""Patrol started mid-air (e.g. from an ROI) on the real DJI bridge.

Field report: switching ROI -> Patrol flew away from where the operator
expected. Region choice was right (rank 1), but every region's sweep was
entered at a fixed end chosen relative to home, never the aircraft. These pin
the fix -- the sweep is entered at the end nearest the aircraft -- and the
surrounding contract: rank-1 selection, waypoint geometry against the
operating radius and the tablet's grid, no stale ROI state, and what a tablet
reconnect re-asserts.
"""

import asyncio
import json
import math

import pytest
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

from adapters.dji.flight_bridge import BridgeConfig
from config.shared_config import OperatingConfig
from controllers.autonomous_drone.flight_logic import FlightState
from realtime_server.protocol import encode_message, operating_config_result
from realtime_server.server import RealtimeServer
from services.commands import CommandName
from services.gps import LocalPosition, local_meters_to_wgs84, wgs84_to_local_meters
from services.patrol_regions import generate_patrol_regions, patrol_region_polygon_points
from services.polygon_coverage import PolygonCoverageConfig, PolygonCoveragePlanner
from tests.test_dji_flight_bridge import _feed_telemetry, _make_bridge, _queue_command

RADIUS_M = 150.0
CRUISE_M = 30.0
FIELD_CONFIG = OperatingConfig(max_flight_radius_m=RADIUS_M, cruise_altitude_m=CRUISE_M)


class Field:
    """A configured bridge hovering at `hover` (local ENU metres from home)."""

    def __init__(self, hover: tuple[float, float]):
        self.now = [0.0]
        self.bridge = _make_bridge(self.now, operating_config=FIELD_CONFIG)
        _feed_telemetry(self.bridge, self.now, altitude_m=0.0)  # grounded fix -> session home
        self.bridge.tick(0.05, now=self.now[0])
        self.hover = hover

    def tick(self):
        self.now[0] += 0.05
        gps = local_meters_to_wgs84(LocalPosition(x=self.hover[0], y=self.hover[1], z=CRUISE_M))
        _feed_telemetry(self.bridge, self.now, is_flying=True, altitude_m=CRUISE_M,
                        flight_mode="VIRTUAL_STICK", lat=gps.lat, lng=gps.lng)
        return self.bridge.tick(0.05, now=self.now[0])

    def command(self, name: str, params: dict | None = None, command_id: str = "c"):
        _queue_command(self.bridge, name, params, command_id=command_id)
        return self.tick()

    def roi_here(self):
        center = local_meters_to_wgs84(LocalPosition(x=self.hover[0], y=self.hover[1], z=0.0))
        self.command(CommandName.SET_ROI.value,
                     {"center": {"lat": center.lat, "lon": center.lng}, "radiusM": 20.0}, "roi")
        assert self.bridge.logic.state in {FlightState.TRANSITING_TO_ROI, FlightState.ROI_PATROL}

    def rank(self, *region_ids: str):
        # A field bridge has been up for minutes; ranks dominate after ~15 s.
        self.bridge.patrol_planner.started_at -= 3600
        self.command(CommandName.SET_PATROL_PRIORITIES.value,
                     {"priorities": [{"regionId": r, "rank": i + 1} for i, r in enumerate(region_ids)]},
                     "prio")

    def patrol(self):
        self.command(CommandName.START_PATROL.value, command_id="patrol")
        assert self.bridge.logic.state is FlightState.PATROLLING

    @property
    def route(self) -> list[tuple[float, float, float]]:
        return [self.bridge.logic.target, *self.bridge._patrol_route_waypoints]


def _compass(dx: float, dy: float) -> float:
    return math.degrees(math.atan2(dx, dy)) % 360.0


def _region(bridge, region_id):
    return bridge.patrol_regions.get(region_id)


# -- the field bug ----------------------------------------------------------------


@pytest.mark.parametrize("hover, ranked", [
    ((30.0, 90.0), "N-3"),    # ROI right of the N axis: enter N-3 from its east end
    ((-30.0, 90.0), "N-3"),   # ...and from its west end when left of it
    ((30.0, 90.0), "N-4"),
    ((80.0, 50.0), "NE-3"),
    ((80.0, 50.0), "E-2"),
    ((-60.0, 70.0), "NW-4"),
])
def test_patrol_from_an_off_center_roi_enters_the_ranked_region_at_its_nearest_end(hover, ranked, capsys):
    field = Field(hover)
    field.tick()
    field.roi_here()
    field.rank(ranked)
    field.patrol()

    assert field.bridge._target_patrol_region_id == ranked
    route = field.route
    first, last = route[0], route[-1]
    leg = math.dist(hover, first[:2])
    assert leg <= math.dist(hover, last[:2]) + 1e-6, "entered at the far end of the sweep"
    # The ranked region's own sweep, entered from either end, is never nearer.
    sweep_ends = [route[0], route[-1]]
    assert leg == pytest.approx(min(math.dist(hover, end[:2]) for end in sweep_ends))
    out = capsys.readouterr().out
    assert f"patrol_plan source=new_coverage region={ranked} rank=1" in out


def test_the_reported_case_no_longer_turns_away_from_the_aircraft_side():
    """ROI 30 m east of the N axis, N-3 ranked: before, the first leg went to
    N-3's west end (bearing ~290); now it goes to its east end."""
    field = Field((30.0, 90.0))
    field.tick()
    field.roi_here()
    field.rank("N-3")
    field.patrol()
    first = field.route[0]
    assert first[0] > 0.0
    assert math.dist((30.0, 90.0), first[:2]) < 30.0


def test_rank_1_wins_even_on_the_far_side_of_home():
    field = Field((0.0, 90.0))
    field.tick()
    field.roi_here()
    field.rank("S-3", "N-2")
    field.patrol()
    assert field.bridge._target_patrol_region_id == "S-3"
    first = field.route[0]
    assert 90.0 < _compass(first[0], first[1]) < 270.0  # south of home


def test_without_ranks_the_shortest_transit_wins():
    """Equal ages (a fresh bridge, nothing ranked): travel decides, measured to
    each region's nearest sweep end -- here N-1's outer end, ~15 m away, beats
    N-2's ~36 m sideways leg even though the aircraft is inside N-2."""
    field = Field((0.0, 90.0))
    field.tick()
    field.roi_here()
    field.patrol()
    assert field.bridge._target_patrol_region_id == "N-1"
    assert math.dist((0.0, 90.0), field.route[0][:2]) < 20.0


# -- waypoint geometry ----------------------------------------------------------------


@pytest.mark.parametrize("ranked", ["N-1", "N-4", "NE-2", "E-3", "SE-4", "S-1", "SW-2", "W-3", "NW-4"])
def test_every_patrol_waypoint_lies_in_the_ranked_cell_of_the_configured_radius(ranked):
    field = Field((40.0, -20.0))
    field.tick()
    field.roi_here()
    field.rank(ranked)
    field.patrol()
    region = _region(field.bridge, ranked)
    assert max(r.outer_radius_m for r in field.bridge.patrol_regions.regions) == pytest.approx(RADIUS_M)
    for x, y, z in field.route:
        distance = math.hypot(x, y)
        assert z == pytest.approx(CRUISE_M)
        assert region.inner_radius_m - 1.0 <= distance <= region.outer_radius_m + 0.5
        offset = (_compass(x, y) - region.direction_deg + 540.0) % 360.0 - 180.0
        if distance > 1.0:  # bearing is undefined at the band-1 apex
            assert abs(offset) <= 22.5 + 1.0


def test_bridge_regions_match_the_tablet_grid_for_the_confirmed_radius():
    """The tablet draws generatePatrolRegions(sessionHome, confirmedRadius)
    (patrolGeometry.ts), the same formula as generate_patrol_regions."""
    field = Field((0.0, 50.0))
    expected = {r.id: r for r in generate_patrol_regions(field.bridge.session_home, RADIUS_M)}
    for region in field.bridge.patrol_regions.regions:
        twin = expected[region.id]
        assert (region.inner_radius_m, region.outer_radius_m) == pytest.approx(
            (twin.inner_radius_m, twin.outer_radius_m))
        assert (region.start_bearing_deg, region.end_bearing_deg) == (twin.start_bearing_deg, twin.end_bearing_deg)
        assert region.center == twin.center


# -- ROI -> Patrol ------------------------------------------------------------------


def test_switching_roi_to_patrol_leaves_no_roi_state_behind():
    field = Field((60.0, 60.0))
    field.tick()
    field.roi_here()
    for _ in range(5):
        field.tick()
    field.rank("W-3")
    field.patrol()
    logic = field.bridge.logic
    assert logic._roi_center is None and logic._roi_radius == 0.0
    assert logic.roi_patrol_planner.snapshot() is None or not logic.roi_patrol_planner.contains((60.0, 60.0, 0.0))
    assert logic._queued_state is None and logic._queued_waypoint is None
    assert not logic.is_target_tracking and logic.target_yaw is None
    assert field.bridge._active_roi_id is None
    assert field.bridge._mission_interruption is None
    assert field.bridge._paused_patrol_coverage is None
    region = _region(field.bridge, "W-3")
    assert field.bridge._target_patrol_region_id == "W-3"
    assert math.hypot(*logic.target[:2]) >= region.inner_radius_m - 1.0
    # And it keeps patrolling: a few ticks later nothing has reverted to ROI.
    for _ in range(20):
        field.tick()
    assert logic.state is FlightState.PATROLLING


# -- tablet reconnect -----------------------------------------------------------------


def test_reasserting_the_same_ranks_after_a_reconnect_keeps_the_active_sweep():
    """realtime_server re-sends priorities to the bridge on every reconnect."""
    field = Field((30.0, 90.0))
    field.tick()
    field.roi_here()
    field.rank("N-3")
    field.patrol()
    before = (field.bridge._target_patrol_region_id, field.route)
    field.command(CommandName.SET_PATROL_PRIORITIES.value,
                  {"priorities": [{"regionId": "N-3", "rank": 1}]}, "reassert")
    assert field.bridge._target_patrol_region_id == before[0]
    assert field.route[-1] == pytest.approx(before[1][-1])


def test_a_reconnecting_tablet_receives_the_confirmed_radius_and_ranks():
    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            base_url = f"ws://127.0.0.1:{server.sockets[0].getsockname()[1]}/ws"
            async with connect(f"{base_url}?role=drone") as drone:
                async with connect(f"{base_url}?role=tablet") as tablet:
                    await asyncio.wait_for(tablet.recv(), 1.0)
                    await tablet.send(encode_message({
                        "type": "command", "commandId": "cfg", "command": "set_operating_config",
                        "params": {"maxFlightRadiusM": RADIUS_M, "cruiseAltitudeM": CRUISE_M}}))
                    forwarded = json.loads(await asyncio.wait_for(drone.recv(), 1.0))
                    await drone.send(encode_message(operating_config_result(forwarded["commandId"], accepted=True)))
                    await asyncio.wait_for(tablet.recv(), 1.0)  # ack
                    await asyncio.wait_for(tablet.recv(), 1.0)  # status
                    await tablet.send(encode_message({
                        "type": "command", "commandId": "prio", "command": "set_patrol_priorities",
                        "params": {"priorities": [{"regionId": "N-3", "rank": 1}]}}))
                    await asyncio.wait_for(drone.recv(), 1.0)
                # Tablet app reloaded: a brand-new connection.
                async with connect(f"{base_url}?role=tablet") as tablet:
                    status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
            assert status["type"] == "mission_status"
            assert status["operatingConfig"]["maxFlightRadiusM"] == RADIUS_M
            assert status["patrol"]["priorities"] == [{"regionId": "N-3", "rank": 1}]
            assert max(r.outer_radius_m for r in realtime_server.patrol_regions.regions) == RADIUS_M

    asyncio.run(scenario())


# -- coverage planner -----------------------------------------------------------------


def _segments(path):
    return {frozenset((a, b)) for a, b in zip(path, path[1:])}


@pytest.mark.parametrize("spacing", [75.0, 20.0])
def test_start_from_only_reorients_the_same_sweep(spacing):
    planner = PolygonCoveragePlanner(PolygonCoverageConfig(lane_spacing_m=spacing))
    home = local_meters_to_wgs84(LocalPosition(0.0, 0.0, 0.0))
    for region in generate_patrol_regions(home, RADIUS_M)[::5]:
        polygon = patrol_region_polygon_points(region, home)
        default = planner.plan(polygon)

        def local(path):
            return [tuple(round(v, 3) for v in (lambda p: (p.x, p.y))(wgs84_to_local_meters(w, home)))
                    for w in path]

        for probe in [(-200.0, 0.0), (0.0, 200.0), (200.0, -50.0), (0.0, 0.0)]:
            start = local_meters_to_wgs84(LocalPosition(probe[0], probe[1], 0.0))
            path = planner.plan(polygon, start_from=start)
            assert len(path) == len(default)
            ends = [local(default)[0], local(default)[-1], local(path)[0]]
            assert math.dist(probe, local(path)[0]) <= min(math.dist(probe, e) for e in ends[:2]) + 1e-3
            if spacing == 75.0:  # one or two lanes: identical ground, possibly reversed
                assert _segments(local(path)) == _segments(local(default)) or len(path) > 2


def test_start_from_absent_keeps_the_previous_plan_exactly():
    planner = PolygonCoveragePlanner()
    home = local_meters_to_wgs84(LocalPosition(0.0, 0.0, 0.0))
    region = generate_patrol_regions(home, 300.0)[3]
    polygon = patrol_region_polygon_points(region, home)
    assert planner.plan(polygon, outward_from=home) == planner.plan(polygon, outward_from=home)
    assert planner.plan(polygon) == planner.plan(polygon, start_from=None)


def test_bridge_lane_spacing_follows_the_same_env_override_as_the_simulator(monkeypatch):
    assert BridgeConfig().patrol_lane_spacing_m == 75.0
    monkeypatch.setenv("DRONTECT_PATROL_LANE_SPACING_M", "30")
    assert BridgeConfig().patrol_lane_spacing_m == 30.0
