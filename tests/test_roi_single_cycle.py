"""Real-demo ROI behaviour: exactly one lawn-mower cycle, then hold station.

The continuous default is the simulator/general behaviour and must stay intact,
so every test here pins both sides of the roi_single_cycle switch.
"""
import random

from controllers.autonomous_drone.flight_logic import (
    FlightConfig,
    FlightLogic,
    FlightState,
)
from controllers.autonomous_drone.roi_patrol import LawnMowerRoiPatrolPlanner

HOME = (0.0, 0.0, 0.1)
ROI_CENTER = (40.0, 0.0)
ROI_RADIUS = 15.0


def _logic(*, single_cycle: bool):
    config = FlightConfig(takeoff_altitude=12.0, roi_single_cycle=single_cycle)
    logic = FlightLogic(
        config=config,
        roi_patrol_planner=LawnMowerRoiPatrolPlanner(),
        rng=random.Random(7),
    )
    logic.initialize_home(HOME)
    return logic


def _airborne_at(logic, xy):
    return (xy[0], xy[1], logic.config.takeoff_altitude)


def _fly_one_cycle(logic, *, max_steps=400):
    """Drive the aircraft onto each target until the state stops being ROI_PATROL."""
    visited = []
    for _ in range(max_steps):
        if logic.state is not FlightState.ROI_PATROL:
            break
        target = logic.target
        assert target is not None
        visited.append(target)
        logic.update(target, 0.1)  # teleport onto the target: it is "reached"
    return visited


def test_planner_reports_cycle_complete_after_every_waypoint_once():
    planner = LawnMowerRoiPatrolPlanner()
    planner.reset(ROI_CENTER, ROI_RADIUS, 12.0, HOME)
    total = len(planner.waypoints)
    assert total > 1
    assert not planner.cycle_complete  # reset() delivered only the first
    for _ in range(total - 1):
        planner.next_waypoint()
    assert planner.cycle_complete


def test_planner_cycle_state_clears_with_the_route():
    planner = LawnMowerRoiPatrolPlanner()
    planner.reset(ROI_CENTER, ROI_RADIUS, 12.0, HOME)
    for _ in range(len(planner.waypoints)):
        planner.next_waypoint()
    assert planner.cycle_complete
    planner.clear()
    assert not planner.cycle_complete
    # A fresh ROI starts a fresh cycle.
    planner.reset(ROI_CENTER, ROI_RADIUS, 12.0, HOME)
    assert not planner.cycle_complete


def test_single_cycle_holds_station_after_one_full_coverage():
    logic = _logic(single_cycle=True)
    logic.request_roi(_airborne_at(logic, HOME), ROI_CENTER, ROI_RADIUS)
    logic.update(_airborne_at(logic, ROI_CENTER), 0.1)  # arrive, enter ROI_PATROL
    assert logic.state is FlightState.ROI_PATROL

    visited = _fly_one_cycle(logic)

    assert logic.state is FlightState.HOLDING
    # Holding its own position is what produces a zero velocity command.
    assert logic.target == visited[-1]
    assert len(visited) == len(logic.roi_patrol_planner.waypoints)


def test_single_cycle_reports_completion_exactly_once():
    logic = _logic(single_cycle=True)
    logic.request_roi(_airborne_at(logic, HOME), ROI_CENTER, ROI_RADIUS)
    logic.update(_airborne_at(logic, ROI_CENTER), 0.1)
    assert not logic.consume_roi_coverage_complete()

    _fly_one_cycle(logic)

    assert logic.consume_roi_coverage_complete()
    assert not logic.consume_roi_coverage_complete()


def test_holding_after_coverage_still_allows_rth_and_manual():
    logic = _logic(single_cycle=True)
    logic.request_roi(_airborne_at(logic, HOME), ROI_CENTER, ROI_RADIUS)
    logic.update(_airborne_at(logic, ROI_CENTER), 0.1)
    _fly_one_cycle(logic)
    assert logic.state is FlightState.HOLDING

    assert logic.request_return_home() is True
    assert logic.state is FlightState.RETURNING_HOME


def test_continuous_default_still_wraps_forever():
    logic = _logic(single_cycle=False)
    logic.request_roi(_airborne_at(logic, HOME), ROI_CENTER, ROI_RADIUS)
    logic.update(_airborne_at(logic, ROI_CENTER), 0.1)
    assert logic.state is FlightState.ROI_PATROL

    total = len(logic.roi_patrol_planner.waypoints)
    for _ in range(total * 2 + 3):
        target = logic.target
        assert target is not None
        logic.update(target, 0.1)

    # Still patrolling well past one full cycle: simulator behaviour unchanged.
    assert logic.state is FlightState.ROI_PATROL
    assert not logic.consume_roi_coverage_complete()
