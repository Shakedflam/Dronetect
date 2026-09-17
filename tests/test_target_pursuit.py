import math
import importlib.util
from collections import deque
from pathlib import Path
import sys
from types import ModuleType
from unittest.mock import Mock

import pytest

from controllers.autonomous_drone.flight_logic import (
    FlightConfig, FlightLogic, FlightState, advance_kinematic_flight,
)
from controllers.autonomous_drone.pursuit import PursuitConfig
from controllers.autonomous_drone import flight_logic
from realtime_server.protocol import validate_telemetry_message
from services.gps import LocalPosition
from services.mission_interruption import InterruptedMissionType
from services.no_fly_router import NoFlyRouter
from services.patrol_regions import PatrolAreaType, PatrolRegionStore
from services.telemetry import build_simulator_telemetry


POSITION = (10.0, 20.0, 15.0)
WAYPOINT = (100.0, 50.0, 15.0)


def make_logic(**options):
    logic = FlightLogic(
        FlightConfig(takeoff_altitude=15, target_standoff_box_area_ratio=0.01,
                     target_standoff_deadband_ratio=0.001,
                     target_lost_search_timeout_s=2.0),
        pursuit_config=PursuitConfig(stable_duration_s=0.3, **options),
        patrol_waypoint_provider=lambda position, completed: WAYPOINT,
    )
    logic.initialize_home((0.0, 0.0, 0.0))
    logic.request_patrol(POSITION)
    return logic


def observe(logic, *, size=100, vx=0, state="confirmed", confidence=0.9,
            track_id="one", cx=500, position=POSITION, yaw=0.0,
            allow_landing_intercept=False):
    logic.observe_detections([{
        "trackId": track_id, "trackState": state, "confidence": confidence,
        "box": [cx-size/2, 500-size/2, cx+size/2, 500+size/2],
        "velocityPxPerSec": {"x": vx, "y": 0},
    }], frame_width=1000, frame_height=1000, position=position, yaw=yaw,
       allow_landing_intercept=allow_landing_intercept)


def enter(logic):
    for _ in range(6):
        observe(logic)
        logic.update(POSITION, 0.1)
    assert logic.state is FlightState.TARGET_PURSUIT


def test_entry_requires_stable_confirmed_centered_history():
    logic = make_logic()
    for _ in range(10):
        observe(logic, cx=750)
        logic.update(POSITION, 0.1)
    assert logic.state is FlightState.TARGET_FOLLOW
    assert logic.target == POSITION
    observe(logic)
    logic.update(POSITION, 0.1)
    assert logic.state is FlightState.TARGET_FOLLOW
    enter(logic)
    assert logic.patrol_resume_checkpoint == POSITION


@pytest.mark.parametrize("size,sign", [(80, 1), (120, -1)])
def test_away_and_approaching_target_motion(size, sign):
    logic = make_logic()
    enter(logic)
    observe(logic, size=size)
    logic.update(POSITION, 0.1)
    assert sign * logic.pursuit.velocity[0] > 0
    assert sign * logic.pursuit.motion[0] > 0


def test_lateral_motion_and_stationary_hover():
    logic = make_logic()
    enter(logic)
    assert logic.pursuit.velocity == (0, 0, 0)
    for _ in range(10):
        observe(logic, vx=200)
        logic.update(POSITION, 0.1)
    assert logic.pursuit.velocity[1] < 0
    assert logic.pursuit.velocity[0] == pytest.approx(0)


def test_centered_target_still_moves_laterally_when_yaw_tracks_its_motion():
    logic = make_logic()
    enter(logic)
    observe(logic, yaw=-0.02, vx=0)
    logic.update(POSITION, 0.1)
    assert logic.pursuit.velocity[1] < 0


def test_camera_rotation_alone_does_not_create_target_motion():
    logic = make_logic()
    enter(logic)
    observe(logic, yaw=-0.02, vx=-200)
    logic.update(POSITION, 0.1)
    assert logic.pursuit.velocity == (0, 0, 0)


@pytest.mark.parametrize("failure", ["missing", "lost", "confidence", "tentative", "stale"])
def test_loss_stops_actual_translation_and_reacquires_pursuit(failure):
    logic = make_logic()
    enter(logic)
    observe(logic, size=80, vx=200)
    logic.update(POSITION, 0.1)
    assert math.hypot(*logic.pursuit.velocity[:2]) > 0
    if failure == "missing":
        logic.observe_detections([], frame_width=1000, frame_height=1000, position=POSITION, yaw=0)
    elif failure == "stale":
        logic.update(POSITION, 1.6)
    else:
        observe(logic, state=failure if failure in {"lost", "tentative"} else "confirmed",
                confidence=0.1 if failure == "confidence" else 0.9)
    assert logic.pursuit.velocity == (0, 0, 0)
    assert logic.diagnostic_state == "TARGET_LOST_SEARCH"
    step = advance_kinematic_flight(position=POSITION, target=POSITION, yaw=0,
                                   horizontal_speed_mps=4, dt=0.1,
                                   velocity_command=logic.pursuit.velocity)
    assert step.position == POSITION
    assert step.horizontal_speed_mps == 0
    observe(logic, size=80)
    logic.update(POSITION, 0.1)
    assert logic.state is FlightState.TARGET_PURSUIT
    assert logic.pursuit.velocity[0] > 0


def test_collision_overrides_forward_motion_and_never_descends():
    logic = make_logic()
    enter(logic)
    observe(logic, size=60)
    logic.update(POSITION, 0.1)
    assert logic.pursuit.velocity[0] > 0
    observe(logic, size=500)
    logic.update(POSITION, 0.1)
    assert logic.pursuit.constraint == "collision"
    assert logic.pursuit.velocity[0] <= 0
    assert logic.pursuit.velocity[2] >= 0


def test_raw_collision_cue_overrides_even_when_smoothed_growth_is_small():
    logic = make_logic()
    enter(logic)
    observe(logic, size=60)
    logic.update(POSITION, 0.1)
    observe(logic, size=95)
    logic.update(POSITION, 0.1)
    assert logic._pursuit_growth < logic.config.target_approach_growth_ratio_per_s
    assert logic.pursuit.constraint == "collision"
    assert logic.pursuit.velocity[0] <= 0


def test_geofence_and_height_override_translation():
    logic = make_logic(geofence_radius_m=30)
    enter(logic)
    observe(logic, size=40)
    logic.update((30, 0, 15), 0.1)
    assert logic.pursuit.constraint == "geofence"
    assert logic.pursuit.velocity[:2] == (0, 0)
    logic.update((10, 20, 3.0), 0.1)
    assert logic.pursuit.constraint == "height_floor"
    assert logic.pursuit.velocity[:2] == (0, 0)
    assert logic.pursuit.velocity[2] > 0
    logic.update((10, 20, 3.5), 0.1)
    assert logic.pursuit.velocity == (0, 0, 0)


def test_velocity_and_vector_acceleration_limits():
    logic = make_logic(max_speed_mps=1, acceleration_mps2=0.5)
    enter(logic)
    previous = logic.pursuit.velocity
    for i in range(30):
        observe(logic, size=60, vx=1000 if i < 15 else -1000)
        logic.update(POSITION, 0.1)
        command = logic.pursuit.velocity
        assert math.dist(previous, command) <= 0.05 + 1e-9
        assert math.hypot(*command[:2]) <= 1 + 1e-9
        previous = command


@pytest.mark.parametrize("complete", [False, True])
def test_permanent_loss_or_completion_returns_to_exact_checkpoint_then_waypoint(complete):
    logic = make_logic()
    enter(logic)
    displaced = (50, 60, 15)
    if complete:
        logic.complete_target_follow()
    else:
        logic.observe_detections([], frame_width=1000, frame_height=1000, position=displaced, yaw=0)
        logic.update(displaced, 2.1)
    assert logic.state is FlightState.TARGET_RETURN
    assert logic.target == POSITION
    assert logic.patrol_resume_checkpoint == POSITION
    assert logic.pursuit.velocity == (0, 0, 0)
    logic.update(displaced, 0.1)
    assert logic.state is FlightState.TARGET_RETURN
    logic.update(POSITION, 0.1)
    assert logic.state is FlightState.PATROLLING
    assert logic.target == WAYPOINT


def test_new_tracker_requires_stability_again_and_rth_cancels_pursuit():
    logic = make_logic()
    enter(logic)
    observe(logic, track_id="two")
    logic.update(POSITION, 0.1)
    assert logic.state is FlightState.TARGET_FOLLOW
    assert logic.pursuit.velocity == (0, 0, 0)
    assert logic.request_return_home()
    assert logic.state is FlightState.RETURNING_HOME
    assert logic.patrol_resume_checkpoint is None


def test_pursuit_diagnostics_survive_realtime_validation():
    logic = make_logic()
    enter(logic)
    observe(logic, vx=200)
    logic.update(POSITION, 0.1)
    diagnostics = logic.target_follow_diagnostics()
    message = build_simulator_telemetry(position_local=LocalPosition(*POSITION),
        heading_deg=0, speed_mps=0, state=logic.state.value,
        target_follow_diagnostics=diagnostics)
    assert validate_telemetry_message(message)["data"]["targetFollowDiagnostics"] == diagnostics


def test_operator_landing_can_intercept_but_safety_landing_cannot():
    normal = make_logic()
    normal.state = FlightState.LANDING
    normal.target = (0.0, 0.0, 0.0)
    assert normal.would_intercept(
        [{"trackId": "one", "trackState": "confirmed", "confidence": 0.9,
          "box": [450, 450, 550, 550], "velocityPxPerSec": {"x": 0, "y": 0}}],
        allow_landing_intercept=True,
    )
    observe_landing = {
        "trackId": "one", "trackState": "confirmed", "confidence": 0.9,
        "box": [450, 450, 550, 550], "velocityPxPerSec": {"x": 0, "y": 0},
    }
    normal.observe_detections([observe_landing], frame_width=1000, frame_height=1000,
                              position=POSITION, yaw=0, allow_landing_intercept=True)
    assert normal.state is FlightState.TARGET_ACQUISITION
    normal.observe_detections([observe_landing], frame_width=1000, frame_height=1000,
                              position=POSITION, yaw=0, allow_landing_intercept=True)
    assert normal.state is FlightState.TARGET_FOLLOW
    assert normal._target_resume_state is FlightState.LANDING

    safety = make_logic()
    safety.state = FlightState.LANDING
    safety.target = (0.0, 0.0, 0.0)
    safety.observe_detections([observe_landing], frame_width=1000, frame_height=1000,
                              position=POSITION, yaw=0, allow_landing_intercept=False)
    assert safety.state is FlightState.LANDING


def test_low_landing_climbs_to_height_floor_before_pursuit():
    logic = make_logic()
    logic.state = FlightState.LANDING
    logic.target = (0.0, 0.0, 0.0)
    low = (10.0, 20.0, 2.0)
    for _ in range(5):
        observe(logic, position=low, allow_landing_intercept=True)
        logic.update(low, 0.1)
    assert logic.state is FlightState.TARGET_FOLLOW
    assert logic.target == (10.0, 20.0, 3.5)


@pytest.fixture
def simulator(monkeypatch):
    """Exercise the real adapter without starting Webots or network threads."""
    controller = ModuleType("controller")
    for name in ("Camera", "GPS", "InertialUnit", "Motor", "Supervisor", "Field"):
        setattr(controller, name, type(name, (), {}))
    monkeypatch.setitem(sys.modules, "controller", controller)
    monkeypatch.setitem(sys.modules, "flight_logic", flight_logic)
    directory = Path(__file__).resolve().parents[1] / "controllers" / "autonomous_drone"
    monkeypatch.syspath_prepend(str(directory))
    spec = importlib.util.spec_from_file_location("simulator_under_test", directory / "autonomous_drone.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    drone = module.AutonomousDrone.__new__(module.AutonomousDrone)
    drone.logic = make_logic()
    drone._kinematic_yaw = 0.0
    drone._horizontal_speed_mps = 0.0
    drone._connection_loss_rth_active = False
    drone._safety_rth_active = False
    drone._safety_rth_reason = None
    drone._active_roi_id = None
    drone._mission_interruption = None
    drone._mission_return_prepared = False
    drone._patrol_route_waypoints = deque()
    drone._target_patrol_region_id = None
    drone._paused_patrol_coverage = None
    drone.patrol_regions = PatrolRegionStore()
    drone.no_fly_router = NoFlyRouter()
    drone.patrol_planner = Mock()
    drone._target_tracking_flight_config = module.AutonomousDrone.SEARCH_KINEMATIC_FLIGHT_CONFIG
    drone.translation = Mock()
    drone.rotation = Mock()
    drone.self_node = Mock()
    return drone


def test_simulator_pursuit_crosses_no_fly_but_collision_still_overrides(simulator):
    from services.patrol_regions import PatrolRegionStore

    simulator.patrol_regions = PatrolRegionStore()
    for region in simulator.patrol_regions.regions:
        simulator.patrol_regions.set_area_type(region.id, "NO_FLY")
    simulator.no_fly_router = Mock()
    simulator.no_fly_router.route.side_effect = AssertionError("Pursuit must bypass NO_FLY routing")
    enter(simulator.logic)
    observe(simulator.logic, size=60)
    simulator.logic.update(POSITION, 0.1)
    moved, _, _, _ = simulator._apply_kinematic_flight(POSITION, 0, 0.1)
    assert moved[0] > POSITION[0]
    observe(simulator.logic, size=500)
    simulator.logic.update(moved, 0.1)
    avoided, _, _, _ = simulator._apply_kinematic_flight(moved, 0, 0.1)
    assert avoided[0] <= moved[0]
    simulator.no_fly_router.route.assert_not_called()


def test_simulator_reaches_checkpoint_exactly_without_waypoint_deadzone(simulator):
    enter(simulator.logic)
    simulator.logic.complete_target_follow()
    position = (POSITION[0] + 0.1, POSITION[1], POSITION[2])
    simulator.logic.update(position, 0.1)
    assert simulator.logic.state is FlightState.TARGET_RETURN
    for _ in range(10):
        position, _, _, _ = simulator._apply_kinematic_flight(position, 0, 0.1)
        simulator.logic.update(position, 0.1)
        if simulator.logic.state is FlightState.PATROLLING:
            break
    assert position == POSITION
    assert simulator.logic.state is FlightState.PATROLLING
    assert simulator.logic.target == WAYPOINT


def test_simulator_blocks_safety_landing_intercept_but_allows_normal_landing(simulator):
    detection_message = {
        "frameWidth": 1000,
        "frameHeight": 1000,
        "detections": [{
            "trackId": "one", "trackState": "confirmed", "confidence": 0.9,
            "box": [450, 450, 550, 550],
            "velocityPxPerSec": {"x": 0, "y": 0},
        }],
    }
    simulator.realtime_client = Mock()
    simulator.camera_gimbal = Mock(pitch=0.0)
    simulator._clear_patrol_progress = Mock()
    simulator.realtime_client.drain_commands.return_value = [{
        "command": "return_home",
        "params": {},
        "safetyReason": "low_battery",
    }]

    simulator._handle_realtime_commands(POSITION)

    assert simulator._safety_rth_active is True
    assert simulator._safety_rth_reason == "low_battery"
    simulator.logic.state = FlightState.LANDING
    simulator.logic.target = (0.0, 0.0, 0.0)
    simulator.realtime_client.drain_detections.return_value = [detection_message]
    simulator._handle_vision_detections(POSITION, 0.0)
    assert simulator.logic.state is FlightState.LANDING

    simulator._safety_rth_active = False
    simulator._safety_rth_reason = None
    simulator._handle_vision_detections(POSITION, 0.0)
    assert simulator.logic.state is FlightState.TARGET_ACQUISITION


def test_patrol_interruption_restores_exact_unfinished_coverage(simulator):
    active = (30.0, 40.0, 15.0)
    remaining = ((31.0, 41.0, 15.0), (32.0, 42.0, 15.0))
    interruption = (11.25, 22.5, 14.75)
    simulator.logic.state = FlightState.PATROLLING
    simulator.logic.target = active
    simulator._target_patrol_region_id = "NE-2"
    simulator._patrol_route_waypoints = deque(remaining)

    simulator._capture_mission_interruption(interruption)

    context = simulator._mission_interruption
    assert context is not None
    assert context.source_type is InterruptedMissionType.PATROL
    assert context.interruption_position == interruption
    assert context.active_id == "NE-2"
    assert context.active_waypoint == active
    assert context.remaining_path == remaining
    assert simulator._target_patrol_region_id is None
    assert not simulator._patrol_route_waypoints

    simulator.logic.target = (99.0, 99.0, 15.0)
    simulator._restore_mission_interruption(context)

    assert simulator.logic.target == active
    assert simulator._target_patrol_region_id == "NE-2"
    assert tuple(simulator._patrol_route_waypoints) == remaining
    assert simulator._next_patrol_waypoint(interruption, completed=True) == remaining[0]
    simulator.patrol_planner.record_observation.assert_not_called()


def test_roi_interruption_restores_id_active_leg_and_exact_planner_cursor(simulator):
    simulator._active_roi_id = "roi:command-7"
    simulator.logic.roi_patrol_planner.reset(
        (0.0, 0.0), 20.0, 15.0, (-20.0, 0.0, 15.0)
    )
    active = simulator.logic.roi_patrol_planner.next_waypoint()
    expected_next = simulator.logic.roi_patrol_planner.snapshot().remaining_path[0]
    simulator.logic.state = FlightState.ROI_PATROL
    simulator.logic.target = active

    simulator._capture_mission_interruption(POSITION)

    context = simulator._mission_interruption
    assert context is not None
    assert context.source_type is InterruptedMissionType.ROI
    assert context.active_id == "roi:command-7"
    assert context.active_waypoint == active
    assert not simulator.logic.roi_patrol_planner.waypoints

    simulator.logic.target = WAYPOINT
    simulator._restore_mission_interruption(context)

    assert simulator._active_roi_id == "roi:command-7"
    assert simulator.logic.target == active
    assert simulator.logic.roi_patrol_planner.next_waypoint() == expected_next


def test_target_return_from_no_fly_egresses_then_restores_enforcement(simulator):
    simulator.patrol_regions.set_area_type("N-1", PatrolAreaType.NO_FLY)
    simulator.patrol_regions.set_area_type("NE-1", PatrolAreaType.NO_FLY)
    interruption = (-100.0, -150.0, 15.0)
    simulator.logic.state = FlightState.PATROLLING
    simulator.logic.target = (30.0, 40.0, 15.0)
    simulator._target_patrol_region_id = "SW-1"
    simulator._capture_mission_interruption(interruption)
    context = simulator._mission_interruption
    assert context is not None

    inside_north_no_fly = (0.0, 60.0, 15.0)
    simulator.logic.state = FlightState.TARGET_RETURN
    simulator._prepare_mission_return(inside_north_no_fly, context)

    assert simulator._mission_return_prepared is True
    route = [simulator.logic.target, *simulator.logic._target_return_waypoints]
    assert route[-1] == interruption
    assert route[0] != inside_north_no_fly
    for left, right in zip(route, route[1:]):
        assert not simulator.no_fly_router.segment_crosses_no_fly(
            left[:2], right[:2], simulator.patrol_regions.regions
        )
    assert simulator._mission_interruption is context
