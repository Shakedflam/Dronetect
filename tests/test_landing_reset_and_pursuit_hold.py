"""FlightLogic: clean IDLE after a confirmed landing, and the pursuit hold.

These pin the state machine itself; the DJI bridge wiring (landing edge,
grounded setpoint suppression, hold delivery while unconfigured) is covered
in tests/test_dji_flight_bridge.py.
"""

import math

import pytest

from controllers.autonomous_drone.flight_logic import FlightConfig, FlightLogic, FlightState
from controllers.autonomous_drone.pursuit import PursuitConfig


HOME = (0.0, 0.0, 0.0)
CRUISE = 15.0
POSITION = (10.0, 20.0, CRUISE)
LOW = (10.0, 20.0, 5.0)
WAYPOINT = (100.0, 50.0, CRUISE)

TARGET_STATES = {
    FlightState.TARGET_ACQUISITION,
    FlightState.TARGET_FOLLOW,
    FlightState.TARGET_PURSUIT,
    FlightState.TARGET_RETURN,
}


def make_logic():
    # Same isolation as tests/test_target_pursuit.py: acquisition is instant
    # so a handful of frames reaches pursuit deterministically.
    logic = FlightLogic(
        FlightConfig(target_observation_s=0.0, takeoff_altitude=CRUISE,
                     target_standoff_box_area_ratio=0.01,
                     target_standoff_deadband_fraction=0.1,
                     target_lost_search_timeout_s=2.0,
                     target_reference_distance_m=2.0,
                     target_reference_area_ratio=0.01),
        pursuit_config=PursuitConfig(stable_duration_s=0.3),
        patrol_waypoint_provider=lambda position, completed: WAYPOINT,
    )
    logic.initialize_home(HOME)
    return logic


def observe(logic, position=POSITION, track_id="one"):
    size = 1000 * math.sqrt(logic._standoff_area_ratio())
    logic.observe_detections([{
        "trackId": track_id, "trackState": "confirmed", "confidence": 0.9,
        "box": [500 - size / 2, 500 - size / 2, 500 + size / 2, 500 + size / 2],
        "velocityPxPerSec": {"x": 0, "y": 0},
    }], frame_width=1000, frame_height=1000, position=position, yaw=0.0,
        camera_pitch=0.0)


def enter_pursuit(logic):
    for _ in range(6):
        observe(logic)
        logic.update(POSITION, 0.1)
    assert logic.state is FlightState.TARGET_PURSUIT


def in_state(state):
    """A FlightLogic driven, through its public API, into `state`."""
    logic = make_logic()
    if state is FlightState.TAKING_OFF:
        logic.request_patrol(LOW)
    elif state is FlightState.PATROLLING:
        logic.request_patrol(POSITION)
    elif state is FlightState.TRANSITING_TO_ROI:
        logic.request_patrol(POSITION)
        logic.request_roi(POSITION, (200.0, 200.0), 20.0)
    elif state is FlightState.TARGET_ACQUISITION:
        logic.request_patrol(POSITION)
        observe(logic)
    elif state is FlightState.TARGET_PURSUIT:
        logic.request_patrol(POSITION)
        enter_pursuit(logic)
    elif state is FlightState.TARGET_RETURN:
        logic.request_patrol(POSITION)
        enter_pursuit(logic)
        logic.complete_target_follow(POSITION)
    elif state is FlightState.RETURNING_HOME:
        logic.request_patrol(POSITION)
        logic.request_return_home()
    assert logic.state is state
    return logic


def assert_clean_idle(logic):
    assert logic.state is FlightState.IDLE
    assert logic.target == HOME
    assert not logic.is_target_tracking
    assert logic._tracked_target_id is None
    assert logic._target_resume_state is None
    assert logic._patrol_resume_target is None
    assert logic._patrol_resume_checkpoint is None
    assert not logic._target_return_waypoints
    assert logic._queued_waypoint is None and logic._queued_state is None
    assert logic._roi_center is None
    assert not logic._reposition_waypoints
    assert logic.pending_mission_state is None
    assert logic.target_yaw is None
    assert not logic.manual_control


# -- landing ------------------------------------------------------------------


@pytest.mark.parametrize("state", [
    FlightState.TAKING_OFF,
    FlightState.PATROLLING,
    FlightState.TRANSITING_TO_ROI,
    FlightState.TARGET_ACQUISITION,
    FlightState.TARGET_PURSUIT,
    FlightState.TARGET_RETURN,
    FlightState.RETURNING_HOME,
])
def test_landing_from_any_flying_state_resets_to_clean_idle(state):
    logic = in_state(state)
    logic.reset_after_landing((10.0, 20.0, 0.0))
    assert_clean_idle(logic)
    # IDLE holds still: a tick on the ground keeps it parked at home.
    logic.update((10.0, 20.0, 0.0), 0.1, yaw=0.0, camera_pitch=0.0)
    assert_clean_idle(logic)


def test_landing_preserves_configuration_home_and_pursuit_hold():
    logic = in_state(FlightState.PATROLLING)
    logic.set_pursuit_blocked(True, POSITION)
    config = logic.config
    logic.reset_after_landing((10.0, 20.0, 0.0))
    assert logic.config is config
    assert logic.home == HOME
    assert logic.pursuit_blocked is True


def test_landing_drops_a_manual_takeover():
    logic = in_state(FlightState.PATROLLING)
    assert logic.engage_manual_control(POSITION)
    logic.reset_after_landing((10.0, 20.0, 0.0))
    assert_clean_idle(logic)


@pytest.mark.parametrize("state", [FlightState.TARGET_PURSUIT, FlightState.TARGET_RETURN])
def test_new_mission_after_landing_starts_cleanly(state):
    logic = in_state(state)
    logic.reset_after_landing((10.0, 20.0, 0.0))

    # Next sortie: a patrol requested on the ground climbs first, toward the
    # configured cruise altitude, with no leftover target or resume point.
    logic.request_patrol((10.0, 20.0, 0.0))
    assert logic.state is FlightState.TAKING_OFF
    assert logic.target == (10.0, 20.0, CRUISE)
    assert logic.pending_mission_state is FlightState.PATROLLING
    logic.update((10.0, 20.0, CRUISE), 0.1, yaw=0.0, camera_pitch=0.0)
    assert logic.state is FlightState.PATROLLING
    assert logic.target == WAYPOINT

    # A previously followed track is not remembered as "already handled".
    observe(logic)
    assert logic.state is FlightState.TARGET_ACQUISITION


# -- pursuit hold -------------------------------------------------------------


def test_patrol_with_hold_ignores_detections_and_keeps_patrolling():
    logic = in_state(FlightState.PATROLLING)
    logic.set_pursuit_blocked(True, POSITION)
    for _ in range(20):
        observe(logic)
        logic.update(POSITION, 0.1, yaw=0.0, camera_pitch=0.0)
        assert logic.state is FlightState.PATROLLING
        assert logic.target == WAYPOINT
    assert not logic.is_target_tracking
    assert logic._target_resume_state is None


def test_takeoff_with_hold_ignores_detections_and_continues_to_patrol():
    logic = in_state(FlightState.TAKING_OFF)
    logic.set_pursuit_blocked(True, LOW)
    for _ in range(10):
        observe(logic, position=LOW)
        logic.update(LOW, 0.1, yaw=0.0, camera_pitch=0.0)
        assert logic.state is FlightState.TAKING_OFF
    observe(logic, position=(10.0, 20.0, CRUISE))
    logic.update((10.0, 20.0, CRUISE), 0.1, yaw=0.0, camera_pitch=0.0)
    assert logic.state is FlightState.PATROLLING
    assert logic.target == WAYPOINT


def test_hold_reports_no_interception_so_the_bridge_never_checkpoints():
    logic = in_state(FlightState.PATROLLING)
    detections = [{"trackId": "one", "trackState": "confirmed", "confidence": 0.9,
                   "box": [450, 450, 550, 550]}]
    assert logic.would_intercept(detections)
    logic.set_pursuit_blocked(True, POSITION)
    assert not logic.would_intercept(detections)


def test_hold_engaged_during_acquisition_resumes_the_mission():
    logic = in_state(FlightState.TARGET_ACQUISITION)
    assert logic.set_pursuit_blocked(True, POSITION)
    # Pursuit never started, so there is no egress leg: straight back to patrol.
    assert logic.state is FlightState.PATROLLING
    assert not logic.is_target_tracking
    observe(logic)
    logic.update(POSITION, 0.1, yaw=0.0, camera_pitch=0.0)
    assert logic.state is FlightState.PATROLLING


def test_hold_engaged_during_pursuit_returns_then_resumes_and_stays_held():
    logic = in_state(FlightState.TARGET_PURSUIT)
    assert logic.set_pursuit_blocked(True, POSITION)
    assert logic.state is FlightState.TARGET_RETURN
    checkpoint = logic.target
    # Detections during the return leg cannot restart a chase.
    observe(logic)
    assert logic.state is FlightState.TARGET_RETURN
    logic.update(checkpoint, 0.1, yaw=0.0, camera_pitch=0.0)
    assert logic.state is FlightState.PATROLLING
    for _ in range(10):
        observe(logic)
        logic.update(POSITION, 0.1, yaw=0.0, camera_pitch=0.0)
        assert logic.state is FlightState.PATROLLING


def test_hold_released_restores_normal_pursuit():
    logic = in_state(FlightState.PATROLLING)
    logic.set_pursuit_blocked(True, POSITION)
    observe(logic)
    assert logic.state is FlightState.PATROLLING
    assert logic.set_pursuit_blocked(False, POSITION)
    enter_pursuit(logic)


def test_roi_resume_without_saved_progress_holds_instead_of_crashing():
    """No valid interrupted mission (the sweep was never restored): the
    aircraft must hold, not raise out of the control loop or loop forever."""
    logic = make_logic()
    logic.request_patrol(POSITION)
    logic.request_roi(POSITION, (POSITION[0], POSITION[1]), 20.0)
    for _ in range(5):
        logic.update(logic.target, 0.1, yaw=0.0, camera_pitch=0.0)
    assert logic.state is FlightState.ROI_PATROL
    observe(logic)
    assert logic.state is FlightState.TARGET_ACQUISITION
    logic.roi_patrol_planner.clear()  # what the bridge does on capture
    for _ in range(40):  # acquisition times out with no restore
        logic.update(logic.target, 0.1, yaw=0.0, camera_pitch=0.0)
    assert logic.state is FlightState.HOLDING
    assert not logic.is_target_tracking


def test_stalled_target_return_is_abandoned_and_mission_resumes():
    logic = in_state(FlightState.TARGET_RETURN)
    far = (POSITION[0] + 30.0, POSITION[1], POSITION[2])
    ticks = round(logic.config.target_return_stall_s / 0.1) + 5
    for _ in range(ticks):
        logic.update(far, 0.1, yaw=0.0, camera_pitch=0.0)
        if logic.state is not FlightState.TARGET_RETURN:
            break
    assert logic.state is FlightState.PATROLLING
    assert not logic.is_target_tracking


def test_target_return_that_keeps_closing_is_not_abandoned():
    logic = in_state(FlightState.TARGET_RETURN)
    checkpoint = logic.target
    # 30 m out, closing at 1 m/s: slower than the watchdog window, never stalled.
    for step in range(320):
        position = (checkpoint[0] + max(0.0, 30.0 - 0.1 * step), checkpoint[1], checkpoint[2])
        logic.update(position, 0.1, yaw=0.0, camera_pitch=0.0)
        if logic.state is not FlightState.TARGET_RETURN:
            break
    # It completed by actually arriving, not by the watchdog giving up.
    assert logic.state is FlightState.PATROLLING
    assert position == checkpoint


def test_without_hold_detection_still_interrupts_patrol():
    logic = in_state(FlightState.PATROLLING)
    assert logic.pursuit_blocked is False
    observe(logic)
    assert logic.state in TARGET_STATES
