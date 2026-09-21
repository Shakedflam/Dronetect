import math
import asyncio
import json
from unittest.mock import AsyncMock

import pytest

from controllers.autonomous_drone.flight_logic import FlightConfig, FlightLogic, FlightState
from controllers.autonomous_drone.target_decisions import ground_projection
from realtime_server.protocol import ProtocolError, validate_command_message, _validate_target_follow_diagnostics
from realtime_server.server import RealtimeServer


POSITION = (10.0, 20.0, 15.0)
WAYPOINT = (100.0, 50.0, 15.0)


def make_logic(**settings):
    logic = FlightLogic(FlightConfig(takeoff_altitude=15, **settings),
                        patrol_waypoint_provider=lambda *args: WAYPOINT)
    logic.initialize_home((0, 0, 0))
    logic.request_patrol(POSITION)
    return logic


def box(logic, *, key="one", confidence=0.5, cx=500, stamp=None, state="confirmed"):
    return {"trackId": key, "trackState": state, "confidence": confidence,
            "box": [cx - 30, 470, cx + 30, 530],
            "lastSeenTimestamp": logic._elapsed_time if stamp is None else stamp,
            "velocityPxPerSec": {"x": 0, "y": 0}}


def observe(logic, items=None, *, position=POSITION, yaw=0, pitch=0.5):
    logic.observe_detections(items if items is not None else [box(logic)],
                            frame_width=1000, frame_height=1000,
                            position=position, yaw=yaw, camera_pitch=pitch)


def tick(logic, dt, *, position=POSITION, yaw=0, pitch=0.5):
    logic.update(position, dt, yaw=yaw, camera_pitch=pitch)


@pytest.mark.parametrize("scores", [(0.5, 0.5, 0.5), (0.5, 0.95, 0.85), (0.9, 0.3, 0.5)])
def test_stationary_weak_or_unstable_confidence_rejected_after_full_window(scores):
    logic = make_logic()
    observe(logic, [box(logic, confidence=scores[0])])
    tick(logic, 0.425)
    observe(logic, [box(logic, confidence=scores[1])])
    # Follow starts immediately once confirmed (a low-confidence sample here
    # may be below target_min_confidence and only count as weak evidence for
    # the parallel false-positive window, not as a confirmation hit).
    tick(logic, 0.425)
    observe(logic, [box(logic, confidence=scores[2])])
    if logic.state is FlightState.TARGET_RETURN:
        # Pursuit had already started (approach begins immediately once
        # usable/safe); the checkpoint return resolves on the next tick.
        tick(logic, 0)
    assert logic.state is FlightState.PATROLLING
    assert logic.target == WAYPOINT
    assert logic.target_decisions.ignored[0].point is not None
    assert logic.target_decision_diagnostics()["lastRejectionReason"] == "stationary_with_weak_confidence"


@pytest.mark.parametrize("change", ["high_confidence", "box_motion", "translation", "yaw", "gimbal", "yaw_out_and_back"])
def test_valid_or_inconclusive_candidate_continues_existing_follow(change):
    logic = make_logic()
    score = 0.9 if change == "high_confidence" else 0.5
    observe(logic, [box(logic, confidence=score)])
    position = (11, 20, 15) if change == "translation" else POSITION
    yaw = 0.1 if change == "yaw" else 0
    pitch = 0.6 if change == "gimbal" else 0.5
    if change == "yaw_out_and_back":
        tick(logic, 0.2, yaw=0.1)
        tick(logic, 0.225, yaw=0)
    else:
        tick(logic, 0.425, position=position, yaw=yaw, pitch=pitch)
    observe(logic, [box(logic, confidence=score)], position=position, yaw=yaw, pitch=pitch)
    tick(logic, 0.425, position=position, yaw=yaw, pitch=pitch)
    observe(logic, [box(logic, confidence=score, cx=550 if change == "box_motion" else 500)],
            position=position, yaw=yaw, pitch=pitch)
    # Approach begins immediately once usable/safe, so pursuit may already
    # have been promoted; the point is that nothing got rejected.
    assert logic.state in {FlightState.TARGET_FOLLOW, FlightState.TARGET_PURSUIT}
    assert not logic.target_decisions.ignored


def test_yaw_wraparound_counts_shortest_angle():
    logic = make_logic()
    observe(logic, yaw=math.radians(179.8))
    tick(logic, 0.425, yaw=math.radians(-179.8))
    observe(logic, yaw=math.radians(-179.8))
    tick(logic, 0.425, yaw=math.radians(-179.8))
    observe(logic, yaw=math.radians(-179.8))
    if logic.state is FlightState.TARGET_RETURN:
        tick(logic, 0)
    assert logic.state is FlightState.PATROLLING


def test_duplicate_controller_ticks_do_not_fabricate_evaluation_evidence():
    logic = make_logic()
    observe(logic, [box(logic, stamp=0)])
    for i in range(4):
        tick(logic, 0.5)
        observe(logic, [box(logic, stamp=0)])
        if i == 0:
            # Two confirmed hits start follow immediately, even though the
            # underlying detection frame (same timestamp) never changed.
            assert logic.state is FlightState.TARGET_FOLLOW
    # Repeated duplicate-timestamped ticks add no new evaluation evidence...
    assert len(logic.target_decisions.samples) == 1
    # ...so even once the observation window has long elapsed, there is
    # never enough evidence to reject.
    assert not logic.target_decisions.ignored
    # A genuinely fresh detection is still tracked normally afterward.
    observe(logic, [box(logic)])
    assert logic.state in {FlightState.TARGET_FOLLOW, FlightState.TARGET_PURSUIT}


def test_lost_predictions_time_out_without_rejection():
    logic = make_logic()
    observe(logic)
    for _ in range(6):
        observe(logic, [box(logic, stamp=0, state="lost")])
        tick(logic, 0.5)
    assert logic.state is FlightState.PATROLLING
    assert not logic.target_decisions.ignored


def test_confirmation_alone_starts_follow_without_bypassing_pursuit_entry():
    """No operator step gates the follow, and none skips the pursuit gate either."""
    logic = make_logic()
    observe(logic, [box(logic, state="tentative")])
    assert logic.state is FlightState.TARGET_ACQUISITION
    tick(logic, 0.1)
    observe(logic, [box(logic, state="tentative")])
    # A tentative track is evidence, not a confirmation: still no follow.
    assert logic.state is FlightState.TARGET_ACQUISITION
    tick(logic, 0.1)
    observe(logic, [box(logic, confidence=0.9)])
    assert logic.state is FlightState.TARGET_FOLLOW
    assert logic.pursuit.velocity == (0, 0, 0)


@pytest.mark.parametrize("mode", ["acquisition", "follow", "pursuit"])
def test_manual_rejection_uses_existing_resume_path_and_expires(mode):
    logic = make_logic()
    observe(logic, [box(logic, confidence=0.9)])
    if mode != "acquisition":
        tick(logic, 0.1)
        observe(logic, [box(logic, confidence=0.9)])
        assert logic.state is FlightState.TARGET_FOLLOW
    if mode == "pursuit":
        tick(logic, logic.pursuit.config.stable_duration_s)
        assert logic.state is FlightState.TARGET_PURSUIT
    assert logic.reject_target("one")
    expiry = logic.target_decisions.ignored[0].expires_at
    assert not logic.reject_target("one")
    assert logic.target_decisions.ignored[0].expires_at == expiry
    if mode == "pursuit":
        assert logic.state is FlightState.TARGET_RETURN
        assert logic.target == POSITION
        tick(logic, 0)
    assert logic.state is FlightState.PATROLLING
    assert logic.target == WAYPOINT
    observe(logic)
    assert logic.state is FlightState.PATROLLING
    tick(logic, logic.config.target_ignore_timeout_s)
    observe(logic)
    assert logic.state is FlightState.TARGET_ACQUISITION


@pytest.mark.parametrize("delay,allowed", [(1.69, True), (1.7, True), (1.71, False)])
def test_reject_grace_after_box_disappears(delay, allowed):
    logic = make_logic()
    observe(logic)
    observe(logic, [])
    tick(logic, delay)
    assert logic.reject_target("one") is allowed


def test_visible_candidate_keeps_reject_available_then_grace_does_not_refresh():
    logic = make_logic()
    observe(logic, [box(logic, confidence=0.9)])
    for _ in range(8):
        tick(logic, 0.5)
        observe(logic, [box(logic, confidence=0.9)])
        assert logic.target_decision_diagnostics()["canReject"]
    observe(logic, [])
    for _ in range(5):
        tick(logic, 0.4)
        observe(logic, [])
    assert not logic.reject_target("one")


@pytest.mark.parametrize("start_state", ["acquisition", "follow"])
def test_reject_succeeds_throughout_the_full_grace_window_and_restores_mission(start_state):
    logic = make_logic()
    observe(logic, [box(logic, confidence=0.9)])
    if start_state == "follow":
        tick(logic, 0.1)
        observe(logic, [box(logic, confidence=0.9)])
        assert logic.state is FlightState.TARGET_FOLLOW
    observe(logic, [])
    tick(logic, 1.69)
    assert logic.reject_target("one")
    if logic.state is FlightState.TARGET_RETURN:
        tick(logic, 0)
    assert logic.state is FlightState.PATROLLING
    assert logic.target == WAYPOINT
    assert logic.target_decisions.ignored[0].track_id == "one"


def test_target_reappearing_during_grace_window_resumes_tracking_without_delay():
    logic = make_logic()
    observe(logic, [box(logic, confidence=0.9, state="tentative")])
    observe(logic, [])
    assert logic._target_visible is False
    tick(logic, 1.0)
    # The box comes back before the grace window/timeout elapses: no reject
    # was issued, so tracking simply resumes -- no re-acquisition wait.
    observe(logic, [box(logic, confidence=0.9, state="tentative")])
    assert logic.tracked_target_id == "one"
    assert logic._target_visible is True
    assert logic.state is FlightState.TARGET_ACQUISITION


def test_static_false_positive_is_dropped_even_after_follow_has_started():
    """The window keeps running once follow begins; nothing locks it out now.

    Confirmation starts the follow on weak evidence, so the filter has to be
    able to undo that. It stays safe for real targets because it requires the
    aircraft to be nearly stationary and the box to hold its size -- neither
    survives an actual pursuit.
    """
    logic = make_logic()
    # Above target_min_confidence, so it counts as a confirmation hit, but
    # below target_low_confidence, so the window judges it weak.
    observe(logic, [box(logic, confidence=0.5)])
    tick(logic, 0.1)
    observe(logic, [box(logic, confidence=0.5)])
    assert logic.state is FlightState.TARGET_FOLLOW

    tick(logic, 0.85)
    observe(logic, [box(logic, confidence=0.5)])
    tick(logic, 0.85)
    observe(logic, [box(logic, confidence=0.5)])

    assert not logic.is_target_tracking
    assert logic.target_decisions.ignored
    assert (
        logic.target_decisions.last_rejection_reason
        == "stationary_with_weak_confidence"
    )


@pytest.mark.parametrize("new_center", [500, 850])
def test_old_candidate_rejection_does_not_abort_new_candidate(new_center):
    logic = make_logic()
    observe(logic)
    tick(logic, 0.1)
    observe(logic, [box(logic, key="two", confidence=0.95, cx=new_center)])
    assert logic.tracked_target_id == "two"
    assert logic.reject_target("one")
    assert logic.state is FlightState.TARGET_ACQUISITION
    assert logic.tracked_target_id == "two"
    assert logic.patrol_resume_checkpoint == POSITION
    assert logic.target_decisions.started_at == pytest.approx(0.1)
    tick(logic, 0.1)
    observe(logic, [box(logic, key="two", confidence=0.95, cx=new_center)])
    assert logic._target_visible
    assert logic.tracked_target_id == "two"


def test_configurable_ignore_radius_and_exact_expiry():
    logic = make_logic(target_ignore_radius_m=2, target_ignore_timeout_s=1.2)
    observe(logic)
    logic.reject_target("one")
    point = logic.target_decisions.ignored[0].point
    assert logic.target_decisions.is_ignored("other", (point[0] + 2, point[1]), 0)
    assert not logic.target_decisions.is_ignored("other", (point[0] + 2.01, point[1]), 0)
    assert logic.target_decisions.is_ignored("one", None, 1.199)
    assert not logic.target_decisions.is_ignored("one", None, 1.2)


@pytest.mark.parametrize("command", ["request_patrol", "request_mission_takeoff", "request_return_home"])
def test_explicit_mission_change_clears_grace(command):
    logic = make_logic()
    observe(logic)
    if command == "request_return_home":
        logic.request_return_home()
    else:
        getattr(logic, command)(POSITION)
    assert not logic.reject_target("one")


def test_ignore_radius_filters_interruption_and_selection_identically():
    logic = make_logic()
    observe(logic)
    logic.reject_target("one")
    near = box(logic, key="new-id")
    args = dict(frame_width=1000, frame_height=1000, position=POSITION, yaw=0, camera_pitch=0.5)
    assert not logic.would_intercept([near], **args)
    logic.observe_detections([near], **args)
    assert logic.state is FlightState.PATROLLING
    far = box(logic, key="far", cx=950)
    assert logic.would_intercept([far], **args)
    logic.observe_detections([far], **args)
    assert logic.tracked_target_id == "far"


def test_no_ground_intersection_falls_back_to_the_range_estimate():
    """An airborne target never yields a ground fix -- the camera ray only
    reaches the ground for something below us. Rejection used to survive as a
    bare track id there, so the tracker re-identifying the same object put the
    aircraft straight back onto it and the operator's reject looked ignored."""
    logic = make_logic()
    observe(logic, pitch=-0.1)
    logic.reject_target("one")

    assert logic.target_decisions.ignored[0].point is not None
    observe(logic, [box(logic, key="new-id")], pitch=-0.1)
    assert logic.tracked_target_id != "new-id"


def test_rejecting_an_unlocatable_target_pauses_new_lock_ons_for_the_window():
    """Last-resort backstop: a target that yields neither a ground fix nor a
    usable range estimate leaves nowhere to pin the rejection, so nothing new
    is picked up until the ignore window expires."""
    logic = make_logic(target_ignore_timeout_s=6.0)
    empty_box = dict(box(logic), box=[500, 500, 500, 500])
    observe(logic, [empty_box])
    logic._tracked_target_id = "one"
    logic.target_decisions.memories["one"].point = None

    assert logic.reject_target("one") is True
    assert logic.target_decisions.is_ignored("anything-else", None, logic._elapsed_time)

    logic.target_decisions.prune(logic._elapsed_time + 6.0)
    assert not logic.target_decisions.is_ignored("anything-else", None, logic._elapsed_time + 6.0)


@pytest.mark.parametrize("state", [FlightState.ROI_PATROL, FlightState.TRANSITING_TO_ROI])
def test_rejection_resumes_exact_roi_state_and_target(state):
    logic = make_logic()
    logic.request_roi(POSITION, (50, 60), 20)
    logic.state = state
    original_target = logic.target
    observe(logic)
    logic.reject_target("one")
    assert logic.state is state
    assert logic.target == original_target
    assert logic._roi_center == (50, 60)
    assert logic._roi_radius == 20


def test_takeoff_detection_starts_follow_immediately_without_reacquisition_wait():
    logic = make_logic()
    logic.request_patrol((0, 0, 0))
    assert logic.state is FlightState.TAKING_OFF
    observe(logic, [box(logic, confidence=0.9)], position=(0, 0, 0))
    assert logic.state is FlightState.TARGET_ACQUISITION
    assert logic._target_resume_state is FlightState.TAKING_OFF
    tick(logic, 0.1, position=(0, 0, 0))
    observe(logic, [box(logic, confidence=0.9)], position=(0, 0, 0))
    # Two confirmed hits start follow immediately -- no observation-window
    # wait, and takeoff is preserved rather than restarting acquisition.
    assert logic.state is FlightState.TARGET_FOLLOW
    assert logic._target_resume_state is FlightState.TAKING_OFF


def test_reject_during_takeoff_resumes_climb_instead_of_returning_to_low_checkpoint():
    logic = FlightLogic(FlightConfig(takeoff_altitude=15), patrol_waypoint_provider=lambda *args: WAYPOINT)
    logic.initialize_home((0.0, 0.0, 0.0))
    logic.request_roi((0.0, 0.0, 0.0), (0.0, 0.0), 20.0)
    assert logic.state is FlightState.TAKING_OFF
    assert logic.pending_mission_state is FlightState.TRANSITING_TO_ROI

    low = (0.0, 0.0, 1.0)
    observe(logic, [box(logic, confidence=0.9)], position=low)
    tick(logic, 0.1, position=low)
    observe(logic, [box(logic, confidence=0.9)], position=low)
    assert logic.state is FlightState.TARGET_FOLLOW
    assert logic._target_resume_state is FlightState.TAKING_OFF

    # Climb well past the pursuit-safety floor so pursuit actually engages.
    climbing = (0.0, 0.0, 6.0)
    tick(logic, 0.1, position=climbing)
    observe(logic, [box(logic, confidence=0.9)], position=climbing)
    tick(logic, 0.1, position=climbing)
    assert logic.state is FlightState.TARGET_PURSUIT

    assert logic.reject_target("one")
    # Must resume the takeoff climb, not fly back down to the low altitude
    # checkpoint where interception happened.
    assert logic.state is FlightState.TAKING_OFF
    assert logic.target == (0.0, 0.0, 15.0)
    assert logic.pending_mission_state is FlightState.TRANSITING_TO_ROI

    # Finishing the climb transitions into the originally-queued ROI mission.
    tick(logic, 0.1, position=(0.0, 0.0, 15.0))
    assert logic.state is FlightState.ROI_PATROL


def test_reject_during_takeoff_already_at_cruise_altitude_skips_stale_reclimb():
    logic = FlightLogic(FlightConfig(takeoff_altitude=15), patrol_waypoint_provider=lambda *args: WAYPOINT)
    logic.initialize_home((0.0, 0.0, 0.0))
    logic.request_roi((0.0, 0.0, 0.0), (0.0, 0.0), 20.0)
    assert logic.state is FlightState.TAKING_OFF

    low = (0.0, 0.0, 1.0)
    observe(logic, [box(logic, confidence=0.9)], position=low)
    tick(logic, 0.1, position=low)
    observe(logic, [box(logic, confidence=0.9)], position=low)
    assert logic.state is FlightState.TARGET_FOLLOW

    # The chase carried the aircraft well past the nominal takeoff altitude,
    # away from the original ground-level anchor.
    cruising = (5.0, 5.0, 15.0)
    tick(logic, 0.1, position=cruising)
    observe(logic, [box(logic, confidence=0.9)], position=cruising)
    tick(logic, 0.1, position=cruising)
    assert logic.state is FlightState.TARGET_PURSUIT

    assert logic.reject_target("one", cruising)
    # Already at cruise altitude -- go straight to the queued ROI mission
    # instead of resuming a stale takeoff climb back at the old anchor,
    # which would otherwise leave state (and the camera) stuck on
    # TAKING_OFF for a long, unnecessary detour.
    assert logic.state is FlightState.TRANSITING_TO_ROI
    assert logic.pending_mission_state is None


def test_ground_projection_geometry_and_invalid_rays():
    def project(yaw=0, pitch=math.pi / 4, box=(450, 450, 550, 550), position=(0, 0, 10)):
        return ground_projection(box, 1000, 1000, position, yaw, pitch, math.pi / 2,
                                 0, math.radians(5), 250)
    assert project() == pytest.approx((10, 0))
    assert project(yaw=math.pi / 2) == pytest.approx((0, 10))
    assert project(box=(700, 450, 800, 550))[1] < 0
    assert project(box=(450, 700, 550, 800))[0] < 10
    assert project(pitch=0) is None
    assert project(pitch=-0.1) is None
    assert project(position=(0, 0, 0)) is None
    assert project(position=(0, 0, 1000)) is None
    assert project(pitch=float("inf")) is None
    assert project(position=(0, 0, float("nan"))) is None


@pytest.mark.parametrize("command", ["reject_target"])
def test_target_commands_validate_and_preserve_track_id(command):
    message = dict(type="command", commandId="decision", command=command, params={"trackId": "one"})
    assert validate_command_message(message)["params"] == {"trackId": "one"}
    for bad in (None, "", " ", 123):
        message["params"] = {"trackId": bad}
        with pytest.raises(ProtocolError):
            validate_command_message(message)


def test_decision_diagnostics_survive_protocol_and_legacy_payloads_still_work():
    logic = make_logic()
    observe(logic)
    data = logic.target_follow_diagnostics()
    assert _validate_target_follow_diagnostics(data)["targetDecision"] == data["targetDecision"]
    del data["targetDecision"]
    assert "targetDecision" not in _validate_target_follow_diagnostics(data)


@pytest.mark.parametrize("command", ["reject_target"])
@pytest.mark.parametrize("safety", [False, True])
def test_commands_route_with_track_id_and_respect_safety_return(command, safety):
    async def scenario():
        server = RealtimeServer()
        server.mission._safety_rth_reason = "test" if safety else None
        server.drone = AsyncMock()
        tablet = AsyncMock()
        payload = dict(type="command", commandId="test", command=command, params={"trackId": "one"})
        await server._route_tablet_message(tablet, json.dumps(payload))
        response = json.loads(tablet.send.call_args.args[0])
        if safety:
            server.drone.send_command.assert_not_called()
            assert response["type"] == "error"
        else:
            server.drone.send_command.assert_awaited_once_with(payload)
            assert response["type"] == "command_ack"
    asyncio.run(scenario())


def growing_box(logic, *, confidence, half_width):
    """A centered box whose apparent size changes between frames."""
    return {
        "trackId": "one", "trackState": "confirmed", "confidence": confidence,
        "box": [500 - half_width, 500 - half_width, 500 + half_width, 500 + half_width],
        "lastSeenTimestamp": logic._elapsed_time,
        "velocityPxPerSec": {"x": 0, "y": 0},
    }


def test_approaching_target_is_never_auto_rejected_as_a_static_false_positive():
    """A drone flying straight at a hovering aircraft has no lateral image motion.

    It is therefore indistinguishable from a fixed false positive by frame
    position alone, and on weak confidence the observation window used to
    discard it -- silently abandoning the most dangerous kind of target.
    Its box is growing, which is what tells the two apart.
    """
    logic = make_logic()
    for half_width in (20, 26, 34, 44):
        observe(logic, [growing_box(logic, confidence=0.5, half_width=half_width)])
        tick(logic, 0.6)

    assert logic.target_decisions.ready(logic._elapsed_time)
    assert not logic.target_decisions.should_reject()
    assert logic.is_target_tracking


def test_fixed_size_weak_target_is_still_auto_rejected():
    """The filter must still discard what it was built for: static objects."""
    logic = make_logic()
    for _ in range(4):
        observe(logic, [growing_box(logic, confidence=0.5, half_width=30)])
        tick(logic, 0.6)

    # Rejection fires inside the window and hands the mission straight back.
    assert not logic.is_target_tracking
    assert logic.state is FlightState.PATROLLING
    assert (
        logic.target_decisions.last_rejection_reason
        == "stationary_with_weak_confidence"
    )


def test_receding_target_is_not_auto_rejected_either():
    logic = make_logic()
    for half_width in (44, 34, 26, 20):
        observe(logic, [growing_box(logic, confidence=0.5, half_width=half_width)])
        tick(logic, 0.6)

    assert logic.target_decisions.ready(logic._elapsed_time)
    assert not logic.target_decisions.should_reject()


def test_pursuit_hold_suppresses_acquisition_entirely():
    logic = make_logic()
    assert not logic.pursuit_blocked
    assert logic.set_pursuit_blocked(True)
    assert logic.pursuit_blocked

    for _ in range(6):
        observe(logic, [box(logic, confidence=0.9)])
        tick(logic, 0.2)

    assert logic.state is FlightState.PATROLLING
    assert not logic.is_target_tracking
    assert logic.pursuit.velocity == (0, 0, 0)


def test_engaging_the_hold_mid_pursuit_disengages_and_resumes_the_mission():
    """A hold that only applied to future targets would leave a chase running."""
    logic = make_logic()
    for _ in range(4):
        observe(logic, [box(logic, confidence=0.9)])
        tick(logic, 0.2)
    assert logic.state is FlightState.TARGET_PURSUIT

    assert logic.set_pursuit_blocked(True, POSITION)

    assert not logic.is_target_tracking
    assert logic.pursuit.velocity == (0, 0, 0)
    tick(logic, 0.1)
    assert logic.state in {FlightState.TARGET_RETURN, FlightState.PATROLLING}


def test_releasing_the_hold_restores_autonomous_pursuit():
    logic = make_logic()
    logic.set_pursuit_blocked(True)
    observe(logic, [box(logic, confidence=0.9)])
    assert logic.state is FlightState.PATROLLING

    assert logic.set_pursuit_blocked(False)
    assert not logic.pursuit_blocked
    for _ in range(4):
        observe(logic, [box(logic, confidence=0.9)])
        tick(logic, 0.2)
    assert logic.state is FlightState.TARGET_PURSUIT


def test_setting_the_hold_to_its_current_value_is_a_no_op():
    logic = make_logic()
    assert not logic.set_pursuit_blocked(False)
    assert logic.set_pursuit_blocked(True)
    assert not logic.set_pursuit_blocked(True)
