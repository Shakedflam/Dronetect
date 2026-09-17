import pytest

from controllers.autonomous_drone.flight_logic import (
    FlightConfig,
    FlightLogic,
    FlightState,
)


POSITION = (10.0, 20.0, 15.0)
PATROL_TARGET = (100.0, 50.0, 15.0)
FRAME_WIDTH = 1000
FRAME_HEIGHT = 1000


def make_patrolling_logic(**overrides) -> FlightLogic:
    config_values = {
        "takeoff_altitude": 15.0,
        "target_min_confidence": 0.50,
        "target_confirmation_hits": 2,
        "target_acquisition_timeout_s": 0.5,
        "target_lost_search_timeout_s": 1.0,
        **overrides,
    }
    logic = FlightLogic(
        config=FlightConfig(**config_values),
        patrol_waypoint_provider=lambda position, completed: PATROL_TARGET,
    )
    logic.initialize_home((0.0, 0.0, 0.0))
    logic.request_patrol(POSITION)
    return logic


def detection(
    *,
    state: str = "tentative",
    box=(700.0, 400.0, 800.0, 500.0),
    velocity=(0.0, 0.0),
    track_id: str = "drone-7",
    confidence: float = 0.9,
) -> dict:
    return {
        "className": "drone",
        "confidence": confidence,
        "box": list(box),
        "trackId": track_id,
        "trackState": state,
        "lastSeenTimestamp": 0.0,
        "velocityPxPerSec": {"x": velocity[0], "y": velocity[1]},
    }


def observe(logic: FlightLogic, item: dict | None, *, yaw: float = 0.0) -> None:
    logic.observe_detections(
        [] if item is None else [item],
        frame_width=FRAME_WIDTH,
        frame_height=FRAME_HEIGHT,
        position=POSITION,
        yaw=yaw,
    )


def confirm(logic: FlightLogic, *, velocity=(0.0, 0.0), yaw=0.0) -> None:
    observe(logic, detection(), yaw=yaw)
    logic.update(POSITION, 0.1)
    observe(logic, detection(state="confirmed", velocity=velocity), yaw=yaw)


def test_single_frame_detection_hovers_immediately_then_resumes_patrol():
    logic = make_patrolling_logic()

    observe(logic, detection())

    assert logic.state is FlightState.TARGET_ACQUISITION
    assert logic.target == POSITION
    assert logic.patrol_resume_checkpoint == POSITION

    observe(logic, None)
    logic.update(POSITION, 0.6)

    assert logic.state is FlightState.PATROLLING
    assert logic.target == PATROL_TARGET


def test_takeoff_detection_holds_horizontal_position_and_keeps_climbing():
    logic = FlightLogic(
        config=FlightConfig(
            takeoff_altitude=15.0,
            target_min_confidence=0.50,
            target_confirmation_hits=2,
        ),
        patrol_waypoint_provider=lambda position, completed: PATROL_TARGET,
    )
    home = (0.0, 0.0, 0.0)
    logic.initialize_home(home)
    logic.request_patrol(home)

    logic.observe_detections(
        [detection(box=(700.0, 400.0, 800.0, 500.0))],
        frame_width=FRAME_WIDTH,
        frame_height=FRAME_HEIGHT,
        position=home,
        yaw=0.0,
    )

    assert logic.state is FlightState.TARGET_ACQUISITION
    assert logic.target == (0.0, 0.0, 15.0)
    assert logic.target_yaw is not None and logic.target_yaw < 0.0

    climbing = (0.0, 0.0, 6.0)
    logic.update(climbing, 0.1)

    assert logic.state is FlightState.TARGET_ACQUISITION
    assert logic.target == (0.0, 0.0, 15.0)


def test_takeoff_starts_horizontal_pursuit_before_normal_cruise_height():
    logic = FlightLogic(
        config=FlightConfig(
            takeoff_altitude=15.0,
            target_min_confidence=0.50,
            target_confirmation_hits=2,
        ),
        patrol_waypoint_provider=lambda position, completed: PATROL_TARGET,
    )
    home = (0.0, 0.0, 0.0)
    logic.initialize_home(home)
    logic.request_patrol(home)
    centered_box = (450.0, 450.0, 550.0, 550.0)

    logic.observe_detections(
        [detection(box=centered_box)],
        frame_width=FRAME_WIDTH,
        frame_height=FRAME_HEIGHT,
        position=home,
        yaw=0.0,
    )
    climbing = (0.0, 0.0, 6.0)
    logic.update(climbing, 0.1)
    logic.observe_detections(
        [detection(state="confirmed", box=(700.0, 450.0, 800.0, 550.0))],
        frame_width=FRAME_WIDTH,
        frame_height=FRAME_HEIGHT,
        position=climbing,
        yaw=0.0,
    )

    assert logic.state is FlightState.TARGET_FOLLOW

    logic.update(climbing, 0.1)

    assert logic.state is FlightState.TARGET_FOLLOW
    assert logic.target == climbing

    logic.observe_detections(
        [detection(state="confirmed", box=centered_box)],
        frame_width=FRAME_WIDTH,
        frame_height=FRAME_HEIGHT,
        position=climbing,
        yaw=0.0,
    )

    logic.update(climbing, logic.pursuit.config.stable_duration_s)

    assert logic.state is FlightState.TARGET_PURSUIT
    assert logic.pursuit.velocity[0] > 0.0
    assert climbing[2] < logic.config.takeoff_altitude


def test_takeoff_pursuit_climbs_and_speed_is_limited_near_height_floor():
    logic = FlightLogic(
        config=FlightConfig(
            takeoff_altitude=15.0,
            target_min_confidence=0.50,
            target_confirmation_hits=2,
        ),
        patrol_waypoint_provider=lambda position, completed: PATROL_TARGET,
    )
    logic.initialize_home((0.0, 0.0, 0.0))
    logic.request_patrol((0.0, 0.0, 0.0))
    low = (0.0, 0.0, 3.75)
    centered_small = (460.0, 460.0, 540.0, 540.0)
    logic.observe_detections(
        [detection(box=centered_small)],
        frame_width=FRAME_WIDTH,
        frame_height=FRAME_HEIGHT,
        position=low,
        yaw=0.0,
    )
    logic.update(low, 0.1)
    logic.observe_detections(
        [detection(state="confirmed", box=centered_small)],
        frame_width=FRAME_WIDTH,
        frame_height=FRAME_HEIGHT,
        position=low,
        yaw=0.0,
    )
    logic.update(low, logic.pursuit.config.stable_duration_s)

    assert logic.state is FlightState.TARGET_PURSUIT
    assert 0.0 < logic.pursuit.velocity[0] <= 1.0
    assert 0.0 < logic.pursuit.velocity[2] <= logic.config.target_takeoff_pursuit_ascent_mps
    assert logic._takeoff_pursuit_altitude() < logic.config.takeoff_altitude


def test_detection_below_fifty_percent_does_not_interrupt_patrol():
    logic = make_patrolling_logic()

    observe(logic, detection(confidence=0.49))

    assert logic.state is FlightState.PATROLLING
    assert logic.target == PATROL_TARGET


def test_detection_at_fifty_percent_starts_hovering():
    logic = make_patrolling_logic()

    observe(logic, detection(confidence=0.50))

    assert logic.state is FlightState.TARGET_ACQUISITION
    assert logic.target == POSITION


def test_confirmed_detection_enters_target_follow():
    logic = make_patrolling_logic()

    confirm(logic)

    assert logic.state is FlightState.TARGET_FOLLOW
    assert logic.tracked_target_id == "drone-7"


def test_confirmed_real_target_replaces_tentative_false_detection():
    logic = make_patrolling_logic()
    observe(logic, detection(track_id="false-track"))
    logic.update(POSITION, 0.1)

    logic.observe_detections(
        [
            detection(track_id="false-track", state="lost", confidence=0.51),
            detection(
                track_id="real-track",
                state="confirmed",
                confidence=0.91,
                box=(450.0, 400.0, 550.0, 500.0),
            ),
        ],
        frame_width=FRAME_WIDTH,
        frame_height=FRAME_HEIGHT,
        position=POSITION,
        yaw=0.0,
    )

    assert logic.state is FlightState.TARGET_ACQUISITION
    assert logic.tracked_target_id == "real-track"
    assert logic.target_yaw == pytest.approx(0.0)


def test_confirmed_new_track_reacquires_during_lost_follow_search():
    logic = make_patrolling_logic()
    confirm(logic)

    logic.observe_detections(
        [
            detection(track_id="drone-7", state="lost"),
            detection(
                track_id="drone-8",
                state="confirmed",
                confidence=0.95,
                box=(450.0, 400.0, 550.0, 500.0),
            ),
        ],
        frame_width=FRAME_WIDTH,
        frame_height=FRAME_HEIGHT,
        position=POSITION,
        yaw=0.0,
    )

    assert logic.state is FlightState.TARGET_FOLLOW
    assert logic.tracked_target_id == "drone-8"
    assert logic.target_yaw == pytest.approx(0.0)


def test_tentative_new_track_stops_search_yaw_and_waits_for_confirmation():
    logic = make_patrolling_logic()
    confirm(logic)

    logic.observe_detections(
        [
            detection(track_id="drone-7", state="lost"),
            detection(
                track_id="drone-8",
                state="tentative",
                confidence=0.85,
                box=(450.0, 400.0, 550.0, 500.0),
            ),
        ],
        frame_width=FRAME_WIDTH,
        frame_height=FRAME_HEIGHT,
        position=POSITION,
        yaw=0.0,
    )

    assert logic.state is FlightState.TARGET_ACQUISITION
    assert logic.tracked_target_id == "drone-8"
    assert logic.target == POSITION
    assert logic.target_yaw == pytest.approx(0.0)


def test_yaw_follows_horizontal_image_center_error():
    logic = make_patrolling_logic()

    observe(logic, detection(box=(700.0, 400.0, 800.0, 500.0)), yaw=0.25)
    right_yaw = logic.target_yaw

    assert right_yaw is not None
    assert right_yaw < 0.25


def test_target_velocity_predictively_influences_yaw():
    stationary = make_patrolling_logic(target_yaw_deadband_ratio=0.0)
    moving = make_patrolling_logic(target_yaw_deadband_ratio=0.0)
    centered_box = (450.0, 400.0, 550.0, 500.0)

    observe(stationary, detection(box=centered_box, velocity=(0.0, 0.0)))
    observe(moving, detection(box=centered_box, velocity=(400.0, 0.0)))

    assert stationary.target_yaw == pytest.approx(0.0)
    assert moving.target_yaw is not None
    assert moving.target_yaw < 0.0


def test_gimbal_tilts_down_for_target_below_aircraft():
    logic = make_patrolling_logic()

    logic.observe_detections(
        [detection(box=(450.0, 800.0, 550.0, 900.0))],
        frame_width=FRAME_WIDTH,
        frame_height=FRAME_HEIGHT,
        position=POSITION,
        yaw=0.0,
        camera_pitch=0.12,
    )

    assert logic.target_camera_pitch is not None
    assert logic.target_camera_pitch > 0.12


def test_target_motion_does_not_translate_before_stable_pursuit():
    right = make_patrolling_logic()
    left = make_patrolling_logic()

    confirm(right, velocity=(200.0, 0.0))
    confirm(left, velocity=(-200.0, 0.0))

    assert right.target is not None and left.target is not None
    assert right.target == POSITION
    assert left.target == POSITION


def test_target_loss_hovers_and_reacquisition_continues_following():
    logic = make_patrolling_logic()
    confirm(logic)

    observe(logic, None)
    logic.update(POSITION, 0.4)

    assert logic.state is FlightState.TARGET_FOLLOW
    assert logic.target == POSITION

    observe(logic, detection(state="confirmed"))
    logic.update(POSITION, 0.1)

    assert logic.state is FlightState.TARGET_FOLLOW
    assert logic.target == POSITION


def test_target_follow_diagnostics_report_search_without_changing_controller_state():
    logic = make_patrolling_logic()
    confirm(logic, velocity=(200.0, -100.0))

    following = logic.target_follow_diagnostics()
    assert following["flightState"] == "TARGET_FOLLOW"
    assert following["trackerId"] == "drone-7"
    assert following["detectionConfidence"] == pytest.approx(0.9)
    assert following["imageCenterError"] == pytest.approx({"x": 0.25, "y": -0.05})
    assert following["confirmation"] == {
        "hits": 2,
        "requiredHits": 2,
        "progress": 1.0,
    }
    assert following["estimatedImageVelocity"]["x"] > 0.0
    assert following["estimatedImageVelocity"]["y"] < 0.0
    assert following["boundingBoxAreaRatio"] == pytest.approx(0.01)
    assert following["standoffError"] == pytest.approx(0.015)
    assert following["patrolInterrupted"] is True
    assert following["patrolResumeState"] == "PATROL"

    observe(logic, None)
    logic.update(POSITION, 0.4)
    searching = logic.target_follow_diagnostics()

    assert logic.state is FlightState.TARGET_FOLLOW
    assert searching["flightState"] == "TARGET_LOST_SEARCH"
    assert searching["targetLostDurationS"] == pytest.approx(0.4)


def test_permanent_loss_resumes_exact_patrol_target():
    logic = make_patrolling_logic()
    confirm(logic)

    observe(logic, None)
    logic.update(POSITION, 1.1)

    assert logic.state is FlightState.PATROLLING
    assert logic.target == PATROL_TARGET


def test_lost_tracker_predictions_do_not_extend_search_timeout():
    logic = make_patrolling_logic()
    confirm(logic)

    for _ in range(3):
        logic.update(POSITION, 0.4)
        observe(logic, detection(state="lost", velocity=(100.0, 0.0)))

    logic.update(POSITION, 0.01)

    assert logic.state is FlightState.PATROLLING
    assert logic.target == PATROL_TARGET


def test_patrol_checkpoint_remains_unchanged_throughout_follow_mode():
    logic = make_patrolling_logic()
    confirm(logic)
    checkpoint = logic.patrol_resume_checkpoint

    logic.update((12.0, 22.0, 15.0), 0.2)
    observe(logic, detection(state="confirmed"), yaw=0.4)
    logic.update((14.0, 24.0, 15.0), 0.2)

    assert logic.state is FlightState.TARGET_FOLLOW
    assert checkpoint == POSITION
    assert logic.patrol_resume_checkpoint == checkpoint


def test_completed_tracker_id_does_not_start_a_duplicate_follow_session():
    logic = make_patrolling_logic()
    confirm(logic)
    observe(logic, None)
    logic.update(POSITION, 1.1)

    observe(logic, detection(state="confirmed"))

    assert logic.state is FlightState.PATROLLING
    assert logic.target == PATROL_TARGET


@pytest.mark.parametrize(
    "roi_state",
    [FlightState.TRANSITING_TO_ROI, FlightState.ROI_PATROL],
)
def test_roi_flight_pauses_for_target_then_resumes_exact_state_and_target(roi_state):
    logic = make_patrolling_logic()
    roi_target = (25.0, 30.0, 15.0)
    logic.state = roi_state
    logic.target = roi_target

    observe(logic, detection())

    assert logic.state is FlightState.TARGET_ACQUISITION
    assert logic.target == POSITION

    observe(logic, None)
    logic.update(POSITION, 0.6)

    assert logic.state is roi_state
    assert logic.target == roi_target


def test_one_fps_detections_have_time_to_confirm_before_resuming():
    logic = make_patrolling_logic(target_acquisition_timeout_s=2.5)

    observe(logic, detection())
    logic.update(POSITION, 1.0)
    observe(logic, detection(state="confirmed"))

    assert logic.state is FlightState.TARGET_FOLLOW


def test_box_growth_does_not_start_translation_before_stable_pursuit():
    logic = make_patrolling_logic(target_approach_growth_ratio_per_s=0.01)
    observe(logic, detection(box=(450.0, 450.0, 550.0, 550.0)))
    logic.update(POSITION, 0.1)
    observe(
        logic,
        detection(state="confirmed", box=(350.0, 350.0, 650.0, 650.0)),
    )

    assert logic.state is FlightState.TARGET_FOLLOW
    assert logic.target is not None
    assert logic.target == POSITION
