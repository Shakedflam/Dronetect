import math
import importlib.util
from collections import deque
from dataclasses import replace
from pathlib import Path
import sys
from types import ModuleType
from unittest.mock import Mock

import pytest

from controllers.autonomous_drone.flight_logic import (
    FlightConfig, FlightLogic, FlightState, advance_kinematic_flight,
)
from controllers.autonomous_drone.pursuit import (
    PursuitConfig,
    estimate_target_altitude,
    time_to_contact_s,
)
from controllers.autonomous_drone.roi_patrol import LawnMowerRoiPatrolPlanner
from controllers.autonomous_drone import flight_logic
from realtime_server.protocol import validate_telemetry_message
from config.shared_config import OperatingConfig
from services.gps import LocalPosition
from services.mission_interruption import InterruptedMissionType
from services.no_fly_router import NoFlyRouteUnavailable, NoFlyRouter
from services.patrol_regions import PatrolAreaType, PatrolRegionStore
from services.patrol_planner import PatrolPlanner
from services.telemetry import build_simulator_telemetry


POSITION = (10.0, 20.0, 15.0)
WAYPOINT = (100.0, 50.0, 15.0)


def make_logic(**options):
    logic = FlightLogic(
        # Isolate pursuit dynamics from the acquisition gate tested separately.
        FlightConfig(target_observation_s=0.0, takeoff_altitude=15, target_standoff_box_area_ratio=0.01,
                     target_standoff_deadband_fraction=0.1,
                     target_lost_search_timeout_s=2.0,
                     # Pinned so these exercise the pursuit laws rather than
                     # whatever range calibration the defaults happen to
                     # derive: the boxes below are sized against this pair.
                     target_reference_distance_m=2.0,
                     target_reference_area_ratio=0.01),
        pursuit_config=PursuitConfig(stable_duration_s=0.3, **options),
        patrol_waypoint_provider=lambda position, completed: WAYPOINT,
    )
    logic.initialize_home((0.0, 0.0, 0.0))
    logic.request_patrol(POSITION)
    return logic


def standoff_size(logic):
    """Box side, in test-frame pixels, that the held standoff subtends.

    The station distance and the range calibration are separate settings, so
    a test that means "at standoff" has to derive the size rather than lean
    on them happening to coincide.
    """
    return 1000 * math.sqrt(logic._standoff_area_ratio())


def observe(logic, *, size=None, scale=1.0, vx=0, state="confirmed", confidence=0.9,
            track_id="one", cx=500, cy=500, position=POSITION, yaw=0.0,
            camera_pitch=0.0,
            allow_landing_intercept=False):
    if size is None:
        # ``scale`` is a multiple of the standoff box: < 1 is farther away.
        size = standoff_size(logic) * scale
    logic.observe_detections([{
        "trackId": track_id, "trackState": state, "confidence": confidence,
        "box": [cx-size/2, cy-size/2, cx+size/2, cy+size/2],
        "velocityPxPerSec": {"x": vx, "y": 0},
    }], frame_width=1000, frame_height=1000, position=position, yaw=yaw,
       camera_pitch=camera_pitch,
       allow_landing_intercept=allow_landing_intercept)


def observe_at_range(logic, distance_m, **options):
    area_ratio = logic.config.target_standoff_box_area_ratio * (
        logic.config.target_standoff_distance_m / distance_m
    ) ** 2
    observe(logic, size=1000 * math.sqrt(area_ratio), **options)


def enter_at_range(logic, distance_m=7.0, **options):
    for _ in range(6):
        observe_at_range(logic, distance_m, **options)
        logic.update(POSITION, 0.1)
    assert logic.state is FlightState.TARGET_PURSUIT


def enter(logic):
    for _ in range(6):
        observe(logic)
        logic.update(POSITION, 0.1)
    assert logic.state is FlightState.TARGET_PURSUIT


def test_entry_starts_immediately_once_confirmed_and_altitude_safe_even_off_center():
    logic = make_logic()
    observe(logic, cx=750)
    logic.update(POSITION, 0.1)
    assert logic.state is FlightState.TARGET_ACQUISITION
    observe(logic, cx=750)
    logic.update(POSITION, 0.1)
    # Two confirmed hits plus altitude safety are enough: approach begins
    # immediately even though the target is still off-center -- yaw keeps
    # correcting toward it in parallel rather than gating movement.
    assert logic.state is FlightState.TARGET_PURSUIT
    assert logic.patrol_resume_checkpoint == POSITION


@pytest.mark.parametrize("scale,sign", [(0.8, 1), (1.2, -1)])
def test_away_and_approaching_target_motion(scale, sign):
    logic = make_logic()
    enter(logic)
    observe(logic, scale=scale)
    logic.update(POSITION, 0.1)
    assert sign * logic.pursuit.velocity[0] > 0
    assert sign * logic.pursuit.motion[0] > 0


def test_far_target_ramps_toward_cruise_speed_not_capped_near_one_mps():
    logic = make_logic()
    # A box much smaller than the standoff size means the target is still
    # far away; held steady (no growth cue) so only the standoff term drives
    # forward speed. Confirm, then let the acceleration-limited ramp settle.
    for _ in range(10):
        observe(logic, scale=0.1)
        logic.update(POSITION, 0.1)
    assert logic.state is FlightState.TARGET_PURSUIT
    # The commanded approach speed should ramp well past the ~1 m/s the
    # unscaled normalized error used to saturate at, toward cruise speed.
    assert logic.pursuit.velocity[0] > 1.0


def test_lateral_motion_and_stationary_hover():
    logic = make_logic()
    enter(logic)
    assert logic.pursuit.velocity[:2] == (0, 0)
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
    assert logic.pursuit.velocity[:2] == (0, 0)


@pytest.mark.parametrize("failure", ["missing", "lost", "confidence", "tentative", "stale"])
def test_loss_stops_actual_translation_and_reacquires_pursuit(failure):
    logic = make_logic()
    enter(logic)
    observe(logic, scale=0.8, vx=200)
    logic.update(POSITION, 0.1)
    assert math.hypot(*logic.pursuit.velocity[:2]) > 0
    if failure == "missing":
        logic.observe_detections([], frame_width=1000, frame_height=1000, position=POSITION, yaw=0)
    elif failure == "stale":
        logic.update(POSITION, 1.6)
    else:
        observe(logic, state=failure if failure in {"lost", "tentative"} else "confirmed",
                confidence=0.1 if failure == "confidence" else 0.9)
    assert logic.diagnostic_state == "TARGET_LOST_SEARCH"
    # A gap decelerates at the acceleration limit instead of snapping to
    # zero, so that one dropped frame does not cost the whole approach ramp,
    # and the search then coasts after the target rather than braking. Either
    # way translation has ended by the time the search window closes.
    window = logic.config.target_lost_search_timeout_s
    speeds = []
    while logic._elapsed_time - logic._target_last_seen_at < window - 0.15:
        logic.update(POSITION, 0.1)
        speeds.append(math.hypot(*logic.pursuit.velocity[:2]))
    # Never accelerating, and down to nothing by the time the search gives up.
    assert speeds == sorted(speeds, reverse=True)
    assert speeds[-1] < 0.2
    step = advance_kinematic_flight(position=POSITION, target=POSITION, yaw=0,
                                   horizontal_speed_mps=4, dt=0.1,
                                   velocity_command=(0.0, 0.0, 0.0))
    assert step.position == POSITION
    assert step.horizontal_speed_mps == 0
    observe(logic, scale=0.8)
    logic.update(POSITION, 0.1)
    assert logic.state is FlightState.TARGET_PURSUIT
    assert logic.pursuit.velocity[0] > 0


def test_collision_overrides_forward_motion_and_never_descends():
    logic = make_logic()
    enter(logic)
    observe(logic, scale=0.6)
    logic.update(POSITION, 0.1)
    assert logic.pursuit.velocity[0] > 0
    observe(logic, scale=5.0)
    logic.update(POSITION, 0.1)
    assert logic.pursuit.constraint == "pass_under_avoidance"
    assert logic.pursuit.velocity[0] <= 0
    assert logic.pursuit.velocity[2] >= 0


def test_raw_collision_cue_dodges_a_frame_before_the_smoothed_one():
    logic = make_logic()
    enter(logic)
    observe(logic, scale=0.6)
    logic.update(POSITION, 0.1)
    observe(logic, scale=0.95)
    logic.update(POSITION, 0.1)
    cfg = logic.pursuit.config
    smoothed = time_to_contact_s(
        logic._pursuit_growth / max(logic._target_area_ratio, 1e-6)
    )
    # The smoothed cue has not caught up yet, but the raw one already puts
    # contact inside the unconditional threshold.
    assert smoothed > cfg.entry_time_to_contact_s
    assert logic.pursuit.time_to_contact_s <= cfg.emergency_time_to_contact_s
    assert logic.pursuit.constraint == "pass_under_avoidance"
    assert logic.pursuit.velocity[0] <= 0


def test_fresh_detection_with_rapid_closing_triggers_avoidance_immediately():
    logic = make_logic()
    observe(logic, scale=0.6)
    logic.update(POSITION, 0.1)
    assert logic.state is FlightState.TARGET_ACQUISITION
    observe(logic, scale=5.0)
    logic.update(POSITION, 0.1)
    # Collision avoidance is reachable within the first two ticks after a
    # fresh detection -- it does not require an established pursuit history.
    assert logic.state is FlightState.TARGET_PURSUIT
    assert logic.pursuit.constraint == "pass_under_avoidance"
    assert logic.pursuit.velocity[0] <= 0


def test_head_on_target_activates_lateral_avoidance_at_five_meters():
    logic = make_logic()
    enter_at_range(logic)

    observe_at_range(logic, 5.0)
    logic.update(POSITION, 0.1)

    assert logic.pursuit.avoidance_active
    assert logic.pursuit.avoidance_side == 1.0
    assert logic.pursuit.constraint == "pass_under_avoidance"
    assert logic.pursuit.velocity[0] <= 0.0
    assert logic.pursuit.velocity[1] < 0.0
    assert 5.0 > logic.config.target_standoff_distance_m


@pytest.mark.parametrize(
    "center_x,expected_side",
    [(450, 1.0), (550, -1.0)],
)
def test_approaching_offset_target_chooses_opposite_lateral_side(
    center_x, expected_side
):
    logic = make_logic()
    enter_at_range(logic, cx=center_x)

    observe_at_range(logic, 5.0, cx=center_x)
    logic.update(POSITION, 0.1)

    assert logic.pursuit.avoidance_side == expected_side


def test_avoidance_yaws_toward_target_while_translating_sideways():
    logic = make_logic()
    enter_at_range(logic)

    observe_at_range(logic, 5.0, yaw=0.0)
    logic.update(POSITION, 0.1)

    assert logic.pursuit.avoidance_side == 1.0
    assert logic.pursuit.velocity[1] < 0.0
    assert logic.target_yaw is not None and logic.target_yaw > 0.0
    # Avoidance must not inflate the yaw limit past its configured band.
    assert (
        logic.config.target_min_yaw_rate_rad_s
        <= logic.target_yaw_rate_limit
        <= logic.config.target_max_yaw_rate_rad_s
    )


def test_avoidance_holds_while_closing_and_releases_once_closure_stops():
    logic = make_logic()
    enter_at_range(logic)
    observe_at_range(logic, 5.0)
    logic.update(POSITION, 0.1)
    assert logic.pursuit.avoidance_active

    # Still being closed on: the dodge must not release just because the
    # aircraft has bought itself a little more range.
    for distance in (4.6, 4.2, 3.8):
        observe_at_range(logic, distance)
        logic.update(POSITION, 0.1)
        assert logic.pursuit.avoidance_active

    # Closure stops: time to contact recovers past the exit threshold and
    # normal pursuit resumes.
    for _ in range(20):
        observe_at_range(logic, 8.0)
        logic.update(POSITION, 0.1)

    assert not logic.pursuit.avoidance_active
    assert logic.pursuit.constraint is None
    assert logic.pursuit.velocity[0] > 0.0


def test_avoidance_side_does_not_switch_with_noisy_horizontal_cues():
    logic = make_logic()
    enter_at_range(logic)
    observe_at_range(logic, 5.0)
    logic.update(POSITION, 0.1)
    assert logic.pursuit.avoidance_side == 1.0

    for distance, center_x in zip(
        (4.8, 4.6, 4.4, 4.2, 4.0), (560, 440, 575, 425, 550)
    ):
        observe_at_range(logic, distance, cx=center_x)
        logic.update(POSITION, 0.1)
        assert logic.pursuit.avoidance_side == 1.0


def test_centered_target_motion_selects_the_opposite_avoidance_side():
    logic = make_logic()
    enter_at_range(logic)

    for _ in range(3):
        observe_at_range(logic, 7.0, vx=200)
        logic.update(POSITION, 0.1)
    observe_at_range(logic, 5.0, vx=200)
    logic.update(POSITION, 0.1)

    assert logic.pursuit.avoidance_side == -1.0


def test_brief_loss_stops_motion_but_preserves_avoidance_side():
    logic = make_logic()
    enter_at_range(logic)
    observe_at_range(logic, 5.0)
    logic.update(POSITION, 0.1)
    assert logic.pursuit.avoidance_side == 1.0

    logic.observe_detections(
        [], frame_width=1000, frame_height=1000, position=POSITION, yaw=0.0
    )
    logic.update(POSITION, 0.4)

    # The search coasts after the target, so what matters is that nothing is
    # driving it any more -- not that the command is exactly zero this tick.
    assert math.hypot(*logic.pursuit.velocity[:2]) < 0.5
    # Vertical is the exception: the search deliberately gains a little height
    # to see over whatever the target went behind.
    assert 0.0 < logic.pursuit.velocity[2] <= logic.config.target_max_vertical_speed_mps
    assert logic.pursuit.avoidance_side == 1.0


def test_geofence_overrides_active_pass_under_translation():
    logic = make_logic(geofence_radius_m=30.0)
    enter_at_range(logic)
    edge = (30.0, 0.0, 15.0)

    observe_at_range(logic, 5.0, position=edge)
    logic.update(edge, 0.1)

    assert logic.pursuit.avoidance_active
    assert logic.pursuit.constraint == "geofence"
    assert logic.pursuit.velocity[:2] == (0.0, 0.0)


def test_height_floor_overrides_active_pass_under_translation():
    logic = make_logic()
    enter_at_range(logic)
    below_floor = (POSITION[0], POSITION[1], 3.0)

    observe_at_range(logic, 5.0, position=below_floor)
    logic.update(below_floor, 0.1)

    assert logic.pursuit.avoidance_active
    assert logic.pursuit.constraint == "height_floor"
    assert logic.pursuit.velocity[:2] == (0.0, 0.0)
    assert logic.pursuit.velocity[2] > 0.0


def test_dodge_times_must_increase_and_leave_room_for_our_own_approach():
    with pytest.raises(ValueError, match="emergency < entry < exit"):
        PursuitConfig(entry_time_to_contact_s=3.0, exit_time_to_contact_s=3.0)
    with pytest.raises(ValueError, match="emergency < entry < exit"):
        PursuitConfig(emergency_time_to_contact_s=1.8, entry_time_to_contact_s=1.8)
    # If our own closure could reach the dodge threshold, approaching the
    # standoff would be indistinguishable from being closed on.
    with pytest.raises(ValueError, match="approach time to contact"):
        PursuitConfig(entry_time_to_contact_s=2.5, approach_time_to_contact_s=2.5)
    with pytest.raises(ValueError, match="pass-under range window"):
        PursuitConfig(pass_under_min_range_m=25.0, pass_under_max_range_m=25.0)


def test_pursuit_never_closes_fast_enough_to_trip_its_own_dodge():
    """Our own approach must stay inside the dodge threshold at every range."""
    cfg = PursuitConfig()
    for distance in (3.0, 5.0, 10.0, 20.0, 40.0):
        closing_cap = distance / cfg.approach_time_to_contact_s
        own_time_to_contact = distance / min(closing_cap, cfg.max_speed_mps)
        assert own_time_to_contact > cfg.entry_time_to_contact_s


def test_brief_target_loss_resumes_pursuit_immediately_without_reacquisition():
    logic = make_logic()
    enter(logic)
    observe(logic, scale=0.8, vx=200)
    logic.update(POSITION, 0.1)
    assert math.hypot(*logic.pursuit.velocity[:2]) > 0
    # The target drops out of the tracker entirely for about half a second.
    logic.observe_detections([], frame_width=1000, frame_height=1000, position=POSITION, yaw=0)
    logic.update(POSITION, 0.5)
    assert math.hypot(*logic.pursuit.velocity[:2]) < 0.5
    assert logic.diagnostic_state == "TARGET_LOST_SEARCH"
    assert logic.state is FlightState.TARGET_PURSUIT
    # It reappears well inside the lost-search timeout: pursuit resumes on
    # this very tick, with no new confirmation hits or observation window.
    observe(logic, scale=0.8)
    logic.update(POSITION, 0.1)
    assert logic.state is FlightState.TARGET_PURSUIT
    assert logic.pursuit.velocity[0] > 0


def test_vertical_image_error_drives_altitude_correction_toward_target():
    logic = make_logic()
    enter(logic)
    assert logic.pursuit.velocity[:2] == (0, 0)
    # Target appears far enough above image center for the estimated metric
    # height error to exceed the configured 0.5 m target band.
    logic.observe_detections([{
        "trackId": "one", "trackState": "confirmed", "confidence": 0.9,
        "box": [450, 50, 550, 150], "velocityPxPerSec": {"x": 0, "y": 0},
    }], frame_width=1000, frame_height=1000, position=POSITION, yaw=0.0)
    logic.update(POSITION, 0.1)
    assert logic._target_altitude_estimate is not None
    assert logic._target_altitude_estimate.height_error_m > 0.5
    assert 0.0 < logic.pursuit.velocity[2] < logic.config.target_max_vertical_speed_mps

    # A target slightly below is still below the station held above it, so
    # the command is a climb: the aircraft levels with the station, not with
    # the target.
    logic = make_logic()
    enter(logic)
    logic.observe_detections([{
        "trackId": "one", "trackState": "confirmed", "confidence": 0.9,
        "box": [450, 850, 550, 950], "velocityPxPerSec": {"x": 0, "y": 0},
    }], frame_width=1000, frame_height=1000, position=POSITION, yaw=0.0)
    logic.update(POSITION, 0.1)
    assert -1.0 < logic._target_altitude_estimate.height_error_m < 0.0
    assert logic.pursuit.velocity[2] > 0.0

    # Only a target further below than the standoff offset earns a descent.
    logic = make_logic()
    enter(logic)
    logic.observe_detections([{
        "trackId": "one", "trackState": "confirmed", "confidence": 0.9,
        "box": [490, 940, 510, 960], "velocityPxPerSec": {"x": 0, "y": 0},
    }], frame_width=1000, frame_height=1000, position=POSITION, yaw=0.0)
    for _ in range(6):
        logic.update(POSITION, 0.1)
    assert (
        logic._target_altitude_estimate.height_error_m
        < -logic.pursuit.config.altitude_offset_m
    )
    assert logic.pursuit.velocity[2] < 0.0


def test_small_estimated_height_offset_holds_inside_target_band():
    logic = make_logic()
    enter(logic)
    # A target sitting one standoff offset below is already on station, so
    # the residual is inside the band and must not produce vertical hunting.
    logic.observe_detections([{
        "trackId": "one", "trackState": "confirmed", "confidence": 0.9,
        "box": [485, 946, 515, 976], "velocityPxPerSec": {"x": 0, "y": 0},
    }], frame_width=1000, frame_height=1000, position=POSITION, yaw=0.0)
    for _ in range(14):
        logic.update(POSITION, 0.1)
    assert logic._target_altitude_estimate is not None
    station_error = (
        logic._target_altitude_estimate.height_error_m
        + logic.pursuit.config.altitude_offset_m
    )
    assert abs(station_error) < 0.5
    assert logic.pursuit.velocity[2] == pytest.approx(0.0, abs=0.01)


def test_gimbal_tilt_sustains_climb_once_image_residual_is_corrected():
    logic = make_logic()
    enter(logic)
    # The gimbal has already tilted up substantially to track the target
    # (a large actual camera tilt fed back this tick), even though the
    # image-frame residual is now ~0 (the gimbal has caught up to it).
    # Relying on the residual alone would drop the climb command back to
    # ~0 here; the sustained gimbal tilt must keep it climbing.
    observe(logic, camera_pitch=-0.3)
    logic.update(POSITION, 0.1, camera_pitch=-0.3)
    assert logic.pursuit.velocity[2] > 0.0


def test_camera_geometry_and_inverse_area_range_estimate():
    centered = estimate_target_altitude(
        box=[450, 450, 550, 550], frame_width=1000, frame_height=1000,
        camera_pitch=0.0, horizontal_fov_rad=1.0,
        reference_distance_m=2.0, reference_area_ratio=0.01,
        max_range_m=100.0,
    )
    quarter_area = estimate_target_altitude(
        box=[475, 475, 525, 525], frame_width=1000, frame_height=1000,
        camera_pitch=0.0, horizontal_fov_rad=1.0,
        reference_distance_m=2.0, reference_area_ratio=0.01,
        max_range_m=100.0,
    )
    above = estimate_target_altitude(
        box=[450, 50, 550, 150], frame_width=1000, frame_height=1000,
        camera_pitch=0.0, horizontal_fov_rad=1.0,
        reference_distance_m=2.0, reference_area_ratio=0.01,
        max_range_m=100.0,
    )
    assert centered is not None and centered.height_error_m == pytest.approx(0.0)
    assert quarter_area is not None
    assert quarter_area.slant_range_m == pytest.approx(2 * centered.slant_range_m)
    assert above is not None and above.elevation_rad > 0 and above.height_error_m > 0


def test_altitude_estimate_rejects_zero_area_and_excessive_range():
    common = dict(
        frame_width=1280, frame_height=720, camera_pitch=0.0,
        horizontal_fov_rad=1.0, reference_distance_m=2.0,
        reference_area_ratio=0.025, max_range_m=10.0,
    )
    assert estimate_target_altitude(box=[10, 10, 10, 20], **common) is None
    assert estimate_target_altitude(box=[10, 10, 11, 11], **common) is None


def test_widescreen_center_ray_uses_gimbal_pitch_as_elevation():
    estimate = estimate_target_altitude(
        box=[590, 310, 690, 410], frame_width=1280, frame_height=720,
        camera_pitch=-0.2, horizontal_fov_rad=1.0,
        reference_distance_m=2.0,
        reference_area_ratio=10000 / (1280 * 720),
        max_range_m=100.0,
    )
    assert estimate is not None
    assert estimate.elevation_rad == pytest.approx(0.2)
    assert estimate.height_error_m == pytest.approx(2.0 * math.sin(0.2))


def test_geofence_and_height_override_translation():
    logic = make_logic(geofence_radius_m=30)
    enter(logic)
    observe(logic, scale=0.4)
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
        observe(logic, scale=0.6, vx=1000 if i < 15 else -1000)
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


def test_new_tracker_resumes_pursuit_immediately_and_rth_cancels_it():
    logic = make_logic()
    enter(logic)
    observe(logic, track_id="two")
    logic.update(POSITION, 0.1)
    # A track-id handoff no longer requires re-achieving stability before
    # resuming pursuit -- approach begins again as soon as it's usable/safe.
    assert logic.state is FlightState.TARGET_PURSUIT
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
    drone._pending_operating_config = None
    drone.patrol_regions = PatrolRegionStore()
    drone.no_fly_router = NoFlyRouter()
    drone.patrol_planner = Mock()
    drone._target_tracking_flight_config = module.AutonomousDrone.SEARCH_KINEMATIC_FLIGHT_CONFIG
    drone._elapsed_time = 0.0
    drone.manual_flight_config = module.build_manual_flight_config()
    drone.manual_input = module.ManualInputRouter(
        command_timeout_s=drone.manual_flight_config.command_timeout_s,
    )
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
    observe(simulator.logic, scale=0.6)
    simulator.logic.update(POSITION, 0.1)
    moved, _, _, _ = simulator._apply_kinematic_flight(POSITION, 0, 0.1)
    assert moved[0] > POSITION[0]
    observe(simulator.logic, scale=5.0)
    simulator.logic.update(moved, 0.1)
    avoided, _, _, _ = simulator._apply_kinematic_flight(moved, 0, 0.1)
    assert avoided[0] <= moved[0]
    simulator.no_fly_router.route.assert_not_called()


def test_controller_applies_operating_config_pair_atomically_while_grounded(simulator):
    simulator.patrol_planner = PatrolPlanner(store=simulator.patrol_regions)
    simulator.realtime_client = Mock()
    simulator.logic.state = FlightState.IDLE

    simulator._handle_operating_config_command(
        {
            "commandId": "config-1",
            "params": {"maxFlightRadiusM": 750.0, "cruiseAltitudeM": 15.25},
        },
        POSITION,
    )

    assert simulator.logic.config.takeoff_altitude == 15.25
    assert simulator.logic.config.z_bounds == (15.25, 15.25)
    assert simulator.logic.pursuit.config.geofence_radius_m == 750.0
    assert simulator.logic.roi_patrol_planner.config.max_flight_radius_m == 750.0
    assert max(region.outer_radius_m for region in simulator.patrol_regions.regions) == 750.0
    assert simulator.logic.state is FlightState.IDLE
    result = simulator.realtime_client.publish_control_message.call_args.args[0]
    assert result == {
        "type": "operating_config_result",
        "commandId": "config-1",
        "accepted": True,
        "pending": False,
    }


def test_radius_change_mid_reposition_applies_now_instead_of_queueing(simulator):
    """Queueing it let the aircraft finish flying to a destination chosen for
    the previous radius -- outward across the new boundary when shrinking --
    and only replan on arrival, re-scoring every region from scratch rather
    than continuing toward the one it was heading for."""
    simulator.patrol_planner = PatrolPlanner(store=simulator.patrol_regions)
    simulator.realtime_client = Mock()
    simulator.logic.pursuit.config = replace(
        simulator.logic.pursuit.config, geofence_radius_m=270.0,
    )
    simulator.logic.start_reposition([(240.0, 0.0, 15.0)], FlightState.PATROLLING)

    simulator._handle_operating_config_command(
        {
            "commandId": "config-mid-leg",
            "params": {"maxFlightRadiusM": 200.0, "cruiseAltitudeM": 15.0},
        },
        (250.0, 0.0, 15.0),
    )

    result = simulator.realtime_client.publish_control_message.call_args.args[0]
    assert result["accepted"] is True
    assert result["pending"] is False
    assert simulator.logic.state is FlightState.REPOSITIONING
    assert simulator.logic.pursuit.config.geofence_radius_m == 200.0
    # The leg now ends at the old destination rescaled by 200/270, pulling the
    # aircraft back inside the new boundary instead of continuing out to 240.
    assert simulator.logic.reposition_destination[0] == pytest.approx(
        240.0 * 200.0 / 270.0
    )


def test_radius_change_during_takeoff_drops_patrol_route_planned_for_the_old_radius(simulator):
    """request_patrol() plans its entire first coverage route immediately, so
    a patrol ordered seconds before a radius change is already holding
    old-radius waypoints while the aircraft is still climbing. Keeping them
    sent the aircraft around the old perimeter -- outside the new boundary
    after a shrink -- before any new-radius planning happened."""
    simulator.patrol_planner = PatrolPlanner(store=simulator.patrol_regions)
    simulator.realtime_client = Mock()
    simulator.logic.state = FlightState.TAKING_OFF
    stale_route = [(9999.0, 9999.0, 15.0)] * 4
    simulator._patrol_route_waypoints = deque(stale_route)
    simulator._target_patrol_region_id = "N-1"

    simulator._handle_operating_config_command(
        {
            "commandId": "config-shrink",
            "params": {"maxFlightRadiusM": 120.0, "cruiseAltitudeM": 15.0},
        },
        POSITION,
    )

    assert simulator.realtime_client.publish_control_message.call_args.args[0]["accepted"] is True
    assert not simulator._patrol_route_waypoints
    assert simulator._target_patrol_region_id is None


def test_altitude_only_change_during_takeoff_keeps_the_planned_patrol_route(simulator):
    simulator.patrol_planner = PatrolPlanner(store=simulator.patrol_regions)
    simulator.realtime_client = Mock()
    simulator.logic.state = FlightState.TAKING_OFF
    radius = simulator.logic.pursuit.config.geofence_radius_m
    simulator._patrol_route_waypoints = deque([(30.0, 40.0, 15.0)])
    simulator._target_patrol_region_id = "N-1"

    simulator._handle_operating_config_command(
        {
            "commandId": "config-altitude",
            "params": {"maxFlightRadiusM": radius, "cruiseAltitudeM": 21.0},
        },
        POSITION,
    )

    assert simulator._target_patrol_region_id == "N-1"
    assert len(simulator._patrol_route_waypoints) == 1


def test_controller_rejects_operating_config_change_while_landing(simulator):
    simulator.patrol_planner = PatrolPlanner(store=simulator.patrol_regions)
    simulator.realtime_client = Mock()
    simulator.logic.state = FlightState.LANDING
    previous_config = simulator.logic.config
    previous_store = simulator.patrol_regions

    simulator._handle_operating_config_command(
        {
            "commandId": "config-2",
            "params": {"maxFlightRadiusM": 750.0, "cruiseAltitudeM": 15.25},
        },
        POSITION,
    )

    assert simulator.logic.config is previous_config
    assert simulator.patrol_regions is previous_store
    result = simulator.realtime_client.publish_control_message.call_args.args[0]
    assert result["accepted"] is False


def test_controller_operating_config_radius_change_mid_patrol_triggers_reposition(simulator):
    simulator.patrol_planner = PatrolPlanner(store=simulator.patrol_regions)
    simulator.realtime_client = Mock()
    old_radius = simulator.logic.pursuit.config.geofence_radius_m
    simulator.logic.state = FlightState.PATROLLING
    simulator.logic.target = WAYPOINT
    simulator._patrol_route_waypoints = deque([WAYPOINT])
    simulator._target_patrol_region_id = "N-1"

    simulator._handle_operating_config_command(
        {
            "commandId": "config-3",
            "params": {"maxFlightRadiusM": old_radius / 2.0, "cruiseAltitudeM": 15.25},
        },
        POSITION,
    )

    assert simulator.logic.state is FlightState.REPOSITIONING
    ratio = (old_radius / 2.0) / old_radius
    assert simulator.logic.target == pytest.approx(
        (POSITION[0] * ratio, POSITION[1] * ratio, 15.25)
    )
    # Stale waypoints/region tracking from the old radius must not survive.
    assert simulator._target_patrol_region_id is None
    assert not simulator._patrol_route_waypoints
    assert simulator.logic.pursuit.config.geofence_radius_m == old_radius / 2.0
    result = simulator.realtime_client.publish_control_message.call_args.args[0]
    assert result == {
        "type": "operating_config_result",
        "commandId": "config-3",
        "accepted": True,
        "pending": False,
    }


def test_controller_operating_config_change_mid_pursuit_is_deferred(simulator):
    simulator.patrol_planner = PatrolPlanner(store=simulator.patrol_regions)
    simulator.realtime_client = Mock()
    enter(simulator.logic)
    previous_config = simulator.logic.config

    simulator._handle_operating_config_command(
        {
            "commandId": "config-4",
            "params": {"maxFlightRadiusM": 750.0, "cruiseAltitudeM": 15.25},
        },
        POSITION,
    )

    assert simulator.logic.state is FlightState.TARGET_PURSUIT
    assert simulator.logic.config is previous_config
    assert simulator._pending_operating_config is not None
    result = simulator.realtime_client.publish_control_message.call_args.args[0]
    assert result == {
        "type": "operating_config_result",
        "commandId": "config-4",
        "accepted": True,
        "pending": True,
    }

    simulator.logic.complete_target_follow(POSITION)
    simulator.logic.update(POSITION, 0.1)  # drains TARGET_RETURN back to PATROLLING
    simulator._consume_pending_operating_config(POSITION)

    assert simulator._pending_operating_config is None
    assert simulator.logic.pursuit.config.geofence_radius_m == 750.0
    # Both halves of the pair must land, not just the radius: checking only
    # the radius here is what let a stale cruise altitude ship.
    assert simulator.logic.config.takeoff_altitude == 15.25
    assert simulator.logic.target is not None
    assert simulator.logic.target[2] == 15.25


def test_operating_config_change_while_repositioning_lands_radius_and_altitude(simulator):
    """Both halves of the pair must land on a leg that is already flying --
    an earlier version deferred the whole change here, and a version before
    that applied only the altitude and silently dropped the radius."""
    simulator.patrol_planner = PatrolPlanner(store=simulator.patrol_regions)
    simulator.realtime_client = Mock()
    simulator.logic.start_reposition([(200.0, 0.0, 15.0)], FlightState.PATROLLING)

    simulator._handle_operating_config_command(
        {
            "commandId": "config-5",
            "params": {"maxFlightRadiusM": 750.0, "cruiseAltitudeM": 15.25},
        },
        POSITION,
    )

    assert simulator.logic.state is FlightState.REPOSITIONING
    assert simulator.logic.config.takeoff_altitude == 15.25
    assert simulator.logic.pursuit.config.geofence_radius_m == 750.0
    assert max(region.outer_radius_m for region in simulator.patrol_regions.regions) == 750.0
    assert simulator._pending_operating_config is None
    result = simulator.realtime_client.publish_control_message.call_args.args[0]
    assert result["accepted"] is True
    assert result["pending"] is False


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
    # Back at the checkpoint, far inside the half-metre dead zone an ordinary
    # waypoint would have settled for -- but not bit-exact, which no real
    # position readback ever is. See FlightConfig.checkpoint_tolerance.
    assert math.dist(position, POSITION) <= simulator.logic.config.checkpoint_tolerance
    assert simulator.logic.state is FlightState.PATROLLING
    assert simulator.logic.target == WAYPOINT


def test_target_return_completes_when_readback_trails_the_commanded_pose(simulator):
    """The controller commands a pose, then reads the next one back off a
    physics body that has been stepped in between, so the two never match
    bit-exactly. An arrival gate demanding equality never opened: the aircraft
    hovered at the checkpoint in TARGET_RETURN forever at zero speed instead
    of resuming its patrol."""
    enter(simulator.logic)
    simulator.logic.complete_target_follow()
    position = (POSITION[0] + 0.1, POSITION[1], POSITION[2])
    simulator.logic.update(position, 0.1)
    assert simulator.logic.state is FlightState.TARGET_RETURN
    for _ in range(50):
        commanded, _, _, _ = simulator._apply_kinematic_flight(position, 0, 0.1)
        position = (commanded[0], commanded[1], commanded[2] - 1e-3)
        simulator.logic.update(position, 0.1)
        if simulator.logic.state is FlightState.PATROLLING:
            break
    assert simulator.logic.state is FlightState.PATROLLING


def test_set_patrol_priorities_replans_active_patrol_immediately(simulator):
    import time

    from services.patrol_planner import PatrolPlanner
    from services.polygon_coverage import PolygonCoveragePlanner

    simulator.patrol_regions = PatrolRegionStore()
    # Started well in the past, like a planner that's been running for a
    # while -- otherwise every region's unobserved age is ~0 at this instant
    # and the priority multiplier has nothing to multiply, letting sheer
    # proximity decide regardless of rank.
    simulator.patrol_planner = PatrolPlanner(
        store=simulator.patrol_regions, started_at=time.time() - 900.0
    )
    simulator.polygon_coverage_planner = PolygonCoveragePlanner()
    simulator.logic.state = FlightState.PATROLLING
    simulator._target_patrol_region_id = "N-1"
    # A long remaining coverage leg: without an immediate re-plan, the
    # planner would not be consulted again -- and the new priority would not
    # take effect -- until this whole queue drains naturally.
    stale_leg = [(9999.0, 9999.0, 15.0)] * 5
    simulator._patrol_route_waypoints = deque(stale_leg)
    simulator.realtime_client = Mock()
    simulator.realtime_client.drain_commands.return_value = [{
        "command": "set_patrol_priorities",
        "params": {"priorities": [{"regionId": "S-1", "rank": 1}]},
    }]

    simulator._handle_realtime_commands(POSITION)

    assert simulator._target_patrol_region_id == "S-1"
    assert list(simulator._patrol_route_waypoints) != stale_leg


def test_set_patrol_priorities_resumes_same_region_instead_of_replanning_it(simulator):
    import time

    from services.patrol_planner import PatrolPlanner
    from services.polygon_coverage import PolygonCoveragePlanner

    simulator.patrol_regions = PatrolRegionStore()
    simulator.patrol_planner = PatrolPlanner(
        store=simulator.patrol_regions, started_at=time.time() - 900.0
    )
    simulator.polygon_coverage_planner = PolygonCoveragePlanner()
    simulator.logic.state = FlightState.PATROLLING

    # Seed real, in-progress coverage of whichever region naturally wins first.
    first_waypoint = simulator._next_patrol_waypoint(POSITION, completed=False)
    simulator.logic.target = first_waypoint
    region_id = simulator._target_patrol_region_id
    remaining_before = list(simulator._patrol_route_waypoints)
    assert region_id is not None
    assert remaining_before, "test needs a region with more than one coverage leg left"

    # An edit that keeps this exact region on top (it's still the only ranked
    # one) must resume its exact remaining coverage, not discard and
    # re-plan a brand-new lawnmower pass over ground already covered.
    simulator.realtime_client = Mock()
    simulator.realtime_client.drain_commands.return_value = [{
        "command": "set_patrol_priorities",
        "params": {"priorities": [{"regionId": region_id, "rank": 1}]},
    }]

    simulator._handle_realtime_commands(POSITION)

    assert simulator._target_patrol_region_id == region_id
    assert simulator._paused_patrol_coverage is None
    assert list(simulator._patrol_route_waypoints) == [first_waypoint, *remaining_before]


def test_set_patrol_priorities_keeps_paused_region_when_another_wins(simulator):
    import time

    from services.patrol_planner import PatrolPlanner
    from services.polygon_coverage import PolygonCoveragePlanner

    simulator.patrol_regions = PatrolRegionStore()
    simulator.patrol_planner = PatrolPlanner(
        store=simulator.patrol_regions, started_at=time.time() - 900.0
    )
    simulator.polygon_coverage_planner = PolygonCoveragePlanner()
    simulator.logic.state = FlightState.PATROLLING

    first_waypoint = simulator._next_patrol_waypoint(POSITION, completed=False)
    simulator.logic.target = first_waypoint
    interrupted_region_id = simulator._target_patrol_region_id
    remaining_before = list(simulator._patrol_route_waypoints)
    assert remaining_before, "test needs a region with more than one coverage leg left"

    # Promote a different region above the one currently being covered.
    other_region_id = next(
        region.id for region in simulator.patrol_regions.regions
        if region.id != interrupted_region_id
    )
    simulator.realtime_client = Mock()
    simulator.realtime_client.drain_commands.return_value = [{
        "command": "set_patrol_priorities",
        "params": {"priorities": [{"regionId": other_region_id, "rank": 1}]},
    }]

    simulator._handle_realtime_commands(POSITION)

    assert simulator._target_patrol_region_id == other_region_id
    # The interrupted region's progress must survive the switch instead of
    # being silently discarded, so it can resume and still be marked
    # observed whenever scoring returns to it.
    paused = simulator._paused_patrol_coverage
    assert paused is not None
    assert paused.region_id == interrupted_region_id
    assert paused.active_waypoint == first_waypoint
    assert list(paused.remaining_waypoints) == remaining_before


def test_patrol_start_from_inside_cleared_roi_no_fly_cell_does_not_stick(simulator):
    """Reproduces: transiting to an ROI inside a NO_FLY cell, clearing the ROI,
    then requesting patrol without setting any priority -- the aircraft is
    left sitting inside that NO_FLY cell and must still route out to a
    legal patrol region instead of holding in place indefinitely.

    Region scoring across every candidate region is expensive (a full
    lawnmower coverage plan plus a no-fly route per candidate), so the
    aircraft flies clear of the NO_FLY cell first -- a cheap,
    destination-agnostic lookup -- and only plans the rest once already
    outside, on the next call once that leg is flown.
    """
    import time

    from services.patrol_planner import PatrolPlanner
    from services.polygon_coverage import PolygonCoveragePlanner

    simulator.patrol_regions = PatrolRegionStore()
    simulator.patrol_planner = PatrolPlanner(
        store=simulator.patrol_regions, started_at=time.time() - 900.0
    )
    simulator.polygon_coverage_planner = PolygonCoveragePlanner()
    simulator.logic.state = FlightState.PATROLLING

    radius_m = math.hypot(POSITION[0], POSITION[1])
    bearing_deg = math.degrees(math.atan2(POSITION[0], POSITION[1])) % 360.0
    containing_region = next(
        region for region in simulator.patrol_regions.regions
        if region.inner_radius_m <= radius_m <= region.outer_radius_m
        and (bearing_deg - region.start_bearing_deg) % 360.0
            <= (region.end_bearing_deg - region.start_bearing_deg) % 360.0
    )
    simulator.patrol_regions.set_area_type(containing_region.id, "NO_FLY")

    exit_waypoint = simulator._next_patrol_waypoint(POSITION, completed=False)

    # First call just clears the NO_FLY cell -- no region committed yet -- and
    # does so immediately rather than after scoring every candidate region.
    assert exit_waypoint != POSITION
    assert simulator._target_patrol_region_id is None

    # Reaching that exit leg triggers a fresh call (completed=True, as
    # FlightLogic does once _has_reached() the current target): now outside
    # the NO_FLY cell, this is where the real region gets chosen.
    waypoint = simulator._next_patrol_waypoint(exit_waypoint, completed=True)

    # A real, routed region was chosen -- not the "no safe coverage, hold in
    # place" fallback -- and it isn't the cell the aircraft started inside.
    assert simulator._target_patrol_region_id is not None
    assert simulator._target_patrol_region_id != containing_region.id
    assert waypoint != exit_waypoint


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


@pytest.mark.parametrize("roi", [False, True])
def test_manual_reject_command_restores_controller_mission_context(simulator, roi):
    simulator.realtime_client = Mock()
    simulator.camera_gimbal = Mock(pitch=0.5)
    simulator.logic.request_patrol(POSITION)
    simulator._target_patrol_region_id = "NE-2"
    simulator._patrol_route_waypoints = deque([(110, 55, 15), (120, 60, 15)])
    if roi:
        simulator.logic.request_roi(POSITION, (10, 20), 25)
        simulator.logic.state = FlightState.ROI_PATROL
        simulator.logic.roi_patrol_planner.reset((10, 20), 25, 15, POSITION)
        simulator.logic.target = simulator.logic.roi_patrol_planner.next_waypoint()
        simulator._active_roi_id = "roi:keep"
    simulator._capture_mission_interruption(POSITION)
    context = simulator._mission_interruption
    observe(simulator.logic)
    simulator.realtime_client.drain_commands.return_value = [{
        "command": "reject_target", "params": {"trackId": "one"},
    }]
    simulator._handle_realtime_commands(POSITION)
    simulator._coordinate_mission_resume(POSITION, simulator.logic.state)
    assert simulator._mission_interruption is None
    assert simulator.logic.state.value == context.source_state
    assert simulator.logic.target == context.active_waypoint
    if roi:
        assert simulator._active_roi_id == "roi:keep"
        assert simulator.logic.roi_patrol_planner.snapshot().remaining_path == context.remaining_path
    else:
        assert simulator._target_patrol_region_id == "NE-2"
        assert tuple(simulator._patrol_route_waypoints) == context.remaining_path


def test_ignored_location_cannot_capture_controller_checkpoint(simulator):
    simulator.camera_gimbal = Mock(pitch=0.5)
    simulator.realtime_client = Mock()
    simulator.logic.request_patrol(POSITION)
    item = {"trackId": "one", "trackState": "confirmed", "confidence": 0.9,
            "box": [450, 450, 550, 550], "lastSeenTimestamp": 0}
    message = {"frameWidth": 1000, "frameHeight": 1000, "detections": [item]}
    simulator.realtime_client.drain_detections.return_value = [message]
    simulator._handle_vision_detections(POSITION, 0)
    simulator.logic.reject_target("one")
    simulator._coordinate_mission_resume(POSITION, FlightState.TARGET_ACQUISITION)
    item["trackId"] = "replacement"
    simulator._handle_vision_detections(POSITION, 0)
    assert simulator._mission_interruption is None
    assert simulator.logic.state is FlightState.PATROLLING


@pytest.mark.parametrize("command", ["reject_target"])
def test_controller_safety_return_blocks_target_commands(simulator, command):
    simulator.logic.request_patrol(POSITION)
    observe(simulator.logic)
    simulator._safety_rth_active = True
    simulator.realtime_client = Mock()
    simulator.realtime_client.drain_commands.return_value = [{
        "command": command, "params": {"trackId": "one"},
    }]
    simulator._handle_realtime_commands(POSITION)
    assert simulator.logic.state is FlightState.TARGET_ACQUISITION
    assert not simulator.logic.target_decisions.ignored


def _controller_module(monkeypatch):
    """Load the Webots controller module with the simulator API stubbed out."""
    controller = ModuleType("controller")
    for name in ("Camera", "GPS", "InertialUnit", "Motor", "Supervisor", "Field"):
        setattr(controller, name, type(name, (), {}))
    monkeypatch.setitem(sys.modules, "controller", controller)
    monkeypatch.setitem(sys.modules, "flight_logic", flight_logic)
    directory = Path(__file__).resolve().parents[1] / "controllers" / "autonomous_drone"
    monkeypatch.syspath_prepend(str(directory))
    spec = importlib.util.spec_from_file_location(
        "controller_config_under_test", directory / "autonomous_drone.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_controller_env_defaults_build_valid_configs(monkeypatch):
    """The shipped environment defaults must satisfy every cross-field rule.

    The simulator fixture builds the controller through ``__new__`` and
    substitutes its own config, so nothing else exercises the defaults that
    ``__init__`` actually uses. A bad combination there is not a test failure
    but a controller that raises at startup and never connects.
    """
    module = _controller_module(monkeypatch)
    flight = module.build_flight_config()
    pursuit = module.build_pursuit_config(geofence_radius_m=2500.0)

    # The yaw band is split across two settings that are easy to move apart.
    assert flight.target_min_yaw_rate_rad_s <= flight.target_max_yaw_rate_rad_s
    assert pursuit.standoff_distance_m > 0.0


def test_controller_env_overrides_are_validated_not_silently_accepted(monkeypatch):
    module = _controller_module(monkeypatch)
    monkeypatch.setenv("DRONTECT_TARGET_MAX_YAW_RATE_DEG_S", "10.0")
    with pytest.raises(ValueError, match="target_min_yaw_rate_rad_s"):
        module.build_flight_config()
    monkeypatch.delenv("DRONTECT_TARGET_MAX_YAW_RATE_DEG_S")

    monkeypatch.setenv("DRONTECT_PURSUIT_DODGE_ENTRY_SECONDS", "4.0")
    with pytest.raises(ValueError, match="emergency < entry < exit"):
        module.build_pursuit_config(geofence_radius_m=2500.0)



@pytest.mark.parametrize("change_radius", [False, True])
def test_altitude_change_retargets_every_stored_waypoint(simulator, change_radius):
    """A new cruise altitude must reach waypoints already planned under the old one.

    The operator changes both values as one unit, so a radius change usually
    rides along and schedules a reposition leg. That leg is flown at the new
    altitude and is then followed by replaying the stored sweep, so retargeting
    cannot be the alternative to repositioning -- nor can it be gated on the
    current state, since a pursuit resumes into whatever it interrupted.
    """
    simulator.patrol_planner = PatrolPlanner(store=simulator.patrol_regions)
    simulator.realtime_client = Mock()

    old_altitude = 10.0
    simulator.logic.config = replace(
        simulator.logic.config, takeoff_altitude=old_altitude
    )
    planner = LawnMowerRoiPatrolPlanner()
    planner.reset((50.0, 50.0), 40.0, old_altitude, (50.0, 50.0))
    simulator.logic.roi_patrol_planner = planner
    simulator.logic.state = FlightState.ROI_PATROL
    simulator.logic.target = (50.0, 50.0, old_altitude)
    simulator._patrol_route_waypoints = deque(
        [(31.0, 41.0, old_altitude), (32.0, 42.0, old_altitude)]
    )

    radius = simulator.logic.pursuit.config.geofence_radius_m
    simulator._apply_operating_config(
        OperatingConfig(
            max_flight_radius_m=750.0 if change_radius else radius,
            cruise_altitude_m=19.0,
        ),
        POSITION,
    )

    assert simulator.logic.config.takeoff_altitude == 19.0
    roi_altitudes = {z for _, _, z in simulator.logic.roi_patrol_planner.snapshot().waypoints}
    assert roi_altitudes == {19.0}
    assert all(z == 19.0 for _, _, z in simulator._patrol_route_waypoints)


def test_altitude_change_mid_pursuit_retargets_the_parked_mission(simulator):
    """The interrupted mission is parked while pursuing, and must resume at the new altitude."""
    simulator.patrol_planner = PatrolPlanner(store=simulator.patrol_regions)
    simulator.realtime_client = Mock()

    old_altitude = 10.0
    simulator.logic.config = replace(
        simulator.logic.config, takeoff_altitude=old_altitude
    )
    active = (30.0, 40.0, old_altitude)
    remaining = ((31.0, 41.0, old_altitude), (32.0, 42.0, old_altitude))
    simulator.logic.state = FlightState.PATROLLING
    simulator.logic.target = active
    simulator._target_patrol_region_id = "NE-2"
    simulator._patrol_route_waypoints = deque(remaining)
    simulator._capture_mission_interruption(POSITION)
    assert simulator._mission_interruption is not None

    enter(simulator.logic)
    simulator._apply_operating_config(
        OperatingConfig(
            max_flight_radius_m=simulator.logic.pursuit.config.geofence_radius_m,
            cruise_altitude_m=19.0,
        ),
        POSITION,
    )

    # Resuming replays the parked progress: it must already be at 19 m.
    context = simulator._mission_interruption
    assert context is not None
    assert context.active_waypoint == (30.0, 40.0, 19.0)
    assert all(z == 19.0 for _, _, z in context.remaining_path)
    simulator._restore_mission_interruption(context)
    assert simulator.logic.target == (30.0, 40.0, 19.0)
    assert all(z == 19.0 for _, _, z in simulator._patrol_route_waypoints)


@pytest.mark.parametrize("pursue_first", [False, True])
def test_reject_during_roi_does_not_kill_the_controller(simulator, pursue_first):
    """Rejecting mid-ROI must not leave a flight state without its plan.

    Capture clears the ROI sweep, and reject ends tracking from the command
    handler -- which run() calls *before* logic.update(). Without resuming in
    the same tick, update() evaluated ROI_PATROL against an empty planner and
    raised out of the main loop, so the aircraft stopped being flown and fell.
    """
    simulator.realtime_client = Mock()
    simulator.camera_gimbal = Mock(pitch=0.5)
    simulator.logic.request_patrol(POSITION)
    simulator.logic.request_roi(POSITION, (10, 20), 25)
    simulator.logic.state = FlightState.ROI_PATROL
    simulator.logic.roi_patrol_planner.reset((10, 20), 25, 15, POSITION)
    simulator.logic.target = simulator.logic.roi_patrol_planner.next_waypoint()
    simulator._active_roi_id = "roi:keep"
    simulator._capture_mission_interruption(POSITION)

    if pursue_first:
        enter(simulator.logic)
    else:
        observe(simulator.logic)

    simulator.realtime_client.drain_commands.return_value = [
        {"command": "reject_target", "params": {"trackId": "one"}}
    ]
    simulator._handle_realtime_commands(POSITION)

    # Drive run()'s ordering: update() must survive the very next tick.
    for _ in range(40):
        here = simulator.logic.target or POSITION
        before = simulator.logic.state
        simulator.logic.update(here, 0.1)
        simulator._coordinate_mission_resume(here, before)

    assert simulator.logic.state is FlightState.ROI_PATROL
    assert simulator._mission_interruption is None
    assert simulator._active_roi_id == "roi:keep"
    assert simulator.logic.roi_patrol_planner.snapshot().waypoints


def test_roi_state_without_a_plan_degrades_instead_of_raising():
    """An exception out of update() does not pause the aircraft, it drops it."""
    logic = make_logic()
    logic.request_patrol(POSITION)
    logic.request_roi(POSITION, (10.0, 20.0), 25.0)
    logic.state = FlightState.ROI_PATROL
    logic.roi_patrol_planner.clear()

    logic.update(POSITION, 0.1)

    assert logic.state is FlightState.PATROLLING
    assert logic.target is not None


@pytest.mark.parametrize("command,params", [
    ("reject_target", {"trackId": "one"}),
    ("set_pursuit_block", {"blocked": True}),
])
@pytest.mark.parametrize("pursue_first", [True, False])
def test_command_that_ends_tracking_resumes_the_roi_not_area_patrol(
    simulator, command, params, pursue_first
):
    """Ending tracking from a command must restore the mission it interrupted.

    Commands are dispatched before this tick's logic.update(), while capture
    has left the ROI sweep cleared. Without resuming in the same tick, update()
    evaluates ROI_PATROL against an empty planner and the aircraft silently
    drops to area patrol -- taking the operator out of their ROI.
    """
    simulator.realtime_client = Mock()
    simulator.camera_gimbal = Mock(pitch=0.5)
    simulator.logic.request_patrol(POSITION)
    simulator.logic.request_roi(POSITION, (10, 20), 25)
    simulator.logic.state = FlightState.ROI_PATROL
    simulator.logic.roi_patrol_planner.reset((10, 20), 25, 15, POSITION)
    simulator.logic.target = simulator.logic.roi_patrol_planner.next_waypoint()
    simulator._active_roi_id = "roi:keep"
    simulator._capture_mission_interruption(POSITION)

    if pursue_first:
        enter(simulator.logic)
    else:
        observe(simulator.logic)

    simulator.realtime_client.drain_commands.return_value = [
        {"command": command, "params": params}
    ]
    simulator._handle_realtime_commands(POSITION)

    for _ in range(60):
        here = simulator.logic.target or POSITION
        before = simulator.logic.state
        simulator.logic.update(here, 0.1)
        simulator._coordinate_mission_resume(here, before)

    assert simulator.logic.state is FlightState.ROI_PATROL
    assert simulator._active_roi_id == "roi:keep"
    assert simulator.logic.roi_patrol_planner.snapshot().waypoints


@pytest.mark.parametrize("vx,turns_right", [(300, True), (-300, False)])
def test_lost_search_yaws_after_the_direction_the_target_was_going(vx, turns_right):
    """A target goes behind cover still moving; it reappears ahead, not where it left.

    Sweeping from where it vanished points the camera at the one place it will
    not be, so the bearing is dead-reckoned at the rate it was last crossing
    frame and the sweep widens the same way.
    """
    logic = make_logic()
    enter(logic)
    for _ in range(4):
        observe(logic, vx=vx)
        logic.update(POSITION, 0.1)
    yaw_at_loss = logic.target_yaw
    assert yaw_at_loss is not None

    logic.observe_detections(
        [], frame_width=1000, frame_height=1000, position=POSITION, yaw=0.0
    )
    for _ in range(4):
        logic.update(POSITION, 0.1)

    assert logic.diagnostic_state == "TARGET_LOST_SEARCH"
    swept = flight_logic.normalized_angle(logic.target_yaw - yaw_at_loss)
    # Image x grows rightward and yaw decreases to look right.
    assert (swept < 0) is turns_right


def test_lost_search_coasts_after_the_target_then_settles():
    """Keep pace with where it was going, but never fly off after a guess."""
    logic = make_logic()
    enter(logic)
    for _ in range(6):
        observe(logic, scale=0.9)
        logic.update(POSITION, 0.1)
    logic._last_target_world_velocity = (2.0, 0.0)

    logic.observe_detections(
        [], frame_width=1000, frame_height=1000, position=POSITION, yaw=0.0
    )
    logic.update(POSITION, 0.1)
    early = logic.pursuit.velocity[0]
    assert early > 0.0

    window = logic.config.target_lost_search_timeout_s
    while logic._elapsed_time - logic._target_last_seen_at < window - 0.15:
        logic.update(POSITION, 0.1)
    assert 0.0 <= logic.pursuit.velocity[0] < early


def test_lost_search_drift_respects_the_geofence():
    logic = make_logic(geofence_radius_m=30.0)
    enter(logic)
    edge = (30.0, 0.0, 15.0)
    observe(logic, position=edge)
    logic.update(edge, 0.1)
    logic._last_target_world_velocity = (6.0, 0.0)

    logic.observe_detections(
        [], frame_width=1000, frame_height=1000, position=edge, yaw=0.0
    )
    logic.update(edge, 0.1)

    assert logic.pursuit.velocity[:2] == (0.0, 0.0)
    assert logic.pursuit.constraint == "geofence"


def _take_manual(simulator):
    simulator.realtime_client.drain_commands.return_value = [
        {"command": "set_manual_control", "params": {"engaged": True}}
    ]
    simulator._handle_realtime_commands(POSITION)


def _give_back_manual(simulator, position):
    simulator.realtime_client.drain_commands.return_value = [
        {"command": "set_manual_control", "params": {"engaged": False}}
    ]
    simulator._handle_realtime_commands(position)


def test_releasing_manual_returns_the_aircraft_to_the_roi_it_was_working(simulator):
    """The operator expects the ROI back, not whatever patrol comes next.

    The sweep is replanned from wherever they left the aircraft rather than
    resumed from a waypoint it may now be a long way from.
    """
    simulator.realtime_client = Mock()
    simulator.logic.request_roi(POSITION, (10.0, 20.0), 25.0)
    simulator.logic.state = FlightState.ROI_PATROL
    simulator.logic.roi_patrol_planner.reset((10.0, 20.0), 25.0, 15, POSITION)

    _take_manual(simulator)
    assert simulator.logic.state is FlightState.MANUAL

    flown_to = (40.0, 60.0, 15.0)
    _give_back_manual(simulator, flown_to)

    assert simulator.logic.state in {
        FlightState.TRANSITING_TO_ROI,
        FlightState.ROI_PATROL,
    }
    assert simulator.logic.roi_patrol_planner.snapshot().waypoints


def test_releasing_manual_from_a_patrol_starts_a_fresh_one(simulator):
    """Half-flown patrol progress is worthless once a human has moved the aircraft.

    Resuming the old coverage route would send it back across ground it may
    have just been flown over by hand.
    """
    simulator.realtime_client = Mock()
    simulator.logic.state = FlightState.PATROLLING
    simulator._patrol_route_waypoints = deque([(1.0, 2.0, 15.0)])
    simulator._target_patrol_region_id = "region-1"

    _take_manual(simulator)
    _give_back_manual(simulator, (40.0, 60.0, 15.0))

    assert simulator.logic.state is FlightState.PATROLLING
    assert simulator._paused_patrol_coverage is None
    assert simulator._target_patrol_region_id is None
    assert simulator.logic.target == WAYPOINT


def test_a_safety_rth_takes_the_aircraft_off_the_operator(simulator):
    """Home outranks the operator, and nothing is handed back on the way."""
    simulator.realtime_client = Mock()
    simulator.logic.state = FlightState.PATROLLING
    _take_manual(simulator)

    simulator.realtime_client.drain_commands.return_value = [
        {
            "command": "return_home",
            "params": {},
            "safetyReason": "low_battery",
        }
    ]
    simulator._handle_realtime_commands(POSITION)

    assert simulator.logic.state is FlightState.RETURNING_HOME
    assert simulator.logic.manual_control is False
    assert simulator._safety_rth_active is True


def test_taking_manual_control_is_refused_during_a_safety_rth(simulator):
    """The takeover that could strand the aircraft is the one that is refused.

    The server refuses it too, but a connection-loss RTH is latched here
    before the server ever hears about it.
    """
    simulator.realtime_client = Mock()
    simulator.logic.state = FlightState.RETURNING_HOME
    simulator._safety_rth_active = True

    _take_manual(simulator)

    assert simulator.logic.manual_control is False
    assert simulator.logic.state is FlightState.RETURNING_HOME


def test_a_link_lost_while_the_operator_is_flying_still_brings_it_home(simulator):
    """The pad is on the far end of the link that just died."""
    simulator.realtime_client = Mock()
    simulator.logic.state = FlightState.PATROLLING
    _take_manual(simulator)

    simulator.realtime_client.connection_lost_for.return_value = True
    simulator._connection_loss_timeout_s = 3.0
    simulator._enforce_connection_loss_rth()

    assert simulator.logic.manual_control is False
    assert simulator.logic.state is FlightState.RETURNING_HOME
    assert simulator._safety_rth_reason == "connection_lost"


def test_a_stick_held_while_manual_flies_the_aircraft_and_releasing_it_hovers(simulator):
    """A held button moves the aircraft; letting go stops it in place."""
    simulator.realtime_client = Mock()
    simulator.logic.state = FlightState.PATROLLING
    _take_manual(simulator)

    simulator.realtime_client.drain_commands.return_value = [
        {
            "command": "set_manual_command",
            "params": {"forward": 1.0, "right": 0.0, "up": 0.0, "yawRate": 0.0},
        }
    ]
    simulator._handle_realtime_commands(POSITION)
    simulator._update_manual_stick()
    simulator.logic.update(POSITION, 0.1)
    moved, _, speed_mps, _ = simulator._apply_kinematic_flight(POSITION, 0.0, 0.1)
    assert moved[0] > POSITION[0]
    assert speed_mps > 0.0

    simulator.realtime_client.drain_commands.return_value = [
        {
            "command": "set_manual_command",
            "params": {"forward": 0.0, "right": 0.0, "up": 0.0, "yawRate": 0.0},
        }
    ]
    simulator._handle_realtime_commands(moved)
    simulator._update_manual_stick()
    simulator.logic.update(moved, 0.1)
    held, _, hover_speed_mps, _ = simulator._apply_kinematic_flight(moved, 0.0, 0.1)
    assert held == moved
    assert hover_speed_mps == 0.0


def test_a_tablet_that_goes_quiet_mid_stick_leaves_the_aircraft_hovering(simulator):
    """Silence is a released stick, not a held one."""
    simulator.realtime_client = Mock()
    simulator.logic.state = FlightState.PATROLLING
    _take_manual(simulator)

    simulator.realtime_client.drain_commands.return_value = [
        {
            "command": "set_manual_command",
            "params": {"forward": 1.0, "right": 0.0, "up": 0.0, "yawRate": 0.0},
        }
    ]
    simulator._handle_realtime_commands(POSITION)
    simulator._update_manual_stick()
    assert simulator.logic.manual_stick.forward == 1.0

    simulator._elapsed_time += simulator.manual_flight_config.command_timeout_s + 0.5
    simulator._update_manual_stick()
    assert simulator.logic.manual_stick.is_centered


def test_the_cockpit_rth_key_hands_the_aircraft_back_too(simulator):
    """The H key must end a takeover exactly as the RTH command does.

    Otherwise the aircraft flies itself home while telemetry still reports the
    operator is flying it, and the stick pad stays on screen over an aircraft
    nobody has the controls of.
    """
    simulator.realtime_client = Mock()
    simulator.keyboard = Mock()
    simulator.keyboard.getKey.side_effect = [ord("H"), -1]
    simulator.logic.state = FlightState.PATROLLING
    _take_manual(simulator)

    simulator.realtime_client.drain_commands.return_value = [
        {
            "command": "set_manual_command",
            "params": {"forward": 1.0, "right": 0.0, "up": 0.0, "yawRate": 0.0},
        }
    ]
    simulator._handle_realtime_commands(POSITION)
    simulator._handle_keyboard()
    simulator._update_manual_stick()

    assert simulator.logic.state is FlightState.RETURNING_HOME
    assert simulator.logic.manual_control is False
    assert simulator.logic.manual_stick.is_centered
    # The held stick is gone too, so a later takeover cannot inherit it.
    assert simulator.manual_input.stick(simulator._elapsed_time).is_centered


def test_a_failing_tick_does_not_stop_the_simulation(simulator, capsys):
    """One exception in the flight logic must not freeze the whole world.

    Webots waits on each synchronous controller, so a controller that raises
    stops calling step() and every other robot stops with it -- with the
    cause buried. Stepping on keeps the traceback visible and the rest of the
    simulation running, and the aircraft goes home rather than flying on
    logic that just failed.
    """
    steps = iter([0, 0, 0, -1])
    simulator.step = lambda _: next(steps)
    simulator.time_step = 32
    simulator.logic.state = FlightState.PATROLLING
    simulator._tick_failures = 0
    simulator._stop_motors = Mock()

    def explode(_dt):
        raise RuntimeError("target decision blew up")

    simulator._tick = explode
    simulator.run()

    assert simulator.logic.state is FlightState.RETURNING_HOME
    assert simulator._safety_rth_active is True
    assert simulator._safety_rth_reason == "flight_logic_error"
    # Every remaining step ran rather than the loop dying on the first one.
    assert simulator._tick_failures == 3
    assert "target decision blew up" in capsys.readouterr().err


def test_repeated_failures_stop_commanding_the_aircraft(simulator):
    """Past a point the logic is not coming back; stop flying on its output."""
    total = simulator.MAX_TICK_FAILURES + 2
    steps = iter([0] * total + [-1])
    simulator.step = lambda _: next(steps)
    simulator.time_step = 32
    simulator.logic.state = FlightState.PATROLLING
    simulator._tick_failures = 0
    simulator._stop_motors = Mock()

    def explode(_dt):
        raise RuntimeError("still broken")

    simulator._tick = explode
    simulator.run()

    assert simulator._stop_motors.called


def _hold_pursuit_during_search(simulator, position):
    simulator.realtime_client.drain_commands.return_value = [
        {"command": "set_pursuit_block", "params": {"blocked": True}}
    ]
    simulator._handle_realtime_commands(position)
    simulator.realtime_client.drain_commands.return_value = []


def test_an_unreachable_checkpoint_does_not_strand_the_aircraft(simulator):
    """Waiting for a route the aircraft cannot influence never ends.

    The return hovers while it looks for a safe way back to where the target
    interrupted the mission -- but hovering cannot change what that route
    depends on, so a refusal repeats every tick forever and the mission never
    comes back. The interruption point is worth less than the mission.
    """
    simulator.realtime_client = Mock()
    simulator.realtime_client.drain_commands.return_value = []
    simulator._mission_return_timeout_s = 2.0
    simulator.no_fly_router = Mock()
    simulator.no_fly_router.egress_route.side_effect = NoFlyRouteUnavailable("boxed in")

    simulator.logic.state = FlightState.PATROLLING
    enter(simulator.logic)
    simulator._capture_mission_interruption(POSITION)
    observe(simulator.logic)
    simulator.logic.observe_detections(
        [], frame_width=1000, frame_height=1000, position=POSITION, yaw=0.0
    )
    _hold_pursuit_during_search(simulator, POSITION)
    assert simulator.logic.state is FlightState.TARGET_RETURN

    for _ in range(60):
        before = simulator.logic.state
        simulator.logic.update(POSITION, 0.1)
        simulator._coordinate_mission_resume(POSITION, before)
        simulator._elapsed_time += 0.1
        if simulator.logic.state is not FlightState.TARGET_RETURN:
            break

    assert simulator.logic.state is FlightState.PATROLLING
    assert simulator._mission_interruption is None


def test_a_route_that_becomes_available_is_still_preferred(simulator):
    """The timeout is a last resort, not a replacement for going back."""
    simulator.realtime_client = Mock()
    simulator.realtime_client.drain_commands.return_value = []
    simulator._mission_return_timeout_s = 10.0
    simulator.no_fly_router = Mock()
    simulator.no_fly_router.egress_route.side_effect = [
        NoFlyRouteUnavailable("boxed in"),
        [(POSITION[0], POSITION[1]), (12.0, 22.0)],
    ]

    simulator.logic.state = FlightState.PATROLLING
    enter(simulator.logic)
    simulator._capture_mission_interruption(POSITION)
    observe(simulator.logic)
    simulator.logic.observe_detections(
        [], frame_width=1000, frame_height=1000, position=POSITION, yaw=0.0
    )
    _hold_pursuit_during_search(simulator, POSITION)

    before = simulator.logic.state
    simulator.logic.update(POSITION, 0.1)
    simulator._coordinate_mission_resume(POSITION, before)

    assert simulator.logic.state is FlightState.TARGET_RETURN
    assert simulator._mission_return_prepared is True
    assert simulator._mission_return_waiting_since is None
