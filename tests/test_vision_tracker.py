import pytest

from vision_server.detector import Detection
from vision_server.tracker import DroneTracker


def test_tracker_keeps_stable_id_for_matching_detection():
    tracker = DroneTracker()

    first = tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=1.0, frame_width=640, frame_height=360)
    second = tracker.update([_detection((14.0, 22.0, 114.0, 122.0))], timestamp=2.0, frame_width=640, frame_height=360)

    assert first[0].track_id == "drone-1"
    assert second[0].track_id == "drone-1"


def test_tracker_assigns_new_id_for_second_target():
    tracker = DroneTracker()

    tracks = tracker.update(
        [
            _detection((10.0, 20.0, 110.0, 120.0)),
            _detection((300.0, 40.0, 360.0, 100.0)),
        ],
        timestamp=1.0,
        frame_width=640,
        frame_height=360,
    )

    assert [track.track_id for track in tracks] == ["drone-1", "drone-2"]


def test_tracker_matches_by_proximity_when_boxes_do_not_overlap():
    tracker = DroneTracker()

    tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=1.0, frame_width=640, frame_height=360)
    tracks = tracker.update([_detection((112.0, 20.0, 212.0, 120.0))], timestamp=2.0, frame_width=640, frame_height=360)

    assert len(tracks) == 1
    assert tracks[0].track_id == "drone-1"
    assert tracks[0].track_state == "confirmed"


def test_tentative_track_becomes_confirmed_after_second_hit():
    tracker = DroneTracker()

    first = tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=1.0, frame_width=640, frame_height=360)
    second = tracker.update([_detection((12.0, 20.0, 112.0, 120.0))], timestamp=2.0, frame_width=640, frame_height=360)

    assert first[0].track_state == "tentative"
    assert second[0].track_state == "confirmed"


def test_small_fast_sweep_detection_confirms_with_frame_relative_gate():
    tracker = DroneTracker()

    first = tracker.update([_detection((100.0, 80.0, 120.0, 100.0))], timestamp=1.0, frame_width=640, frame_height=360)
    second = tracker.update([_detection((220.0, 94.0, 240.0, 114.0))], timestamp=1.3, frame_width=640, frame_height=360)

    assert first[0].track_id == "drone-1"
    assert second[0].track_id == "drone-1"
    assert second[0].track_state == "confirmed"


def test_frame_relative_gate_still_rejects_distant_unrelated_detection():
    tracker = DroneTracker()

    tracker.update([_detection((100.0, 80.0, 120.0, 100.0))], timestamp=1.0, frame_width=640, frame_height=360)
    tracks = tracker.update([_detection((400.0, 220.0, 420.0, 240.0))], timestamp=1.3, frame_width=640, frame_height=360)

    assert [track.track_id for track in tracks] == ["drone-2", "drone-1"]
    assert tracks[0].track_state == "tentative"
    assert tracks[1].track_state == "lost"


def test_single_miss_marks_track_lost_but_keeps_it_alive():
    tracker = DroneTracker()

    tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=1.0, frame_width=640, frame_height=360)
    tracks = tracker.update([], timestamp=1.3, frame_width=640, frame_height=360)

    assert len(tracks) == 1
    assert tracks[0].track_id == "drone-1"
    assert tracks[0].track_state == "lost"
    assert tracks[0].last_seen_timestamp == 1.0


def test_track_is_removed_after_elapsed_timeout():
    tracker = DroneTracker(lost_after_seconds=0.25, remove_after_seconds=1.0)

    tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=1.0, frame_width=640, frame_height=360)
    tracker.update([], timestamp=1.5, frame_width=640, frame_height=360)
    tracks = tracker.update([], timestamp=2.0, frame_width=640, frame_height=360)

    assert tracks == []


def test_tracker_estimates_center_velocity():
    tracker = DroneTracker()

    tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=1.0, frame_width=640, frame_height=360)
    tracks = tracker.update([_detection((20.0, 10.0, 120.0, 110.0))], timestamp=3.0, frame_width=640, frame_height=360)

    assert tracks[0].velocity_px_per_sec == pytest.approx((5.0, -5.0))


def test_tracker_velocity_uses_variable_detection_intervals():
    tracker = DroneTracker()

    tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=1.0, frame_width=640, frame_height=360)
    tracks = tracker.update([_detection((40.0, 20.0, 140.0, 120.0))], timestamp=1.5, frame_width=640, frame_height=360)

    assert tracks[0].velocity_px_per_sec == pytest.approx((60.0, 0.0))


def test_zero_or_invalid_dt_does_not_break_tracker():
    tracker = DroneTracker()

    tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=2.0, frame_width=640, frame_height=360)
    tracks = tracker.update([_detection((40.0, 20.0, 140.0, 120.0))], timestamp=2.0, frame_width=640, frame_height=360)
    earlier_tracks = tracker.update([_detection((70.0, 20.0, 170.0, 120.0))], timestamp=1.5, frame_width=640, frame_height=360)

    assert tracks[0].velocity_px_per_sec == (0.0, 0.0)
    assert earlier_tracks[0].velocity_px_per_sec == (0.0, 0.0)
    assert earlier_tracks[0].last_seen_timestamp == 2.0


def test_prediction_advances_track_in_velocity_direction_during_gap():
    tracker = DroneTracker(lost_after_seconds=0.5, remove_after_seconds=2.0)

    tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=1.0, frame_width=640, frame_height=360)
    tracker.update([_detection((30.0, 20.0, 130.0, 120.0))], timestamp=2.0, frame_width=640, frame_height=360)
    tracks = tracker.update([], timestamp=2.25, frame_width=640, frame_height=360)

    assert tracks[0].box == pytest.approx((35.0, 20.0, 135.0, 120.0))
    assert tracks[0].track_state == "confirmed"


def test_association_uses_predicted_position():
    tracker = DroneTracker(max_center_distance_diagonals=0.5)

    tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=1.0, frame_width=640, frame_height=360)
    tracker.update([_detection((60.0, 20.0, 160.0, 120.0))], timestamp=2.0, frame_width=640, frame_height=360)
    tracks = tracker.update([_detection((110.0, 20.0, 210.0, 120.0))], timestamp=3.0, frame_width=640, frame_height=360)

    assert len(tracks) == 1
    assert tracks[0].track_id == "drone-1"


def test_confirmed_track_survives_temporary_detector_gap():
    tracker = DroneTracker(lost_after_seconds=0.25, remove_after_seconds=0.8)

    tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=1.0, frame_width=640, frame_height=360)
    tracker.update([_detection((20.0, 20.0, 120.0, 120.0))], timestamp=1.1, frame_width=640, frame_height=360)
    tracks = tracker.update([], timestamp=1.2, frame_width=640, frame_height=360)

    assert len(tracks) == 1
    assert tracks[0].track_id == "drone-1"
    assert tracks[0].track_state == "confirmed"


def test_lost_and_removed_states_are_time_based():
    tracker = DroneTracker(lost_after_seconds=0.25, remove_after_seconds=0.8)

    tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=1.0, frame_width=640, frame_height=360)
    lost = tracker.update([], timestamp=1.3, frame_width=640, frame_height=360)
    removed = tracker.update([], timestamp=1.8, frame_width=640, frame_height=360)

    assert lost[0].track_state == "lost"
    assert removed == []


def _detection(box):
    return Detection(class_name="drone", confidence=0.87, box=box)
