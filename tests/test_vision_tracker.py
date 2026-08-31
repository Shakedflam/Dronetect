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


def test_single_miss_marks_track_lost_but_keeps_it_alive():
    tracker = DroneTracker()

    tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=1.0, frame_width=640, frame_height=360)
    tracks = tracker.update([], timestamp=2.0, frame_width=640, frame_height=360)

    assert len(tracks) == 1
    assert tracks[0].track_id == "drone-1"
    assert tracks[0].track_state == "lost"
    assert tracks[0].last_seen_timestamp == 1.0


def test_track_is_removed_after_consecutive_misses():
    tracker = DroneTracker(max_consecutive_misses=2)

    tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=1.0, frame_width=640, frame_height=360)
    tracker.update([], timestamp=2.0, frame_width=640, frame_height=360)
    tracks = tracker.update([], timestamp=3.0, frame_width=640, frame_height=360)

    assert tracks == []


def test_tracker_estimates_center_velocity():
    tracker = DroneTracker()

    tracker.update([_detection((10.0, 20.0, 110.0, 120.0))], timestamp=1.0, frame_width=640, frame_height=360)
    tracks = tracker.update([_detection((20.0, 10.0, 120.0, 110.0))], timestamp=3.0, frame_width=640, frame_height=360)

    assert tracks[0].velocity_px_per_sec == pytest.approx((5.0, -5.0))


def _detection(box):
    return Detection(class_name="drone", confidence=0.87, box=box)
