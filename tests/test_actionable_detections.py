import pytest

from services.actionable_detections import ActionableDetectionConfig, ActionableDetectionFilter

W, H = 960, 540


def det(track_id="t1", confidence=0.9, box=(400, 200, 480, 260), state="confirmed"):
    return {"trackId": track_id, "trackState": state, "confidence": confidence,
            "box": list(box), "className": "drone"}


def make_filter(**config):
    logs: list[str] = []
    return ActionableDetectionFilter(ActionableDetectionConfig(**config), log=logs.append), logs


def frame(f, *detections, passthrough=None):
    return f.update(list(detections), frame_width=W, frame_height=H,
                    passthrough_track_id=passthrough)


def test_one_frame_false_positive_is_never_actionable():
    f, logs = make_filter()
    assert frame(f, det(confidence=0.95)).actionable == []
    for _ in range(5):
        assert frame(f).actionable == []
    assert not f.is_actionable("t1")
    assert any("reason=not_persistent frames=1/3 lastConf=0.95" in line for line in logs)


def test_three_consecutive_frames_make_a_track_actionable():
    f, logs = make_filter()
    assert frame(f, det(confidence=0.81)).actionable == []
    assert frame(f, det(confidence=0.82)).actionable == []
    result = frame(f, det(confidence=0.83))
    assert [d["trackId"] for d in result.actionable] == ["t1"]
    assert result.newly_accepted == ("t1",)
    assert any(line.startswith("detection=accepted track=t1 conf=0.83 frames=3/3") for line in logs)


@pytest.mark.parametrize("required", [3, 4, 5])
def test_required_frames_is_configurable(required):
    f, _ = make_filter(required_frames=required)
    for _ in range(required - 1):
        assert frame(f, det()).actionable == []
    assert frame(f, det()).actionable


def test_one_missed_frame_does_not_reset_the_streak():
    f, _ = make_filter()
    frame(f, det())
    frame(f, det())
    frame(f)  # a single dropped vision frame
    assert frame(f, det()).actionable  # third hit -> actionable


def test_two_missed_frames_reset_the_streak():
    f, _ = make_filter()
    frame(f, det())
    frame(f, det())
    frame(f)
    frame(f)
    assert frame(f, det()).actionable == []
    assert frame(f, det()).actionable == []
    assert frame(f, det()).actionable


def test_actionable_track_survives_one_miss_then_is_released():
    f, logs = make_filter()
    for _ in range(3):
        frame(f, det())
    frame(f)
    assert f.is_actionable("t1")
    assert frame(f, det()).actionable
    frame(f)
    frame(f)
    assert not f.is_actionable("t1")
    assert any("detection=released track=t1 reason=missed_frames" in line for line in logs)


def test_confidence_threshold_starts_at_0_6():
    f, logs = make_filter()
    for _ in range(5):
        assert frame(f, det(confidence=0.59)).actionable == []
    assert any("reason=low_confidence conf=0.59" in line for line in logs)
    g, _ = make_filter()
    for _ in range(2):
        frame(g, det(confidence=0.6))
    assert frame(g, det(confidence=0.6)).actionable


def test_confidence_hysteresis_keeps_an_actionable_track():
    f, _ = make_filter()
    for _ in range(3):
        frame(f, det(confidence=0.9))
    assert frame(f, det(confidence=0.55)).actionable  # above keep_confidence 0.5
    # A brand-new track at the same confidence still has to clear 0.6.
    for _ in range(5):
        assert [d["trackId"] for d in frame(f, det(confidence=0.9), det("t2", 0.55)).actionable] == ["t1"]


@pytest.mark.parametrize(("box", "state", "reason"), [
    ((400, 200, 404, 204), "confirmed", "box_too_small"),       # 4x4 px
    ((400, 200, 409, 209), "confirmed", "box_too_small"),       # 9x9 px, under the area floor
    ((0, 0, 900, 500), "confirmed", "box_too_large"),
    ((100, 200, 700, 260), "confirmed", "box_implausible_aspect"),  # 10:1
    ((480, 260, 400, 200), "confirmed", "box_invalid"),
    ((2000, 2000, 2080, 2060), "confirmed", "box_outside_frame"),
    ((400, 200, 480, 260), "lost", "track_lost"),
])
def test_implausible_or_lost_boxes_are_never_actionable(box, state, reason):
    f, logs = make_filter()
    for _ in range(6):
        assert frame(f, det(box=box, state=state)).actionable == []
    assert any(f"reason={reason}" in line for line in logs)


def test_rejection_is_logged_once_per_reason_not_every_frame():
    f, logs = make_filter()
    for _ in range(20):
        frame(f, det(confidence=0.3))
    assert len([line for line in logs if "reason=low_confidence" in line]) == 1
    assert not f.is_actionable("t1")


def test_tracked_target_passes_through_unfiltered():
    f, _ = make_filter()
    weak = det(confidence=0.45, state="tentative")
    result = frame(f, weak, passthrough="t1")
    assert result.actionable == [weak]


def test_reset_forgets_all_streaks():
    f, _ = make_filter()
    for _ in range(3):
        frame(f, det())
    f.reset()
    assert frame(f, det()).actionable == []
