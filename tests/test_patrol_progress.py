from services.patrol_progress import PausedPatrolCoverage


def test_resume_returns_to_interruption_before_unfinished_active_waypoint():
    progress = PausedPatrolCoverage(
        region_id="NE-2",
        interruption_position=(12.0, 4.0, 15.0),
        active_waypoint=(20.0, 4.0, 15.0),
        remaining_waypoints=((20.0, 8.0, 15.0), (5.0, 8.0, 15.0)),
    )

    resumed = progress.resumed_waypoints(
        ((7.0, 1.0, 15.0), (12.0, 4.0, 15.0))
    )

    assert resumed == (
        (7.0, 1.0, 15.0),
        (12.0, 4.0, 15.0),
        (20.0, 4.0, 15.0),
        (20.0, 8.0, 15.0),
        (5.0, 8.0, 15.0),
    )


def test_resume_does_not_drop_active_waypoint_when_interrupted_at_lane_start():
    progress = PausedPatrolCoverage(
        region_id="N-1",
        interruption_position=(0.0, 0.0, 15.0),
        active_waypoint=(30.0, 0.0, 15.0),
        remaining_waypoints=(),
    )

    assert progress.resumed_waypoints(()) == (
        (0.0, 0.0, 15.0),
        (30.0, 0.0, 15.0),
    )
