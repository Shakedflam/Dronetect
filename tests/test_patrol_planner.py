import math

import pytest

from services.gps import DEFAULT_HOME
from services.patrol_planner import (
    PatrolPlanner,
    PatrolRegionScorer,
    PatrolScoringConfig,
)
from services.patrol_regions import PatrolAreaType, PatrolRegionStore


def test_priority_rank_increases_score_for_equal_age_and_distance():
    store = PatrolRegionStore()
    ranked = store.set_priority("N-1", 1)
    unranked = store.get("S-1")
    scorer = PatrolRegionScorer()
    current = DEFAULT_HOME

    ranked_score = scorer.score(
        ranked, current, now=100.0, unobserved_since=0.0
    )
    unranked_score = scorer.score(
        unranked, current, now=100.0, unobserved_since=0.0
    )

    assert ranked_score.distance_m == pytest.approx(unranked_score.distance_m)
    assert ranked_score.priority_multiplier == pytest.approx(4.0)
    assert ranked_score.total > unranked_score.total


def test_rank_to_weight_conversion_is_defined_by_the_scorer():
    scorer = PatrolRegionScorer(PatrolScoringConfig(priority_boost=3.0))

    assert scorer.priority_multiplier(None) == pytest.approx(1.0)
    assert scorer.priority_multiplier(1) == pytest.approx(4.0)
    assert scorer.priority_multiplier(2) == pytest.approx(2.5)


def test_travel_cost_prefers_closer_region_when_age_and_priority_match():
    store = PatrolRegionStore()
    near = store.get("N-1")
    far = store.get("N-4")
    scorer = PatrolRegionScorer()

    near_score = scorer.score(
        near, DEFAULT_HOME, now=100.0, unobserved_since=0.0
    )
    far_score = scorer.score(
        far, DEFAULT_HOME, now=100.0, unobserved_since=0.0
    )

    assert near_score.age_seconds == far_score.age_seconds
    assert near_score.priority_multiplier == far_score.priority_multiplier
    assert near_score.distance_m < far_score.distance_m
    assert near_score.total > far_score.total


def test_old_low_priority_region_eventually_beats_recent_high_priority_region():
    store = PatrolRegionStore()
    high = store.set_priority("N-1", 1)
    store.record_observation(high.id, observed_at=990.0)
    low = store.get("S-1")
    scorer = PatrolRegionScorer()

    high_score = scorer.score(
        store.get(high.id), DEFAULT_HOME, now=1000.0, unobserved_since=0.0
    )
    low_score = scorer.score(
        low, DEFAULT_HOME, now=1000.0, unobserved_since=0.0
    )

    assert low_score.priority_multiplier < high_score.priority_multiplier
    assert low_score.total > high_score.total


def test_repeated_planning_eventually_visits_every_region():
    store = PatrolRegionStore()
    store.set_priority("N-1", 1)
    planner = PatrolPlanner(store=store, started_at=0.0)
    current = DEFAULT_HOME
    visited: set[str] = set()

    for step in range(1, 257):
        now = step * 1000.0
        decision = planner.choose_next(current, now=now)
        visited.add(decision.region.id)
        current = decision.region.center
        planner.record_observation(decision.region.id, observed_at=now)
        if len(visited) == len(store.regions):
            break

    assert visited == {region.id for region in store.regions}


def test_recording_observation_updates_store_and_reduces_revisit_score():
    planner = PatrolPlanner(started_at=0.0)
    decision = planner.choose_next(DEFAULT_HOME, now=1000.0)
    before = decision.score.total

    updated = planner.record_observation(decision.region.id, observed_at=1000.0)
    after = planner.scorer.score(
        updated,
        DEFAULT_HOME,
        now=1000.0,
        unobserved_since=planner.started_at,
    )

    assert updated.last_observed == pytest.approx(1000.0)
    assert after.age_seconds == 0.0
    assert after.total < before


def test_selection_does_not_mark_region_observed():
    planner = PatrolPlanner(started_at=10.0)

    decision = planner.choose_next(DEFAULT_HOME, now=20.0)

    assert decision.score.age_seconds == pytest.approx(10.0)
    assert planner.store.get(decision.region.id).last_observed is None


def test_tie_breaking_is_stable_for_identical_inputs():
    planner = PatrolPlanner(started_at=0.0)

    first = planner.choose_next(DEFAULT_HOME, now=100.0)
    second = planner.choose_next(DEFAULT_HOME, now=100.0)

    assert second.region.id == first.region.id
    assert second.score == first.score


def test_changed_priority_set_affects_next_selection_without_new_planner():
    store = PatrolRegionStore()
    planner = PatrolPlanner(store=store, started_at=0.0)

    before = planner.choose_next(DEFAULT_HOME, now=1000.0)
    store.replace_priorities([("N-4", 1)])
    after = planner.choose_next(DEFAULT_HOME, now=1000.0)

    assert before.region.band_index == 1
    assert after.region.id == "N-4"


def test_patrol_area_is_selectable():
    store = PatrolRegionStore()
    store.replace_area_types(
        [
            (region.id, PatrolAreaType.IGNORE)
            for region in store.regions
            if region.id != "SE-3"
        ]
    )
    planner = PatrolPlanner(store=store, started_at=0.0)

    assert planner.choose_next(DEFAULT_HOME, now=100.0).region.id == "SE-3"


def test_ignore_area_is_not_selectable():
    store = PatrolRegionStore()
    store.set_area_type("N-1", PatrolAreaType.IGNORE)
    store.set_priority("N-1", 1)
    planner = PatrolPlanner(store=store, started_at=0.0)

    assert planner.choose_next(DEFAULT_HOME, now=100.0).region.id != "N-1"


def test_no_fly_area_is_not_selectable():
    store = PatrolRegionStore()
    store.set_area_type("N-1", PatrolAreaType.NO_FLY)
    store.set_priority("N-1", 1)
    planner = PatrolPlanner(store=store, started_at=0.0)

    assert planner.choose_next(DEFAULT_HOME, now=100.0).region.id != "N-1"


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("age_weight", -1.0),
        ("priority_boost", math.inf),
        ("travel_weight", math.nan),
        ("nominal_travel_speed_mps", 0.0),
    ],
)
def test_invalid_scoring_configuration_is_rejected(field, value):
    values = {
        "age_weight": 1.0,
        "priority_boost": 3.0,
        "travel_weight": 1.0,
        "nominal_travel_speed_mps": 7.0,
    }
    values[field] = value

    with pytest.raises(ValueError, match=field):
        PatrolScoringConfig(**values)


@pytest.mark.parametrize("timestamp", [-1.0, math.inf, math.nan, True])
def test_invalid_planner_timestamps_are_rejected(timestamp):
    planner = PatrolPlanner(started_at=0.0)

    with pytest.raises(ValueError, match="timestamp"):
        planner.choose_next(DEFAULT_HOME, now=timestamp)


def test_unknown_region_observation_is_rejected():
    planner = PatrolPlanner(started_at=0.0)

    with pytest.raises(KeyError, match="Unknown patrol region"):
        planner.record_observation("missing", observed_at=1.0)
