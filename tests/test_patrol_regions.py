import math

import pytest

from services.gps import DEFAULT_HOME
from services.patrol_regions import (
    PATROL_BAND_COUNT,
    PATROL_DIRECTIONS,
    PatrolAreaType,
    PatrolRegionStore,
    generate_patrol_regions,
    patrol_band_outer_radii_m,
    patrol_region_polygon_points,
)


def test_generates_32_regions_covering_every_direction_and_band():
    regions = generate_patrol_regions()

    assert len(regions) == 32
    ids = {region.id for region in regions}
    for _, label in PATROL_DIRECTIONS:
        for band_index in range(1, PATROL_BAND_COUNT + 1):
            assert f"{label}-{band_index}" in ids


def test_bands_cover_approximately_equal_area():
    outer_radii = patrol_band_outer_radii_m(2500.0)
    inner_radii = [0.0, *outer_radii[:-1]]

    areas = [
        math.pi * (outer**2 - inner**2)
        for inner, outer in zip(inner_radii, outer_radii)
    ]

    for area in areas[1:]:
        assert area == pytest.approx(areas[0], rel=1e-9)


def test_2500_meter_band_boundaries_match_equal_area_proportions():
    assert patrol_band_outer_radii_m(2500.0) == pytest.approx(
        [1250.0, 1767.76695297, 2165.06350946, 2500.0]
    )


def test_band_radii_scale_proportionately_when_home_radius_grows():
    base_radii = patrol_band_outer_radii_m(2500.0)

    for factor in (0.1, 2.0, 5.0):
        scaled_radii = patrol_band_outer_radii_m(2500.0 * factor)
        for base, scaled in zip(base_radii, scaled_radii):
            assert scaled == pytest.approx(base * factor)


def test_first_band_starts_at_home_and_last_band_ends_at_home_radius():
    regions = generate_patrol_regions(home_radius_m=2500.0)

    innermost = [r for r in regions if r.band_index == 1]
    outermost = [r for r in regions if r.band_index == PATROL_BAND_COUNT]

    assert all(region.inner_radius_m == 0.0 for region in innermost)
    assert all(
        region.outer_radius_m == pytest.approx(2500.0) for region in outermost
    )


def test_north_region_bearings_wrap_around_zero():
    regions = generate_patrol_regions()
    north_band_1 = next(r for r in regions if r.id == "N-1")

    assert north_band_1.start_bearing_deg == pytest.approx(337.5)
    assert north_band_1.end_bearing_deg == pytest.approx(22.5)


def test_region_center_sits_on_its_direction_bearing_from_home():
    regions = generate_patrol_regions()
    east_band_1 = next(r for r in regions if r.id == "E-1")

    assert east_band_1.center.lat == pytest.approx(DEFAULT_HOME.lat)
    assert east_band_1.center.lng > DEFAULT_HOME.lng


def test_region_polygon_traces_outer_and_inner_sector_boundaries():
    region = next(region for region in generate_patrol_regions() if region.id == "E-2")

    points = patrol_region_polygon_points(region, arc_step_deg=5.0)

    assert len(points) == 20
    assert points == patrol_region_polygon_points(region, arc_step_deg=5.0)


def test_priority_rank_defaults_to_unranked():
    regions = generate_patrol_regions()

    assert all(region.priority_rank is None for region in regions)


def test_regions_default_to_patrol_area_type():
    regions = generate_patrol_regions()

    assert all(region.area_type is PatrolAreaType.PATROL for region in regions)


def test_replacing_area_types_defaults_omitted_regions_back_to_patrol():
    store = PatrolRegionStore()
    store.replace_area_types(
        [("N-1", PatrolAreaType.IGNORE), ("N-2", PatrolAreaType.NO_FLY)]
    )

    store.replace_area_types([("N-2", PatrolAreaType.NO_FLY)])

    assert store.get("N-1").area_type is PatrolAreaType.PATROL
    assert store.get("N-2").area_type is PatrolAreaType.NO_FLY


def test_non_patrol_area_loses_priority_and_later_ranks_are_compacted():
    store = PatrolRegionStore()
    store.replace_priorities([("N-1", 1), ("N-2", 2), ("N-3", 3), ("N-4", 4)])

    store.set_area_type("N-2", PatrolAreaType.IGNORE)

    assert store.get("N-1").priority == 1
    assert store.get("N-2").priority is None
    assert store.get("N-3").priority == 2
    assert store.get("N-4").priority == 3


def test_regions_have_stable_ids_centers_priority_and_observation_fields():
    first = generate_patrol_regions()
    second = generate_patrol_regions()

    assert [region.id for region in first] == [region.id for region in second]
    assert all(region.center is not None for region in first)
    assert all(region.priority is None for region in first)
    assert all(region.last_observed is None for region in first)


def test_store_owns_priority_and_last_observed_without_changing_geometry():
    store = PatrolRegionStore()
    original = store.get("NE-3")

    prioritized = store.set_priority("NE-3", 2)
    observed = store.record_observation("NE-3", observed_at=1234.5)

    assert prioritized.priority == 2
    assert observed.priority == 2
    assert observed.last_observed == pytest.approx(1234.5)
    assert observed.id == original.id
    assert observed.center == original.center
    assert observed.inner_radius_m == original.inner_radius_m
    assert observed.outer_radius_m == original.outer_radius_m


@pytest.mark.parametrize("radius", [0.0, -1.0, math.inf, math.nan])
def test_invalid_home_radius_is_rejected(radius):
    with pytest.raises(ValueError, match="home_radius_m"):
        generate_patrol_regions(home_radius_m=radius)


@pytest.mark.parametrize("priority", [0, -1, 1.5, True])
def test_invalid_priority_is_rejected(priority):
    store = PatrolRegionStore()

    with pytest.raises(ValueError, match="priority"):
        store.set_priority("N-1", priority)


def test_unknown_region_cannot_be_updated():
    store = PatrolRegionStore()

    with pytest.raises(KeyError, match="Unknown patrol region"):
        store.record_observation("missing", observed_at=1.0)


def test_replace_priorities_clears_old_ranks_and_accepts_empty_set():
    store = PatrolRegionStore()
    store.replace_priorities([("N-4", 1), ("NE-4", 2)])

    store.replace_priorities([("S-4", 1)])

    assert store.get("N-4").priority is None
    assert store.get("NE-4").priority is None
    assert store.get("S-4").priority == 1

    store.replace_priorities([])
    assert all(region.priority is None for region in store.regions)


@pytest.mark.parametrize(
    "priorities",
    [
        [("missing", 1)],
        [("N-4", 1), ("N-4", 2)],
        [("N-4", 1), ("NE-4", 1)],
        [("N-4", 1), ("NE-4", 3)],
        [("N-4", True)],
        [("N-4", 1.0)],
    ],
)
def test_replace_priorities_rejects_invalid_sets_atomically(priorities):
    store = PatrolRegionStore()
    store.replace_priorities([("S-4", 1)])

    with pytest.raises((KeyError, ValueError)):
        store.replace_priorities(priorities)

    assert store.get("S-4").priority == 1
    assert sum(region.priority is not None for region in store.regions) == 1
