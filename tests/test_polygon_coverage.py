import math

import pytest
from shapely.geometry import LineString, Polygon

from services.gps import (
    DEFAULT_HOME,
    LocalPosition,
    local_meters_to_wgs84,
    wgs84_to_local_meters,
)
from services.polygon_coverage import (
    PolygonCoverageConfig,
    PolygonCoveragePlanner,
)


def _geo(x: float, y: float):
    return local_meters_to_wgs84(LocalPosition(x=x, y=y, z=0.0), DEFAULT_HOME)


def _local_path(path):
    return [
        (local.x, local.y)
        for point in path
        for local in [wgs84_to_local_meters(point, DEFAULT_HOME)]
    ]


def _assert_segments_covered(path, free_space):
    for start, end in zip(path, path[1:]):
        # Geographic/local round-tripping introduces sub-millimeter noise.
        assert free_space.buffer(1e-3).covers(LineString([start, end]))


def test_rectangle_uses_long_axis_with_spaced_alternating_lanes():
    polygon = [_geo(0, 0), _geo(30, 0), _geo(30, 10), _geo(0, 10)]
    planner = PolygonCoveragePlanner(
        PolygonCoverageConfig(
            lane_spacing_m=3.0,
            boundary_inset_m=0.0,
            no_fly_clearance_m=1.0,
        )
    )

    path = _local_path(planner.plan(polygon, outward_from=_geo(15, -20)))

    assert len(path) == 8
    lane_ys = [path[index][1] for index in range(0, len(path), 2)]
    assert lane_ys == pytest.approx([0.5, 3.5, 6.5, 9.5], abs=1e-4)
    assert [lane_ys[index + 1] - lane_ys[index] for index in range(3)] == pytest.approx(
        [3.0, 3.0, 3.0], abs=1e-4
    )
    horizontal_directions = [
        math.copysign(1.0, path[index + 1][0] - path[index][0])
        for index in range(0, len(path), 2)
    ]
    assert horizontal_directions == [1.0, -1.0, 1.0, -1.0]
    _assert_segments_covered(path, Polygon([(0, 0), (30, 0), (30, 10), (0, 10)]))


def test_irregular_concave_polygon_keeps_every_segment_inside():
    local_polygon = [(0, 0), (32, 0), (32, 9), (20, 9), (20, 21), (0, 21)]
    polygon = [_geo(x, y) for x, y in local_polygon]
    planner = PolygonCoveragePlanner(
        PolygonCoverageConfig(
            lane_spacing_m=4.0,
            boundary_inset_m=0.0,
            no_fly_clearance_m=1.0,
        )
    )

    first = _local_path(planner.plan(polygon, outward_from=_geo(15, -30)))
    second = _local_path(planner.plan(polygon, outward_from=_geo(15, -30)))

    assert len(first) == len(second)
    for actual, expected in zip(first, second):
        assert actual == pytest.approx(expected, abs=1e-7)
    assert len(first) >= 8
    _assert_segments_covered(first, Polygon(local_polygon))


def test_no_fly_hole_is_buffered_and_ignore_geometry_is_not_subtracted():
    outer_local = [(0, 0), (40, 0), (40, 20), (0, 20)]
    no_fly_local = [(16, 6), (24, 6), (24, 14), (16, 14)]
    ignored_local = [(2, 2), (8, 2), (8, 8), (2, 8)]
    outer = [_geo(x, y) for x, y in outer_local]
    no_fly = [_geo(x, y) for x, y in no_fly_local]
    ignored = [_geo(x, y) for x, y in ignored_local]
    planner = PolygonCoveragePlanner(
        PolygonCoverageConfig(
            lane_spacing_m=4.0,
            boundary_inset_m=0.0,
            no_fly_clearance_m=1.0,
        )
    )

    path = _local_path(
        planner.plan(
            outer,
            no_fly_polygons=[no_fly],
            outward_from=_geo(20, -30),
        )
    )

    free_space = Polygon(outer_local).difference(Polygon(no_fly_local).buffer(1.0, quad_segs=2))
    _assert_segments_covered(path, free_space)
    assert any(
        Polygon(ignored_local).intersects(LineString([start, end]))
        for start, end in zip(path, path[1:])
    )
    repeated = _local_path(
        planner.plan(
            outer,
            no_fly_polygons=[no_fly],
            outward_from=_geo(20, -30),
        )
    )
    assert len(path) == len(repeated)
    for actual, expected in zip(path, repeated):
        assert actual == pytest.approx(expected, abs=1e-7)


@pytest.mark.parametrize("spacing", [0.0, -1.0, math.inf, math.nan])
def test_invalid_lane_spacing_is_rejected(spacing):
    with pytest.raises(ValueError):
        PolygonCoverageConfig(lane_spacing_m=spacing)
