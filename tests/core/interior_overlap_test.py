"""A chord that runs ALONG a hull edge does not enter the polygon.

`poly.intersection(line).length` measures the overlap with the CLOSED polygon
(interior UNION boundary), so an edge-following chord scores its whole
edge-following stretch — kilometres — while its interior overlap is zero. That
made the collision test reject exactly the merged chord the smoother needs in
order to drop a waypoint sitting on the edge (batch_random_test seeds 194, 257:
11533.475 m and 8597.245 m of "overlap", every metre of it on the boundary).

`interior_overlap_length` subtracts the boundary part, and both planners share
it with the oracle so there is one answer to "how far inside is this chord".
"""

from shapely.geometry import LineString, Polygon

from path_planning.validation import oracle as pv


SQUARE = Polygon([(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0), (0.0, 1000.0)])


def test_chord_along_an_edge_has_no_interior_overlap():
    along = LineString([(-500.0, 0.0), (1500.0, 0.0)])  # runs down the bottom edge
    assert SQUARE.intersection(along).length == 1000.0, (
        "closed overlap is the whole edge"
    )
    assert pv.interior_overlap_length(SQUARE, along) == 0.0


def test_a_real_crossing_is_measured_in_full():
    across = LineString(
        [(-500.0, 500.0), (1500.0, 500.0)]
    )  # straight through the middle
    assert pv.interior_overlap_length(SQUARE, across) == 1000.0


def test_edge_following_then_cutting_in_counts_only_the_penetration():
    """Along the bottom edge, then diagonally through the interior."""
    mixed = LineString([(0.0, 0.0), (600.0, 0.0), (600.0, 400.0)])
    assert pv.interior_overlap_length(SQUARE, mixed) == 400.0


def test_segment_clear_accepts_the_edge_follower_and_rejects_the_crossing():
    coords = list(SQUARE.exterior.coords)
    assert pv._segment_clear((-500.0, 0.0), (1500.0, 0.0), [], [coords]) is True
    assert pv._segment_clear((-500.0, 500.0), (1500.0, 500.0), [], [coords]) is False
