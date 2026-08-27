"""The bbox prefilters must not change a single verdict.

Measured over 40 scenarios, 82% of the circle tests in `_is_collision_free` and
97.6% of those in `_is_corner_arc_clear` were against an obstacle that cannot reach
the query at all — 93% of all point-to-segment distance work in the planner. The
prefilter is sound because a point further than `radius` outside the query's
bounding box is further than `radius` from the query itself; these tests hold it
to that, against a brute-force check that keeps no prefilter.
"""

import random

from shapely.geometry import LineString

from path_planning import config, planner as astar
from path_planning.geometry import spatial as su
from path_planning.scenario import preprocessing as prep, presets as mg


def _brute_force_clear(planner, p1, p2):
    """`_is_collision_free` with every prefilter removed."""
    for cx, cy, radius in planner._circles:
        if su.point_to_line_distance((cx, cy), p1, p2) < radius:
            return False
    line = LineString([p1, p2])
    for poly in planner._polygons:
        if poly.relate_pattern(line, "T********"):
            return False
    if planner._safezone is not None and not planner._safezone.covers(line):
        return False
    return True


def _planner():
    scen = mg.get_all_scenarios()["scenario_16_extreme_complexity"]()
    return astar.KinodynamicAstar(prep.prepare_scenario(scen))


def test_prefiltered_and_brute_force_agree_on_random_chords():
    planner = _planner()
    assert planner._circles and planner._polygons, "scenario lost its obstacles"
    rng = random.Random(7)
    w, h = config.MAP_WIDTH, config.MAP_HEIGHT
    disagreements = []
    blocked = 0
    for _ in range(1500):
        a = (rng.uniform(0, w), rng.uniform(0, h))
        b = (rng.uniform(0, w), rng.uniform(0, h))
        fast = planner._is_collision_free(a, b)
        slow = _brute_force_clear(planner, a, b)
        blocked += not fast
        if fast != slow:
            disagreements.append((a, b, fast, slow))
    assert not disagreements, disagreements[:3]
    assert blocked > 100, "sample never hit an obstacle; the test proves nothing"


def test_prefilter_keeps_a_chord_that_only_just_reaches_a_circle():
    """The dangerous case for a bbox test: the centre is outside the chord's own
    bounding box, and only the grown box catches it."""
    planner = _planner()
    (cx, cy, r) = planner._circles[0]
    # A short chord ending just short of the circle, offset so the centre lies
    # outside the chord's raw bbox but within `r` of it.
    a = (cx - 3 * r, cy - r * 0.5)
    b = (cx - r * 0.5, cy - r * 0.5)
    assert not (a[0] <= cx <= b[0] and min(a[1], b[1]) <= cy <= max(a[1], b[1])), (
        "centre must be OUTSIDE the raw bbox for this test to bite"
    )
    assert planner._is_collision_free(a, b) is _brute_force_clear(planner, a, b)
