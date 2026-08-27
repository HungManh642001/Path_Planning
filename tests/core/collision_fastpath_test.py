"""Semantic-invariance tests for the collision fast paths.

The bbox prefilters in _is_collision_free/_is_sector_clear and the State dedup-key
cache are pure refactors: results must agree exactly with an unfiltered
reference (every polygon tested, no prefilter). These tests pin that
agreement on real random maps so a prefilter slip (wrong bbox order, strict
vs non-strict overlap, stale cache) can never change planner behaviour
silently.
"""

import math
import random

from shapely.geometry import LineString, Polygon

from path_planning.core import (
    arc_geometry as ag,
    kinodynamic_astar as astar,
    map_generator as mg,
    preprocessing as prep,
    spatial_utils as su,
)
from path_planning.core.map_generator import generate_random_scenario


# Seeds chosen for polygon variety: 964 has 15 islands, 86/155 mixed.
MAP_SEEDS = [964, 86, 155]


def _reference_check_collision(planner, p1, p2):
    """Unfiltered reference: exact same predicates, every obstacle tested."""
    p1x, p1y = p1
    sx, sy = p2[0] - p1x, p2[1] - p1y
    dd = sx * sx + sy * sy
    for (cx, cy), radius in planner.scenario["circle_obstacles"]:
        relx, rely = cx - p1x, cy - p1y
        if dd == 0.0:
            if relx * relx + rely * rely < radius * radius:
                return False
            continue
        t = max(0.0, min(1.0, (relx * sx + rely * sy) / dd))
        ex, ey = relx - t * sx, rely - t * sy
        if ex * ex + ey * ey < radius * radius:
            return False
    line = LineString([p1, p2])
    for poly in planner._polygons:
        if poly.relate_pattern(line, "T********"):
            return False
    if planner._safezone is not None and not planner._safezone.covers(line):
        return False
    return True


def _reference_sector_clear(planner, center, r_in, r_out, phi_a, phi_b):
    lo, hi = (phi_a, phi_b) if phi_a <= phi_b else (phi_b, phi_a)
    for c2, r2 in planner.scenario["circle_obstacles"]:
        dx, dy = c2[0] - center[0], c2[1] - center[1]
        d = math.hypot(dx, dy)
        if d - r2 >= r_out or d + r2 <= r_in:
            continue
        if d <= r2:
            return False
        theta = math.atan2(dy, dx)
        half = math.asin(min(1.0, r2 / d))
        if ag.has_angular_overlap(theta - half, theta + half, lo, hi):
            return False
    quad = Polygon(ag.sector_polygon(center, r_in, r_out, lo, hi))
    for poly in planner._polygons:
        if poly.relate_pattern(quad, "T********"):
            return False
    return True


def test_check_collision_agrees_with_unfiltered_reference():
    rng = random.Random(3)
    for seed in MAP_SEEDS:
        planner = astar.KinodynamicAstar(
            prep.prepare_scenario(generate_random_scenario(seed=seed))
        )
        for _ in range(400):
            p1 = (rng.uniform(0, 500000), rng.uniform(0, 500000))
            # mix of long chords and short local segments
            if rng.random() < 0.5:
                p2 = (rng.uniform(0, 500000), rng.uniform(0, 500000))
            else:
                p2 = (
                    p1[0] + rng.uniform(-20000, 20000),
                    p1[1] + rng.uniform(-20000, 20000),
                )
            assert planner._is_collision_free(p1, p2) == _reference_check_collision(
                planner, p1, p2
            ), (seed, p1, p2)


def test_sector_clear_agrees_with_unfiltered_reference():
    rng = random.Random(5)
    for seed in MAP_SEEDS:
        planner = astar.KinodynamicAstar(
            prep.prepare_scenario(generate_random_scenario(seed=seed))
        )
        for _ in range(200):
            center = (rng.uniform(0, 500000), rng.uniform(0, 500000))
            r_in = rng.uniform(5000, 60000)
            r_out = r_in * (1.0 / math.cos(math.pi / 8.0))
            phi_a = rng.uniform(-math.pi, math.pi)
            phi_b = phi_a + rng.uniform(0.01, 0.5)
            assert planner._is_sector_clear(
                center, r_in, r_out, phi_a, phi_b
            ) == _reference_sector_clear(planner, center, r_in, r_out, phi_a, phi_b), (
                seed,
                center,
            )


def test_state_dedup_key_matches_lattice_semantics():
    rng = random.Random(9)
    for _ in range(300):
        wp = (rng.uniform(0, 500000), rng.uniform(0, 500000))
        h = rng.uniform(-math.pi, math.pi)
        a = astar.State(wp, h)
        # same lattice cell via a sub-quantum nudge
        b = astar.State((wp[0] + 1e-9, wp[1] - 1e-9), h + 1e-12)
        assert (
            su.state_to_tuple(a.waypoint, a.heading)
            == su.state_to_tuple(b.waypoint, b.heading)
        ) == (a == b)
        if a == b:
            assert hash(a) == hash(b)
