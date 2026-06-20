import math
import pytest
import preprocessing as prep
import kinodynamic_astar as astar
import path_validation as pv
import config
import spatial_utils as su


def _simple_pre(circles=(), polys=(), start=(2000, 2000), goal=(100000, 0)):
    scenario = {
        'start': start, 'start_heading': 0.0,
        'goal': goal, 'goal_heading': 0.0,
        'obstacles': [{'type': 'circle', 'center': c, 'radius': r} for c, r in circles]
                     + [{'type': 'polygon', 'polygon': p} for p in polys],
        'islands': [], 'sam_sites': [],
    }
    return prep.prepare_scenario(scenario)


def test_polygons_are_prebuilt_shapely_objects():
    pre = _simple_pre(polys=[[(0, 0), (10, 0), (10, 10), (0, 10)]])
    planner = astar.KinodynamicAstar(pre)
    from shapely.geometry import Polygon
    assert hasattr(planner, '_polygons')
    assert all(isinstance(p, Polygon) for p in planner._polygons)
    assert len(planner._polygons) == 1


def test_state_tuple_buckets_nearby_states_together():
    a = su.state_to_tuple((123456.0, 7000.0), 0.10)
    b = su.state_to_tuple((123456.0 + 200.0, 7000.0 + 200.0), 0.10 + math.radians(1.0))
    assert a == b, "states within one lattice cell must hash equal"


def test_state_tuple_distinguishes_far_states():
    a = su.state_to_tuple((0.0, 0.0), 0.0)
    b = su.state_to_tuple((5000.0, 0.0), 0.0)
    assert a != b


def test_finds_valid_path_around_single_circle():
    # Circle at (150000,0) r=20000 -> inflated ~30.3 km. Goal far enough right
    # that the goal waypoint (offset back by DSS) clears the inflated circle,
    # but the circle still blocks the direct start->goal line, forcing a detour.
    pre = _simple_pre(circles=[((150000.0, 0.0), 20000.0)], goal=(300000.0, 0.0))
    planner = astar.KinodynamicAstar(pre)
    path = planner.search()
    assert path is not None, "planner must find a route around one circle"
    assert pv.segments_clear(path, pre['circle_obstacles'], pre['polygon_obstacles'])




def test_no_dead_stuck_counter():
    import inspect
    src = inspect.getsource(astar.KinodynamicAstar.search)
    assert 'iterations_without_expansion' not in src, \
        "dead early-exit counter must be removed (C4)"


def test_produced_path_is_fully_valid_around_circle():
    pre = _simple_pre(circles=[((150000.0, 0.0), 20000.0)], goal=(300000.0, 0.0))
    path = astar.KinodynamicAstar(pre).search()
    assert path is not None
    assert pv.turn_angles_ok(path, pre['alpha_max_rad'])
    assert pv.arcs_clear(path, pre['turn_radius'],
                         pre['circle_obstacles'], pre['polygon_obstacles'])


def test_arc_clear_method_removed():
    import inspect
    assert not hasattr(astar.KinodynamicAstar, '_arc_clear'), \
        "arc clearance is guaranteed by inflation; the runtime check must be removed"
    src = inspect.getsource(astar.KinodynamicAstar.get_next_states)
    assert '_arc_clear' not in src
    src2 = inspect.getsource(astar.KinodynamicAstar.smooth_path)
    assert '_arc_clear' not in src2


def test_smoothing_reduces_or_keeps_waypoints_and_stays_valid():
    pre = _simple_pre(circles=[((150000.0, 0.0), 20000.0)], goal=(300000.0, 0.0))
    raw = astar.KinodynamicAstar(pre).search()                # un-smoothed search path
    result = astar.plan_trajectory(pre, verbose=False)        # search + smooth
    smoothed = result['path']
    print(f"\nraw waypoints={len(raw) if raw else 0}, smoothed waypoints={len(smoothed) if smoothed else 0}")
    assert raw is not None and smoothed is not None
    assert len(smoothed) <= len(raw), f"smoothed {len(smoothed)} > raw {len(raw)}"
    # smoothed path must remain fully valid
    assert pv.segments_clear(smoothed, pre['circle_obstacles'], pre['polygon_obstacles'])
    assert pv.turn_angles_ok(smoothed, pre['alpha_max_rad'])
    assert pv.arcs_clear(smoothed, pre['turn_radius'],
                         pre['circle_obstacles'], pre['polygon_obstacles'])


def test_turn_penalty_makes_turning_cost_more_than_straight():
    # Goal directly behind the start heading (turn 180deg > alpha_max) => no graph
    # candidate is valid => radial fallback fires, giving the 11-way fan to compare.
    pre = _simple_pre(goal=(-50000.0, 0.0))
    planner = astar.KinodynamicAstar(pre)
    succ = planner.get_next_states(planner.start_state)
    assert len(succ) >= 2
    straight = min(succ, key=lambda s: abs(astar._angle_diff(s[0].heading, planner.start_state.heading)))
    turned = max(succ, key=lambda s: abs(astar._angle_diff(s[0].heading, planner.start_state.heading)))
    assert turned[1] > straight[1], "a sharper turn must cost more (turn penalty)"


def test_heuristic_is_euclidean_lower_bound():
    pre = _simple_pre()
    planner = astar.KinodynamicAstar(pre)
    s = planner.start_state
    g = planner.goal_state
    euclid = math.hypot(g.waypoint[0] - s.waypoint[0], g.waypoint[1] - s.waypoint[1])
    h = planner.heuristic(s, g)
    assert abs(h - euclid) < 1e-6, "heuristic must equal Euclidean distance (admissible)"

    # Strengthen: build a state with a heading DIFFERENT from goal_state.heading so
    # the old `dist + R * heading_diff` formula would return dist + R * pi/2, not
    # just dist.  The heuristic must still equal pure Euclidean distance regardless
    # of heading, so the old formula is guaranteed to fail this assertion.
    differing_heading_state = astar.State(s.waypoint, g.heading + math.pi / 2)
    h2 = planner.heuristic(differing_heading_state, g)
    # euclid distance is identical (same waypoints); heading must NOT add anything
    assert abs(h2 - euclid) < 1e-6, \
        f"heuristic with heading diff pi/2 returned {h2:.2f}, expected Euclidean {euclid:.2f}; " \
        f"heading penalty R*pi/2 = {planner.R * math.pi / 2:.2f} must be removed"


def test_goal_directed_successor_heads_straight_at_goal():
    pre = _simple_pre()  # empty map; Strategy 2 + goal-directed only
    planner = astar.KinodynamicAstar(pre)
    succ = planner.get_next_states(planner.start_state)
    gh = su.angle_to_heading(planner.start_state.waypoint, planner.goal_state.waypoint)
    # The goal-directed successor heads EXACTLY at the goal; the radial fan headings
    # are offsets of the current heading and won't match gh exactly. Use a tight bound.
    assert any(abs(astar._angle_diff(s[0].heading, gh)) < math.radians(0.5) for s in succ), \
        "a goal-directed successor pointing straight at the goal must exist"


import random as _random
from shapely.geometry import Polygon as _Poly, LineString as _Line


def _brute_force_clear(p1, p2, circles, polys):
    import spatial_utils as su
    for c, r in circles:
        if su.point_to_line_distance(c, p1, p2) < r - 1e-6:
            return False
    line = _Line([p1, p2])
    return not any(line.intersects(_Poly(poly)) for poly in polys)


def test_check_collision_matches_bruteforce_with_spatial_index():
    polys = [[(10000, 10000), (30000, 10000), (30000, 30000), (10000, 30000)],
             [(60000, 5000), (90000, 5000), (75000, 40000)]]
    circles = [((50000.0, 50000.0), 8000.0)]
    pre = _simple_pre(circles=[((50000.0, 50000.0), 8000.0)],
                      polys=polys)
    planner = astar.KinodynamicAstar(pre)
    rng = _random.Random(1234)
    for _ in range(400):
        p1 = (rng.uniform(0, 100000), rng.uniform(0, 100000))
        p2 = (rng.uniform(0, 100000), rng.uniform(0, 100000))
        got = planner._check_collision(p1, p2)
        want = _brute_force_clear(p1, p2, pre['circle_obstacles'], pre['polygon_obstacles'])
        assert got == want, f"mismatch on {p1}->{p2}: got {got} want {want}"
    assert hasattr(planner, '_poly_tree')


import spatial_utils as su2  # alias to avoid clashing if su already imported


def test_dynamic_successors_include_circle_tangents():
    # Circle nearly straight ahead and small, so BOTH tangent directions are within
    # alpha_max of the start heading (this task runs at the default alpha_max=30 deg).
    # From start, successors must include the circle's tangent points (computed against
    # the INFLATED radius, which is what the planner stores) — no tangent_graph used.
    circle_raw = ((60000.0, 6000.0), 8000.0)
    pre = _simple_pre(circles=[circle_raw], goal=(120000.0, 0.0))
    inflated_center, inflated_radius = pre['circle_obstacles'][0]
    planner = astar.KinodynamicAstar(pre)
    succ = planner.get_next_states(planner.start_state)
    tang = su2.circle_tangent_points(planner.start_state.waypoint, inflated_center, inflated_radius)
    succ_wps = [s[0].waypoint for s in succ]
    assert tang, "expected two tangent points from start to the inflated circle"
    assert any(min(math.hypot(w[0]-t[0], w[1]-t[1]) for w in succ_wps) < 1.0 for t in tang), \
        "dynamic successors must include circle tangent points"


def test_radial_fallback_when_no_graph_candidate():
    # empty map: no obstacles, so tangent set is just the goal. If the goal is directly
    # reachable it is a successor; force the not-directly-reachable case by a goal behind
    # the start heading so the direct jump turn > alpha_max, exercising the radial fallback.
    pre = _simple_pre(goal=(-50000.0, 0.0))   # goal behind start (start_heading=0)
    planner = astar.KinodynamicAstar(pre)
    succ = planner.get_next_states(planner.start_state)
    assert len(succ) > 0, "radial fallback must produce successors when no graph candidate is valid"


import time as _time


def test_search_respects_time_budget(monkeypatch):
    # Force a tiny budget and a scenario that would otherwise run to the iteration cap;
    # search must return within a small multiple of the budget.
    monkeypatch.setattr(config, 'TIME_BUDGET_S', 0.05)
    # dense-ish: several polygons straddling the route so the graph can't trivially solve it
    polys = [[(40000+i*1000, 0), (60000+i*1000, 0), (50000+i*1000, 40000)] for i in range(6)]
    pre = _simple_pre(polys=polys, goal=(120000.0, 0.0))
    planner = astar.KinodynamicAstar(pre)
    t0 = _time.perf_counter()
    planner.search()
    dt = _time.perf_counter() - t0
    assert dt < 0.5, f"search ignored the 0.05s budget (took {dt:.3f}s)"


def test_polygon_vertex_is_reachable_successor():
    # A polygon straddling the path ahead. Its hull vertices must be reachable
    # successors: a segment that ENDS AT a polygon vertex (touching the boundary
    # at its endpoint) must NOT be rejected as a collision. Before the fix every
    # vertex was rejected because shapely `intersects` flags the endpoint touch.
    poly = [(60000.0, -20000.0), (60000.0, 20000.0), (90000.0, 20000.0), (90000.0, -20000.0)]
    pre = _simple_pre(polys=[poly], goal=(150000.0, 0.0))
    planner = astar.KinodynamicAstar(pre)
    from shapely.geometry import Polygon
    hull = list(Polygon(pre['polygon_obstacles'][0]).convex_hull.exterior.coords[:-1])
    succ = planner.get_next_states(planner.start_state)
    succ_wps = [s[0].waypoint for s in succ]
    assert any(
        min(math.hypot(w[0] - v[0], w[1] - v[1]) for w in succ_wps) < 1.0
        for v in hull
    ), "at least one polygon hull vertex must be a reachable successor"


def test_collision_through_polygon_interior_still_blocked():
    # The endpoint-trim fix must NOT weaken real collision detection: a segment
    # that cuts straight through the polygon interior is still a collision.
    poly = [(60000.0, -20000.0), (60000.0, 20000.0), (90000.0, 20000.0), (90000.0, -20000.0)]
    pre = _simple_pre(polys=[poly])
    planner = astar.KinodynamicAstar(pre)
    inflated = pre['polygon_obstacles'][0]
    xs = [p[0] for p in inflated]; ys = [p[1] for p in inflated]
    cx = (min(xs) + max(xs)) / 2
    left = (min(xs) - 5000.0, (min(ys) + max(ys)) / 2)
    right = (max(xs) + 5000.0, (min(ys) + max(ys)) / 2)
    assert planner._check_collision(left, right) is False, \
        "a segment through the polygon interior must be blocked"


def test_goal_candidate_rejected_when_final_turn_exceeds_amax():
    # goal_heading is WEST (180 deg); approaching W_{n-1} from the west means the
    # terminal turn (approach -> goal_heading) is ~180 deg > alpha_max. The goal
    # must NOT be offered as a direct successor (final-turn constraint at W_{n-1}).
    scenario = {
        'start': (0.0, 0.0), 'start_heading': 0.0,
        'goal': (200000.0, 0.0), 'goal_heading': math.radians(180.0),
        'obstacles': [], 'islands': [], 'sam_sites': [],
    }
    pre = prep.prepare_scenario(scenario)
    planner = astar.KinodynamicAstar(pre)
    gwp = planner.goal_state.waypoint
    succ = planner.get_next_states(planner.start_state)
    assert not any(math.hypot(s[0].waypoint[0] - gwp[0], s[0].waypoint[1] - gwp[1]) < 1.0
                   for s in succ), \
        "goal requiring a >alpha_max terminal turn must be rejected"


def test_goal_candidate_accepted_when_final_turn_ok():
    # goal_heading EAST (0 deg), approach also roughly east => terminal turn ~0.
    # The goal must be a direct successor.
    scenario = {
        'start': (0.0, 0.0), 'start_heading': 0.0,
        'goal': (200000.0, 0.0), 'goal_heading': 0.0,
        'obstacles': [], 'islands': [], 'sam_sites': [],
    }
    pre = prep.prepare_scenario(scenario)
    planner = astar.KinodynamicAstar(pre)
    gwp = planner.goal_state.waypoint
    succ = planner.get_next_states(planner.start_state)
    assert any(math.hypot(s[0].waypoint[0] - gwp[0], s[0].waypoint[1] - gwp[1]) < 1.0
               for s in succ), \
        "goal with a feasible terminal turn must be a direct successor"


def test_segment_along_polygon_edge_is_clear():
    # Boundary-following: a segment that runs ALONG a polygon edge (on the boundary,
    # not through the interior) must be allowed, so the planner can hug an obstacle
    # boundary to route around it. Only interior penetration is a collision.
    poly = [(60000.0, -20000.0), (60000.0, 20000.0),
            (90000.0, 20000.0), (90000.0, -20000.0)]
    pre = _simple_pre(polys=[poly])
    planner = astar.KinodynamicAstar(pre)
    inflated = pre['polygon_obstacles'][0]
    # two consecutive vertices of the inflated polygon -> the edge between them
    a, b = inflated[0], inflated[1]
    assert planner._check_collision(a, b) is True, \
        "a segment running along a polygon edge (boundary) must be allowed"


def test_wrap_step_successor_added_when_on_circle_boundary():
    # When the current waypoint sits ON a circle's inflated boundary, get_next_states
    # must add a STRAIGHT continuation successor (same heading, WRAP_STEP_M ahead).
    # This step is zero-turn so it is NOT subject to the đoản trình arc constraint:
    # WRAP_STEP_M (2000m) < R*tan(alpha_max/2) (8000m), so if đoản trình were applied
    # the step would be rejected. Its presence proves the constraint is skipped.
    circle_raw = ((100000.0, 0.0), 20000.0)
    pre = _simple_pre(circles=[circle_raw], goal=(300000.0, 0.0))
    c, r = pre['circle_obstacles'][0]                 # inflated circle
    P = (c[0], c[1] + r)                              # top of the inflated circle
    h = 0.0                                           # tangent at the top (horizontal)
    planner = astar.KinodynamicAstar(pre)
    succ = planner.get_next_states(astar.State(P, h))
    expected = (P[0] + config.WRAP_STEP_M * math.cos(h),
                P[1] + config.WRAP_STEP_M * math.sin(h))
    assert any(
        math.hypot(s[0].waypoint[0] - expected[0], s[0].waypoint[1] - expected[1]) < 1.0
        and abs(astar._angle_diff(s[0].heading, h)) < 1e-9
        for s in succ), "a straight wrap-step successor must be added on a circle boundary"


def test_wrap_step_not_added_when_off_circle():
    # Far from any circle boundary, no wrap-step successor (heading-preserving point
    # exactly WRAP_STEP_M ahead) should be generated.
    circle_raw = ((100000.0, 0.0), 20000.0)
    pre = _simple_pre(circles=[circle_raw], goal=(300000.0, 0.0))
    planner = astar.KinodynamicAstar(pre)
    P = (5000.0, 90000.0)                             # nowhere near the circle boundary
    h = 0.0
    succ = planner.get_next_states(astar.State(P, h))
    straight = (P[0] + config.WRAP_STEP_M, P[1])
    assert not any(
        math.hypot(s[0].waypoint[0] - straight[0], s[0].waypoint[1] - straight[1]) < 1.0
        for s in succ), "no wrap-step should be added when not on a circle boundary"


# --------------------------------------------------------------------------
# Approach-heading feasibility: the planner must not accept reaching the goal
# region with an arrival heading that would force a > alpha_max terminal turn,
# and smoothing must not introduce such a turn either. (Regression: a path that
# wrap-stepped straight into the goal region arrived ~113 deg off goal_heading.)
# --------------------------------------------------------------------------
def _all_turns_deg(path, goal_heading):
    """Turn angle (deg) at each interior waypoint of [path] + the terminal turn
    from the last leg onto goal_heading."""
    pts = [wp for wp, _ in path]
    legs = [math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0])
            for i in range(len(pts) - 1)]
    turns = []
    for i in range(len(legs) - 1):
        d = abs(math.atan2(math.sin(legs[i + 1] - legs[i]), math.cos(legs[i + 1] - legs[i])))
        turns.append(math.degrees(d))
    # terminal turn from the final leg onto the required approach heading
    term = abs(math.atan2(math.sin(goal_heading - legs[-1]), math.cos(goal_heading - legs[-1])))
    turns.append(math.degrees(term))
    return turns


def _dense_scenario(start_heading_deg, goal_heading_deg):
    return {
        'start': (150000.0, 80000.0), 'start_heading': math.radians(start_heading_deg),
        'goal': (370000.0, 470000.0), 'goal_heading': math.radians(goal_heading_deg),
        'obstacles': [
            {'type': 'circle', 'center': (255000.0, 285000.0), 'radius': 42000.0},
            {'type': 'circle', 'center': (330000.0, 250000.0), 'radius': 35000.0},
            {'type': 'polygon', 'polygon': [(150000, 240000), (210000, 250000),
                                            (220000, 310000), (160000, 330000), (130000, 290000)]},
            {'type': 'polygon', 'polygon': [(230000, 110000), (345000, 120000),
                                            (340000, 200000), (240000, 205000)]},
        ], 'islands': [], 'sam_sites': [],
    }


def test_solved_path_never_exceeds_alpha_max_at_any_waypoint():
    # Sweep a dense map with varied launch/approach headings. Any path the planner
    # reports as successful must respect the max turn angle at EVERY waypoint,
    # including the terminal turn onto the approach heading.
    tol = 0.5
    for sh in (30, 60, 90, 120, 150):
        for gh in (70, 160, 200, 230, 250, 270, 290):
            scenario = _dense_scenario(sh, gh)
            pre = prep.prepare_scenario(scenario)
            res = astar.plan_trajectory(pre, verbose=False)
            if not res['success']:
                continue
            amax = math.degrees(pre['alpha_max_rad'])
            turns = _all_turns_deg(res['path'], pre['goal_state']['heading'])
            worst = max(turns)
            assert worst <= amax + tol, (
                f"sh={sh} gh={gh}: turn {worst:.1f} deg exceeds alpha_max {amax} "
                f"(turns={[round(t,1) for t in turns]})")
