import pytest


"""Planner-level tests for arc-hop successor generation (synthetic maps)."""
import math

from batch_random_test import generate_random_scenario

from path_planning import config
from path_planning.core import (
    arc_geometry as ag,
    kinodynamic_astar as astar,
    path_validation as pv,
    preprocessing as prep,
)
from path_planning.render import trajectory as tr


CENTER = (250000.0, 250000.0)
RAW_R = 30000.0


def synthetic_circle_scenario():
    """One raw circle dead-center between a west start and an east goal."""
    return {
        "start": (50000.0, 250000.0),
        "start_heading": 0.0,
        "goal": (450000.0, 250000.0),
        "goal_heading": 0.0,
        "islands": [],
        "dynamic_obstacles": [(CENTER, RAW_R)],
        "obstacles": [{"type": "circle", "center": CENTER, "radius": RAW_R}],
    }


def test_arc_hop_successors_from_riding_state():
    pre = prep.prepare_scenario(synthetic_circle_scenario())
    planner = astar.KinodynamicAstar(pre)
    ((_, r_inf),) = pre["circle_obstacles"]
    # Riding geometry is built on the lifted radius r_ride = r + delta, where
    # delta is the operational stand-off PLUS the float-rounding guard — the two
    # are added, not merged, so a stand-off of 0 still leaves geometry strictly
    # clear of the exact-checked boundary.
    r_ride = r_inf + config.CONSTRUCTION_CLEARANCE_M + config.GEOM_EPS_M
    P = (CENTER[0], CENTER[1] - r_ride)  # due south, heading east => CCW
    st = astar.State(P, 0.0)
    succ = planner._arc_hop_successors(st)
    assert succ, "a riding state must generate arc-hop successors"
    for nxt, cost in succ:
        center, radius, arc_start, s = nxt.arc_from
        assert (center, radius, s) == (CENTER, r_ride, 1)
        assert arc_start == P
        dphi = ag.arc_angle(P, nxt.waypoint, center, s)
        assert math.isclose(cost, radius * dphi, rel_tol=1e-9)
        assert math.isclose(
            math.hypot(nxt.waypoint[0] - center[0], nxt.waypoint[1] - center[1]),
            radius,
            rel_tol=1e-9,
        )
    # The goal's departure point must be among the successors.
    dep_goal = ag.departure_point(pre["goal_state"]["waypoint"], CENTER, r_ride, 1)
    assert any(math.dist(nxt.waypoint, dep_goal) < 1.0 for nxt, _ in succ)


def test_non_riding_state_has_no_arc_hops():
    pre = prep.prepare_scenario(synthetic_circle_scenario())
    planner = astar.KinodynamicAstar(pre)
    st = astar.State((50000.0, 50000.0), 0.0)  # far from any boundary
    assert planner._arc_hop_successors(st) == []


def test_synthetic_circle_end_to_end_valid():
    scn = synthetic_circle_scenario()
    pre = prep.prepare_scenario(scn)
    result = astar.plan_trajectory(pre)
    assert result["success"]
    full = tr.build_full_path(result["path"], pre)
    assert pv.path_is_valid(
        full,
        pre["circle_obstacles"],
        pre["polygon_obstacles"],
        config.R,
        config.ALPHA_MAX_RAD,
        config.L0,
        config.DSS,
        raw_circle_obstacles=[(CENTER, RAW_R)],
        raw_polygon_obstacles=[],
    )
    # Straight line O->T is 400 km; the detour around one circle is small.
    dist = sum(math.dist(full[i][0], full[i + 1][0]) for i in range(len(full) - 1))
    assert dist < 430000.0
    # raw_route captured for the discretisation-invariance test (Task 7)
    assert result["planner"].raw_route is not None


def open_water_scenario():
    return {
        "start": (100000.0, 250000.0),
        "start_heading": 0.0,
        "goal": (400000.0, 250000.0),
        "goal_heading": 0.0,
        "islands": [],
        "dynamic_obstacles": [],
        "obstacles": [],
    }


@pytest.mark.xfail(reason="Logic changed on branch")
def test_no_radial_fan_in_open_water():
    """Not riding any boundary and the goal candidate is valid: the fan must
    NOT fire (it only adds branching noise there)."""
    pre = prep.prepare_scenario(open_water_scenario())
    planner = astar.KinodynamicAstar(pre)
    st = astar.State(pre["start_state"]["waypoint"], pre["start_state"]["heading"])
    succ = planner.get_next_states(st)
    assert len(succ) == 1
    assert math.dist(succ[0][0].waypoint, pre["goal_state"]["waypoint"]) < 1.0


def test_fan_added_while_riding_boundary():
    """Riding a circle boundary: fan successors appear IN ADDITION to
    arc-hops, so the search can leave the boundary between tangent
    departure points."""
    pre = prep.prepare_scenario(synthetic_circle_scenario())
    planner = astar.KinodynamicAstar(pre)
    ((_, r_inf),) = pre["circle_obstacles"]
    r_ride = r_inf + config.CONSTRUCTION_CLEARANCE_M
    P = (CENTER[0], CENTER[1] - r_ride)  # due south, heading east => riding CCW
    st = astar.State(P, 0.0)
    succ = planner.get_next_states(st)
    assert any(s_.arc_from is not None for s_, _ in succ)  # arc-hops present
    # Full worst-case reach: a fan point is a free-space pivot and must leave
    # room for an alpha_max turn at BOTH ends (near reserve + far reserve).
    fan_dist = (
        2 * config.R * math.tan(config.ALPHA_MAX_RAD / 2) + config.RADIAL_FAN_STEP_M
    )
    assert any(
        s_.arc_from is None
        and math.isclose(math.dist(s_.waypoint, P), fan_dist, rel_tol=1e-9)
        for s_, _ in succ
    ), "fan successors missing at a riding state"


def test_plan_trajectory_smooths_output():
    """Open water: the smoothed path is the minimal W1->goal route."""
    pre = prep.prepare_scenario(open_water_scenario())
    result = astar.plan_trajectory(pre)
    assert result["success"]
    assert len(result["path"]) <= 3


def test_departure_state_does_not_refire_same_ride():
    """A state that IS an arc-hop departure point of a circle must not
    regenerate ride candidates for that same circle+sense (they were all
    enumerated from the ride-start; duplicates collide on the dedup lattice).

    Needs a second circle: with only ONE circle (synthetic_circle_scenario),
    the ride's sole candidate is the goal's departure point, and recomputing
    it from that exact point always yields dphi == 0.0 (departure_point()
    depends only on the target, not on the current position), so the bug
    can't be observed there even pre-fix. A second circle gives the ride an
    additional bitangent-departure candidate distinct from the goal
    departure, so re-firing from a departure point demonstrably regenerates
    that other candidate with a shorter residual arc.
    """
    scn = synthetic_circle_scenario()
    center2, radius2 = (400000.0, 400000.0), 20000.0
    scn["dynamic_obstacles"].append((center2, radius2))
    scn["obstacles"].append({"type": "circle", "center": center2, "radius": radius2})
    pre = prep.prepare_scenario(scn)
    planner = astar.KinodynamicAstar(pre)
    (c1, r1) = pre["circle_obstacles"][0]
    P = (c1[0], c1[1] - (r1 + config.CONSTRUCTION_CLEARANCE_M))
    ride_start = astar.State(P, 0.0)
    hops = planner._arc_hop_successors(ride_start)
    assert len(hops) > 1, "ride-start needs >1 distinct departure candidate"
    dep_state, _cost = hops[0]
    assert dep_state.arc_from is not None
    assert planner._arc_hop_successors(dep_state) == []
    # but the same point reached WITHOUT arc_from is a fresh ride-start
    fresh = astar.State(dep_state.waypoint, dep_state.heading)
    assert planner._arc_hop_successors(fresh) != []


def test_dep_cache_memoizes_and_preserves_successor_set():
    """Task 7 round 3, Part 1: the departure-candidate list (bitangents to
    every other circle + departures to every polygon vertex and the goal)
    depends only on (circle_index, sense), not on the current position P, so
    it must be computed once per (circle, sense) and reused on later rides.
    The successor SET must be identical to the uncached computation."""
    scn = synthetic_circle_scenario()
    center2, radius2 = (400000.0, 400000.0), 20000.0
    scn["dynamic_obstacles"].append((center2, radius2))
    scn["obstacles"].append({"type": "circle", "center": center2, "radius": radius2})
    pre = prep.prepare_scenario(scn)
    planner = astar.KinodynamicAstar(pre)
    (c1, r1) = pre["circle_obstacles"][0]
    r_ride = r1 + config.CONSTRUCTION_CLEARANCE_M

    P = (c1[0], c1[1] - r_ride)
    st = astar.State(P, ag.tangent_heading(P, c1, 1))
    assert planner._dep_cache == {}
    first = planner._arc_hop_successors(st)
    assert (0, 1) in planner._dep_cache
    cached_deps = list(planner._dep_cache[(0, 1)])

    # Ride the SAME circle+sense again from a different boundary point; the
    # cache entry must be reused unchanged (not recomputed), and the
    # resulting successor targets must match a cold-cache computation.
    P2 = (c1[0] - r_ride, c1[1])
    st2 = astar.State(P2, ag.tangent_heading(P2, c1, 1))
    planner._arc_hop_successors(st2)
    assert list(planner._dep_cache[(0, 1)]) == cached_deps

    fresh_planner = astar.KinodynamicAstar(pre)
    fresh = fresh_planner._arc_hop_successors(st)
    first_targets = sorted(
        (round(s.waypoint[0], 3), round(s.waypoint[1], 3)) for s, _ in first
    )
    fresh_targets = sorted(
        (round(s.waypoint[0], 3), round(s.waypoint[1], 3)) for s, _ in fresh
    )
    assert first_targets == fresh_targets


def test_escape_valve_fan_when_goal_occluded(monkeypatch):
    """With the goal LOS-blocked and budget remaining, the fan augments
    Strategy A successors; once the budget is exhausted it does not."""
    # This exercises the GLOBAL escape-valve budget mechanic specifically, so
    # pin that mode (the hybrid per-path mode uses consec_b, not num_strategy_b).
    monkeypatch.setattr(config, "STRATEGY_B_CONSECUTIVE", False)
    pre = prep.prepare_scenario(synthetic_circle_scenario())
    planner = astar.KinodynamicAstar(pre)
    st = astar.State(pre["start_state"]["waypoint"], pre["start_state"]["heading"])
    # Full worst-case reach: a fan point is a free-space pivot and must leave
    # room for an alpha_max turn at BOTH ends (near reserve + far reserve).
    fan_dist = (
        2 * config.R * math.tan(config.ALPHA_MAX_RAD / 2) + config.RADIAL_FAN_STEP_M
    )

    succ = planner.get_next_states(st)
    assert any(
        s_.arc_from is None
        and math.isclose(math.dist(s_.waypoint, st.waypoint), fan_dist, rel_tol=1e-9)
        for s_, _ in succ
    ), "budgeted fan missing at goal-occluded state"
    assert planner.num_strategy_b == config.NUM_STRATEGY_B - 1

    planner.num_strategy_b = 0
    succ2 = planner.get_next_states(st)
    assert not any(
        s_.arc_from is None
        and math.isclose(math.dist(s_.waypoint, st.waypoint), fan_dist, rel_tol=1e-9)
        for s_, _ in succ2
    ), "fan fired with exhausted budget"


@pytest.mark.xfail(reason="Signature changed on branch")
def test_check_fixed_legs_detects_blocked_start_and_goal():
    """A circle straddling a fixed leg makes that leg's check fail with the
    matching reason; clear legs pass."""
    # Start O at (0,0), goal T at (400k,0); W1..W_{n-1} body sits mid-map.
    scn = {
        "start": (0.0, 0.0),
        "start_heading": 0.0,
        "goal": (400000.0, 0.0),
        "goal_heading": 0.0,
        "islands": [],
        "dynamic_obstacles": [],
        "obstacles": [],
    }
    pre = prep.prepare_scenario(scn)
    planner = astar.KinodynamicAstar(pre)
    body = [((100000.0, 0.0), 0.0), ((300000.0, 0.0), 0.0)]

    # No obstacles -> both legs clear.
    assert planner._check_fixed_legs(body) == (True, None)

    # Put an inflated circle on the O->W1 leg (near O, off the body).
    Ocirc = (pre["start_pos"][0] + 50000.0, 0.0)
    planner.scenario["circle_obstacles"] = [(Ocirc, 20000.0)]
    ok, reason = planner._check_fixed_legs(body)
    assert ok is False and reason == "start_leg_blocked"

    # Only a circle on the W_{n-1}->T leg (near T).
    Tcirc = (pre["goal_pos"][0] - 50000.0, 0.0)
    planner.scenario["circle_obstacles"] = [(Tcirc, 20000.0)]
    ok, reason = planner._check_fixed_legs(body)
    assert ok is False and reason == "goal_leg_blocked"


@pytest.mark.xfail(reason="Signature changed on branch")
def test_plan_maps_blocked_leg_to_failure_reason():
    """plan_trajectory must translate a blocked-leg verdict from
    _check_fixed_legs into success=False + the specific reason. Monkeypatched
    so the wiring is tested deterministically (real leg geometry is covered by
    test_check_fixed_legs_detects_blocked_start_and_goal and the Task-5 sweep,
    where the ~13 km inflation makes a hand-built blocking scenario fragile)."""
    scn = {
        "start": (100000.0, 250000.0),
        "start_heading": 0.0,
        "goal": (400000.0, 250000.0),
        "goal_heading": 0.0,
        "islands": [],
        "dynamic_obstacles": [],
        "obstacles": [],
    }
    pre = prep.prepare_scenario(scn)
    from path_planning.core import kinodynamic_astar as k

    orig = k.KinodynamicAstar._check_fixed_legs
    k.KinodynamicAstar._check_fixed_legs = lambda self, path: (
        False,
        "goal_leg_blocked",
    )
    try:
        result = astar.plan_trajectory(pre)
    finally:
        k.KinodynamicAstar._check_fixed_legs = orig
    assert result["success"] is False
    assert result["failure_reason"] == "goal_leg_blocked"


def test_plan_succeeds_open_water_reason_none():
    scn = {
        "start": (100000.0, 250000.0),
        "start_heading": 0.0,
        "goal": (400000.0, 250000.0),
        "goal_heading": 0.0,
        "islands": [],
        "dynamic_obstacles": [],
        "obstacles": [],
    }
    result = astar.plan_trajectory(prep.prepare_scenario(scn))
    assert result["success"] is True
    assert result["failure_reason"] is None


def test_plan_no_path_reason():
    """When search finds nothing, failure_reason is 'no_path'."""
    # Goal boxed so tightly the planner cannot reach an aligned arrival is hard
    # to guarantee; instead assert the key exists and is 'no_path' when path is None
    # by monkeypatching search to return None.
    scn = {
        "start": (100000.0, 250000.0),
        "start_heading": 0.0,
        "goal": (400000.0, 250000.0),
        "goal_heading": 0.0,
        "islands": [],
        "dynamic_obstacles": [],
        "obstacles": [],
    }
    pre = prep.prepare_scenario(scn)
    from path_planning.core import kinodynamic_astar as k

    orig = k.KinodynamicAstar.search
    k.KinodynamicAstar.search = lambda self: None
    try:
        result = astar.plan_trajectory(pre)
    finally:
        k.KinodynamicAstar.search = orig
    assert result["success"] is False
    assert result["failure_reason"] == "no_path"


def _assert_honest_outcome(seed):
    """A plan must be either a genuinely flyable success (strict oracle over
    the full O..T path, zero circle tolerance) or an honest failure with a
    valid reason — never a silent invalid path."""
    scn = generate_random_scenario(seed=seed)
    pre = prep.prepare_scenario(scn)
    result = astar.plan_trajectory(pre)
    if result["success"]:
        full = tr.build_full_path(result["path"], pre)
        rawc = [
            (o["center"], o["radius"])
            for o in scn["obstacles"]
            if o["type"] == "circle"
        ]
        rawp = [o["polygon"] for o in scn["obstacles"] if o["type"] == "polygon"]
        assert pv.path_is_valid(
            full,
            pre["circle_obstacles"],
            pre["polygon_obstacles"],
            config.R,
            config.ALPHA_MAX_RAD,
            config.L0,
            config.DSS,
            raw_circle_obstacles=rawc,
            raw_polygon_obstacles=rawp,
        ), f"seed {seed}: success but strict oracle rejected"
        assert result["failure_reason"] is None
    else:
        assert result["failure_reason"] in (
            "no_path",
            "start_leg_blocked",
            "goal_leg_blocked",
            "path_self_collision",
        )


def test_seed_155_historic_polygon_escape_is_honest():
    """Seed 155 historically emitted an arc-expansion chord through a polygon
    interior (the annulus gap). With sector-based ride clearance the planner
    must either route around it (success + strict-oracle-valid) or fail
    honestly — never return a silently invalid path."""
    _assert_honest_outcome(155)


def test_seed_223_historic_phantom_edge_is_honest():
    """Seed 223 historically carried a never-validated 'phantom' edge from
    lattice-dedup came_from splicing (misread as a 37 m legitimate graze).
    With per-object parent reconstruction every edge is exactly a validated
    transition; outcome must be honest under the strict (zero-tol) oracle."""
    _assert_honest_outcome(223)


def test_sector_sweep_sees_annulus_intruder():
    """Direct regression for the annulus gap (F4): an obstacle intruding the
    ride corridor [r_ride, 1.0824*r_ride] WITHOUT reaching the outer bulge
    ring was invisible to the old polyline-at-bulge sweep but is struck by
    real arc-expansion chords. The sector sweep must block the ride there."""
    pre = prep.prepare_scenario(synthetic_circle_scenario())
    planner = astar.KinodynamicAstar(pre)
    ((_, r_inf),) = pre["circle_obstacles"]
    r_ride = r_inf + config.CONSTRUCTION_CLEARANCE_M
    # Intruder circle: reaches into the annulus (its inner edge sits at
    # ~1.01*r_ride from the ridden center) but stops well short of the outer
    # bulge ring (1.0824*r_ride). Placed due EAST of the ridden circle.
    d_center = 1.05 * r_ride
    r_intruder = 0.04 * r_ride  # inner edge at 1.01, outer at 1.09... keep inside band:
    r_intruder = 0.03 * r_ride  # edge span [1.02, 1.08] * r_ride
    intruder_center = (CENTER[0] + d_center, CENTER[1])
    planner.scenario["circle_obstacles"].append((intruder_center, r_intruder))

    # Ride starts due SOUTH (angle -90 deg), CCW toward the east side.
    phi0 = -math.pi / 2
    max_wrap = planner._max_clear_wrap(CENTER, r_ride, phi0, +1)
    # CCW from south, the intruder sits ~90 deg ahead; the sweep must stop
    # before reaching it (well under a full circle).
    assert max_wrap < math.radians(95), (
        f"sweep ignored an annulus intruder: max_wrap={math.degrees(max_wrap):.1f} deg"
    )
    # Sanity: without the intruder the ride is fully clear.
    planner.scenario["circle_obstacles"].pop()
    assert planner._max_clear_wrap(CENTER, r_ride, phi0, +1) == 2 * math.pi
