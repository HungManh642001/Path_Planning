"""
Characterization (golden-master) tests for the Kinodynamic A* planner.

PURPOSE
-------
These tests LOCK IN the *current* behaviour of the planner on a handful of
representative scenarios so that future algorithm changes can be evaluated
against a known baseline. They are NOT correctness tests — they assert
"the algorithm does today exactly what it did when this baseline was
captured", nothing more.

DO NOT change the algorithm to make these pass. If a deliberate algorithm
change moves a number, re-capture the baseline (see `_print_actual` output
when a test fails) and update the EXPECTED table in a separate, reviewed step.

Four representative scenarios (per request):
  1. empty       - open field, no obstacles
  2. one_circle  - a single circular obstacle straddling the direct line
  3. one_island  - a single polygon island straddling the direct line
  4. mixed       - one circle + one island

Run:
    pytest characterization_test.py -v -s      # -s shows the printed metrics
    python characterization_test.py            # standalone report table
"""

import math
import time

import config
import preprocessing as prep
import kinodynamic_astar as astar


# A heading change larger than this (radians) counts as a "turn".
TURN_THRESHOLD_RAD = math.radians(1.0)


# --------------------------------------------------------------------------
# Scenario construction (explicit & deterministic — no random generation)
# --------------------------------------------------------------------------
def _build_scenario(start, start_heading, goal, goal_heading, islands, circles):
    """Build a raw scenario dict in the shape map_generator produces."""
    obstacles = []
    for poly in islands:
        obstacles.append({'type': 'polygon', 'polygon': poly})
    for center, radius in circles:
        obstacles.append({'type': 'circle', 'center': center, 'radius': radius})
    return {
        'start': start,
        'start_heading': start_heading,
        'goal': goal,
        'goal_heading': goal_heading,
        'map_bounds': (config.MAP_WIDTH, config.MAP_HEIGHT),
        'islands': islands,
        'sam_sites': [(c, r) for c, r in circles],
        'obstacles': obstacles,
    }


# Common start/goal on the SW->NE diagonal so obstacles near the centre block
# the direct line.
_START = (2000, 2000)
_GOAL = (450000, 450000)
_H = math.pi / 4  # 45 degrees

# A square island centred on the diagonal.
_CENTRE_ISLAND = [
    (206000, 206000),
    (246000, 206000),
    (246000, 246000),
    (206000, 246000),
]
# A second island for the mixed scenario, further along the diagonal.
_FAR_ISLAND = [
    (300000, 300000),
    (340000, 300000),
    (340000, 340000),
    (300000, 340000),
]


def scenario_empty():
    """Open field, no obstacles."""
    return _build_scenario(_START, _H, _GOAL, _H, islands=[], circles=[])


def scenario_one_circle():
    """A single circular obstacle straddling the direct line."""
    return _build_scenario(_START, _H, _GOAL, _H,
                           islands=[], circles=[((226000, 226000), 40000)])


def scenario_one_island():
    """A single polygon island straddling the direct line."""
    return _build_scenario(_START, _H, _GOAL, _H,
                           islands=[_CENTRE_ISLAND], circles=[])


def scenario_mixed():
    """One circle + one island."""
    return _build_scenario(_START, _H, _GOAL, _H,
                           islands=[_FAR_ISLAND], circles=[((150000, 150000), 30000)])


def scenario_two_circles_gap():
    """Two circles offset from the SW->NE diagonal; the direct route is not blocked, so a path exists (regression anchor for a solvable multi-circle map)."""
    return _build_scenario(_START, _H, _GOAL, _H, islands=[],
                           circles=[((180000, 240000), 35000),
                                    ((280000, 200000), 35000)])


def scenario_circle_and_island():
    """A circle and an island that both straddle the diagonal; the current planner cannot route around them (locked as no-path baseline)."""
    return _build_scenario(_START, _H, _GOAL, _H,
                           islands=[_FAR_ISLAND],
                           circles=[((150000, 180000), 30000)])


SCENARIOS = {
    'empty': scenario_empty,
    'one_circle': scenario_one_circle,
    'one_island': scenario_one_island,
    'mixed': scenario_mixed,
    'two_circles_gap': scenario_two_circles_gap,
    'circle_and_island': scenario_circle_and_island,
}


# --------------------------------------------------------------------------
# Metric extraction
# --------------------------------------------------------------------------
def _path_is_collision_free(planner, path):
    """Re-check every segment of the produced path against inflated obstacles
    using the planner's own collision routine."""
    for i in range(len(path) - 1):
        if not planner._check_collision(path[i][0], path[i + 1][0]):
            return False
    return True


def _path_length(path):
    total = 0.0
    for i in range(len(path) - 1):
        (x1, y1), _ = path[i]
        (x2, y2), _ = path[i + 1]
        total += math.hypot(x2 - x1, y2 - y1)
    return total


def _num_turns(path):
    """Number of junctions where the heading changes by more than the
    turn threshold."""
    turns = 0
    for i in range(len(path) - 1):
        _, h1 = path[i]
        _, h2 = path[i + 1]
        delta = math.atan2(math.sin(h2 - h1), math.cos(h2 - h1))
        if abs(delta) > TURN_THRESHOLD_RAD:
            turns += 1
    return turns


def measure(scenario_key):
    """Run the full pipeline for one scenario and return a metrics dict."""
    scenario = SCENARIOS[scenario_key]()
    pre = prep.prepare_scenario(scenario)

    t0 = time.perf_counter()
    result = astar.plan_trajectory(pre, verbose=False)
    runtime = time.perf_counter() - t0

    path = result['path']
    metrics = {
        'success': result['success'],
        'runtime_s': runtime,
        'waypoints': 0,
        'collision_free': None,
        'valid': result['success'],
        'total_length_m': 0.0,
        'num_turns': 0,
    }
    if result['success'] and path:
        collision_free = _path_is_collision_free(result['planner'], path)
        metrics.update({
            'waypoints': len(path),
            'collision_free': collision_free,
            'valid': result['success'] and collision_free,
            'total_length_m': _path_length(path),
            'num_turns': _num_turns(path),
        })
    return metrics


def _print_metrics(name, m):
    valid = "yes" if m['valid'] else "no"
    print(
        f"\n[{name}] valid={valid} | "
        f"length={m['total_length_m']/1000:9.3f} km ({m['total_length_m']:.1f} m) | "
        f"turns={m['num_turns']:3d} | waypoints={m['waypoints']:3d} | "
        f"runtime={m['runtime_s']*1000:8.1f} ms | collision_free={m['collision_free']}"
    )


# --------------------------------------------------------------------------
# Golden-master baseline (captured from the current algorithm).
# Length is asserted with a small absolute tolerance; runtime is NOT asserted
# for an exact value (only an upper bound) because it is machine-dependent.
# --------------------------------------------------------------------------
LENGTH_ABS_TOL_M = 1.0          # meters
# NOTE: runtime is machine-dependent and NOT part of the locked behaviour.
# This ceiling is only a sanity guard against a pathological hang. The
# obstacle scenarios currently take ~25-140s each *because they exhaust the
# search and fail* (see the captured baseline below) -- that slowness is
# itself a baseline characteristic worth watching, not an assertion target.
RUNTIME_CEILING_S = 300.0

# Baseline captured from the current algorithm (2026-06-18), after Task 4:
# dynamic tangent successors (circle tangent points + polygon hull vertices +
# goal). The one_circle scenario is now SOLVED (valid=True). Polygon-only
# scenarios (one_island, mixed, circle_and_island) still fail — radial fallback
# runs out of iterations before routing around pure polygon obstacles.
EXPECTED = {
    'empty':      {'valid': True,  'waypoints': 2, 'num_turns': 0, 'total_length_m': 602280.4888642486},
    # one_circle: dynamic tangent successors now route around the circle (was False/0/0/0)
    'one_circle': {'valid': True,  'waypoints': 4, 'num_turns': 3, 'total_length_m': 610745.7365217851},
    'one_island': {'valid': False, 'waypoints': 0, 'num_turns': 0, 'total_length_m': 0.0},
    'mixed':      {'valid': False, 'waypoints': 0, 'num_turns': 0, 'total_length_m': 0.0},
}

# two_circles_gap: length shifted by ~2.4 m due to changed successor geometry
EXPECTED['two_circles_gap'] = {'valid': True, 'waypoints': 3, 'num_turns': 2, 'total_length_m': 602709.8018659366}
EXPECTED['circle_and_island'] = {'valid': False, 'waypoints': 0, 'num_turns': 0, 'total_length_m': 0.0}


def _check(scenario_key):
    m = measure(scenario_key)
    _print_metrics(scenario_key, m)
    exp = EXPECTED[scenario_key]
    assert m['runtime_s'] < RUNTIME_CEILING_S, \
        f"runtime {m['runtime_s']:.2f}s exceeded ceiling {RUNTIME_CEILING_S}s"
    assert m['valid'] == exp['valid'], f"valid: got {m['valid']}, expected {exp['valid']}"
    assert m['waypoints'] == exp['waypoints'], \
        f"waypoints: got {m['waypoints']}, expected {exp['waypoints']}"
    assert m['num_turns'] == exp['num_turns'], \
        f"num_turns: got {m['num_turns']}, expected {exp['num_turns']}"
    assert abs(m['total_length_m'] - exp['total_length_m']) <= LENGTH_ABS_TOL_M, \
        f"total_length_m: got {m['total_length_m']:.3f}, expected {exp['total_length_m']:.3f}"


def test_empty_field():
    _check('empty')


def test_one_circle():
    _check('one_circle')


def test_one_island():
    _check('one_island')


def test_mixed():
    _check('mixed')


def test_two_circles_gap():
    _check('two_circles_gap')


def test_circle_and_island():
    _check('circle_and_island')


# --------------------------------------------------------------------------
# Standalone report (python characterization_test.py)
# --------------------------------------------------------------------------
def main():
    print("=" * 96)
    print("  CHARACTERIZATION BASELINE - Kinodynamic A* planner")
    print("=" * 96)
    print(f"  R={config.R} m | alpha_max={config.ALPHA_MAX} deg | "
          f"SAFE_MARGIN={config.SAFE_MARGIN} m | MAX_ITER={config.MAX_ITERATIONS}")
    header = (f"\n{'Scenario':<14}{'Valid':<7}{'Length (km)':>14}"
              f"{'Turns':>8}{'Waypoints':>12}{'Runtime (ms)':>15}")
    print(header)
    print("-" * 96)
    for key in SCENARIOS:
        m = measure(key)
        valid = "yes" if m['valid'] else "no"
        print(f"{key:<14}{valid:<7}{m['total_length_m']/1000:>14.3f}"
              f"{m['num_turns']:>8}{m['waypoints']:>12}{m['runtime_s']*1000:>15.1f}")
    print("-" * 96)


if __name__ == "__main__":
    main()
