"""A/B benchmark: base optimal A* vs focal (A*epsilon) planner.

Reuses the deterministic random-scenario generator from batch_random_test so
comparisons run on identical maps. For each seed it records success, path
cost, iterations, and wall time for both planners, and verifies the focal
path never exceeds the (1 + focal_eps) bound.

Usage:  python -m ml_planner.run_ml            # default seeds 0..49
        python -m ml_planner.run_ml 200        # seeds 0..199
"""

import math
import sys
import time

import core.preprocessing as prep
import core.kinodynamic_astar as astar
from batch_random_test import generate_random_scenario
from ml_planner.plan import plan_trajectory_focal, path_length
import ml_planner.config as mlcfg


def _timed(fn):
    t0 = time.perf_counter()
    res = fn()
    return res, time.perf_counter() - t0


def _mission_cost(pre, path):
    # Corner-invariant total cost from O: include the O->first-corner takeoff
    # leg the returned body omits, so equally-optimal runs that settle on
    # different tied start corners are compared fairly.
    return math.dist(pre['start_pos'], path[0][0]) + path_length(path)


def compare_seed(seed, focal_eps=None):
    eps = mlcfg.FOCAL_EPS if focal_eps is None else focal_eps
    scenario = generate_random_scenario(seed=seed)
    pre = prep.prepare_scenario(scenario)

    base_res, base_time = _timed(lambda: astar.plan_trajectory(pre, verbose=False))
    # Re-preprocess so the two runs never share mutable state.
    pre2 = prep.prepare_scenario(scenario)
    focal_res, focal_time = _timed(lambda: plan_trajectory_focal(pre2, focal_eps=eps))

    base_ok = base_res['success']
    focal_ok = focal_res['success']
    base_cost = _mission_cost(pre, base_res['path']) if base_ok else float('nan')
    focal_cost = _mission_cost(pre2, focal_res['path']) if focal_ok else float('nan')
    cost_ratio = (focal_cost / base_cost) if (base_ok and focal_ok and base_cost > 0) else float('nan')
    within = True
    if base_ok and focal_ok and base_cost > 0:
        within = focal_cost <= (1.0 + eps) * base_cost + 1e-6

    return {
        'seed': seed,
        'base_success': base_ok,
        'focal_success': focal_ok,
        'base_cost': base_cost,
        'focal_cost': focal_cost,
        'cost_ratio': cost_ratio,
        'within_bound': within,
        'base_iters': base_res['stats']['iterations'],
        'focal_iters': focal_res['stats']['iterations'],
        'base_time': base_time,
        'focal_time': focal_time,
    }


def run_benchmark(seeds, focal_eps=None):
    return [compare_seed(s, focal_eps=focal_eps) for s in seeds]


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 50
    rows = run_benchmark(range(n))
    solved = [r for r in rows if r['base_success'] and r['focal_success']]
    viol = [r for r in rows if not r['within_bound']]
    print(f"seeds={n}  both-solved={len(solved)}  bound-violations={len(viol)}")
    if solved:
        avg_ratio = sum(r['cost_ratio'] for r in solved) / len(solved)
        base_it = sum(r['base_iters'] for r in solved)
        focal_it = sum(r['focal_iters'] for r in solved)
        base_t = sum(r['base_time'] for r in solved)
        focal_t = sum(r['focal_time'] for r in solved)
        print(f"avg cost ratio (focal/base) = {avg_ratio:.4f}")
        print(f"total iterations: base={base_it} focal={focal_it}  "
              f"({100.0 * (1 - focal_it / base_it):.1f}% fewer)" if base_it else "")
        print(f"total time (s):  base={base_t:.2f} focal={focal_t:.2f}  "
              f"({100.0 * (1 - focal_t / base_t):.1f}% faster)" if base_t else "")
    if viol:
        print(f"WARNING: {len(viol)} seeds violated the epsilon bound: "
              f"{[r['seed'] for r in viol]}")


if __name__ == '__main__':
    main()
