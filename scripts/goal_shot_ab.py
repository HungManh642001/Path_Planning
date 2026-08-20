"""A/B: goal-shot ON vs OFF on the adverse-heading random seeds.

Reports, per seed: solved?, iterations, planning time, path length, and
oracle validity. Flood seeds should flip from FAIL/slow to solved-fast with no
length regression on the seeds that already passed.

Run: PYTHONPATH=. python scripts/goal_shot_ab.py
"""
import math
import time

import config
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import core.path_validation as pv
import render.trajectory as tr
from batch_random_test import generate_random_scenario

# Seeds 5, 7, 8 are FEASIBILITY failures (start in obstacle / DSS leg blocked),
# out of scope for the shot; skip them here.
SEEDS = [0, 1, 2, 3, 4, 6, 9]


def _run(seed):
    scen = generate_random_scenario(seed=seed)
    pre = prep.prepare_scenario(scen)
    t0 = time.perf_counter()
    res = astar.plan_trajectory(pre)
    dt = time.perf_counter() - t0
    plen = 0.0
    valid = None
    if res['success'] and res['path']:
        for a, b in zip(res['path'][:-1], res['path'][1:]):
            plen += math.dist(a[0], b[0])
        full = tr.build_full_path(res['path'], pre)
        # Same verdict plan_trajectory reaches: INFLATED obstacles for straights
        # AND arcs. This used to pass raw_circle_obstacles/raw_polygon_obstacles,
        # which path_is_valid documents as a legacy escape hatch for reproducing
        # the old inflation model -- it validates arcs against the uninflated
        # obstacle, so a path dipping inside SAFE_MARGIN reads as valid here and
        # invalid to the planner. Invisible today only because SAFE_MARGIN is 0.
        valid, _reason = pv.path_is_valid(
            full, pre['circle_obstacles'], pre['polygon_obstacles'],
            config.R, config.ALPHA_MAX_RAD, config.L0, config.DSS)
    return res['success'], res['stats']['iterations'], dt, plen, valid


def main():
    results = {}  # enabled -> {seed: (ok, it, dt, plen, valid)}
    for enabled in (False, True):
        config.GOAL_SHOT_ENABLED = enabled
        print(f"\n=== GOAL_SHOT_ENABLED = {enabled} ===")
        per_seed = {}
        for seed in SEEDS:
            ok, it, dt, plen, valid = _run(seed)
            per_seed[seed] = (ok, it, dt, plen, valid)
            print(f"  seed {seed}: solved={ok!s:5s} iters={it:6d} "
                  f"t={dt:5.2f}s len={plen/1000:7.1f}km oracle={valid}")
        results[enabled] = per_seed

        solved = [v for v in per_seed.values() if v[0]]
        n_solved = len(solved)
        mean_it = (sum(v[1] for v in solved) / n_solved) if n_solved else float('nan')
        mean_t = (sum(v[2] for v in solved) / n_solved) if n_solved else float('nan')
        print(f"  --- SUMMARY (shot={enabled}): solved {n_solved}/{len(SEEDS)}, "
              f"mean iters (solved) = {mean_it:.1f}, mean time (solved) = {mean_t:.2f}s")

    off, on = results[False], results[True]
    print("\n=== PER-SEED COMPARISON (OFF -> ON) ===")
    header = (f"  {'seed':>4} {'off':>6} {'on':>6} {'speedup':>9} "
              f"{'len_off(km)':>12} {'len_on(km)':>12} {'delta(km)':>10} "
              f"{'ok_off':>7} {'ok_on':>6}")
    print(header)
    for seed in SEEDS:
        ok_off, it_off, _, plen_off, _ = off[seed]
        ok_on, it_on, _, plen_on, _ = on[seed]
        if it_off > 0 and it_on > 0:
            speedup = it_off / it_on
            speedup_s = f"{speedup:8.2f}x"
        else:
            speedup_s = f"{'n/a':>9}"
        len_off_km = plen_off / 1000 if ok_off else float('nan')
        len_on_km = plen_on / 1000 if ok_on else float('nan')
        delta_km = (len_on_km - len_off_km) if (ok_off and ok_on) else float('nan')
        print(f"  {seed:>4} {it_off:>6} {it_on:>6} {speedup_s} "
              f"{len_off_km:>12.1f} {len_on_km:>12.1f} {delta_km:>10.1f} "
              f"{str(ok_off):>7} {str(ok_on):>6}")

    n_off = sum(1 for v in off.values() if v[0])
    n_on = sum(1 for v in on.values() if v[0])
    solved_off = [v for v in off.values() if v[0]]
    solved_on = [v for v in on.values() if v[0]]
    mean_it_off = (sum(v[1] for v in solved_off) / len(solved_off)) if solved_off else float('nan')
    mean_it_on = (sum(v[1] for v in solved_on) / len(solved_on)) if solved_on else float('nan')
    print("\n=== OVERALL SUMMARY ===")
    print(f"  OFF: solved {n_off}/{len(SEEDS)}, mean iters (solved) = {mean_it_off:.1f}")
    print(f"  ON : solved {n_on}/{len(SEEDS)}, mean iters (solved) = {mean_it_on:.1f}")
    if solved_off and solved_on:
        print(f"  Mean iters speedup (OFF/ON, over each side's own solved set) = "
              f"{mean_it_off / mean_it_on:.2f}x")


if __name__ == "__main__":
    main()
