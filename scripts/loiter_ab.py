"""A/B for the turn-around levers on adverse-heading scenarios.

  Lever A = LOITER_ENABLED         (virtual turning-circle turn-around macro)
  Lever B = ARC_CLEARANCE_CHECK    (reject corners whose fillet arc clips)

Compares four configs (off/off, A only, B only, A+B) on the two named
run_test.py scenarios (the ones whose PNGs the user flagged as non-optimal /
FAILED against a reference planner) plus a sample of adverse random seeds.
Reports per scenario: solved?, ORACLE-validity, path length, iters, time.

Run: PYTHONPATH=. python scripts/loiter_ab.py
"""
import ast
import math
import time

import config
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import core.path_validation as pv
from batch_random_test import generate_random_scenario

# The two run_test.py scenarios (free-goal, adverse start heading). Reference
# planner (same R/L0/DSS/alpha_max) got 179.1 / 224.0 km, both PASS; the
# current planner produced longer, oracle-INVALID paths.
KW = dict(R=8000, L0=4000, alpha_max_rad=math.pi / 2, DSS=20000)
NAMED_REF = {'scenario_test': 179.14, '2': 223.96}
RANDOM_SEEDS = [0, 1, 2, 3, 4, 6, 9]

CONFIGS = [('off/off', False, False),
           ('A only ', True, False),
           ('B only ', False, True),
           ('A+B    ', True, True)]


def _named_scenarios():
    src = open("run_test.py").read().splitlines()
    out = {}
    for lineno, name in ((208, 'scenario_test'), (209, '2')):
        line = src[lineno - 1].strip()
        if line.startswith("#"):
            line = line[1:].strip()
        out[name] = ast.literal_eval(line.split("=", 1)[1].strip())
    return out


def _validate(res, pre, kw):
    if not res.get('path'):
        return False, float('nan')
    full = astar._full_mission_path(res['path'], pre)
    pts = [w[0] for w in full]
    dist = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)) / 1000.0
    ok = pv.path_is_valid(
        full, pre['circle_obstacles'], pre['polygon_obstacles'],
        pre['turn_radius'], pre['alpha_max_rad'], config.L0, kw['DSS'],
        raw_circle_obstacles=pre.get('raw_circle_obstacles'),
        raw_polygon_obstacles=pre.get('raw_polygon_obstacles'),
        circle_tol=config.CIRCLE_GRAZE_TOL_M)
    return ok, dist


def _run(pre, kw):
    t = time.perf_counter()
    res = astar.plan_trajectory(pre, verbose=False)
    dt = time.perf_counter() - t
    ok, dist = _validate(res, pre, kw)
    it = res.get('stats', {}).get('iterations', -1) if res.get('stats') else -1
    return res.get('success'), ok, dist, it, dt, res.get('failure_reason')


def _report(title, pre, kw, ref=None):
    print("=" * 78)
    print(title + (f"   ref(other algo)={ref}km" if ref else ""))
    for lbl, lo, ar in CONFIGS:
        config.LOITER_ENABLED = lo
        config.ARC_CLEARANCE_CHECK = ar
        s, ok, d, it, dt, rs = _run(pre, kw)
        tag = "VALID" if ok else ("(invalid)" if s else "FAIL")
        print(f"  {lbl}  succ={str(s):5} {tag:9} dist={d:7.1f}km "
              f"it={it:5} t={dt:5.2f}s  {rs or ''}")


def main():
    for name, scen in _named_scenarios().items():
        pre = prep.prepare_scenario(scen, **KW)
        config.RADIAL_FAN_DIRECTIONS = 3
        config.TIME_BUDGET_S = 15
        _report(f"NAMED {name!r}", pre, KW, ref=NAMED_REF[name])

    for seed in RANDOM_SEEDS:
        scen = generate_random_scenario(seed=seed)
        pre = prep.prepare_scenario(scen)
        kw = dict(DSS=config.DSS)
        _report(f"RANDOM seed {seed}", pre, kw)


if __name__ == "__main__":
    main()
