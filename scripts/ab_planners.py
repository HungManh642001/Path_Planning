"""A/B harness for the two planners: run a seeded scenario sweep, dump every
waypoint, and diff two runs.

The point of dumping full waypoint lists (not just summary stats) is that most
optimisations in this codebase are supposed to be BIT-IDENTICAL. A summary that
matches proves nothing -- two different routes can share a length to 4 decimals.
`compare` reports the exact number of seeds whose paths differ at all, and only
then falls back to comparing quality.

    python scripts/ab_planners.py run  --planner v0 --seeds 200 --out base.json
    python scripts/ab_planners.py compare base.json new.json

Scenario generation MIRRORS batch_random_test.generate_random_scenario call for
call, so a seed here is the same map that harness draws (it cannot be imported:
it pulls matplotlib at module level). Default mode is 'free' (goal_heading None)
because that is what batch_random_test -- the production harness -- runs;
'fixed' re-uses the identical map and only adds an approach heading, drawn from
a separate RNG so the obstacle field stays byte-identical between the modes.
"""

import argparse
import json
import math
import random
import time

from path_planning import config
from path_planning.core import (
    map_generator as mg,
    mission as mission,
    preprocessing as prep,
    spatial_utils as su,
)


def make_scenario(seed, mode="free"):
    """One scenario. Mirrors batch_random_test.generate_random_scenario."""
    random.seed(seed)
    map_bounds = (config.MAP_WIDTH, config.MAP_HEIGHT)
    width, height = map_bounds
    while True:
        start = (
            random.uniform(width * 0.1, width * 0.9),
            random.uniform(height * 0.1, height * 0.9),
        )
        goal = (
            random.uniform(width * 0.1, width * 0.9),
            random.uniform(height * 0.1, height * 0.9),
        )
        if 200000 < su.distance(start, goal) < 280000:
            break
    heading_start_to_goal = su.angle_to_heading(start, goal)
    topology = random.choices(
        ["random", "center_cluster", "wall_block"], weights=[0.2, 0.3, 0.5]
    )[0]
    cfg = {
        "map_bounds": map_bounds,
        "start": start,
        "start_heading": heading_start_to_goal
        + random.uniform(-math.pi / 2, math.pi / 2),
        "goal": goal,
        "goal_heading": None,
        "num_islands": random.randint(0, 20),
        "num_dynamic_obstacles": random.randint(0, 20),
        "topology": topology,
        "seed": seed,
    }
    if mode == "fixed":
        # Separate stream: the map above must not shift when a heading is added.
        cfg["goal_heading"] = heading_start_to_goal + random.Random(
            seed + 1000000
        ).uniform(-math.pi / 2, math.pi / 2)
    return mg.create_scenario(cfg)


def _load_planner(name):
    if name == "v0":
        from path_planning.core import kinodynamic_astar_v0 as m
    elif name == "main":
        from path_planning.core import kinodynamic_astar as m
    else:
        raise SystemExit(f"unknown planner {name!r}")
    return m


def _full_length(module, path, pre):
    full = mission.full_mission_path(path, pre)
    return sum(math.dist(full[i][0], full[i + 1][0]) for i in range(len(full) - 1))


def _apply_overrides(pairs):
    """Set config constants BEFORE the planner module is imported.

    Both planners derive module-level constants from config at import time
    (`_CAND_MIN_D2`, `_MIN_STRAIGHT_M`, ...), so an override applied afterwards
    would be silently ignored -- the run would look like an A/B and measure
    nothing. Hence: override, then import.
    """
    for pair in pairs or ():
        key, _, value = pair.partition("=")
        if not hasattr(config, key):
            raise SystemExit(f"config has no attribute {key!r}")
        setattr(config, key, _coerce(value))
        print(f"  config.{key} = {getattr(config, key)!r}")


def _coerce(v):
    """Parse an override literal, keeping integers INTEGRAL.

    `float()` for everything looks harmless and is not: a count that reaches
    `range()` raises TypeError, and one that only reaches arithmetic silently
    measures a different planner than the one you meant to measure. Hit for real
    with `--set GOAL_SHOT_CONE=25`.
    """
    try:
        return int(v)
    except ValueError:
        pass
    try:
        return float(v)
    except ValueError:
        return v


def run(args):
    _apply_overrides(args.set)
    module = _load_planner(args.planner)
    seeds = list(range(args.start, args.start + args.seeds))
    results = {}
    t_all = time.perf_counter()
    for seed in seeds:
        pre = prep.prepare_scenario(make_scenario(seed, args.mode))
        t0 = time.perf_counter()
        res = module.plan_trajectory(pre)
        dt = time.perf_counter() - t0
        path = res.get("path") or []
        results[str(seed)] = {
            "success": bool(res["success"]),
            "failure_reason": res["failure_reason"],
            "time_s": dt,
            "length_m": _full_length(module, path, pre) if res["success"] else None,
            "waypoints": [[w[0], w[1], h] for w, h in path],
            "iterations": res["stats"]["iterations"],
        }
        if args.verbose:
            r = results[str(seed)]
            print(
                f"  seed {seed:4d}  {'OK ' if r['success'] else 'FAIL'} "
                f"{r['failure_reason'] or ''} {dt:.2f}s"
            )
    payload = {
        "planner": args.planner,
        "mode": args.mode,
        "overrides": args.set or [],
        "seeds": [args.start, args.start + args.seeds],
        "total_time_s": time.perf_counter() - t_all,
        "results": results,
    }
    with open(args.out, "w") as fh:
        json.dump(payload, fh)
    _summary(payload)
    print(f"written: {args.out}")


def _summary(payload):
    res = payload["results"]
    solved = [k for k, v in res.items() if v["success"]]
    total_len = sum(res[k]["length_m"] for k in solved)
    print(
        f"{payload['planner']:>4} [{payload['mode']}] "
        f"seeds={len(res)} solved={len(solved)} "
        f"time={payload['total_time_s']:.1f}s "
        f"length={total_len / 1000:.1f}km "
        f"waypoints={sum(len(res[k]['waypoints']) for k in solved)}"
    )


def compare(args):
    a = json.load(open(args.base))
    b = json.load(open(args.new))
    _summary(a)
    _summary(b)
    ra, rb = a["results"], b["results"]
    keys = sorted(set(ra) & set(rb), key=int)
    if len(keys) != len(ra) or len(keys) != len(rb):
        print(f"WARNING: seed sets differ ({len(ra)} vs {len(rb)}, {len(keys)} shared)")

    identical = sum(1 for k in keys if ra[k]["waypoints"] == rb[k]["waypoints"])
    print(f"\nbit-identical paths: {identical}/{len(keys)}")
    if identical != len(keys):
        diff = [k for k in keys if ra[k]["waypoints"] != rb[k]["waypoints"]]
        print(f"  differing seeds: {diff[:20]}{' ...' if len(diff) > 20 else ''}")

    gained = [k for k in keys if rb[k]["success"] and not ra[k]["success"]]
    lost = [k for k in keys if ra[k]["success"] and not rb[k]["success"]]
    print(
        f"solved: {sum(ra[k]['success'] for k in keys)} -> "
        f"{sum(rb[k]['success'] for k in keys)}   gained={gained} lost={lost}"
    )

    both = [k for k in keys if ra[k]["success"] and rb[k]["success"]]
    if both:
        la = sum(ra[k]["length_m"] for k in both)
        lb = sum(rb[k]["length_m"] for k in both)
        wa = sum(len(ra[k]["waypoints"]) for k in both)
        wb = sum(len(rb[k]["waypoints"]) for k in both)
        ia = sum(ra[k]["iterations"] for k in both)
        ib = sum(rb[k]["iterations"] for k in both)
        print(
            f"length:     {la / 1000:.1f}km -> {lb / 1000:.1f}km  ({100 * (lb - la) / la:+.4f}%)"
        )
        print(f"waypoints:  {wa} -> {wb}")
        print(f"iterations: {ia} -> {ib}  ({100 * (ib - ia) / ia:+.2f}%)")
    ta, tb = a["total_time_s"], b["total_time_s"]
    print(
        f"time:       {ta:.1f}s -> {tb:.1f}s  ({100 * (tb - ta) / ta:+.2f}%)"
        "   [single runs drift ~5%; pair repeats before trusting this]"
    )


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="sweep seeds with one planner")
    r.add_argument("--planner", choices=["v0", "main"], required=True)
    r.add_argument("--seeds", type=int, default=200)
    r.add_argument("--start", type=int, default=0)
    r.add_argument("--mode", choices=["free", "fixed"], default="free")
    r.add_argument("--out", required=True)
    r.add_argument("--verbose", action="store_true")
    r.add_argument(
        "--set",
        action="append",
        metavar="KEY=VALUE",
        help="override a config constant before importing the planner",
    )
    r.set_defaults(func=run)

    c = sub.add_parser("compare", help="diff two run dumps")
    c.add_argument("base")
    c.add_argument("new")
    c.set_defaults(func=compare)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
