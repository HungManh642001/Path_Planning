"""Comprehensive evaluator for the CNN guidance heuristic.

Two levels (see ml_planner/EVAL.md for the methodology):
  1. OFFLINE model accuracy  — Spearman rank correlation (the metric that
     matters for a ranking heuristic) + MAE on held-out hard maps.
  2. END-TO-END planner       — base vs focal-hand vs focal-guided on held-out
     easy AND hard seeds: iterations, wall-time (guided incl. build_field),
     search mission-cost ratio, real flight-path distance ratio, epsilon-bound
     violations, per-scenario win-rate. Splits by difficulty and prints a
     verdict. Per-scenario rows are saved to ml_planner/data/benchmark_results.csv.

Usage:
  python -m ml_planner.benchmark [--offline-n N] [--bench-n N]
                                 [--hard-start S] [--easy-start S]
                                 [--eps E] [--budget SEC]
"""
import argparse
import contextlib
import csv
import math
import os
import statistics
import time

import numpy as np

import config
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import render.trajectory as tr
from batch_random_test import generate_random_scenario
import ml_planner.dataset_gen as dg
from ml_planner.build_dataset import hard_scenario
from ml_planner.guidance import Guidance
from ml_planner.graph_guidance import GraphGuidance
from ml_planner.plan import plan_trajectory_focal, path_length

EPS = 0.05
OUT_CSV = os.path.join(os.path.dirname(__file__), "data", "benchmark_results.csv")


# ---------------------------------------------------------------- helpers ----
def mission_cost(pre, path):
    """Corner-invariant search cost: O->corner takeoff leg + interior polyline."""
    return math.dist(pre['start_pos'], path[0][0]) + path_length(path)


def flight_distance(pre, path):
    """Real flown distance: length of the filleted O->T flight path (spans the
    whole mission incl. takeoff + terminal DSS run-in, so it is larger than the
    search mission-cost; compare planners via ratios, not absolute magnitude)."""
    full = tr.build_full_path(path, pre)
    pts = tr.sample_trajectory(full, pre['turn_radius'], mode='dubins')
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


@contextlib.contextmanager
def _capped_oracle(budget=15.0):
    """Cap the data-gen oracle so a pathological held-out seed cannot hang."""
    orig = dg._no_budget

    @contextlib.contextmanager
    def capped():
        ot, oi = config.TIME_BUDGET_S, config.MAX_ITERATIONS
        config.TIME_BUDGET_S = budget
        config.MAX_ITERATIONS = 2_000_000
        try:
            yield
        finally:
            config.TIME_BUDGET_S = ot
            config.MAX_ITERATIONS = oi
    dg._no_budget = capped
    try:
        yield
    finally:
        dg._no_budget = orig


@contextlib.contextmanager
def _planner_budget(budget):
    """Give base/focal planners a common (generous) time budget for hard maps."""
    old = config.TIME_BUDGET_S
    config.TIME_BUDGET_S = budget
    try:
        yield
    finally:
        config.TIME_BUDGET_S = old


# ------------------------------------------------------- offline model eval ---
def offline_eval(guidance, n, hard_start, grid_res=None):
    """Rank correlation + MAE of the model's field vs true cost-to-go on the
    labeled cells of n held-out hard scenarios."""
    from scipy.stats import spearmanr
    gr = grid_res or guidance.grid_res
    rows = []
    for k in range(n):
        seed = hard_start + k
        sample = dg.generate_sample(hard_scenario(seed), grid_res=gr)
        if sample is None:
            continue
        pre = prep.prepare_scenario(hard_scenario(seed))
        try:
            guidance.build_field(pre)
        except Exception:
            continue
        pred = guidance.field
        x0, y0, scale, g = sample['affine']
        diag = math.sqrt(2.0) * (g / scale)
        label_n = sample['label'] / diag                 # true normalized cost-to-go
        m = sample['mask'] > 0
        p, t = pred[m], label_n[m]
        if len(t) < 5:
            continue
        rho = spearmanr(p, t).correlation
        mae = float(np.mean(np.abs(p - t)))
        rows.append(dict(seed=seed, n_labeled=int(m.sum()),
                         spearman=float(rho) if rho == rho else 0.0,
                         mae_norm=mae, mae_m=mae * diag))
    return rows


def offline_graph_eval(gg, n, hard_start):
    """Spearman of predicted node values vs snapped oracle labels on held-out
    hard seeds (spec §2 early-stop gate: mean >= 0.8)."""
    from scipy.stats import spearmanr
    from ml_planner.graph_dataset import generate_graph_sample
    rows = []
    for k in range(n):
        seed = hard_start + k
        sample = generate_graph_sample(hard_scenario(seed))
        if sample is None:
            continue
        try:
            gg.build_field(prep.prepare_scenario(hard_scenario(seed)))
        except Exception:
            continue
        m = sample['mask'] > 0
        if m.sum() < 5:
            continue
        rho = spearmanr(gg.values[m], sample['label'][m]).correlation
        rows.append(dict(seed=seed, n_labeled=int(m.sum()),
                         spearman=float(rho) if rho == rho else 0.0))
    return rows


# ------------------------------------------------------ end-to-end benchmark --
def _guided_plan(scen, guidance, eps):
    """Focal plan with the CNN field as secondary; time INCLUDES build_field.
    Falls back to hand-crafted (secondary=None) if no/failed model."""
    pre = prep.prepare_scenario(scen)
    sec = None
    if guidance is not None and guidance.available:
        try:
            guidance.build_field(pre)
            sec = lambda st: guidance.lookup(st.waypoint)
        except Exception:
            sec = None
    return plan_trajectory_focal(pre, focal_eps=eps, secondary=sec)


def _gnn_plan(scen, graph_guidance, eps):
    """Focal plan with the GNN per-node field as secondary; time INCLUDES
    build_field (graph build + numpy forward). Falls back to hand-crafted."""
    pre = prep.prepare_scenario(scen)
    sec = None
    if graph_guidance is not None and graph_guidance.available:
        try:
            graph_guidance.build_field(pre)
            sec = lambda st: graph_guidance.lookup(st.waypoint)
        except Exception:
            sec = None
    return plan_trajectory_focal(pre, focal_eps=eps, secondary=sec)


def _safe(fn):
    try:
        r, dt = None, 0.0
        t0 = time.perf_counter()
        r = fn()
        dt = time.perf_counter() - t0
        return r, dt
    except Exception:
        return None, 0.0


def compare_one(scen_func, seed, difficulty, guidance, eps, graph_guidance=None):
    scen = scen_func(seed)
    pm = prep.prepare_scenario(scen)                     # for cost/flight metrics

    rb, tb = _safe(lambda: astar.plan_trajectory(prep.prepare_scenario(scen), verbose=False))
    rh, th = _safe(lambda: plan_trajectory_focal(prep.prepare_scenario(scen), focal_eps=eps, secondary=None))
    rg, tg = _safe(lambda: _guided_plan(scen, guidance, eps))
    rn, tn = _safe(lambda: _gnn_plan(scen, graph_guidance, eps))

    def ok(r):
        return bool(r and r.get('success'))

    row = {'seed': seed, 'difficulty': difficulty,
           'base_success': ok(rb), 'hand_success': ok(rh), 'guided_success': ok(rg),
           'base_iters': rb['stats']['iterations'] if ok(rb) else '',
           'hand_iters': rh['stats']['iterations'] if ok(rh) else '',
           'guided_iters': rg['stats']['iterations'] if ok(rg) else '',
           'base_time': round(tb, 3), 'hand_time': round(th, 3), 'guided_time': round(tg, 3)}
    row['gnn_success'] = ok(rn)
    row['gnn_iters'] = rn['stats']['iterations'] if ok(rn) else ''
    row['gnn_time'] = round(tn, 3)

    for name, r in (('base', rb), ('hand', rh), ('guided', rg), ('gnn', rn)):
        row[f'{name}_mission'] = round(mission_cost(pm, r['path']), 1) if ok(r) else ''
        row[f'{name}_flight'] = round(flight_distance(pm, r['path']), 1) if ok(r) else ''

    if ok(rb) and row['base_mission']:
        bm, bf = row['base_mission'], row['base_flight']
        row['hand_cost_ratio'] = round(row['hand_mission'] / bm, 4) if ok(rh) else ''
        row['guided_cost_ratio'] = round(row['guided_mission'] / bm, 4) if ok(rg) else ''
        row['guided_flight_ratio'] = round(row['guided_flight'] / bf, 4) if ok(rg) and bf else ''
        row['hand_bound_ok'] = (row['hand_mission'] <= 1.05 * bm + 1e-6) if ok(rh) else ''
        row['guided_bound_ok'] = (row['guided_mission'] <= 1.05 * bm + 1e-6) if ok(rg) else ''
        row['gnn_cost_ratio'] = round(row['gnn_mission'] / bm, 4) if ok(rn) else ''
        row['gnn_flight_ratio'] = round(row['gnn_flight'] / bf, 4) if ok(rn) and bf else ''
        row['gnn_bound_ok'] = (row['gnn_mission'] <= 1.05 * bm + 1e-6) if ok(rn) else ''
    else:
        for c in ('hand_cost_ratio', 'guided_cost_ratio', 'guided_flight_ratio', 'hand_bound_ok', 'guided_bound_ok',
                  'gnn_cost_ratio', 'gnn_flight_ratio', 'gnn_bound_ok'):
            row[c] = ''

    row['guided_beats_hand_iters'] = (ok(rg) and ok(rh) and rg['stats']['iterations'] < rh['stats']['iterations'])
    row['gnn_beats_hand_iters'] = (ok(rn) and ok(rh) and rn['stats']['iterations'] < rh['stats']['iterations'])
    return row


def planner_benchmark(guidance, graph_guidance=None, easy_seeds=(), hard_seeds=(), eps=EPS):
    rows = []
    for seed in easy_seeds:
        rows.append(compare_one(generate_random_scenario, seed, 'easy', guidance, eps, graph_guidance=graph_guidance))
    for seed in hard_seeds:
        rows.append(compare_one(hard_scenario, seed, 'hard', guidance, eps, graph_guidance=graph_guidance))
    return rows


CSV_COLUMNS = [
    'seed', 'difficulty', 'base_success', 'hand_success', 'guided_success', 'gnn_success',
    'base_iters', 'hand_iters', 'guided_iters', 'gnn_iters',
    'base_time', 'hand_time', 'guided_time', 'gnn_time',
    'base_mission', 'hand_mission', 'guided_mission', 'gnn_mission',
    'base_flight', 'hand_flight', 'guided_flight', 'gnn_flight',
    'hand_cost_ratio', 'guided_cost_ratio', 'guided_flight_ratio',
    'hand_bound_ok', 'guided_bound_ok', 'guided_beats_hand_iters',
    'gnn_cost_ratio', 'gnn_flight_ratio', 'gnn_bound_ok', 'gnn_beats_hand_iters',
]


def write_csv(rows, path=OUT_CSV):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, '') for c in CSV_COLUMNS})
    return path


# --------------------------------------------------------------- reporting ---
def _summ(rows, diff):
    sub = [r for r in rows if r['difficulty'] == diff
           and r['base_success'] and r['hand_success'] and r['guided_success']
           and r['gnn_success']]
    print(f"\n--- {diff.upper()}  ({len(sub)} solved by all three) ---")
    if not sub:
        print("  (none)")
        return
    def col(k):
        return [r[k] for r in sub]
    it_b, it_h, it_g = sum(col('base_iters')), sum(col('hand_iters')), sum(col('guided_iters'))
    t_b, t_h, t_g = sum(col('base_time')), sum(col('hand_time')), sum(col('guided_time'))
    it_n, t_n = sum(col('gnn_iters')), sum(col('gnn_time'))
    print(f"  iterations   base={it_b}  hand={it_h}  guided={it_g}  gnn={it_n}")
    if it_h:
        print(f"    guided vs hand: {100*(1-it_g/it_h):+.1f}% iters | median guided/hand "
              f"{statistics.median(col('guided_iters'))}/{statistics.median(col('hand_iters'))}")
    print(f"  wall time(s) base={t_b:.1f}  hand={t_h:.1f}  guided={t_g:.1f}  gnn={t_n:.1f}"
          + (f"  (guided vs hand {100*(1-t_g/t_h):+.1f}%)" if t_h else ""))
    print(f"  mission-cost ratio vs base:  hand={statistics.mean(col('hand_cost_ratio')):.4f}  "
          f"guided={statistics.mean(col('guided_cost_ratio')):.4f}  "
          f"gnn={statistics.mean(col('gnn_cost_ratio')):.4f}")
    fr = [r['guided_flight_ratio'] for r in sub if r['guided_flight_ratio'] != '']
    if fr:
        print(f"  flight-distance ratio guided/base: {statistics.mean(fr):.4f}")
    print(f"  epsilon-bound violations: hand={sum(1 for r in sub if r['hand_bound_ok'] is False)}  "
          f"guided={sum(1 for r in sub if r['guided_bound_ok'] is False)}")
    wins = sum(1 for r in sub if r['guided_beats_hand_iters'])
    print(f"  guided beats hand (fewer iters): {wins}/{len(sub)} scenarios ({100*wins/len(sub):.0f}%)")
    wins_gnn = sum(1 for r in sub if r['gnn_beats_hand_iters'])
    print(f"  gnn beats hand (fewer iters): {wins_gnn}/{len(sub)}")
    return dict(it_h=it_h, it_g=it_g, t_h=t_h, t_g=t_g, wins=wins, n=len(sub),
                it_n=it_n, t_n=t_n,
                cost_h=statistics.mean(col('hand_cost_ratio')),
                cost_n=statistics.mean(col('gnn_cost_ratio')))


def gnn_acceptance(it_g, it_h, t_g, t_h, cost_g, cost_h):
    """Spec 2026-07-19 §2: win at least one axis, don't lose the other.
    speed win  = fewer total iterations AND less net wall-time than hand;
    quality win = lower mean cost-ratio than hand;
    not-worse: wall-time within +5% of hand / cost-ratio within +0.002."""
    speed_win = (it_g < it_h) and (t_g < t_h)
    quality_win = cost_g < cost_h
    time_ok = t_g <= 1.05 * t_h
    cost_ok = cost_g <= cost_h + 0.002
    return (speed_win and cost_ok) or (quality_win and time_ok)


def _verdict(hard):
    print("\n=== VERDICT (hard maps) ===")
    if not hard or hard['n'] == 0:
        print("  inconclusive — no hard scenario solved by all three.")
        return
    faster_iter = hard['it_g'] < hard['it_h']
    faster_time = hard['t_g'] < hard['t_h']
    if faster_iter and faster_time:
        print("  ✅ CNN-guided WINS on hard maps (fewer iterations AND less net wall-time).")
    elif faster_iter:
        print("  ⚠️ CNN-guided expands fewer nodes but is NOT faster in net wall-time "
              "(build_field overhead). Consider a smaller/faster field or lower GRID_RES.")
    else:
        print("  ❌ CNN-guided does NOT beat the hand-crafted heuristic on hard maps. "
              "Hand-crafted stays the default; CNN remains an optional accelerator.")

    if hard and hard['n'] and 'it_n' in hard:
        ok = gnn_acceptance(hard['it_n'], hard['it_h'], hard['t_n'], hard['t_h'],
                            hard['cost_n'], hard['cost_h'])
        print("\n=== GNN PROTOTYPE ACCEPTANCE (hard maps, vs hand-crafted) ===")
        print(f"  iters {hard['it_n']} vs {hard['it_h']} | time {hard['t_n']:.1f}s "
              f"vs {hard['t_h']:.1f}s | cost {hard['cost_n']:.4f} vs {hard['cost_h']:.4f}")
        print("  ✅ PASS — GNN wins at least one axis without losing the other."
              if ok else
              "  ❌ FAIL — GNN neither wins an axis nor holds the other (spec §2).")


def main():
    ap = argparse.ArgumentParser(description="Evaluate the CNN guidance (offline + end-to-end).")
    ap.add_argument('--offline-n', type=int, default=10)
    ap.add_argument('--gnn-offline-n', type=int, default=10)
    ap.add_argument('--bench-n', type=int, default=30, help="scenarios per difficulty")
    ap.add_argument('--hard-start', type=int, default=5000, help="held-out hard seed base (disjoint from training)")
    ap.add_argument('--easy-start', type=int, default=7000)
    ap.add_argument('--eps', type=float, default=EPS)
    ap.add_argument('--budget', type=float, default=15.0, help="planner time budget (s) for hard maps")
    args = ap.parse_args()

    g = Guidance()
    print(f"guidance model: {'AVAILABLE' if g.available else 'NOT FOUND'} "
          f"({g.model_path}) | GRID_RES={g.grid_res}")

    # 1) offline model accuracy
    if g.available and args.offline_n > 0:
        with _capped_oracle(args.budget):
            off = offline_eval(g, args.offline_n, args.hard_start)
        if off:
            import statistics as st
            print(f"\n=== OFFLINE model accuracy ({len(off)} hard held-out) ===")
            print(f"  Spearman rank corr (higher=better ranking): "
                  f"mean {st.mean(r['spearman'] for r in off):.3f}  "
                  f"min {min(r['spearman'] for r in off):.3f}")
            print(f"  MAE (meters):    mean {st.mean(r['mae_m'] for r in off):.0f}  "
                  f"| labeled/scn mean {st.mean(r['n_labeled'] for r in off):.0f}")
        else:
            print("\n=== OFFLINE model accuracy: no held-out sample solved ===")
    else:
        print("\n=== OFFLINE model accuracy skipped (no model) ===")

    gg = GraphGuidance()
    print(f"graph model:   {'AVAILABLE' if gg.available else 'NOT FOUND'} "
          f"({gg.model_path})")
    if gg.available and args.gnn_offline_n > 0:
        with _capped_oracle(args.budget):
            goff = offline_graph_eval(gg, args.gnn_offline_n, args.hard_start)
        if goff:
            import statistics as st
            print(f"\n=== OFFLINE GNN accuracy ({len(goff)} hard held-out) ===")
            print(f"  Spearman: mean {st.mean(r['spearman'] for r in goff):.3f}  "
                  f"min {min(r['spearman'] for r in goff):.3f}  (gate: mean >= 0.8)")

    # 2) end-to-end benchmark (hard seeds disjoint from the offline ones)
    easy = range(args.easy_start, args.easy_start + args.bench_n)
    hard = range(args.hard_start + 1000, args.hard_start + 1000 + args.bench_n)
    with _planner_budget(args.budget), _capped_oracle(args.budget):
        rows = planner_benchmark(g, graph_guidance=gg, easy_seeds=easy, hard_seeds=hard, eps=args.eps)
    _summ(rows, 'easy')
    hard_summ = _summ(rows, 'hard')
    _verdict(hard_summ)
    path = write_csv(rows)
    print(f"\nper-scenario results -> {path}  ({len(rows)} rows)")


if __name__ == '__main__':
    main()
