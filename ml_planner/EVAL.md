# Evaluating the CNN guidance model

The guidance CNN is a **ranking heuristic**: it only orders which node the
focal search expands next. It can **never** affect correctness — the ε=5%
bound and the exact collision check are independent of it. So "model quality"
is *not* "how precisely it regresses cost-to-go" (raw MSE). It is:

1. **How well it ranks** (offline), and
2. **Whether it makes the planner faster** (end-to-end).

Always evaluate on seeds **disjoint from training** (generalization).

## 1. Offline / intrinsic accuracy (fast, no planner)

On held-out labeled cells, compare the model's predicted field to the true
cost-to-go:

- **Spearman rank correlation** — the PRIMARY metric. The planner consumes the
  field only to rank nodes, so rank agreement is what matters. `> ~0.8` is
  strong; near `0` means the model adds no useful ordering over Euclid.
- **MAE (meters)** — secondary/interpretable. A model can have mediocre MAE yet
  excellent rank correlation and still guide well.

## 2. End-to-end / extrinsic (the real test)

Run base A* vs focal + hand-crafted vs focal + CNN-guided on held-out **easy**
and **hard** maps:

- **Node expansions (iterations)** — primary. Fewer = better ranking.
- **Net wall-time incl. `build_field`** — primary. The CNN pays a per-problem
  forward pass; on *easy* searches this overhead is not amortized, so guided
  can expand fewer nodes yet still be slower. On *hard* searches it amortizes.
- **Per-scenario win-rate** — in how many scenarios guided expands fewer nodes
  than hand-crafted.
- **Mission-cost ratio vs base** — must stay `<= 1.05` (the ε bound); a value
  above it is a real defect.
- **Flight-path distance** ("quãng đường bay") — the real flown km along the
  filleted O→T path. It spans the WHOLE mission (takeoff leg + interior +
  terminal DSS run-in into T), so it is larger than the search mission-cost
  (which stops at W_{n-1}); they are not directly comparable in magnitude, but
  each is measured identically across planners so the ratios are meaningful.

Split by difficulty: the CNN's value proposition differs between easy and hard.

### Go / no-go criterion

**CNN-guided must beat hand-crafted on iterations AND net wall-time on the HARD
subset.** If it wins → the CNN is the right secondary for hard maps (it is
auto-selected whenever a model is present; hand-crafted stays the fallback). If
it does not → hand-crafted remains the default and the CNN stays an optional
accelerator.

## Commands

```bash
# Full evaluation (offline rank-corr/MAE + end-to-end easy+hard + verdict);
# writes per-scenario ml_planner/data/benchmark_results.csv for drill-down.
python -m ml_planner.benchmark --offline-n 10 --bench-n 30

# Quick smoke (fewer scenarios)
python -m ml_planner.benchmark --offline-n 5 --bench-n 10

# Inspect a specific scenario's numbers
#   open ml_planner/data/benchmark_results.csv  (one row per seed+difficulty)
```

With no model in `ml_planner/models/guidance.onnx`, the benchmark still runs
base-vs-hand and reports guided == hand-crafted (clean fallback).

## Training a model (any GPU machine, off Colab)

```bash
python ml_planner/train/train_guidance.py \
    --data-dir ml_planner/data --out ml_planner/models/guidance.onnx \
    --epochs 120 --base 48
```

Consumes the `guidance_dataset*.npz` shards from `ml_planner.build_dataset`,
trains the residual-over-Euclid U-Net, and exports a single-file ONNX matching
the `channels`→`cost_to_go` / `GRID_RES²` contract.

## GNN tangent-graph guidance (prototype)

The GNN secondary runs on an explicit tangent/bitangent graph (numpy MPNN,
ms-scale build+forward) — see docs/superpowers/specs/2026-07-19-gnn-guidance-design.md.

```bash
# 1) Build the graph dataset (parallel oracle solves; ~same cost as the CNN one)
python -m ml_planner.graph_dataset 0 2400 6 2000

# 2) Train on any GPU machine (torch only needed here)
python ml_planner/train/train_graph.py \
    --data-dir ml_planner/data --out ml_planner/models/graph_guidance.npz

# 3) Evaluate: offline Spearman gate (>= 0.8) + 4-way end-to-end + acceptance
python -m ml_planner.benchmark --offline-n 10 --gnn-offline-n 10 --bench-n 30
```

Acceptance (spec §2): on hard held-out maps vs hand-crafted, the GNN must win
speed (total iterations AND net wall-time) or quality (mean cost-ratio), and
must not lose the other axis (time within +5%, cost-ratio within +0.002).
With no `models/graph_guidance.npz` the gnn columns fall back to hand-crafted.

### Prototype result (2026-07-19, models/graph_guidance.npz, train seeds 0..2399)

Offline gate PASSED (Spearman mean 0.896–0.907 on held-out hard maps) but the
end-to-end acceptance FAILED: on 23 hard held-out maps solved by all four
planners, the GNN won the quality axis vs hand-crafted (mean cost-ratio
1.0067 vs 1.0089) but lost wall-time by +38% (52.1 s vs 37.8 s; iterations
26 252 vs 16 711) — far outside the +5% not-worse margin. Structure: median
iterations are near parity (617 vs 554); the totals are dominated by a few
tail blowups where the model buys a cheaper homotopy class with extra
expansions (e.g. seed 6018: 2 088 -> 9 317 iters for cost 1.0085 -> 0.9807),
plus a fixed ~0.16 s/problem graph-build+forward cost and a ~100 µs/call kNN
lookup (+0.11 s/scenario at iso-iterations). Qualitatively the same behavior
as the CNN. Hand-crafted stays the default; the GNN (like the CNN) is a
quality-mode option, not an accelerator, in its current form.

## Lazy focal + AI corridor (bound-preserving)

Collision checks are deferred to pop time (optimistic nodes keep f_min a
valid lower bound, so the 1.05x guarantee is intact — see
docs/superpowers/specs/2026-07-19-lazy-corridor-design.md). The corridor
(GNN value field -> boolean grid) is a FOCAL ordering tiebreak (in-corridor
nodes expand first; FOCAL keeps every in-band node); a wrong model can only
cost time. Benchmark columns: `lazy_*` (mechanism baseline, no
model), `lcor_*` (lazy + corridor), plus real-check counters
`hand_checks/lazy_checks/lcor_checks` for attribution.

Acceptance: lcor must beat hand-crafted on hard-map wall-time with ZERO
epsilon-bound violations. Early-stop: if pure lazy already fails to beat
hand, stop — the corridor layer is moot.

```bash
python -m ml_planner.benchmark --offline-n 0 --gnn-offline-n 0 --bench-n 30
```

### Go/no-go result (2026-07-19, 30 seeds/tier, commits 58152a2 + 2fbb321)

The first two runs exposed two real bugs, both fixed with regression tests
before any verdict (per the plan's stop-and-debug rule):

1. **Liveness churn** (fix `58152a2`): invalidated deferred states stayed
   "live" in OPEN after their `g_scores` deletion, were re-admitted to FOCAL
   every refill and re-paid the real collision check on every pop (98k
   rejected re-validations in 321 iterations on seed 6001). This exhausted
   the wall-clock budget and reported spurious no-path on maps the eager
   planner solves — hard-tier success collapsed to 8/30. Fixed with an
   `edge_dead` flag excluded by `_is_live`.
2. **Admission gate broke the ε bound** (fix `2fbb321`): seed 6011 violated
   the bound (lcor 1.0567; eager+gate 1.0668 — the gate, not laziness, was
   the cause). In a non-reopening search, withholding in-band nodes from
   FOCAL lets worse paths permanently close lattice cells; the locked-in
   inflation is not limited by the 5% band. The corridor was demoted to a
   FOCAL ordering tiebreak (spec Amendment 2).

Final run (hard tier, 24 maps solved by all six planners):

| planner | wall time | iterations | real checks | bound violations |
|---|---|---|---|---|
| hand (eager focal) | 34.1 s | 22 261 | 838 080 | 0 |
| **lazy (pure mechanism)** | **17.7 s** | 21 483 | **116 456** | **0** |
| lcor (lazy + GNN corridor) | 41.5 s | 51 125 | 240 724 | 0 |

Easy tier: lazy 0.7 s vs hand 1.1 s, checks 5 903 vs 28 341; zero
violations anywhere.

**Verdict: the mechanism wins, the AI layer loses.** Pure lazy evaluation
beats the hand-crafted default by **-48% wall-time** and **-86% real
collision checks** on hard maps at identical quality guarantees (0
violations, same ε=5% envelope) — the speed axis the two learned-secondary
prototypes could not win. The GNN corridor's increment is negative
(2.4x the iterations of pure lazy: preferring in-corridor nodes overrides
the better-informed hand secondary ordering, plus ~0.16 s/map build cost) —
formally FAIL per spec §2, consistent with the CNN/GNN secondary verdicts
that learned guidance does not buy wall-time in this planner. **Recommended
fast mode: `plan_trajectory_lazy(pre, corridor=None)`** — no model needed.
