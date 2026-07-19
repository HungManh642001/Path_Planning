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
