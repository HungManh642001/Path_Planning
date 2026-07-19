"""Phase-1 (+ Phase-2 placeholder) tunables for the ml_planner variant.

Kept separate from the root config.py so the base planner is never touched.
"""

import os

# ====== FOCAL SEARCH (A*epsilon) ======
# Bounded-suboptimality factor: the returned path cost is guaranteed
# <= (1 + FOCAL_EPS) * optimal. 0.0 reproduces exact-optimal A*.
FOCAL_EPS = 0.05
FOCAL_WEIGHT = 1.0 + FOCAL_EPS

# ====== PHASE-2 PLACEHOLDERS (CNN guidance map; unused in Phase 1) ======
# Fixed grid resolution for the per-problem cost-to-go field. Higher = finer
# cells (better resolves narrow gaps on hard maps) at higher build_field cost.
GRID_RES = 384
# Path to the exported ONNX guidance model (produced off-machine on Colab).
# Missing file => planner falls back to the hand-crafted secondary heuristic.
MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "guidance.onnx")
# Tangent-graph GNN guidance weights (numpy .npz, produced by
# ml_planner/train/train_graph.py). Missing file => planner falls back to the
# hand-crafted secondary heuristic, exactly like the CNN model above.
GRAPH_MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "graph_guidance.npz")

# ====== LAZY FOCAL + AI CORRIDOR ======
# Corridor slack: a cell belongs to the corridor when
# dist(start, cell) + V_hat(cell) <= (1 + CORRIDOR_DELTA) * V_hat(start).
CORRIDOR_DELTA = 0.15
# Boolean admission grid resolution (contains() is one array index).
CORRIDOR_GRID_RES = 128
