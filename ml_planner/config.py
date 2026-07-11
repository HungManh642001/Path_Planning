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
# Fixed grid resolution for the per-problem cost-to-go field.
GRID_RES = 256
# Path to the exported ONNX guidance model (produced off-machine on Colab).
# Missing file => planner falls back to the hand-crafted secondary heuristic.
MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "guidance.onnx")
