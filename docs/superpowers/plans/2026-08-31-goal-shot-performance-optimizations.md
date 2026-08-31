# Plan: Goal-Shot Performance Optimizations

## Task List

- [ ] **Task 1: Update `geometry.goal_shot` with dataclass fields, `build_goal_cone`, and optimized candidates generation**
  - Update `TwoCornerCandidate` to include `leg1_len: float` and `leg2_len: float`.
  - Add `build_goal_cone(goal_heading, turn_radius, alpha_max, num_cone)`.
  - Update `two_corner_candidates` to accept optional precomputed `cone`, eliminate redundant `_angdiff`, and return `leg1_len`/`leg2_len`.
  - Update unit tests in `tests/path_planning/unit/geometry/test_goal_shot.py`.
  - Verify with pytest.

- [ ] **Task 2: Update `SuccessorGenerator` to precompute cone and use `leg1_len`/`leg2_len`**
  - Precompute `self.goal_shot_cone` in `SuccessorGenerator.__init__`.
  - In `try_goal_shot`, pass `self.goal_shot_cone` to `two_corner_candidates`.
  - Replace `math.dist(position, corner)` and `math.dist(corner, goal_wp)` with `candidate.leg1_len` and `candidate.leg2_len`.
  - Verify with pytest.

- [ ] **Task 3: Update `AstarSearchEngine` with search-scoped `leg2_memo`**
  - Add `self.leg2_memo` in `AstarSearchEngine.__init__`.
  - Pass `self.leg2_memo` to `try_goal_shot` across iterations.
  - Verify with pytest.

- [ ] **Task 4: Full Test Suite Verification, Linting, and Type Checking**
  - Run `ruff check .` and `ruff format .`.
  - Run `pyright`.
  - Run `pytest tests/ -v`.
  - Commit all changes with standard commit messages.
