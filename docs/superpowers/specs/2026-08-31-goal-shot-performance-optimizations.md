# Spec: Goal-Shot Performance Optimizations

## 1. Context & Motivation
The `goal_shot` mechanism is an analytical two-corner shortcut used in `KinodynamicAstar` to rapidly connect expanding states to the final mission goal under heading reversal constraints.
Profiling and algorithmic analysis revealed 4 computational bottlenecks:
1. Re-evaluating terminal arrival cone trigonometry on every node expansion.
2. Invoking expensive `_angdiff` (atan2/sin/cos) for linear angle increments.
3. Re-calculating Euclidean distances (`math.dist`) already obtained from ray intersection linear solver.
4. Resetting ray memoization cache for leg 2 on every node expansion despite terminal destination and arrival headings being constant.

## 2. Technical Architecture & Modifications

### 2.1 `path_planning.geometry.goal_shot`
- **`TwoCornerCandidate` Dataclass**:
  - Add `leg1_len: float` and `leg2_len: float`.
- **`build_goal_cone`**:
  - Helper function `build_goal_cone(goal_heading: float, turn_radius: float, alpha_max: float, num_cone: int) -> list[tuple[float, float, float, float]]`.
- **`two_corner_candidates`**:
  - Accept optional `cone: Sequence[tuple[float, float, float, float]] | None = None`.
  - Replace `turn_at_position = abs(_angdiff(leg1_heading, heading))` with `abs(offset)`.
  - Populate `leg1_len` and `leg2_len` in returned `TwoCornerCandidate` instances.

### 2.2 `path_planning.search.successors`
- **`SuccessorGenerator.__init__`**:
  - Precompute `self.goal_shot_cone` using `build_goal_cone`.
- **`SuccessorGenerator.try_goal_shot`**:
  - Pass `self.goal_shot_cone` into `two_corner_candidates`.
  - Use `candidate.leg1_len` and `candidate.leg2_len` instead of `math.dist(...)`.

### 2.3 `path_planning.search.astar`
- **`AstarSearchEngine.__init__`**:
  - Initialize `self.leg2_memo: dict[float, list[float]] = {}`.
- **`AstarSearchEngine.search`**:
  - Pass `self.leg2_memo` across iterations into `self.successors.try_goal_shot(current, {}, self.leg2_memo)`.

## 3. Verification Criteria
- All existing and new unit tests in `tests/path_planning/unit/geometry/test_goal_shot.py` pass.
- All 205+ tests in the test suite pass with 100% success rate.
- Benchmark and runtime checks verify zero regression in trajectory optimality, safety, and validity.
