# pyright: reportMissingTypeArgument=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownParameterType=false
"""Động cơ lập kế hoạch quỹ đạo Kinodynamic A* và giao diện cấp cao.

Module cung cấp lớp :class:`KinodynamicAstar` và hàm giao diện :func:`plan_trajectory`.
Thuật toán tính toán đường bay tối ưu, không va chạm và thỏa mãn các ràng buộc
động học: bán kính quay tối thiểu R, góc chuyển hướng tối đa alpha_max,
chiều dài ổn định L0 và khoảng cách tiếp cận thẳng DSS.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from shapely.geometry import MultiPolygon, Polygon
from shapely.prepared import PreparedGeometry

from path_planning import config
from path_planning.collision.detector import CollisionDetector
from path_planning.geometry import spatial
from path_planning.search.astar import AstarSearchEngine
from path_planning.search.heuristic import euclidean_heuristic
from path_planning.search.state import State
from path_planning.search.successors import SuccessorGenerator
from path_planning.trajectory.mission_path import full_mission_path
from path_planning.trajectory.smoothing import smooth_path
from path_planning.validation import oracle


if TYPE_CHECKING:
    from path_planning.types import (
        PlannerState,
        PlanResult,
        Point,
        PreprocessedScenario,
        SearchStats,
    )

logger = logging.getLogger(__name__)


class KinodynamicAstar:
    """Bộ lập kế hoạch Kinodynamic A* trên không gian trạng thái (waypoint, heading).

    Tìm kiếm đường bay có tổng chiều dài ngắn nhất từ điểm cất cánh đến mục tiêu,
    đáp ứng các ràng buộc: bán kính quay tối thiểu R, góc chuyển hướng tối đa
    alpha_max, chiều dài ổn định sau cất cánh L0 và khoảng cách tiếp cận thẳng DSS.

    Attributes:
        scenario: Preprocessed scenario containing inflated obstacles and limits.
        time_budget_s: Maximum allocated search time in seconds.
        goal_state: Terminal goal state representation.
        collision_detector: Spatial collision detection engine.
        successor_generator: Lattice state expansion and candidate generator.
        search_engine: Core priority queue search loop and budgeting controller.
        raw_route: Unsmoothed sequence of states found by A* before shortcutting.
        start_corners: Seeded takeoff corner states along the initial climb ray.
        R: Minimum turn radius in metres.
        alpha_max_rad: Maximum allowed turning angle per corner in radians.
    """

    def __init__(
        self,
        preprocessed_scenario: PreprocessedScenario,
        time_budget_s: float | None = None,
    ) -> None:
        """Khởi tạo bộ lập kế hoạch Kinodynamic A* từ kịch bản đã tiền xử lý.

        Args:
            preprocessed_scenario: Output dictionary from
                :func:`path_planning.scenario.preprocessing.prepare_scenario`.
            time_budget_s: Maximum search duration in seconds. If None, falls back
                to :data:`path_planning.config.TIME_BUDGET_S`.

        Raises:
            ValueError: If start/goal are missing in `preprocessed_scenario`,
                or if `time_budget_s` is non-positive / invalid.
        """
        self.scenario = preprocessed_scenario
        self.time_budget_s = config.resolve_time_budget_s(
            time_budget_s if time_budget_s is not None else config.TIME_BUDGET_S
        )

        origin = preprocessed_scenario.get("start_pos")
        target = preprocessed_scenario.get("goal_pos")
        if origin is None or target is None:
            raise ValueError(
                "preprocessed scenario needs both start_pos and goal_pos; "
                "build it with path_planning.scenario.preprocessing.prepare_scenario"
            )
        self._origin: Point = origin
        self._target: Point = target
        self._l0 = preprocessed_scenario["start_state"].get(
            "straight_length", config.L0
        )
        self._dss = preprocessed_scenario["goal_state"].get(
            "engagement_distance", config.DSS
        )
        self.R = preprocessed_scenario["turn_radius"]
        self.alpha_max_rad = preprocessed_scenario["alpha_max_rad"]
        self._alpha_build = self.alpha_max_rad - config.GEOM_EPS_RAD
        self._free_goal: bool = (
            preprocessed_scenario.get("goal_heading") is None
            or preprocessed_scenario.get("is_goal_heading_free", False)
            or preprocessed_scenario["goal_state"]["heading"] is None
        )

        goal_wp = preprocessed_scenario["goal_state"]["waypoint"]
        goal_h = (
            None if self._free_goal else preprocessed_scenario["goal_state"]["heading"]
        )
        self.goal_state = State(goal_wp, goal_h)

        # Collision & Successors
        self.collision_detector = CollisionDetector(
            preprocessed_scenario, turn_radius=self.R
        )
        self.successor_generator = SuccessorGenerator(
            preprocessed_scenario,
            self.collision_detector,
            turn_radius=self.R,
            alpha_max_rad=self.alpha_max_rad,
            l0=self._l0,
            dss=self._dss,
            origin=self._origin,
            target=self._target,
            goal_state=self.goal_state,
            is_goal_heading_free=self._free_goal,
        )

        self.raw_route: list[PlannerState] | None = None
        self.start_corners = self.successor_generator.seed_start_corners()
        self.search_engine = AstarSearchEngine(
            self.start_corners,
            self.goal_state,
            self.successor_generator,
            self.collision_detector,
            time_budget_s=self.time_budget_s,
            origin=self._origin,
            target=self._target,
            is_goal_heading_free=self._free_goal,
            turn_radius=self.R,
            dss=self._dss,
            l0=self._l0,
            alpha_build=self._alpha_build,
        )

    # --- Delegated properties and methods for 100% backward compatibility ---

    @property
    def _turn_cos_guard(self) -> float:
        """Return cosine threshold guard for fast turn prefiltering."""
        return self.successor_generator.turn_cos_guard

    @property
    def _shot_armed(self) -> bool:
        """Return whether analytic two-corner goal shot is armed."""
        return self.search_engine.shot_armed

    @_shot_armed.setter
    def _shot_armed(self, value: bool) -> None:
        self.search_engine.shot_armed = value

    @property
    def num_strategy_b(self) -> int:
        """Return remaining Strategy B radial fan expansion quota."""
        return self.successor_generator.num_strategy_b

    @num_strategy_b.setter
    def num_strategy_b(self, value: int) -> None:
        self.successor_generator.num_strategy_b = value

    @property
    def _fan_rungs(self) -> list[float]:
        """Return distance rungs for radial fan exploration in metres."""
        return self.successor_generator.fan_rungs

    @property
    def _last_reject(self) -> str | None:
        """Return rejection reason identifier for the most recent candidate state."""
        return self.successor_generator.last_reject

    @property
    def _poly_vertices(self) -> list[Point]:
        """Return inflated obstacle polygon vertices used for candidate generation."""
        return self.successor_generator.poly_vertices

    @property
    def _polygons(self) -> list[Polygon]:
        """Return Shapely polygon instances of static obstacles."""
        return self.collision_detector.polygons

    @property
    def _poly_bboxes(self) -> list[tuple[float, float, float, float]]:
        """Return bounding boxes (minx, miny, maxx, maxy) of polygon obstacles."""
        return self.collision_detector.poly_bboxes

    @property
    def _circles(self) -> list[tuple[float, float, float]]:
        """Return circular obstacles as (center_x, center_y, radius) in metres."""
        return self.collision_detector.circles

    @property
    def _safezone(self) -> Polygon | MultiPolygon | None:
        """Return combined operational safezone boundary polygon if configured."""
        return self.collision_detector.safezone

    @property
    def _safezone_prep(self) -> PreparedGeometry | None:
        """Return prepared spatial geometry of safezone for fast containment queries."""
        return self.collision_detector.safezone_prep

    @property
    def _has_explicit_bounds(self) -> bool:
        """Return True if map dimensions were explicitly specified in the scenario."""
        return self.collision_detector.has_explicit_bounds

    @property
    def _bounds_w(self) -> float:
        """Return operational area width in metres."""
        return self.collision_detector.bounds_w

    @property
    def _bounds_h(self) -> float:
        """Return operational area height in metres."""
        return self.collision_detector.bounds_h

    @property
    def _construct_delta(self) -> float:
        """Return construction stand-off buffer added to obstacles in metres."""
        return self.collision_detector.construct_delta

    @property
    def open_set(self) -> list[tuple[float, int, State]]:
        """Return the priority queue of pending states sorted by f-score."""
        return self.search_engine.open_set

    @property
    def closed_set(self) -> set[State]:
        """Return the set of already expanded lattice states."""
        return self.search_engine.closed_set

    @property
    def g_scores(self) -> dict[State, float]:
        """Return map of lowest known cost-to-come per lattice state."""
        return self.search_engine.g_scores

    @property
    def iteration_count(self) -> int:
        """Return total number of main loop iterations executed."""
        return self.search_engine.iteration_count

    @iteration_count.setter
    def iteration_count(self, value: int) -> None:
        self.search_engine.iteration_count = value

    @property
    def is_budget_bound(self) -> bool:
        """Return True if search terminated due to time budget limit exhaustion."""
        return self.search_engine.is_budget_bound

    @property
    def is_search_failed(self) -> bool:
        """Return True if the open priority queue emptied without finding a path."""
        return self.search_engine.is_search_failed

    def heuristic(self, state: State, goal_state: State) -> float:
        """Estimate remaining Euclidean flight distance from state to goal.

        Args:
            state: Current lattice state.
            goal_state: Target goal state.

        Returns:
            Admissible straight-line distance heuristic in metres.
        """
        return euclidean_heuristic(state, goal_state)

    def get_next_states(self, current_state: State) -> list[tuple[State, float]]:
        """Generate all kinematically feasible successor states from current state.

        Args:
            current_state: State currently being expanded.

        Returns:
            List of tuples (successor_state, step_cost).
        """
        return self.successor_generator.get_next_states(current_state)

    def _is_collision_free(self, p1: Point, p2: Point) -> bool:
        """Test whether the straight line segment p1 -> p2 is collision-free."""
        return self.collision_detector.is_collision_free(p1, p2)

    def _is_corner_arc_clear(self, h_in: float, w: Point, w_next: Point) -> bool:
        """Test whether radius-R fillet arc rounding corner w is collision-free."""
        return self.collision_detector.is_corner_arc_clear(h_in, w, w_next)

    def _is_sector_clear(
        self, center: Point, r_in: float, r_out: float, phi_a: float, phi_b: float
    ) -> bool:
        """Test whether an annular sector around circle center is free of obstacles."""
        return self.collision_detector.is_sector_clear(
            center, r_in, r_out, phi_a, phi_b
        )

    def _is_in_bounds(self, point: Point) -> bool:
        """Test whether point lies within valid operational boundaries."""
        return self.collision_detector.is_in_bounds(point)

    def _on_circle_boundary(self, point: Point, tol: float | None = None) -> bool:
        """Test whether point lies on the outer boundary of any circular obstacle."""
        return self.collision_detector.on_circle_boundary(point, tol)

    def _ray_chord_clear(
        self,
        memo: dict[float, list[float]],
        ray: float,
        dist: float,
        p1: Point,
        p2: Point,
    ) -> bool:
        """Collision-test a chord, reusing what is already known about its ray."""
        span = memo.get(ray)
        if span is None:
            span = memo[ray] = [0.0, float("inf")]
        if dist <= span[0]:
            return True
        if dist >= span[1]:
            return False
        if self._is_collision_free(p1, p2):
            span[0] = dist
            return True
        span[1] = dist
        return False

    def _check_fixed_legs(self) -> bool:
        """Test whether mandatory takeoff and approach legs are collision-free."""
        return self.collision_detector.check_fixed_legs(
            self.goal_state.waypoint, self._target
        )

    def _seed_start_corners(self) -> list[State]:
        """Seed initial search states along takeoff ray with straight length >= L0."""
        return self.successor_generator.seed_start_corners()

    def _doan_trinh(
        self,
        current: State,
        leg_len: float,
        turn: float,
        far_reserve: float = 0.0,
        advance: float = 0.0,
    ) -> float | None:
        """Validate and update straight budget for a candidate turn and leg length."""
        return self.successor_generator.doan_trinh(
            current, leg_len, turn, far_reserve, advance
        )

    def _pivot_candidate(
        self, current: State, node: Point, advance: float
    ) -> tuple[State, float] | None:
        """Validate a candidate waypoint transition from current state to node."""
        return self.successor_generator.pivot_candidate(current, node, advance)

    def _slide_pivot(self, current: State, node: Point) -> tuple[State, float] | None:
        """Attempt to recover arc-blocked candidate by sliding pivot along ray."""
        return self.successor_generator.slide_pivot(current, node)

    def _try_goal_shot(self, current: State) -> State | None:
        """Attempt direct two-corner terminal connection to target goal."""
        return self.successor_generator.try_goal_shot(current, {}, {})

    def search(self) -> list[PlannerState] | None:
        """Execute the A* search loop until a path is found or time budget expires.

        Returns:
            List of (waypoint, heading) states if found, or None on failure/timeout.
        """
        return self.search_engine.search()

    def _is_goal_reached(self, current: State) -> bool:
        """Check whether current state satisfies terminal arrival conditions."""
        return self.search_engine.is_goal_reached(current)

    def _reconstruct_path(self, state: State) -> list[PlannerState]:
        """Backtrack parent pointers from terminal state to build waypoint path."""
        return self.search_engine.reconstruct_path(state)

    def get_search_stats(self) -> SearchStats:
        """Return diagnostic metrics and counters from search execution.

        Returns:
            Dictionary containing search iteration count, set sizes, and budget status.
        """
        return self.search_engine.get_search_stats()

    def smooth_path(self, path: list[PlannerState]) -> list[PlannerState]:
        """Optimize and shortcut path using dynamic programming subsequence selection.

        Args:
            path: Raw waypoint sequence from A* search.

        Returns:
            Optimized sequence of waypoints maintaining all kinematic invariants.
        """
        start_h = self.scenario["start_state"]["heading"]
        goal_h = self.scenario.get("goal_heading")
        return smooth_path(
            path,
            self._origin,
            self._target,
            self.collision_detector,
            turn_radius=self.R,
            alpha_max_rad=self.alpha_max_rad,
            l0=self._l0,
            dss=self._dss,
            start_heading=start_h,
            goal_heading=goal_h,
            is_goal_heading_free=self._free_goal,
        )

    def plan(self, *, verbose: bool = False) -> PlanResult:
        """Thực thi toàn bộ quy trình: kiểm tra, tìm kiếm, làm mượt và kiểm định.

        Args:
            verbose: If True, log detailed search progress to standard logger.

        Returns:
            PlanResult dictionary containing trajectory path, success flag, reason,
            and execution statistics.
        """
        if not self.start_corners:
            return self._result(None, False, "start_leg_blocked")
        if not self._check_fixed_legs():
            return self._result(None, False, "goal_leg_blocked")

        if verbose:
            logger.info("Starting A* search...")
        path = self.search()
        if verbose:
            stats = self.get_search_stats()
            logger.info(
                f"Search completed: {stats['iterations']} iterations in "
                f"<= {stats['time_budget_s']:g} s"
                + (" (budget exhausted)" if stats["is_budget_bound"] else "")
            )
        if path is None:
            return self._result(None, False, "no_path")
        self.raw_route = list(path)
        path = self.smooth_path(path)
        full = full_mission_path(path, self.scenario)
        res = oracle.path_is_valid(
            full,
            self.scenario["circle_obstacles"],
            self.scenario["polygon_obstacles"],
            turn_radius=self.scenario["turn_radius"],
            alpha_max_rad=self.scenario["alpha_max_rad"],
            l0=self._l0,
            dss=self._dss,
        )
        if not res.is_ok:
            return self._result(full, False, res.detail)

        if verbose:
            logger.info(f"Path found with {len(full)} waypoints")
        return self._result(full, True, None)

    def _result(
        self, path: list[PlannerState] | None, is_success: bool, reason: str | None
    ) -> PlanResult:
        """Package internal search outcomes into canonical PlanResult dictionary."""
        return {
            "path": path,
            "is_success": is_success,
            "failure_reason": reason,
            "stats": self.get_search_stats(),
            "planner": self,
        }


def plan_trajectory(
    preprocessed_scenario: PreprocessedScenario,
    *,
    verbose: bool = False,
    time_budget_s: float | None = None,
) -> PlanResult:
    """Lập kế hoạch đường bay tự hành hoàn chỉnh từ kịch bản tiền xử lý.

    Args:
        preprocessed_scenario: Prepared mission dictionary containing endpoints,
            inflated obstacles, turn limits, and safezone geometries.
        verbose: If True, logs step-by-step solver progression.
        time_budget_s: Wall-clock computation budget limit in seconds.

    Returns:
        Canonical PlanResult containing the smoothed flyable path and diagnostics.
    """
    if verbose:
        logger.info("Initializing Kinodynamic A*...")
    return KinodynamicAstar(preprocessed_scenario, time_budget_s=time_budget_s).plan(
        verbose=verbose
    )


# Aliases for test compatibility
_angle_diff = spatial.angle_diff
_MIN_STRAIGHT_M = config.MIN_STRAIGHT_M
