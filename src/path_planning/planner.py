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

from path_planning import config
from path_planning.collision.detector import CollisionDetector
from path_planning.search.astar import AstarSearchEngine
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

    def search(self) -> list[PlannerState] | None:
        """Execute the A* search loop until a path is found or time budget expires.

        Returns:
            List of (waypoint, heading) states if found, or None on failure/timeout.
        """
        return self.search_engine.search()

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
        if not self.collision_detector.is_collision_free(
            self.goal_state.waypoint, self._target
        ):
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
