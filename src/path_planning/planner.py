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
        scenario: Kịch bản tiền xử lý chứa chướng ngại vật đã giãn nở và các giới hạn.
        time_budget_s: Thời gian tìm kiếm tối đa được phân bổ tính bằng giây.
        goal_state: Biểu diễn trạng thái đích cuối cùng.
        collision_detector: Động cơ phát hiện va chạm không gian.
        successor_generator: Bộ sinh trạng thái kế tiếp và ứng viên trên lưới.
        search_engine: Vòng lặp tìm kiếm hàng đợi ưu tiên và bộ kiểm soát quỹ thời gian.
        raw_route: Chuỗi trạng thái thô chưa làm mịn tìm thấy bởi A* trước khi đi tắt.
        start_corners: Các trạng thái góc cất cánh được gieo mầm dọc theo tia
            leo cao ban đầu.
        R: Bán kính quay vòng tối thiểu tính bằng mét.
        alpha_max_rad: Góc chuyển hướng tối đa cho phép tại mỗi góc cua (rad).
    """

    def __init__(
        self,
        preprocessed_scenario: PreprocessedScenario,
        time_budget_s: float | None = None,
    ) -> None:
        """Khởi tạo bộ lập kế hoạch Kinodynamic A* từ kịch bản đã tiền xử lý.

        Args:
            preprocessed_scenario: Dictionary đầu ra từ
                :func:`path_planning.scenario.preprocessing.prepare_scenario`.
            time_budget_s: Thời lượng tìm kiếm tối đa tính bằng giây. Nếu là None,
                sử dụng :data:`path_planning.config.TIME_BUDGET_S`.

        Raises:
            ValueError: Nếu thiếu start/goal trong `preprocessed_scenario`,
                hoặc nếu `time_budget_s` không dương / không hợp lệ.
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
        """Thực thi vòng lặp tìm kiếm A* cho đến khi tới đích hoặc hết thời gian.

        Returns:
            Danh sách các trạng thái (waypoint, heading) nếu tìm thấy,
            hoặc None nếu thất bại / hết giờ.
        """
        return self.search_engine.search()

    def get_search_stats(self) -> SearchStats:
        """Trả về các số liệu thống kê và bộ đếm chẩn đoán từ quá trình tìm kiếm.

        Returns:
            Dictionary chứa số lượt lặp tìm kiếm, kích thước tập hợp và
            trạng thái quỹ thời gian.
        """
        return self.search_engine.get_search_stats()

    def smooth_path(self, path: list[PlannerState]) -> list[PlannerState]:
        """Tối ưu và đi tắt đường bay bằng quy hoạch động chọn dãy con.

        Chỉ chạy thuật toán làm mịn khi số điểm waypoint thô nằm trong giới hạn
        an toàn xử lý để kiểm soát thời gian tính toán.

        Args:
            path: Chuỗi trạng thái thô (waypoint, heading) dọc theo đường bay.

        Returns:
            Chuỗi trạng thái rút gọn đã làm mịn thỏa mãn mọi ràng buộc động học.
        """
        raw_waypoints = [w for w, _ in path]
        if len(raw_waypoints) > config.SMOOTH_MAX_NODES:
            return path
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
            verbose: Nếu True, ghi nhật ký chi tiết tiến trình tìm kiếm ra
                logger tiêu chuẩn.

        Returns:
            Dictionary PlanResult chứa đường bay quỹ đạo, cờ thành công, lý do,
            và số liệu thống kê thực thi.
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
        """Đóng gói kết quả tìm kiếm nội bộ vào dictionary chuẩn PlanResult."""
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
        preprocessed_scenario: Dictionary kịch bản nhiệm vụ đã tiền xử lý chứa
            điểm đầu cuối, vật cản đã giãn nở, giới hạn góc rẽ và vùng an toàn.
        verbose: Nếu True, ghi log chi tiết từng bước giải thuật toán.
        time_budget_s: Giới hạn quỹ thời gian tính toán thực tế tính bằng giây.

    Returns:
        Dictionary PlanResult chuẩn chứa đường bay đã làm mịn khả thi
        và số liệu chẩn đoán.
    """
    if verbose:
        logger.info("Initializing Kinodynamic A*...")
    return KinodynamicAstar(preprocessed_scenario, time_budget_s=time_budget_s).plan(
        verbose=verbose
    )
