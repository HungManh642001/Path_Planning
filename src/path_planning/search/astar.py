"""Động cơ tìm kiếm đồ thị A* động học với hàng đợi ưu tiên và hạn mức thời gian."""

from __future__ import annotations

import heapq
import math
import time
from collections import defaultdict
from collections.abc import Callable
from typing import TYPE_CHECKING

from path_planning import config
from path_planning.geometry import spatial
from path_planning.search.heuristic import euclidean_heuristic
from path_planning.search.state import State


if TYPE_CHECKING:
    from path_planning.collision.detector import CollisionDetector
    from path_planning.search.successors import SuccessorGenerator
    from path_planning.types import PlannerState, Point, SearchStats


class AstarSearchEngine:
    """Vòng lặp tìm kiếm A* chính trên hàng đợi ưu tiên kèm kiểm soát deadline.

    Attributes:
        start_corners: Danh sách các trạng thái điểm rẽ xuất phát được gieo mầm.
        goal_state: Trạng thái đích mục tiêu.
        successors: Bộ sinh các ứng viên trạng thái kế tiếp.
        collision: Bộ kiểm tra va chạm không gian hình học.
        time_budget_s: Thời gian tìm kiếm tối đa cho phép tính bằng giây.
        origin: Tọa độ điểm cất cánh của máy bay (O).
        target: Tọa độ đích mục tiêu nhiệm vụ (T).
        is_goal_heading_free: Cho biết hướng tiếp cận đích có tự do hay không.
        turn_radius: Bán kính quay vòng tối thiểu tính bằng mét.
        dss: Chiều dài đoản trình tiếp cận cảm biến thẳng về đích (m).
        l0: Chiều dài đoạn bay thẳng ổn định sau cất cánh (m).
        alpha_build: Giới hạn góc rẽ dùng trong khâu dựng hình (rad).
        heuristic_fn: Hàm heuristic ước lượng khoảng cách tới đích.
        open_set: Hàng đợi ưu tiên chứa các trạng thái mở sắp theo f-score.
        closed_set: Tập hợp các trạng thái đã duyệt đóng.
        g_scores: Bản đồ lưu chi phí g_cost nhỏ nhất đã biết tới mỗi State.
        iteration_count: Số vòng lặp tìm kiếm đã thực thi.
        nodes_expanded: Số lượng nút trạng thái đã mở rộng từ open_set.
        is_budget_bound: True nếu tìm kiếm bị dừng sớm do hết thời gian.
        is_search_failed: True nếu hàng đợi rỗng mà không tìm thấy đường tới đích.
        shot_armed: True nếu cơ chế giải tích bắn 2 góc rẽ về đích được kích hoạt.
    """

    def __init__(
        self,
        start_corners: list[State],
        goal_state: State,
        successor_generator: SuccessorGenerator,
        collision_detector: CollisionDetector,
        *,
        time_budget_s: float,
        origin: Point,
        target: Point,
        is_goal_heading_free: bool = False,
        turn_radius: float = config.R,
        dss: float = config.DSS,
        l0: float = config.L0,
        alpha_build: float,
        heuristic_fn: Callable[[State, State], float] = euclidean_heuristic,
    ) -> None:
        """Khởi tạo động cơ tìm kiếm A* với các điểm rẽ xuất phát và hạn mức thời gian.

        Args:
            start_corners: Danh sách các trạng thái điểm rẽ xuất phát.
            goal_state: Trạng thái đích mục tiêu.
            successor_generator: Thực thể bộ sinh trạng thái kế tiếp.
            collision_detector: Thực thể bộ phát hiện va chạm hình học.
            time_budget_s: Hạn mức thời gian tìm kiếm tối đa (giây).
            origin: Tọa độ điểm cất cánh O.
            target: Tọa độ điểm mục tiêu đích T.
            is_goal_heading_free: True nếu hướng tiếp cận đích không bị ràng buộc.
            turn_radius: Bán kính quay vòng tối thiểu của phương tiện (m).
            dss: Khoảng cách bay thẳng đoản trình khóa mục tiêu (m).
            l0: Chiều dài đoạn bay thẳng ổn định sau cất cánh (m).
            alpha_build: Góc rẽ tối đa cho phép khi dựng hình (rad).
            heuristic_fn: Hàm heuristic ước tính chi phí (state, goal) -> float.
        """
        self.start_corners = start_corners
        self.goal_state = goal_state
        self.successors = successor_generator
        self.collision = collision_detector
        self.time_budget_s = time_budget_s
        self.origin = origin
        self.target = target
        self.is_goal_heading_free = is_goal_heading_free
        self.turn_radius = turn_radius
        self.dss = dss
        self.l0 = l0
        self.alpha_build = alpha_build
        self.heuristic_fn = heuristic_fn

        self.open_set: list[tuple[float, int, State]] = []
        self.closed_set: set[State] = set()
        self.g_scores: defaultdict[State, float] = defaultdict(lambda: float("inf"))
        self.iteration_count = 0
        self.nodes_expanded = 0
        self.is_budget_bound = False
        self.is_search_failed = False
        self.leg2_memo: dict[float, list[float]] = {}

        self.shot_armed = False
        if not self.is_goal_heading_free:
            goal_h = self.goal_state.heading
            if goal_h is not None:
                travel = spatial.angle_to_heading(self.origin, self.target)
                reversal = abs(spatial.angle_diff(goal_h, travel))
                self.shot_armed = reversal >= config.deg_to_rad(
                    config.GOAL_SHOT_MIN_REVERSAL_DEG
                )

        for corner in self.start_corners:
            corner.h_cost = self.heuristic_fn(corner, self.goal_state)
            heapq.heappush(
                self.open_set,
                (
                    corner.g_cost + config.HEURISTIC_WEIGHT * corner.h_cost,
                    self.iteration_count,
                    corner,
                ),
            )
            if corner.g_cost < self.g_scores[corner]:
                self.g_scores[corner] = corner.g_cost

    def is_goal_reached(self, current: State) -> bool:
        """Kiểm tra trạng thái hiện tại đã đạt điều kiện tới đích hay chưa.

        Args:
            current: Trạng thái cần đánh giá.

        Returns:
            True nếu trạng thái thỏa mãn các điều kiện tiếp cận đích; ngược lại False.
        """
        if self.is_goal_heading_free:
            parent = current.parent
            if parent is None or parent.heading is None:
                return False
            seg = math.dist(parent.waypoint, current.waypoint)
            bearing = spatial.angle_to_heading(parent.waypoint, current.waypoint)
            turn_at_prev = abs(spatial.angle_diff(bearing, parent.heading))
            return seg - self.turn_radius * math.tan(turn_at_prev / 2.0) >= self.dss

        goal_heading = self.goal_state.heading
        if goal_heading is None or current.heading is None:
            return False
        return (
            abs(spatial.angle_diff(goal_heading, current.heading)) <= self.alpha_build
        )

    def reconstruct_path(self, state: State) -> list[PlannerState]:
        """Truy vết ngược các con trỏ parent về xuất phát để tạo chuỗi waypoint.

        Args:
            state: Trạng thái đích đã chạm tới.

        Returns:
            Chuỗi các trạng thái (waypoint, heading) từ xuất phát tới đích.

        Raises:
            TypeError: Nếu bất kỳ trạng thái nào thiếu thông tin góc hướng bay.
        """
        states: list[State] = []
        current: State | None = state
        while current is not None:
            states.append(current)
            current = current.parent
        states.reverse()

        path: list[PlannerState] = []
        for st in states:
            if st.via is not None:
                path.append(st.via)
            heading = st.heading
            if heading is None:
                raise TypeError("reconstructed path contains a headingless state")
            path.append((st.waypoint, heading))
        return path

    def get_search_stats(self) -> SearchStats:
        """Trả về thống kê chẩn đoán hiệu năng của quá trình tìm kiếm.

        Returns:
            Từ điển chứa số vòng lặp, kích thước tập đóng/mở, ngân sách thời gian,
            và cờ báo trạng thái kết thúc tìm kiếm.
        """
        return {
            "iterations": self.iteration_count,
            "closed_set_size": len(self.closed_set),
            "time_budget_s": self.time_budget_s,
            "is_budget_bound": self.is_budget_bound,
            "open_set_size": len(self.open_set),
            "is_search_failed": self.is_search_failed,
        }

    def search(self) -> list[PlannerState] | None:
        """Thực thi vòng lặp tìm kiếm A* cho đến khi tới đích hoặc hết thời gian.

        Returns:
            Chuỗi đường bay dạng các tuple (waypoint, heading), hoặc None nếu thất bại.
        """
        started_at = time.perf_counter()
        budget_s = self.time_budget_s

        if not self.start_corners:
            self.is_search_failed = True
            return None
        if not self.collision.check_fixed_legs(self.goal_state.waypoint, self.target):
            self.is_search_failed = True
            return None

        while self.open_set:
            if (time.perf_counter() - started_at) > budget_s:
                self.is_budget_bound = True
                break

            self.iteration_count += 1
            _, _, current = heapq.heappop(self.open_set)

            if current in self.closed_set:
                continue

            self.closed_set.add(current)
            self.nodes_expanded += 1

            if len(self.open_set) <= 1 and self.successors.num_strategy_b <= 0:
                self.successors.num_strategy_b = config.NUM_STRATEGY_B

            if (
                config.GOAL_SHOT_ENABLED
                and self.shot_armed
                and (self.iteration_count % config.GOAL_SHOT_EVERY_N) == 0
            ):
                shot = self.successors.try_goal_shot(current, {}, self.leg2_memo)
                if shot is not None and shot.g_cost < self.g_scores.get(
                    shot, float("inf")
                ):
                    self.g_scores[shot] = shot.g_cost
                    shot.h_cost = 0.0
                    heapq.heappush(
                        self.open_set,
                        (
                            shot.g_cost + config.HEURISTIC_WEIGHT * shot.h_cost,
                            self.iteration_count,
                            shot,
                        ),
                    )

            dist_to_goal = math.sqrt(
                (current.waypoint[0] - self.goal_state.waypoint[0]) ** 2
                + (current.waypoint[1] - self.goal_state.waypoint[1]) ** 2
            )

            if dist_to_goal < config.GOAL_THRESHOLD and self.is_goal_reached(current):
                return self.reconstruct_path(current)

            for next_state, transition_cost in self.successors.get_next_states(current):
                if next_state in self.closed_set:
                    continue

                tentative_g = self.g_scores[current] + transition_cost
                if tentative_g < self.g_scores.get(next_state, float("inf")):
                    next_state.parent = current
                    self.g_scores[next_state] = tentative_g
                    next_state.g_cost = tentative_g
                    next_state.h_cost = self.heuristic_fn(next_state, self.goal_state)
                    heapq.heappush(
                        self.open_set,
                        (
                            next_state.g_cost
                            + config.HEURISTIC_WEIGHT * next_state.h_cost,
                            self.iteration_count,
                            next_state,
                        ),
                    )

        self.is_search_failed = True
        return None
