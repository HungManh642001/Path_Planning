"""Bộ sinh trạng thái kế tiếp cho các bước mở rộng ô lưới A* động học."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from path_planning import config
from path_planning.geometry import spatial as su
from path_planning.geometry.goal_shot import two_corner_candidates
from path_planning.search.state import State


if TYPE_CHECKING:
    from path_planning.collision.detector import CollisionDetector
    from path_planning.types import Point, PreprocessedScenario


_CAND_MIN_D2 = config.CANDIDATE_MIN_DIST_M * config.CANDIDATE_MIN_DIST_M


class SuccessorGenerator:
    """Sinh các trạng thái ứng viên kế tiếp và các điểm rẽ xuất phát ban đầu.

    Attributes:
        scenario: Preprocessed mission scenario parameters.
        collision_detector: Spatial collision checker.
        turn_radius: Minimum vehicle turn radius in metres.
        alpha_max_rad: Maximum allowed turning angle per corner in radians.
        l0: Takeoff level-flight straight stabilisation distance in metres.
        dss: Terminal sensor engagement distance in metres.
        origin: Vehicle takeoff point O.
        target: Mission destination point T.
        goal_state: Terminal goal state.
        is_goal_heading_free: Whether terminal heading constraint is relaxed.
        construct_delta: Obstacle inflation buffer in metres.
        alpha_build: Construction turn angle limit (alpha_max - eps).
        turn_cos_guard: Cosine threshold for rapid turn angle prefiltering.
        poly_vertices: Inflated obstacle polygon vertices for candidate generation.
        fan_rungs: Distance rungs for Strategy B radial fan expansions.
        last_reject: Rejection code for the most recently tested candidate.
        num_strategy_b: Remaining budget for Strategy B valve expansions.
    """

    def __init__(
        self,
        scenario: PreprocessedScenario,
        collision_detector: CollisionDetector,
        *,
        turn_radius: float = config.R,
        alpha_max_rad: float = config.ALPHA_MAX_RAD,
        l0: float = config.L0,
        dss: float = config.DSS,
        origin: Point,
        target: Point,
        goal_state: State,
        is_goal_heading_free: bool = False,
    ) -> None:
        """Khởi tạo bộ sinh trạng thái kế tiếp với các giới hạn và đỉnh vật cản.

        Args:
            scenario: Prepared scenario dictionary.
            collision_detector: Collision detector engine.
            turn_radius: Minimum vehicle turning radius in metres.
            alpha_max_rad: Maximum allowable corner turn angle in radians.
            l0: Level flight stabilization length after takeoff in metres.
            dss: Final straight approach distance to target in metres.
            origin: Takeoff coordinate (x, y).
            target: Destination coordinate (x, y).
            goal_state: Target goal state representation.
            is_goal_heading_free: If True, arrival heading is unconstrained.
        """
        self.scenario = scenario
        self.collision_detector = collision_detector
        self.turn_radius = turn_radius
        self.alpha_max_rad = alpha_max_rad
        self.l0 = l0
        self.dss = dss
        self.origin = origin
        self.target = target
        self.goal_state = goal_state
        self.is_goal_heading_free = is_goal_heading_free

        self.construct_delta = config.CONSTRUCTION_CLEARANCE_M + config.GEOM_EPS_M
        self.alpha_build = alpha_max_rad - config.GEOM_EPS_RAD
        self.turn_cos_guard = math.cos(
            min(
                math.pi - config.GEOM_EPS_RAD,
                self.alpha_build + config.TURN_PREFILTER_BAND_RAD,
            )
        )

        self.poly_vertices: list[Point] = []
        for poly in collision_detector.polygons:
            hull = poly.convex_hull.buffer(self.construct_delta, join_style="mitre")
            self.poly_vertices.extend(
                (float(x), float(y)) for x, y in hull.exterior.coords[:-1]
            )

        num_rungs = max(1, int(config.NUM_FAN_DISTANCES))
        tan_half_max = math.tan(self.alpha_build / 2.0)
        self.fan_rungs: list[float] = [
            self.turn_radius * (j / num_rungs) * tan_half_max + config.RADIAL_FAN_STEP_M
            for j in range(1, num_rungs + 1)
        ]

        self.last_reject: str | None = None
        self.num_strategy_b = config.NUM_STRATEGY_B

    def seed_start_corners(self) -> list[State]:
        """Gieo trạng thái góc rẽ xuất phát dọc theo tia cất cánh (chiều dài >= L0).

        Returns:
            List of valid collision-free start corner states satisfying l1 >= L0.
        """
        takeoff_heading = self.scenario["start_state"]["heading"]
        num_corners = max(1, int(config.NUM_START_CORNERS))
        tan_max = math.tan(self.alpha_build / 2.0)

        corners: list[State] = []
        for i in range(1, num_corners + 1):
            distance = (
                self.l0
                + config.GEOM_EPS_M
                + self.turn_radius * (i / num_corners) * tan_max
            )
            corner = (
                self.origin[0] + distance * math.cos(takeoff_heading),
                self.origin[1] + distance * math.sin(takeoff_heading),
            )
            if not self.collision_detector.is_in_bounds(corner):
                continue
            if not self.collision_detector.is_collision_free(self.origin, corner):
                continue
            state = State(corner, takeoff_heading)
            state.g_cost = distance
            state.straight_budget = distance
            state.min_straight_in = self.l0
            state.is_start_corner = True
            corners.append(state)
        return corners

    def doan_trinh(
        self,
        current: State,
        leg_len: float,
        turn: float,
        far_reserve: float = 0.0,
        advance: float = 0.0,
    ) -> float | None:
        """Kiểm tra và cập nhật ngân sách đoạn bay thẳng cho một bước chuyển tiếp.

        Args:
            current: Preceding state.
            leg_len: Length of the candidate chord in metres.
            turn: Turn angle required at current state in radians.
            far_reserve: Turn fillet reserve required at the far end in metres.
            advance: Distance slid along heading ray before turning in metres.

        Returns:
            Updated straight budget on the new leg if feasible; None if infeasible.
        """
        reserve = self.turn_radius * math.tan(turn / 2.0)
        if current.straight_budget + advance - reserve < current.min_straight_in:
            return None
        budget = leg_len - reserve
        if budget - far_reserve < config.MIN_STRAIGHT_M:
            return None
        return budget

    def get_next_states(self, current_state: State) -> list[tuple[State, float]]:
        """Sinh các trạng thái kế tiếp khả thi từ trạng thái current_state.

        Args:
            current_state: State currently being expanded.

        Returns:
            List of (successor_state, step_cost) pairs.

        Raises:
            TypeError: If current_state has no heading.
        """
        heading = current_state.heading
        if heading is None:
            raise TypeError("cannot expand a headingless goal target")

        successors: list[tuple[State, float]] = []
        position = current_state.waypoint

        # Wrap step off circle boundary
        if self.collision_detector.on_circle_boundary(position):
            forward = (
                position[0] + config.WRAP_STEP_M * math.cos(heading),
                position[1] + config.WRAP_STEP_M * math.sin(heading),
            )
            if self.collision_detector.is_in_bounds(
                forward
            ) and self.collision_detector.is_collision_free(position, forward):
                successors.append((State(forward, heading), config.WRAP_STEP_M))

        # Strategy A: candidates
        goal_wp = self.goal_state.waypoint
        candidates: list[Point] = []
        for center, radius in self.scenario["circle_obstacles"]:
            candidates.extend(
                su.circle_tangent_points(
                    position, center, radius + self.construct_delta
                )
            )
        candidates.extend(self.poly_vertices)
        candidates.append(goal_wp)

        goal_reject: str | None = None
        for node in candidates:
            dx = node[0] - position[0]
            dy = node[1] - position[1]
            if dx * dx + dy * dy < _CAND_MIN_D2:
                continue
            result = self.pivot_candidate(current_state, node, 0.0)
            if (
                result is None
                and config.NUM_PIVOT_SLIDES > 0
                and self.last_reject == "arc"
            ):
                result = self.slide_pivot(current_state, node)
            if result is not None:
                successors.append(result)
            elif node is goal_wp:
                goal_reject = self.last_reject

        if (
            successors
            and not self.collision_detector.is_collision_free(position, goal_wp)
            and not current_state.is_start_corner
        ):
            if self.num_strategy_b <= 0:
                return successors
            self.num_strategy_b -= 1
        elif (
            config.FAN_SKIP_ON_SHORT_RUNIN
            and self.is_goal_heading_free
            and goal_reject == "goal"
            and successors
            and not current_state.is_start_corner
        ):
            return successors

        # Strategy B: radial fan
        num_directions = config.RADIAL_FAN_DIRECTIONS
        for i in range(num_directions):
            heading_offset = -self.alpha_build + 2 * self.alpha_build * i / (
                num_directions - 1
            )
            next_heading = heading + heading_offset
            near_reserve = math.tan(abs(heading_offset) / 2.0) * self.turn_radius
            turn = abs(su.angle_diff(next_heading, heading))
            cos_next = math.cos(next_heading)
            sin_next = math.sin(next_heading)
            for rung in self.fan_rungs:
                distance_next = near_reserve + rung
                next_waypoint = (
                    position[0] + distance_next * cos_next,
                    position[1] + distance_next * sin_next,
                )
                budget = self.doan_trinh(current_state, distance_next, turn)
                if budget is None:
                    continue
                if not self.collision_detector.is_in_bounds(next_waypoint):
                    continue
                if not self.collision_detector.is_collision_free(
                    position, next_waypoint
                ):
                    continue
                if (
                    config.ARC_CLEARANCE_CHECK
                    and not self.collision_detector.is_corner_arc_clear(
                        heading,
                        position,
                        next_waypoint,
                    )
                ):
                    continue
                successor = State(next_waypoint, next_heading)
                successor.straight_budget = budget
                successors.append(
                    (successor, distance_next + config.TURN_PENALTY_WEIGHT * turn)
                )
        return successors

    def pivot_candidate(
        self, current: State, node: Point, advance: float
    ) -> tuple[State, float] | None:
        """Xác thực tính khả thi của bước chuyển tiếp từ trạng thái tới điểm node.

        Args:
            current: Origin state.
            node: Target coordinate to transition to.
            advance: Distance to advance along current heading before turning (m).

        Returns:
            Tuple (successor_state, step_cost) if feasible; None otherwise.

        Raises:
            TypeError: If current state lacks heading or trigonometric caches.
        """
        position = current.waypoint
        heading = current.heading
        ux = current.cos_h
        uy = current.sin_h
        if heading is None or ux is None or uy is None:
            raise TypeError("cannot expand a headingless goal target")

        pivot = (
            (position[0] + advance * ux, position[1] + advance * uy)
            if advance > 0.0
            else position
        )
        dx = node[0] - pivot[0]
        dy = node[1] - pivot[1]
        seg_len = math.hypot(dx, dy)

        if dx * ux + dy * uy < self.turn_cos_guard * seg_len:
            self.last_reject = "turn"
            return None
        heading_to_node = su.angle_to_heading(pivot, node)
        turn = abs(su.angle_diff(heading_to_node, heading))
        if turn > self.alpha_build:
            self.last_reject = "turn"
            return None

        far_reserve = 0.0
        if node is self.goal_state.waypoint:
            if self.is_goal_heading_free:
                if seg_len - self.turn_radius * math.tan(turn / 2.0) < self.dss:
                    self.last_reject = "goal"
                    return None
            else:
                goal_heading = self.goal_state.heading
                if goal_heading is not None:
                    final_turn = abs(su.angle_diff(goal_heading, heading_to_node))
                    if final_turn > self.alpha_build:
                        self.last_reject = "goal"
                        return None
                    far_reserve = self.turn_radius * math.tan(final_turn / 2.0)

        budget = self.doan_trinh(current, seg_len, turn, far_reserve, advance)
        if budget is None:
            self.last_reject = "doan_trinh"
            return None
        if advance > 0.0:
            if not self.collision_detector.is_in_bounds(pivot):
                self.last_reject = "bounds"
                return None
            if not self.collision_detector.is_collision_free(position, pivot):
                self.last_reject = "ext_leg"
                return None
        if not self.collision_detector.is_collision_free(pivot, node):
            self.last_reject = "los"
            return None
        if (
            config.ARC_CLEARANCE_CHECK
            and not self.collision_detector.is_corner_arc_clear(
                heading,
                pivot,
                node,
            )
        ):
            self.last_reject = "arc"
            return None

        self.last_reject = None
        successor = State(node, heading_to_node)
        successor.straight_budget = budget
        if advance > 0.0:
            successor.via = (pivot, heading)
        return successor, advance + seg_len + config.TURN_PENALTY_WEIGHT * turn

    def slide_pivot(self, current: State, node: Point) -> tuple[State, float] | None:
        """Thử cứu ứng viên bị cản cung lượn bằng cách trượt điểm rẽ dọc tia hướng.

        Args:
            current: Current state.
            node: Target coordinate.

        Returns:
            Tuple (successor_state, step_cost) if a slid pivot succeeds; None otherwise.

        Raises:
            TypeError: If current state has no heading.
        """
        position = current.waypoint
        heading = current.heading
        if heading is None:
            raise TypeError("cannot expand a headingless goal target")
        ux = current.cos_h if current.cos_h is not None else math.cos(heading)
        uy = current.sin_h if current.sin_h is not None else math.sin(heading)
        vx = node[0] - position[0]
        vy = node[1] - position[1]
        along = vx * ux + vy * uy
        if along <= 0.0:
            return None
        cross = abs(vx * -uy + vy * ux)
        if cross < config.GEOM_EPS_M:
            return None

        turn_without_slide = math.atan2(cross, along)
        num_slides = int(config.NUM_PIVOT_SLIDES)
        tan_half_max = math.tan(self.alpha_build / 2.0)
        for i in range(1, num_slides + 1):
            turn_i = 2.0 * math.atan((i / num_slides) * tan_half_max)
            if turn_i <= turn_without_slide:
                continue
            if turn_i >= math.pi / 2.0 - config.GEOM_EPS_RAD:
                slide = along
            else:
                slide = along - cross / math.tan(turn_i)
            if slide <= config.MIN_PIVOT_SLIDE_M:
                continue
            result = self.pivot_candidate(current, node, slide)
            if result is not None:
                return result
        return None

    def try_goal_shot(
        self,
        current: State,
        leg1_memo: dict[float, list[float]],
        leg2_memo: dict[float, list[float]],
    ) -> State | None:
        """Xây dựng phương án cơ động giải tích 2 góc rẽ bắn thẳng về mục tiêu đích.

        Args:
            current: Current search state.
            leg1_memo: Clearance memoization dictionary for leg 1 rays.
            leg2_memo: Clearance memoization dictionary for leg 2 rays.

        Returns:
            Terminal goal state linked via corner if shot succeeds; None otherwise.

        Raises:
            TypeError: If current heading or goal heading is missing in fixed-goal mode.
        """
        if self.is_goal_heading_free:
            return None
        goal_wp = self.goal_state.waypoint
        goal_heading = self.goal_state.heading
        heading = current.heading
        if goal_heading is None or heading is None:
            raise TypeError("the goal shot requires both headings in fixed-goal mode")

        candidates = two_corner_candidates(
            current.waypoint,
            heading,
            goal_wp,
            goal_heading,
            self.turn_radius,
            self.alpha_build,
            config.MIN_STRAIGHT_M,
            current.straight_budget,
            current.min_straight_in,
            num_dir=config.GOAL_SHOT_DIRS,
            num_cone=config.GOAL_SHOT_CONE,
        )
        if not candidates:
            return None

        base_g = current.g_cost
        position = current.waypoint
        for candidate in candidates:
            corner = candidate.corner
            leg1_heading = candidate.leg1_heading
            arrival_heading = candidate.arrival_heading
            if not self.collision_detector.ray_chord_clear(
                leg1_memo,
                leg1_heading,
                math.dist(position, corner),
                position,
                corner,
            ):
                continue
            if not self.collision_detector.ray_chord_clear(
                leg2_memo,
                arrival_heading,
                math.dist(corner, goal_wp),
                corner,
                goal_wp,
            ):
                continue
            if config.ARC_CLEARANCE_CHECK:
                if not self.collision_detector.is_corner_arc_clear(
                    heading, current.waypoint, corner
                ):
                    continue
                if not self.collision_detector.is_corner_arc_clear(
                    leg1_heading, corner, goal_wp
                ):
                    continue
                if not self.collision_detector.is_corner_arc_clear(
                    arrival_heading, goal_wp, self.target
                ):
                    continue

            corner_state = State(corner, leg1_heading)
            corner_state.parent = current
            turn_1 = abs(su.angle_diff(leg1_heading, heading))
            corner_state.g_cost = (
                base_g
                + math.dist(current.waypoint, corner)
                + config.TURN_PENALTY_WEIGHT * turn_1
            )
            corner_state.straight_budget = candidate.budget_corner

            goal_state = State(goal_wp, arrival_heading)
            goal_state.parent = corner_state
            turn_2 = abs(su.angle_diff(arrival_heading, leg1_heading))
            goal_state.g_cost = (
                corner_state.g_cost
                + math.dist(corner, goal_wp)
                + config.TURN_PENALTY_WEIGHT * turn_2
            )
            goal_state.straight_budget = candidate.budget_goal
            return goal_state
        return None
