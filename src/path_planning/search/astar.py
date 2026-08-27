"""Kinodynamic A* priority queue graph search engine."""

from __future__ import annotations

import heapq
import math
import time
from collections import defaultdict
from collections.abc import Callable
from typing import TYPE_CHECKING

from path_planning import config
from path_planning.geometry import spatial as su
from path_planning.search.heuristic import euclidean_heuristic
from path_planning.search.state import State


if TYPE_CHECKING:
    from path_planning.collision.detector import CollisionDetector
    from path_planning.search.successors import SuccessorGenerator
    from path_planning.types import PlannerState, Point, SearchStats


class AstarSearchEngine:
    """Core A* priority queue loop with deadline budgeting.

    Attributes:
        start_corners: Seeded initial search corner states.
        goal_state: Target goal state.
        successors: Successor candidate generator.
        collision: Spatial collision checker.
        time_budget_s: Maximum allowable search duration in seconds.
        origin: Aircraft takeoff location.
        target: Mission target destination.
        is_goal_heading_free: Whether goal approach heading is unconstrained.
        turn_radius: Minimum turning radius in metres.
        dss: Seeker approach distance in metres.
        l0: Takeoff straight stabilization distance in metres.
        alpha_build: Construction turn angle limit in radians.
        heuristic_fn: Distance estimation heuristic function.
        open_set: Priority queue of active states sorted by f-score.
        closed_set: Set of visited states.
        g_scores: Map from State to minimum known cost-to-come.
        iteration_count: Number of loop iterations completed.
        nodes_expanded: Number of distinct nodes popped from open set.
        is_budget_bound: True if search was truncated by time budget.
        is_search_failed: True if search queue emptied without reaching goal.
        shot_armed: True if analytic two-corner goal shot solver is active.
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
        """Initialize A* search engine with start corners and budget.

        Args:
            start_corners: List of seeded initial corner states.
            goal_state: Target goal state.
            successor_generator: Successor state generator instance.
            collision_detector: Collision detection engine instance.
            time_budget_s: Maximum allowed search time in seconds.
            origin: Aircraft takeoff point O.
            target: Mission target point T.
            is_goal_heading_free: True if arrival heading is unconstrained.
            turn_radius: Minimum vehicle turn radius in metres.
            dss: Terminal sensor engagement distance in metres.
            l0: Takeoff stabilization distance in metres.
            alpha_build: Construction turn angle limit in radians.
            heuristic_fn: Distance heuristic function (state, goal) -> float.
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

        self.shot_armed = False
        if not self.is_goal_heading_free:
            goal_h = self.goal_state.heading
            if goal_h is not None:
                travel = su.angle_to_heading(self.origin, self.target)
                reversal = abs(su.angle_diff(goal_h, travel))
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
        """Test whether a state within the goal threshold may terminate.

        Args:
            current: State to evaluate.

        Returns:
            True if state satisfies terminal approach conditions; False otherwise.
        """
        if self.is_goal_heading_free:
            parent = current.parent
            if parent is None or parent.heading is None:
                return False
            seg = math.dist(parent.waypoint, current.waypoint)
            bearing = su.angle_to_heading(parent.waypoint, current.waypoint)
            turn_at_prev = abs(su.angle_diff(bearing, parent.heading))
            return seg - self.turn_radius * math.tan(turn_at_prev / 2.0) >= self.dss

        goal_heading = self.goal_state.heading
        if goal_heading is None or current.heading is None:
            return False
        return abs(su.angle_diff(goal_heading, current.heading)) <= self.alpha_build

    def reconstruct_path(self, state: State) -> list[PlannerState]:
        """Walk parent pointers back to the start, expanding pivot slides.

        Args:
            state: Reached terminal goal state.

        Returns:
            Sequence of (waypoint, heading) states from start to goal.

        Raises:
            TypeError: If any reconstructed state is missing a heading.
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
        """Return diagnostic counters for the search run.

        Returns:
            Dictionary containing iteration count, closed/open set sizes, time budget,
            and timeout flags.
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
        """Run the A* search loop until goal reached or deadline expires.

        Returns:
            Reconstructed path of (waypoint, heading) tuples, or None on failure.
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
                shot = self.successors.try_goal_shot(current, {}, {})
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
