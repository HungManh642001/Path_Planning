"""Focal (A*epsilon) variant of the Kinodynamic A* planner.

Subclasses the base planner and overrides only search() to expand the
FOCAL-best node (minimum secondary heuristic) while an admissible Euclid
OPEN guarantees the (1 + focal_eps) bound. All geometry, collision, arc-hop,
smoothing, and start-corner logic is inherited unchanged.
"""

import heapq
import itertools
import math
import time

import config
import core.spatial_utils as su
from core.kinodynamic_astar import KinodynamicAstar, _angle_diff

import ml_planner.config as mlcfg
from ml_planner.secondary import handcrafted_secondary


class FocalKinodynamicAstar(KinodynamicAstar):
    def __init__(self, preprocessed_scenario, focal_eps=None, secondary=None):
        self.collision_checks = 0    # REAL collision checks paid (lazy A/B metric)
        self._admit_all = False      # drain-path override of _focal_admissible
        super().__init__(preprocessed_scenario)
        self.collision_checks = 0    # Reset after initialization (counts search phase only)
        self.focal_eps = mlcfg.FOCAL_EPS if focal_eps is None else focal_eps
        self._secondary = secondary  # Callable[[State], float] or None

    def secondary_h(self, state):
        """Ranking heuristic for FOCAL (need not be admissible)."""
        if self._secondary is not None:
            return self._secondary(state)
        return handcrafted_secondary(
            state.waypoint,
            self.goal_state.waypoint,
            self.scenario['circle_obstacles'],
        )

    def _check_collision(self, p1, p2):
        self.collision_checks += 1
        return super()._check_collision(p1, p2)

    # ---- extension points for the lazy variant (behavior-neutral here) ----
    def _focal_admissible(self, state):
        """FOCAL admission filter; the lazy+corridor subclass narrows this.
        Rejected states stay in OPEN (still bounding f_min); the drain path
        retries with _admit_all so filtering can never starve the search."""
        return True

    def _validate_on_pop(self, state):
        """Last-moment edge validation; the lazy subclass defers collision
        checks to here. Returning False discards the pop (state NOT closed)."""
        return True

    def _goal_reached(self, current):
        """Return the reconstructed path if `current` is an accepted goal
        arrival, else None. Mirrors the base search()'s goal-acceptance rules
        (free run-in >= DSS, or aligned arrival within alpha_max)."""
        dist = math.hypot(
            current.waypoint[0] - self.goal_state.waypoint[0],
            current.waypoint[1] - self.goal_state.waypoint[1],
        )
        if dist >= config.GOAL_THRESHOLD:
            return None
        if self._free_goal:
            if current.parent is not None:
                seg = math.dist(current.parent.waypoint, current.waypoint)
                bearing = su.angle_to_heading(current.parent.waypoint, current.waypoint)
                turn_at_prev = abs(_angle_diff(bearing, current.parent.heading))
                usable = seg - self.R * math.tan(turn_at_prev / 2.0)
                if usable >= self._dss - config.EPS:
                    return self._reconstruct_path(current)
            return None
        approach_turn = abs(_angle_diff(self.goal_state.heading, current.heading))
        if approach_turn <= self.alpha_max_rad:
            return self._reconstruct_path(current)
        return None

    def search(self):
        """Focal (A*epsilon) search. OPEN is ordered by the admissible
        f = g + h_euclid (weight 1) so f_min bounds the optimum; FOCAL holds
        every live OPEN node with f <= w * f_min and is expanded by minimum
        secondary_h. Guarantees returned cost <= w * optimal."""
        _start = time.perf_counter()
        _budget = config.TIME_BUDGET_S
        w = 1.0 + self.focal_eps

        if not self.start_corners:
            self.search_failed = True
            return None

        counter = itertools.count()
        open_heap = []      # (f, count, state) — all inserted OPEN nodes
        focal_heap = []     # (secondary, count, state) — nodes with f <= w*f_min
        in_focal = set()    # id(state) currently pushed to focal_heap
        self.open_set = open_heap  # keep get_search_stats() meaningful

        # Admissible OPEN uses heuristic weight 1 intentionally (NOT
        # config.HEURISTIC_WEIGHT) so f = g + h stays a true lower bound and
        # the focal bound holds; focal_eps is the only suboptimality knob.
        w = 1.0 + self.focal_eps

        for corner in self.start_corners:
            corner.h_cost = self.heuristic(corner, self.goal_state)
            if corner.g_cost < self.g_scores[corner]:
                self.g_scores[corner] = corner.g_cost
            heapq.heappush(open_heap, (corner.g_cost + corner.h_cost, next(counter), corner))

        def _is_live(state):
            # edge_dead: set by the lazy subclass when a deferred edge fails
            # its real collision check. The g_scores test alone cannot retire
            # such a state — its entry is deleted to keep the lattice cell
            # re-discoverable, which makes `g_cost <= inf` vacuously true and
            # would leave the corpse re-admittable (and re-paying the real
            # check) forever. Never set in eager mode.
            return (not getattr(state, 'edge_dead', False) and
                    state not in self.closed_set and
                    state.g_cost <= self.g_scores.get(state, float('inf')))

        def _clean_open_top():
            while open_heap and not _is_live(open_heap[0][2]):
                heapq.heappop(open_heap)

        def _refill_focal(f_bound):
            # Tolerant admission: g_cost and h_cost for two nodes with the
            # SAME true admissible f (e.g. multiple seeded start corners
            # collinear with O and the goal) are computed via independent
            # floating-point paths (ray-distance vs. hypot) and can land ~1
            # ULP apart at these magnitudes (~1e5-1e6 m). A strict f <=
            # f_bound at eps=0 (w=1.0, zero-width window) then silently
            # excludes the true optimum from FOCAL, so the tie-break falls to
            # a worse node — breaking the eps=0 == optimal guarantee. EPS is
            # metres, orders of magnitude above the float noise, negligible
            # against real path costs.
            for f, c, st in open_heap:
                if (f <= f_bound + config.EPS and id(st) not in in_focal
                        and _is_live(st)
                        and (self._admit_all or self._focal_admissible(st))):
                    heapq.heappush(focal_heap, (self.secondary_h(st), c, st))
                    in_focal.add(id(st))

        _clean_open_top()
        f_min = open_heap[0][0] if open_heap else None
        if f_min is not None:
            _refill_focal(w * f_min)

        while open_heap and self.iteration_count < self.max_iterations:
            if _budget is not None and (time.perf_counter() - _start) > _budget:
                break
            self.iteration_count += 1

            # Select the FOCAL-best live node; if FOCAL drained, refill and retry.
            current = None
            while focal_heap:
                _, _, cand = heapq.heappop(focal_heap)
                in_focal.discard(id(cand))
                if _is_live(cand) and self._validate_on_pop(cand):
                    current = cand
                    break
            if current is None:
                _clean_open_top()
                if not open_heap:
                    break
                f_min = open_heap[0][0]
                _refill_focal(w * f_min)
                if not focal_heap and open_heap:
                    # Admission filtering (corridor) found nothing in band:
                    # admit unconditionally so filtering can only cost time,
                    # never starve the search or fake a no-path.
                    self._admit_all = True
                    try:
                        _refill_focal(w * f_min)
                    finally:
                        self._admit_all = False
                continue

            self.closed_set.add(current)

            # Escape-valve re-arm (mirrors base search): give the fan a fresh
            # budget as a last resort when the frontier is nearly dead.
            if len(open_heap) <= 1 and self.num_strategy_b <= 0:
                self.num_strategy_b = config.NUM_STRATEGY_B

            reached = self._goal_reached(current)
            if reached is not None:
                return reached

            for next_state, transition_cost in self.get_next_states(current):
                if next_state in self.closed_set:
                    continue
                tentative_g = self.g_scores[current] + transition_cost
                if tentative_g < self.g_scores.get(next_state, float('inf')):
                    next_state.parent = current
                    self.g_scores[next_state] = tentative_g
                    next_state.g_cost = tentative_g
                    next_state.h_cost = self.heuristic(next_state, self.goal_state)
                    f = tentative_g + next_state.h_cost
                    c = next(counter)
                    heapq.heappush(open_heap, (f, c, next_state))
                    if (f_min is not None and f <= w * f_min + config.EPS
                            and (self._admit_all or self._focal_admissible(next_state))):
                        heapq.heappush(focal_heap, (self.secondary_h(next_state), c, next_state))
                        in_focal.add(id(next_state))

            # Update f_min after expansion; widen FOCAL if it rose.
            _clean_open_top()
            if open_heap:
                new_fmin = open_heap[0][0]
                if f_min is None or new_fmin > f_min:
                    f_min = new_fmin
                    _refill_focal(w * f_min)
            else:
                f_min = None

        self.search_failed = True
        return None
