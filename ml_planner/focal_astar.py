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
        super().__init__(preprocessed_scenario)
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
