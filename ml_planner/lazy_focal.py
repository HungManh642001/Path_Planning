"""Bound-preserving lazy variant of the focal planner.

Strategy-A / fan chord collision checks are DEFERRED at generation (the
successor is created optimistically with edge_validated=False) and paid only
when the node is actually popped for expansion. An optimistic f is <= the
true f (a collision check can only delete an edge, never cheapen it), so
f_min over OPEN remains a valid lower bound and the focal (1+eps) guarantee
is unchanged. An optional Corridor (ml_planner/corridor.py) acts as a FOCAL
ordering tiebreak: FOCAL still holds every in-band node (identical safety
envelope to the eager focal search), in-corridor nodes are merely expanded
first, so a wrong model can only cost time, never the bound.

Never deferred: chords to the goal waypoint (keeps the valve LOS test at the
same call site honest, and guarantees every accepted goal arrival rides a
validated edge), đoản-trình checks (run before collision in the core loop),
and arc-hop geometry (_max_clear_wrap/_sector_clear, not _check_collision).
"""

from ml_planner.focal_astar import FocalKinodynamicAstar


class LazyFocalKinodynamicAstar(FocalKinodynamicAstar):
    def __init__(self, preprocessed_scenario, focal_eps=None, corridor=None):
        # Set BEFORE super().__init__: corner seeding in KinodynamicAstar's
        # __init__ calls _check_collision, and the trap reads _lazy_ctx (None
        # here, so seeding stays fully eager — those legs must be real).
        self._lazy_ctx = None       # state currently generating successors
        self._deferred_now = set()  # waypoints deferred during this arm
        super().__init__(preprocessed_scenario, focal_eps=focal_eps, secondary=None)
        self.corridor = corridor

    # ---- defer-at-generation trap -------------------------------------
    def get_next_states(self, current_state):
        self._lazy_ctx = current_state
        self._deferred_now = set()
        try:
            successors = super().get_next_states(current_state)
        finally:
            self._lazy_ctx = None
        for st, _cost in successors:
            if st.waypoint in self._deferred_now:
                st.edge_validated = False
        return successors

    def _check_collision(self, p1, p2):
        ctx = self._lazy_ctx
        if (ctx is not None and p1 == ctx.waypoint
                and p2 != self.goal_state.waypoint):
            # Optimistically clear; the real check runs at pop time.
            self._deferred_now.add(p2)
            return True
        return super()._check_collision(p1, p2)

    # ---- validate-on-pop ----------------------------------------------
    def _validate_on_pop(self, state):
        if getattr(state, 'edge_validated', True):
            return True
        ok = self._check_collision(state.parent.waypoint, state.waypoint)
        if ok:
            state.edge_validated = True
            return True
        # Dead edge: forget this g so the lattice cell stays re-discoverable
        # through other (possibly valid) incoming edges, and kill THIS state
        # object so _is_live retires it from OPEN/FOCAL — without the flag the
        # deleted g_scores entry makes the liveness test vacuously true and
        # the corpse is re-admitted (re-paying the real check) every refill.
        state.edge_dead = True
        if self.g_scores.get(state) == state.g_cost:
            del self.g_scores[state]
        return False

    # ---- corridor is a FOCAL ordering tiebreak, never a gate ------------
    # Admission gating broke the epsilon bound (benchmark seed 6011, ratio
    # 1.0567; eager+gate 1.0668, so the gate itself was the cause): in a
    # non-reopening search, holding in-band nodes out of FOCAL lets worse
    # paths close their lattice cells first, and that locked-in inflation is
    # not limited by the (1+eps) band. Preferring in-corridor nodes inside a
    # FOCAL that still holds EVERY in-band node keeps the exact safety
    # envelope of the eager focal search.
    def secondary_h(self, state):
        s = super().secondary_h(state)
        if self.corridor is None:
            return s
        inside = self.corridor.contains(state.waypoint[0], state.waypoint[1])
        return (0 if inside else 1, s)
