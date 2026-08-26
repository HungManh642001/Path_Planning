"""Pure-geometry tests for the 2-corner goal-shot candidate enumerator."""
import math

from path_planning.core import goal_shot as gs

# Non-degenerate 2-corner geometry: at the origin heading EAST, goal to the
# north-east, approach heading NORTH. The vehicle turns onto an intermediate
# leg and then onto the northern approach — a genuine 2-corner maneuver with a
# well-defined intermediate corner. (The exact-opposite-heading case at the
# origin is DEGENERATE: the ideal corner collapses onto the start point
# (leg1 length 0), so no candidate is returned and the search must first
# travel before a 2-corner shot becomes feasible — that is expected, correct
# behavior, not a bug.)
P = (0.0, 0.0)
H = 0.0              # heading east
GOAL = (100000.0, 50000.0)
GH = math.pi / 2     # approach heading north
R = 8000.0
AMAX = math.pi / 2   # 90 deg


def test_two_corner_candidates_are_feasible():
    cands = gs.two_corner_candidates(P, H, GOAL, GH, R, AMAX,
                                     10.0, 1e9, 10.0, num_dir=9, num_cone=9)
    assert cands, "adverse-launch open geometry must yield 2-corner candidates"
    for total_len, C, d1, phi, budget_C, budget_W in cands:
        a1 = abs(gs._angdiff(d1, H))          # turn at P
        a2 = abs(gs._angdiff(phi, d1))        # turn at C
        at = abs(gs._angdiff(GH, phi))        # terminal turn at the goal
        assert a1 <= AMAX + 1e-9
        assert a2 <= AMAX + 1e-9
        assert at <= AMAX + 1e-9
        # C is consistent with the reported leg headings.
        assert abs(gs._angdiff(math.atan2(C[1] - P[1], C[0] - P[0]), d1)) < 1e-6
        assert abs(gs._angdiff(math.atan2(GOAL[1] - C[1], GOAL[0] - C[0]), phi)) < 1e-6
        # Both legs keep the far-end reserve + min straight.
        assert budget_C - R * math.tan(a2 / 2.0) >= 10.0 - 1e-6
        assert budget_W - R * math.tan(at / 2.0) >= 10.0 - 1e-6


def test_candidates_sorted_shortest_first():
    cands = gs.two_corner_candidates(P, H, GOAL, GH, R, AMAX, 10.0, 1e9, 10.0)
    lengths = [c[0] for c in cands]
    assert lengths == sorted(lengths)


def test_reserves_reject_everything_when_min_straight_huge():
    cands = gs.two_corner_candidates(P, H, GOAL, GH, R, AMAX, 1e12, 1e9, 10.0)
    assert cands == []


def test_incoming_budget_gate_blocks_first_turn():
    # A tiny incoming straight budget cannot afford the near reserve of a large
    # first turn, so no candidate whose turn-at-P exceeds the budget survives.
    cands = gs.two_corner_candidates(P, H, GOAL, GH, R, AMAX,
                                     10.0, 100.0, 10.0)  # budget_in = 100 m
    for _tot, _C, d1, _phi, _bC, _bW in cands:
        a1 = abs(gs._angdiff(d1, H))
        assert 100.0 - R * math.tan(a1 / 2.0) >= 10.0 - 1e-9
