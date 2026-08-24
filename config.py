"""
Configuration Module for Autonomous Aircraft Path Planning System
Defines operational parameters and parameters
"""

import math

# ====== DYNAMIC CONSTRAINTS ======
# Turn radius (m) - fixed for entire trajectory
R = 8000.0

# Maximum turn angle allowed (degrees)
ALPHA_MAX = 90.0  # in degrees, will be converted to radians

# Minimum distance for level flight and stabilization (m)
# After takeoff, distance to stabilize
L0 = 4000.0

# Distance for terminal camera sensor lock(m)
DSS = 23000.0

# Minimum usable straight run between two consecutive turns (m), i.e. the
# đoản-trình floor for every INTERIOR segment (the first and last legs use the
# larger L0 / DSS instead). Read by both planners.
MIN_STRAIGHT_M = 10.0

# How far the first smoothed chord may deviate from start_heading (rad). No turn
# is available at the takeoff point O, so the first kept waypoint must sit on
# the takeoff ray; this is an exactness guard, not a slack allowance.
TAKEOFF_RAY_TOL_RAD = 1e-9

# The mirror of the above at the other end (rad): in FIXED-goal mode the seeker
# run-in into T must be flown along goal_heading, so the last smoothed chord has
# to lie on the approach ray. Without this a subsequence smoother happily drops
# W_{n-1} and connects to T from wherever, which shortens the path while
# silently arriving on the wrong heading (measured up to 68 deg off).
APPROACH_RAY_TOL_RAD = 1e-9

# Takeoff angle (degrees) - angle from horizontal at start of trajectory
START_ANGLE_MIN = -180.0
START_ANGLE_MAX = 180.0
START_ANGLE_DEFAULT = 15.0

# Approach angle (degrees) - angle from horizontal at goal approach
APPROACH_ANGLE_MIN = -180.0
APPROACH_ANGLE_MAX = 180.0
APPROACH_ANGLE_DEFAULT = 30.0

# ====== SAFETY & OBSTACLE HANDLING ======
# Safety margin buffer (m) - distance to expand obstacle boundaries
SAFE_MARGIN = 0.0

# Polygon inflation join style: 'mitre' keeps sharp corners so each obstacle
# yields a few real corner vertices (used as navigation waypoints) instead of
# ~70 rounded arc points. mitre_limit caps the corner-spike length; it is large
# enough that the mitre polygon always CONTAINS the exact round Minkowski buffer
# (preserving the arc-clearance guarantee) for the convex-ish islands here.
POLYGON_MITRE_LIMIT = 5.0

# Angular step (deg) for expanding a circle-boundary arc into waypoint
# vertices (circumscribed polygon) at OUTPUT time. Max supported 45. Search
# connectivity does NOT depend on it: arc clearance is checked at the fixed
# 45-deg bulge radius r/cos(pi/8), which covers any expansion step <= 45 deg.
ARC_WAYPOINT_STEP_DEG = 30.0

# Angular step (deg) for sampling arc clearance during search.
ARC_SAMPLE_STEP_DEG = 5.0

# Collision checking is EXACT: any penetration of a circle's INFLATED
# boundary (dist < radius) is a collision — zero tolerance. Feasibility of
# boundary-riding geometry is achieved on the CONSTRUCTION side instead:
# all riding geometry (tangent points, bitangent departures, circumscribed
# arc vertices) is built on radius r + CONSTRUCTION_CLEARANCE_M, so every
# planner-made chord keeps at least that much true clearance — float noise
# (~mm) is absorbed by construction, never forgiven by validation.
# DEPRECATED, kept at 0.0 only because gui/params.py still exposes a slider
# for it; the planner treats it as exactly zero semantics.
CIRCLE_GRAZE_TOL_M = 0.0

# Construction clearance (m): the extra radius on which boundary-riding
# geometry is BUILT (r_ride = inflated radius + this). Keeps constructed
# tangent chords strictly outside the inflated boundary so the exact
# (zero-tolerance) collision check accepts them with margin far above float
# noise. Geometrically negligible: ~1 m against 23-63 km inflated radii.
CONSTRUCTION_CLEARANCE_M = 1.0

# ====== COORDINATE SYSTEM ======
# Map bounds (meters) for simulation
MAP_WIDTH = 500000.0
MAP_HEIGHT = 500000.0

# ====== A* SEARCH ======
# Number of seeded start-corner states on the start-heading ray. Corner i
# (i = 1..K, tan-uniform buckets: tan(a_i/2) = (i/K) * tan(alpha_max/2)) sits
# at d_i = L0 + R*tan(a_i/2) and affords first turns alpha <= a_i while
# keeping the takeoff straight l1 >= L0 exactly. Bucket K reproduces the
# legacy worst-case W1, so NUM_START_CORNERS = 1 is exactly legacy behaviour.
NUM_START_CORNERS = 4

# Wall-clock budget for a single search (seconds). This is the search's ONLY
# stop condition -- the old MAX_ITERATIONS cap is gone: a count of iterations
# is not a quantity an operator can reason about, and two independent stop
# conditions meant a search could end for a reason the result never named.
#
# Must be a finite number > 0; there is no "unlimited" any more. A search with
# no cap at all can hang a GUI or a service worker forever, and the value that
# used to express it (None) also read as "0.0 = no limit" on the service wire,
# which is how a request-level budget of zero silently became infinity.
#
# This is only the DEFAULT. Every entry point takes `time_budget_s=` and falls
# back here only when the caller passes nothing.
#
# NOTE this makes the search wall-clock dependent: the same mission on a slower
# machine explores fewer nodes and can return a different path. Take
# instrumented measurements with a budget large enough that nothing is
# budget-bound.
TIME_BUDGET_S: float = 15.0


def resolve_time_budget_s(value: float | None = None) -> float:
    """Resolve a caller-supplied search budget against the configured default.

    One definition of "is this a usable budget", shared by both planners and by
    the DDS service, so a value that is refused in one place is refused in all
    of them.

    Args:
        value: Seconds, or ``None`` for "the caller did not say". ``None`` is
            NOT "unlimited": an uncapped search can hold a GUI or a service
            worker forever, so there is no way to ask for one.

    Returns:
        The budget to run under, in seconds.

    Raises:
        ValueError: If the resolved budget is not a finite number > 0. NaN is
            refused explicitly because every comparison against it is false, so
            it would silently disable the deadline rather than trip it.
    """
    budget = TIME_BUDGET_S if value is None else value
    if not isinstance(budget, (int, float)) or math.isnan(budget):
        raise ValueError(f"time budget must be a finite number of seconds, got {budget!r}")
    budget = float(budget)
    if budget <= 0.0 or math.isinf(budget):
        raise ValueError(f"time budget must be finite and > 0 seconds, got {budget!r}")
    return budget

# State-lattice quantisation for A* de-duplication
STATE_POS_QUANTUM = 1000.0          # meters
STATE_HEADING_QUANTUM_DEG = 3.0     # degrees

# Heuristic weight (1.0 = Dijkstra, > 1.0 = more greedy)
HEURISTIC_WEIGHT = 1.0

# Threshold for considering a point as reached (meters)
GOAL_THRESHOLD = 1.0  # meters; reachable given STATE_POS_QUANTUM

# Cost added per radian of heading change at a transition (meters per radian)
TURN_PENALTY_WEIGHT = 0  # 4000.0

# Fallback strategy for A* when no valid successors are found: radial fan of directions
RADIAL_FAN_DIRECTIONS = 3  # number of directions in the fan

# Number of DISTANCE rungs emitted per fan direction. A fan leg must cover
# near reserve + far reserve + the doan-trinh minimum:
#
#     d_j = R*tan(theta/2) + R*tan(beta_j/2) + RADIAL_FAN_STEP_M
#
# where theta is the fan's own turn (known) and beta_j the NEXT turn at the
# pivot (deferred by _doan_trinh). The old code hardcoded beta = alpha_max, so
# every leg paid the worst-case far reserve even when the pivot barely turns —
# an unconditional bulge on fan-routed paths in open water.
#
# Rung j is instead "the shortest leg that still affords a next turn
# beta <= beta_j", tan-uniform exactly like NUM_START_CORNERS:
# tan(beta_j/2) = (j/M)*tan(alpha_max/2), so the far reserve is simply
# R*(j/M)*tan(alpha_max/2) — linear in j, no trig in the loop.
#
# Rung spacing is R*tan(alpha_max/2)/M, which must stay above
# STATE_POS_QUANTUM or adjacent rungs collapse onto the same dedup cell
# => M <= 8 at the default R / alpha_max / quantum. M = 2 gives 4 km.
#
# M = 2 is MEASURED, not assumed. Obstacle-free adverse-heading missions
# (the case the ladder targets), legacy vs M, 22 start/goal heading pairs:
#
#     M=2  0 fails, 12 better, net -49.6 km   <- best
#     M=3  4 fails,  7 better, net  +2.1 km
#     M=4  4 fails, 12 better, net -23.5 km
#
# M >= 3 pushes hard cases past the search budget (branching x3-x4), losing
# missions that legacy solved — and the lost cases are where the ladder wins
# biggest (start 90 / goal 90: 458.4 -> 410.6 km at M=2, FAIL at M=3+).
# The relation is NOT monotone in M: the coarse dedup lattice makes M=3 worse
# than both neighbours. Re-measure before changing this.
#
# A/B knob: NUM_FAN_DISTANCES = 1 together with RADIAL_FAN_STEP_M = 1000.0
# reproduces the legacy single worst-case leg exactly.
NUM_FAN_DISTANCES = 2

# Straight pad added to every fan rung on top of the two turn reserves. The
# theoretical minimum is the planner's _MIN_STRAIGHT_M (10 m), but _doan_trinh
# recomputes R*tan(turn/2) from an angle that has round-tripped through
# _angle_diff, so a rung built to land exactly on the threshold can be
# rejected by float noise. 100 m clears that by ~10 orders of magnitude while
# still shedding 90% of the old 1000 m pad.
RADIAL_FAN_STEP_M = 100.0

# Along-ray pivot slide: number of retry positions tried when a Strategy-A
# candidate is rejected (usually by _corner_arc_clear at a polygon hull vertex,
# where the fillet folds into the polygon the vertex belongs to).
#
# The pivot slides FORWARD along the incoming heading, P' = P + d*h_in, so the
# incoming leg keeps its DIRECTION and only grows: the parent's corner, its
# turn reserve and every ancestor stay valid by construction, and the straight
# budget can only increase. Sliding along the outer bisector instead would
# rotate the incoming leg and force the ancestors to be re-validated (which is
# the non-terminating version of this idea).
#
# With h_in as the x-axis and V - P = (a, b), the new turn is
# |atan2(b, a - d)|, which INCREASES with d — so the slide is capped at
# d_max = a - |b|/tan(alpha_max) and the fillet bulge grows as it is repaired.
# Retry positions are therefore parametrised by the RESULTING turn, in
# tan-uniform capability buckets tan(alpha_i/2) = (i/K)*tan(alpha_max/2) — the
# same idiom as NUM_START_CORNERS and NUM_FAN_DISTANCES — and the first bucket
# that clears wins (smallest slide = shortest detour).
#
# 0 disables the mechanism entirely (the A/B knob).
NUM_PIVOT_SLIDES = 4

# Shortest slide worth emitting (m). A bucket whose d falls below this differs
# from the un-slid corner only by float noise, so it would cost a full
# collision + arc check to re-test the geometry that was just rejected.
MIN_PIVOT_SLIDE_M = 1.0

# Straight continuation step off an inflated circle boundary (m), used only by
# the readability-first core/kinodynamic_astar_v0.py; the main planner replaced
# this wrap step with _arc_hop_successors, which has no step parameter at all.
WRAP_STEP_M = 10000.0

# Largest O..T node count smooth_path will run its exact DP on. The DP is
# O(m^3) transitions with one turn-arc check each, which is nothing at the sizes
# this planner produces (measured over 114 paths: median 9 nodes, max 21, 2.4 ms
# per path) but would be wasteful on a pathological input. Above this the path
# is returned unsmoothed rather than spending the time budget here.
SMOOTH_MAX_NODES = 64

# Tie-break for smooth_path's DP: metres of path length that one kept waypoint
# is worth. The DP minimises length alone, and a waypoint the aircraft flies
# STRAIGHT through costs exactly zero length -- measured bit-for-bit equal on
# seed 34, where the chord across three pivot-slide/fan waypoints came to
# 28299.999971999972 m either way -- so with no tie-break the DP keeps or drops
# such waypoints arbitrarily, and the delivered plan carries waypoints that mark
# no manoeuvre. Charging every waypoint a metre makes the shortest path also the
# one with the fewest waypoints. It also bounds what that preference may cost:
# at most this many metres per waypoint dropped, ~5 ppm of a 200 km mission.
# This is a TIE-BREAK, not an objective -- raising it far enough to buy real
# length is a different decision and would need its own measurement.
SMOOTH_NODE_PENALTY_M = 1.0

# Inset (m) of the shrunk polygon copies used ONLY to short-circuit the exact
# interior-overlap measurement in _check_collision. A chord whose interior
# reaches into the shrunk copy is overlapping the real polygon by more than this
# and is unambiguously blocked, so it never needs measuring; measuring costs
# 63 us against 12 us for the predicate, and 14.4% of collision calls hit a
# polygon (measured over 60 scenarios). Only chords that graze the boundary
# shallower than this fall through to the exact test -- 8 of 77333 hits. This is
# a PERFORMANCE gate, not a tolerance: it can only skip work on chords that are
# already blocked, never forgive one. Keep it far above POLYGON_TOUCH_TOL_M (the
# real threshold, 1e-6 m) and far below anything operational.
POLYGON_DEEP_HIT_INSET_M = 1e-3

# Guard band (rad) for the cheap turn prefilter in _pivot_candidate. The exact
# gate is |turn| <= alpha_build with turn from atan2; the equivalent dot-product
# form (dot >= cos(alpha_build) * seg_len) is mathematically identical but not
# bit-identical near the limit, and turns land ON the limit routinely here (0.31%
# of turn decisions sit within 1e-12 rad of alpha_max). So the prefilter rejects
# only what is over the limit BY MORE THAN THIS, and anything inside the band
# falls through to the exact test -- the cheap form can never be the one that
# decides a borderline case. 1e-6 rad is ~1e10 times the dot product's own
# relative error and still narrow enough that 55% of candidates skip the atan2s.
TURN_PREFILTER_BAND_RAD = 1e-6

# Escape-valve budget: number of expansions that may ALSO get the radial fan
# while the goal is line-of-sight blocked (cheap reorientation moves, e.g.
# recovering from an adverse initial heading). Fallback/riding fans are not
# budgeted.
#
# WARNING — ONE NAME, TWO MEANINGS. This is the exception to the "one constant
# per meaning" rule, and it is not a deliberate one:
#   v0 (the SHIPPED planner) reads it as a GLOBAL counter for the whole search,
#     with start corners EXEMPT, re-armed when the frontier nearly dies. v0 does
#     not read STRATEGY_B_CONSECUTIVE or STRATEGY_B_GLOBAL_CAP at all, so the
#     block below describes main only.
#   main reads it as the PER-PATH consecutive cap (State.consec_b), because
#     STRATEGY_B_CONSECUTIVE is True.
# So "3" is 3-in-the-whole-search in one planner and 3-in-a-row-per-path in the
# other. Do not reason about one planner's behaviour from the other's number.
#
# What the global form actually does, measured on v0 over 100 free seeds: the
# gate is reached 8,967 times and SUPPRESSES 8,747 of them (97.5%) — the budget
# empties almost immediately and then stays empty, so it behaves as an off
# switch for the occluded-goal fan rather than as an allowance. Only 220 firings
# ever spent it. Meanwhile 89% of the fan's 2,016 actual firings never consult
# it (start corner 348, goal already clear, or the no-successor fallback), and
# it bounds nothing consecutive: fan chains still reach depth 6, and 16% of
# firings are on a state already 3+ fan legs deep.
NUM_STRATEGY_B = 3

# FREE-GOAL MODE ONLY: skip the radial fan when the goal is line-of-sight clear
# and the only thing wrong with the direct leg is that it cannot supply the DSS
# run-in. Read by core/kinodynamic_astar_v0.py; False is exactly legacy.
#
# The fan cannot solve that problem. Every fan leg departs at +-alpha_max or
# straight ahead, at a fixed rung distance, and none is aimed at the goal, so
# firing here only floods the lattice near the target. Measured over 300
# free-goal seeds: 1,108 firings contributing ZERO waypoints to any delivered
# route.
#
# SCOPED TO FREE GOALS ON PURPOSE, because the same rejection means something
# else in fixed-goal mode: there it is "cannot turn onto goal_heading", the
# terminal approach problem. That one is 43.6% of all firings and DOES carry the
# route (143 waypoints over 300 seeds); dropping the fan there costs +0.426%
# with one seed at +40%. The fan is a poor tool for it -- the right tool is the
# analytic goal shot, which v0 does not have -- but a poor tool beats none.
#
# Note this is deliberately NARROWER than "the goal is line-of-sight clear".
# The fan's other goal-clear firings are lattice diversity rather than progress:
# widening the skip to all of them costs seed 51 +73.5%. And note the negative
# result next door -- skipping the fan when the goal is ALREADY an accepted
# successor also looks free (zero route yield) but measured WORSE on both axes
# (+0.0376% length, +0.93% iterations), so zero yield is not zero value here.
FAN_SKIP_ON_SHORT_RUNIN = True

# Interpretation of NUM_STRATEGY_B — IN THE MAIN PLANNER ONLY. v0 never reads
# this flag; it is hard-wired to the "False" branch described below.
#   False (legacy) = a GLOBAL budget: at most NUM_STRATEGY_B occluded-reorient
#     fan expansions in the WHOLE search (start corners exempt), re-armed when
#     the frontier nearly dies.
#   True (HYBRID) = PER-PATH cap of NUM_STRATEGY_B fan waypoints IN A ROW on any
#     single path (each State carries the running consecutive-B count; a non-fan
#     step resets it — the original intent, Wi..Wi+k all Strategy-B) PLUS a
#     global safety valve of STRATEGY_B_GLOBAL_CAP TOTAL occluded-reorient fan
#     firings. The per-path cap governs normal maps (better adverse quality);
#     the global valve (NO re-arm) stops the frontier blow-up the pure per-path
#     rule causes on pathological maps (e.g. a valid seed that otherwise times
#     out). Start corners are NOT exempt (consec_b starts at 0).
#
# DEFAULT True (hybrid) since fd7584d, which adopted it as part of removing the
# loiter macro. It was introduced as OPT-IN in 0a0eacf, and this paragraph used
# to still say so long after the value had flipped — read the value, not the
# prose. Setting it False gives the legacy global budget, which is also exactly
# what v0 does.
#
# Adopted on 40 random adverse seeds (per-path NUM_STRATEGY_B=3, GLOBAL_CAP=50)
# vs global5: +6 seeds shorter (net -33.9 km, none longer) and it fixes the
# scenario_3 wide-loop (307.8 -> 291.7 km, 17 -> 7 wp). It was NOT a strict win
# at the time — one seed regressed valid -> path_self_collision (the extra fan
# exploration surfaces a shorter path whose FINAL oracle validation fails;
# search-time checks did not then match the oracle). That caveat is recorded as
# HISTORY: it has not been re-measured since, and much of the search/oracle
# mismatch it blamed has been closed in the meantime (the fillet-arc gate, the
# interior-overlap measurement, the exact-turn smoother gate). Re-measure before
# quoting it as a live risk.
STRATEGY_B_CONSECUTIVE = True

# Global safety valve for the HYBRID Strategy-B mode: the maximum TOTAL number
# of occluded-reorient fan firings in one search before the fan is cut off
# (hard cap, no re-arm). Large enough that adverse missions (which solve in a
# few hundred expansions) keep their per-path fan chains; small enough to stop
# a per-path blow-up before the time budget. Only consulted when
# STRATEGY_B_CONSECUTIVE is True, i.e. by the MAIN planner only.
#
# 50 was the validated value in 0a0eacf (fixes seed32/seed18 where lower caps
# still failed/were-longer, no worse than 150 on the rest). It is 100 now, and
# that is NOT a measured number: fd7584d took it to 500 alongside the loiter
# removal, and 60359b1 — a smooth_path fix whose message never mentions the fan
# — brought it back to 100. Treat 100 as unvalidated until someone re-runs it.
#
# Despite the framing above ("safety valve"), this cap is what actually governs
# main: over 100 free seeds it suppresses 16,806 firings against the per-path
# cap's 2,457, so it is the primary limiter and NUM_STRATEGY_B is the secondary
# one — the reverse of what the two names suggest.
STRATEGY_B_GLOBAL_CAP = 100

# ====== GOAL SHOT (analytic terminal connect) ======
# Hybrid-A*-style analytic expansion. From each popped state the planner tries
# a 2-corner vehicle-legal maneuver straight to the goal, arriving within
# alpha_max of goal_heading; a collision-free valid one is INJECTED into OPEN
# with its true g (h=0), not returned immediately — the normal goal-accept
# block only takes it once it surfaces as the cheapest frontier node, so the
# shot prunes the adverse-approach flood (the Euclid heuristic is blind to
# the terminal heading, so misaligned states pile up near the goal) WITHOUT
# regressing path quality against a plain A* run. Fixed-goal mode only —
# free-goal is already fast.
GOAL_SHOT_ENABLED = True

# Attempt the shot every N popped states. The check is cheap (angle filter,
# then at most a few 2-segment collision checks), so 1 (every pop) is fine;
# raise it only to cap per-pop cost on ultra-dense maps where the shot rarely
# connects.
GOAL_SHOT_EVERY_N = 1

# Candidate scan resolution: turn-at-P directions across [h ± alpha_max] and
# arrival headings across [goal_heading ± alpha_max]. The grid brackets the
# shortest 2-corner maneuver; a finer grid finds a shorter (and sometimes a
# VALID where the coarse grid only found an đoản-trình-violating) one.
# MEASURED on 40 open-water adverse seeds (oracle-validated, so invalid
# re-selections count as failures, not silent successes):
#   9x9   -> 38/40 valid, mean length gap vs Dubins-LB 6.8%
#   25x25 -> 40/40 valid, 5.3%  (fixes seeds 15,35 that 9x9 could only reach
#            with an unflyable path; -1.5 pts on seeds valid at both)
# 15x15 is non-monotone (fixes 15,35 but breaks 2,10) and 19x19 fails the
# full-reversal synthetic — 25x25 is the sweet spot. Per-pop cost is flat
# (the shot returns early via inject); raise GOAL_SHOT_EVERY_N if an
# obstacle-dense map ever makes the every-pop 25x25 scan bite. A/B before
# changing — quality is non-monotone in successor density here.
GOAL_SHOT_DIRS = 25

# The two axes are NOT symmetric, and 25x25 hid that for a long time.
#
# The 2-corner length has an exact closed form (verified to 1.2e-10 relative
# error over 50,111 valid configurations):
#
#     L = |PG| * cos((a - b)/2) / cos((a + b)/2)
#
# with a, b the deviations at P and at G from the P->G bearing. Since |a + b| is
# exactly the turn at the intermediate corner C, the denominator is
# cos(turn_C/2): the length is governed by that turn, and reaching a REVERSED
# goal needs maximum angular swing, which pushes the arrival heading to the EDGE
# of its cone. Measured on 80 reversed missions, where the winning candidate sat:
#
#     arrival cone   90.5% exactly at an edge, 99.99% within 2 samples of one,
#                    0.0% in the middle          -> 25 samples resolve nothing
#     turn-at-P      0.13% within 2 of an edge, spread through the interior
#                    -> this axis genuinely needs its 25
#
# So the cone is cut to its three extremes and DIRS is left alone. Measured,
# adverse suite / 40 reversed benchmark maps, versus 25x25:
#
#     25x3 (here)  same missions solved, +0.125% length, grid 625 -> 75 points,
#                  adverse 27.6s -> 20.0s
#     13x3         same solved, +0.101%, but 113,951 adverse iterations vs 81,174
#      9x3         same solved, +0.340%, 136,687 iterations
#      5x3         same solved, +0.493%, 227,659 iterations and SLOWER overall
#
# Note the trap in that table: a smaller grid is not automatically cheaper. The
# shot connects less often, so the search does more work, and 5x3 ends up slower
# than 25x25 despite an 8x cheaper grid. Cut the axis that resolves nothing, not
# the one that does. (The older note that "9x9 -> 38/40 valid" does not conflict:
# 9x9 cuts BOTH axes, and DIRS is the one that must not be cut.)
#
# BOTH planners read this, and they are in different regimes. v0 arms the shot
# only on reversed approaches (GOAL_SHOT_MIN_REVERSAL_DEG), which is where the
# cone collapses to its edges, so 3 is free there — both standard sweeps stay
# bit-identical because the shot never fires on them. The MAIN planner still
# arms the shot on every fixed-goal mission, including the aligned ones where a
# finer cone occasionally helps, so it pays a little: measured 300 seeds fixed,
# 243 -> 243 solved, length +0.0261% (21 seeds longer, 1 shorter, 221 identical,
# worst seed 138 at +1.457%), iterations +3.23%. Giving main the same reversal
# gate would remove that cost; until then it is the price of one constant with
# one meaning.
GOAL_SHOT_CONE = 3

# Arm the shot only on missions whose APPROACH IS REVERSED: the angle between
# goal_heading and the start->goal bearing must be at least this many degrees.
# Read by core/kinodynamic_astar_v0.py; 0.0 arms it on every fixed-goal mission
# (the legacy behaviour, and what the main planner still does).
#
# ALPHA_MAX is the threshold for a reason, not by tuning. Below it a straight
# run at the goal can still turn onto goal_heading in ONE corner, which the
# ordinary Strategy-A goal candidate already builds; above it one corner cannot
# and the terminal needs the two the shot synthesises. So this arms the shot
# exactly where it is the only thing that can finish the mission.
#
# It matters because the shot is INSURANCE, not a speedup, and the premium is
# steep. Measured on v0, fixed-goal, 300 random seeds: the shot is attempted
# 55,184 times, connects 4,663 times, and 87 of those (0.16% of attempts) reach
# the delivered path -- for -0.22% length at +26% wall-clock. On missions that
# actually reverse, the same code is worth 10 extra solved missions and -77%
# wall-clock (144-case adverse suite: 131/144 at 1,177,550 iterations without,
# 141/144 at 78,979 with).
#
# Neither current benchmark contains a reversed approach: the 16 named scenarios
# top out at 45 deg and the 300-seed sweep at 89.5 deg. So this gate switches the
# shot OFF everywhere they measure and ON for the case it was written for --
# which is the point, not an oversight. Do NOT read "it never fires on the
# benchmarks" as "it is dead"; read it as "the benchmarks contain no
# turn-around mission".
GOAL_SHOT_MIN_REVERSAL_DEG = ALPHA_MAX

# ====== TURN-ARC CLEARANCE (search-time) ======
# During expansion the search only collision-checks the STRAIGHT leg of each
# successor; the radius-R fillet arc that rounds the corner AT the current
# waypoint is left to the final oracle. A free corner's arc bulges inward by
# R*(1/cos(alpha/2)-1) and can penetrate a nearby obstacle the straight legs
# cleared, so the search commits to a path only the oracle then rejects
# (path_self_collision) - no amount of successor diversity fixes it. With this
# on, each successor's corner arc is checked against the INFLATED obstacles --
# the same set the straight legs clear, same as path_validation.arcs_clear.
#
# It used to be the RAW set, and this comment said so long after it stopped
# being true: back when inflation carried an R*(1/cos(alpha_max/2)-1) turn term,
# a fillet was DESIGNED to bulge into exactly that band, so raw was the correct
# reference. With the turn term gone there is no band, and checking against raw
# would let a turn dip inside the operator's stand-off (measured: 97.9 m of true
# clearance on a run configured for 500 m). ARC_CLEARANCE_CHECK = False is
# legacy.
ARC_CLEARANCE_CHECK = True

# Arc sample count for the search-time corner-arc check. The oracle samples 24;
# 12 is enough to catch the shallow (tens-of-metres) bulges seen in practice
# while halving the per-corner cost. Segments (not just points) are checked, so
# a thin obstacle between samples is still caught along the chord.
ARC_CHECK_SAMPLES = 12

# ====== VISUALIZATION ======
PLOT_BUFFER_ZONES = True
PLOT_START_END_MARKERS = True

# Figure DPI for saving
FIGURE_DPI = 200

# ====== SCENARIO GENERATION (map_generator) ======
# Obstacle detection radius (m)
OBSTACLE_RADIUS_MIN = 10000.0
OBSTACLE_RADIUS_MAX = 50000.0

# Island polygon size
ISLAND_SIZE_MIN = 5000.0
ISLAND_SIZE_MAX = 30000.0

# Number of vertices for irregular polygons
ISLAND_VERTICES_MIN = 4
ISLAND_VERTICES_MAX = 8

# Minimum gap (m) between two generated islands, mirroring the separation the
# circle generator has always enforced. Without it islands overlap freely:
# measured over 200 scenarios, 183 contained overlapping pairs (median 7, max
# 62) and 21.2% of polygon hull vertices sat buried INSIDE another polygon —
# candidates the search re-tests and re-rejects on every expansion.
# This is a shape constraint, not a flyability one: at R = 8000 m a 500 m
# corridor is unusable either way, the point is that the obstacle set stays
# geometrically well-formed.
ISLAND_MIN_SEPARATION_M = 500.0

# Same rule for circles, measured between the two BOUNDARIES: two sites are
# separated when dist(centres) >= r_i + r_j + this. The old code compared
# against a flat 2*OBSTACLE_RADIUS_MAX + 500 = 100.5 km instead, charging every
# pair the worst-case radius — on a 500 km map that capped the field at ~13
# circles no matter how many were asked for (measured: median 6, max 13 when
# requesting 0-50).
CIRCLE_MIN_SEPARATION_M = 500.0

# Minimum clearance (m) required between start/goal and any generated obstacle.
# This used to read config.EPS, i.e. 1e-6 m — a buffer that permitted an
# obstacle to touch the start point. Measured at that setting: 16% of scenarios
# put start or goal closer than L0 to an obstacle, so the mandatory takeoff or
# run-in leg was born blocked.
SPAWN_CLEARANCE_M = 5000.0

# ====== UTILS ======

def deg_to_rad(degrees: float) -> float:
    """Convert degrees to radians"""
    return math.radians(degrees)

def rad_to_deg(radians: float) -> float:
    """Convert radians to degrees"""
    return math.degrees(radians)

# Pre-compute often-used values
ALPHA_MAX_RAD = deg_to_rad(ALPHA_MAX)

EPS = 1e-6

# ====== ROUNDING GUARDS (construction side only) ======
# Float64 rounding pads. These exist so that geometry BUILT to sit exactly on a
# limit is not rejected by the exact check that follows: a chord constructed
# tangent to a circle has distance-to-centre == r in exact arithmetic, but lands
# a few ULP either side in practice. Measured on this planner (2440 tangents,
# coordinates ~2e5 m): |dist - r| median 7.3e-12 m, max 7.5e-11 m ~= 1 ULP, and
# 43.3% of tangents fell INSIDE the circle, i.e. self-rejected against the exact
# `dist < radius` test.
#
# Use them ONLY when CONSTRUCTING geometry, never to soften a check — a check
# with slack forgives a real intrusion and destroys the guarantee that a
# reported clearance is the true one.
#
# Pad TOWARDS FEASIBILITY, which is not always "+": add to an obstacle radius,
# but SUBTRACT from a turn limit (build alpha <= alpha_max - GEOM_EPS_RAD) and
# build straight runs LONGER than their floor. Adding to alpha_max would
# construct the very violation the pad is meant to prevent.
#
# Magnitude: an absolute pad must clear one ULP at the largest coordinate in
# play. ULP is ~1.2e-10 m at the 500 km map edge and ~2.3e-10 m at y ~ 1.15e6
# (real missions), so 1e-8 m keeps ~25x headroom while remaining 1e-12 of the
# 8 km turn radius, i.e. geometrically meaningless. Do NOT raise it into
# millimetres to "be safe": measured, a 1e-3 m pad already costs 1.7-2.5% path
# length, and a 1 m pad costs the same while buying nothing extra. That cost is
# a STAND-OFF, which is what SAFE_MARGIN / CONSTRUCTION_CLEARANCE_M are for.
GEOM_EPS_M = 1e-8

# Shortest Strategy-A candidate edge worth evaluating (m). A candidate this
# close to the current waypoint is skipped before _pivot_candidate runs.
#
# The two planners disagreed here, and not by style: main skipped anything
# within 100 m (a bare `< 10000` squared-metres literal in the successor loop),
# v0 skipped only the genuinely degenerate zero-length edge. So main was
# discarding candidates v0 evaluated, which is a difference in the search graph,
# not in taste.
#
# The floor is GEOM_EPS_M regardless of this value: a zero-length edge has no
# heading and must never reach _pivot_candidate. Set 0 for "degenerate only"
# (v0's historical rule).
#
# MEASURED over 300 seeds x both goal modes, main at 100 m vs at 0:
#   - every path bit-identical, every mission solved, same lengths
#   - _pivot_candidate calls 3,836,890 -> 3,845,944, i.e. +0.236%
# So the two rules are outcome-equivalent on this distribution and the 100 m
# skip was buying 0.2% of candidate evaluations. 0 wins on risk rather than on
# the clock: 100 m is a number with no derivation, and it silently discards a
# legitimate tangent point or hull vertex on any map whose features are closer
# together than that -- these maps have 20-60 km islands, a real one need not.
#
# Do not tune by intuition if you revisit this: quality is non-monotone in
# successor count here (see NUM_FAN_DISTANCES), so re-measure, and measure the
# CALL COUNTS rather than the clock (config.TIME_BUDGET_S makes wall-clock
# results machine-dependent -- see scripts/BENCHMARKS.md).
CANDIDATE_MIN_DIST_M = 0.0

# Angular twin of GEOM_EPS_M. Measured: 0.31% of turn decisions sit within
# 1e-12 rad of alpha_max (they are the fan rungs and start corners built to
# afford exactly alpha_max), and widening the band from 1e-12 to 1e-3 rad only
# grows that set from 1410 to 1707 — so the population is genuinely AT the
# limit, and 1e-9 rad clears it with three orders to spare. 1e-9 rad over an
# 8 km radius is 8 microns of arc.
GEOM_EPS_RAD = 1e-9