"""
Configuration Module for Autonomous Aircraft Path Planning System
Defines operational parameters and parameters
"""

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
MAP_ORIGIN = (0.0, 0.0)

# ====== A* SEARCH ======
# Number of seeded start-corner states on the start-heading ray. Corner i
# (i = 1..K, tan-uniform buckets: tan(a_i/2) = (i/K) * tan(alpha_max/2)) sits
# at d_i = L0 + R*tan(a_i/2) and affords first turns alpha <= a_i while
# keeping the takeoff straight l1 >= L0 exactly. Bucket K reproduces the
# legacy worst-case W1, so NUM_START_CORNERS = 1 is exactly legacy behaviour.
NUM_START_CORNERS = 4

# Maximum iterations for A* search
MAX_ITERATIONS = 50000

# Wall-clock budget for a single search (seconds). None = no time limit.
TIME_BUDGET_S = 10  # 0.9

# State-lattice quantisation for A* de-duplication
STATE_POS_QUANTUM = 1000.0          # meters
STATE_HEADING_QUANTUM_DEG = 3.0     # degrees

# Heuristic weight (1.0 = Dijkstra, > 1.0 = more greedy)
HEURISTIC_WEIGHT = 1.0

# Grid resolution (cells on the long side) for the admissible goal-distance
# field heuristic (core/heuristic_field.py). The field only ever tightens h
# via max(euclid, field), so a coarser grid degrades toward plain Euclid.
HEURISTIC_GRID_N = 256

# Lazy-build threshold: the goal-distance field is built only when the
# search reaches this many iterations without finishing (proof of real
# Euclid flooding). 734/869 solved random seeds finish under 300 iterations
# and would pay the ~0.3 s build for nothing (measured +185 s over a
# 1000-seed sweep when building eagerly at init).
HEURISTIC_FIELD_LAZY_ITERS = 300

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
# M >= 3 pushes hard cases past MAX_ITERATIONS (branching x3-x4), losing
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

# Escape-valve budget: number of expansions that may ALSO get the radial fan
# while the goal is line-of-sight blocked (cheap reorientation moves, e.g.
# recovering from an adverse initial heading). Fallback/riding fans are not
# budgeted.
NUM_STRATEGY_B = 5

# ====== VISUALIZATION ======
PLOT_BUFFER_ZONES = True
PLOT_START_END_MARKERS = True

# Figure DPI for saving
FIGURE_DPI = 150

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

# ====== UTILS ======
import math

def deg_to_rad(degrees):
    """Convert degrees to radians"""
    return math.radians(degrees)

def rad_to_deg(radians):
    """Convert radians to degrees"""
    return math.degrees(radians)

# Pre-compute often-used values
ALPHA_MAX_RAD = deg_to_rad(ALPHA_MAX)

EPS = 1e-6