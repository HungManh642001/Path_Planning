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
SAFE_MARGIN = 10000.0

# Polygon inflation join style: 'mitre' keeps sharp corners so each obstacle
# yields a few real corner vertices (used as navigation waypoints) instead of
# ~70 rounded arc points. mitre_limit caps the corner-spike length; it is large
# enough that the mitre polygon always CONTAINS the exact round Minkowski buffer
# (preserving the arc-clearance guarantee) for the convex-ish islands here.
POLYGON_MITRE_LIMIT = 5.0

# DEPRECATED: the planner no longer reads this (arc-hop successors replaced
# the wrap step). Kept only because gui/params.py still exposes a slider that
# writes it; delete together with the GUI panel update.
WRAP_STEP_M = 10000.0

# Angular step (deg) for expanding a circle-boundary arc into waypoint
# vertices (circumscribed polygon) at OUTPUT time. Max supported 45. Search
# connectivity does NOT depend on it: arc clearance is checked at the fixed
# 45-deg bulge radius r/cos(pi/8), which covers any expansion step <= 45 deg.
ARC_WAYPOINT_STEP_DEG = 30.0

# Angular step (deg) for sampling arc clearance during search.
ARC_SAMPLE_STEP_DEG = 5.0

# Tolerance (m) by which a straight segment may graze inside a circle's
# INFLATED boundary. Shared by the planner (_check_collision) and, when
# validating planner output, the oracle (path_is_valid(..., circle_tol=...)).
# Set from measurement: the max body graze of legitimate raw-safe paths was
# ~20.97 m (arc-expansion chords / tangent segments dipping into the inflation
# band); this value adds headroom. It is a tiny fraction of the ~13.3 km
# inflation band and never approaches the raw obstacle. Polygon interior is
# checked tolerance-free.
CIRCLE_GRAZE_TOL_M = 23.0

# ====== COORDINATE SYSTEM ======
# Map bounds (meters) for simulation
MAP_WIDTH = 500000.0
MAP_HEIGHT = 500000.0
MAP_ORIGIN = (0.0, 0.0)

# ====== A* SEARCH ======
# Maximum iterations for A* search
MAX_ITERATIONS = 50000

# Wall-clock budget for a single search (seconds). None = no time limit.
TIME_BUDGET_S = 5  # 0.9

# State-lattice quantisation for A* de-duplication
STATE_POS_QUANTUM = 1000.0          # meters
STATE_HEADING_QUANTUM_DEG = 3.0     # degrees

# Heuristic weight (1.0 = Dijkstra, > 1.0 = more greedy)
HEURISTIC_WEIGHT = 1.0

# Threshold for considering a point as reached (meters)
GOAL_THRESHOLD = 1000.0  # meters; reachable given STATE_POS_QUANTUM

# Cost added per radian of heading change at a transition (meters per radian)
TURN_PENALTY_WEIGHT = 0  # 4000.0

# Fallback strategy for A* when no valid successors are found: radial fan of directions
RADIAL_FAN_DIRECTIONS = 3  # number of directions in the fan
RADIAL_FAN_STEP_M = 1000.0  # step size for the radial fan

# Escape-valve budget: number of expansions that may ALSO get the radial fan
# while the goal is line-of-sight blocked (cheap reorientation moves, e.g.
# recovering from an adverse initial heading). Fallback/riding fans are not
# budgeted.
NUM_STRATEGY_B = 3

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