"""
Configuration Module for Missile Path Planning System
Defines tactical constants and parameters
"""

import math

# ====== DYNAMIC CONSTRAINTS ======
# Turn radius (m) - fixed for entire trajectory
R = 500.0

# Maximum turn angle allowed (degrees)
ALPHA_MAX = 30.0  # in degrees, will be converted to radians

# Convert ALPHA_MAX to radians for calculations
ALPHA_MAX_RAD = math.radians(ALPHA_MAX)

# Aliases for GUI compatibility
TURN_RADIUS = R  # Alias for R (turn radius in meters)

# Minimum distance for level flight and stabilization (m)
# After launch, distance to descend and stabilize for sea-skimming
L0 = 4000.0

# Distance for seeker to lock and guide to target (m)
DSS = 23000.0

# Launch angle (degrees) - angle from horizontal at launch
LAUNCH_ANGLE_MIN = 5.0     # Minimum launch angle
LAUNCH_ANGLE_MAX = 25.0    # Maximum launch angle
LAUNCH_ANGLE_DEFAULT = 15.0  # Default launch angle

# Approach angle (degrees) - angle from horizontal at target approach
APPROACH_ANGLE_MIN = 10.0     # Minimum approach angle
APPROACH_ANGLE_MAX = 45.0     # Maximum approach angle
APPROACH_ANGLE_DEFAULT = 30.0  # Default approach angle

# ====== SAFETY & OBSTACLE HANDLING ======
# Safety margin buffer (m) - distance to expand obstacle boundaries
SAFE_MARGIN = 100.0

# ====== COORDINATE SYSTEM ======
# Map bounds (meters) for simulation
MAP_WIDTH = 500000.0
MAP_HEIGHT = 500000.0
MAP_ORIGIN = (0.0, 0.0)

# ====== A* SEARCH ======
# Maximum iterations for A* search
MAX_ITERATIONS = 50000

# Heuristic weight (1.0 = Dijkstra, > 1.0 = more greedy)
HEURISTIC_WEIGHT = 1.2

# Threshold for considering a point as reached (meters)
GOAL_THRESHOLD = 200.0

# ====== TANGENT GRAPH ======
# Minimum angle between tangent lines (degrees)
MIN_TANGENT_ANGLE = 10.0

# Number of sampling points on each tangent
TANGENT_SAMPLES = 50

# ====== VISUALIZATION ======
PLOT_BITANGENTS = True
PLOT_BUFFER_ZONES = True
PLOT_START_END_MARKERS = True

# Figure DPI for saving
FIGURE_DPI = 150

# ====== SCENARIO PARAMETERS ======
# Scenario 1: Open ocean
SCENARIO1_ISLANDS = 0
SCENARIO1_SAM_SITES = 0

# Scenario 2: Single obstacle
SCENARIO2_ISLANDS = 1
SCENARIO2_SAM_SITES = 1

# Scenario 3: Narrow gap
SCENARIO3_ISLANDS = 2
SCENARIO3_SAM_SITES = 0

# Scenario 4: Complex maze
SCENARIO4_ISLANDS = 20
SCENARIO4_SAM_SITES = 10

# SAM detection radius (m)
SAM_RADIUS_MIN = 3000.0
SAM_RADIUS_MAX = 5000.0

# Island polygon size
ISLAND_SIZE_MIN = 500.0
ISLAND_SIZE_MAX = 3000.0

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
