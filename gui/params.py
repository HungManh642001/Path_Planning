"""Parameter specifications for the planner GUI and config-override application.

The planner reads several tunables directly from the `config` module, so the GUI
applies panel values by writing them onto `config` before each run (single process).
Physical params consumed by prepare_scenario are returned as keyword arguments.
"""

import math

import config

# key, label, group, min, max, default
PARAM_SPECS = [
    # --- tactical (physical) ---
    {'key': 'turn_radius',  'label': 'Turn Radius R (m)',   'group': 'tactical', 'min': 3000.0,  'max': 15000.0, 'default': config.R},
    {'key': 'alpha_max_deg','label': 'Max Turn Angle (deg)','group': 'tactical', 'min': 10.0,    'max': 90.0,    'default': config.ALPHA_MAX},
    {'key': 'l0',           'label': 'L0 stabilize (m)',    'group': 'tactical', 'min': 1000.0,  'max': 20000.0, 'default': config.L0},
    {'key': 'dss',          'label': 'Seeker DSS (m)',      'group': 'tactical', 'min': 5000.0,  'max': 60000.0, 'default': config.DSS},
    {'key': 'safe_margin',  'label': 'Safe Margin (m)',     'group': 'tactical', 'min': 0.0,     'max': 30000.0, 'default': config.SAFE_MARGIN},
    {'key': 'launch_angle', 'label': 'Launch Angle (deg)',  'group': 'tactical', 'min': config.LAUNCH_ANGLE_MIN,   'max': config.LAUNCH_ANGLE_MAX,   'default': config.LAUNCH_ANGLE_DEFAULT},
    {'key': 'approach_angle','label': 'Approach Angle (deg)','group': 'tactical','min': config.APPROACH_ANGLE_MIN, 'max': config.APPROACH_ANGLE_MAX, 'default': config.APPROACH_ANGLE_DEFAULT},
    # --- run ---
    {'key': 'time_budget_s','label': 'Time Budget (s)',     'group': 'run',      'min': 0.1,     'max': 10.0,    'default': config.TIME_BUDGET_S},
    {'key': 'max_iterations','label': 'Max Iterations',     'group': 'run',      'min': 1000.0,  'max': 200000.0,'default': float(config.MAX_ITERATIONS)},
    # --- advanced (search internals) ---
    {'key': 'state_pos_quantum',     'label': 'Pos Quantum (m)',     'group': 'advanced', 'min': 100.0,  'max': 5000.0,  'default': config.STATE_POS_QUANTUM},
    {'key': 'state_heading_quantum', 'label': 'Heading Quantum (deg)','group': 'advanced','min': 0.5,    'max': 15.0,    'default': config.STATE_HEADING_QUANTUM_DEG},
    {'key': 'heuristic_weight',      'label': 'Heuristic Weight',    'group': 'advanced', 'min': 1.0,    'max': 5.0,     'default': config.HEURISTIC_WEIGHT},
    {'key': 'turn_penalty_weight',   'label': 'Turn Penalty (m/rad)','group': 'advanced', 'min': 0.0,    'max': 20000.0, 'default': config.TURN_PENALTY_WEIGHT},
    {'key': 'goal_threshold',        'label': 'Goal Threshold (m)',  'group': 'advanced', 'min': 100.0,  'max': 5000.0,  'default': config.GOAL_THRESHOLD},
    {'key': 'polygon_mitre_limit',   'label': 'Polygon Mitre Limit', 'group': 'advanced', 'min': 1.0,    'max': 5.0,     'default': config.POLYGON_MITRE_LIMIT},
    {'key': 'wrap_step_m',           'label': 'Wrap Step (m)',       'group': 'advanced', 'min': 500.0,  'max': 20000.0, 'default': config.WRAP_STEP_M},
    {'key': 'circle_graze_tol_m',    'label': 'Circle Graze Tol (m)','group': 'advanced', 'min': 0.0,    'max': 500.0,   'default': config.CIRCLE_GRAZE_TOL_M},
    {'key': 'obstacle_ring_samples', 'label': 'Ring Samples',        'group': 'advanced', 'min': 6.0,    'max': 64.0,    'default': float(config.OBSTACLE_RING_SAMPLES)},
]


def default_values():
    return {s['key']: s['default'] for s in PARAM_SPECS}


def apply_overrides(values):
    """Write GUI values onto config.* and return prepare_scenario kwargs."""
    config.TIME_BUDGET_S = float(values['time_budget_s'])
    config.MAX_ITERATIONS = int(values['max_iterations'])
    config.STATE_POS_QUANTUM = float(values['state_pos_quantum'])
    config.STATE_HEADING_QUANTUM_DEG = float(values['state_heading_quantum'])
    config.HEURISTIC_WEIGHT = float(values['heuristic_weight'])
    config.TURN_PENALTY_WEIGHT = float(values['turn_penalty_weight'])
    config.GOAL_THRESHOLD = float(values['goal_threshold'])
    config.POLYGON_MITRE_LIMIT = float(values['polygon_mitre_limit'])
    config.WRAP_STEP_M = float(values['wrap_step_m'])
    config.CIRCLE_GRAZE_TOL_M = float(values['circle_graze_tol_m'])
    config.OBSTACLE_RING_SAMPLES = int(values['obstacle_ring_samples'])
    # ALPHA_MAX is stored in degrees in config; keep it consistent.
    config.ALPHA_MAX = float(values['alpha_max_deg'])
    return {
        'R': float(values['turn_radius']),
        'L0': float(values['l0']),
        'DSS': float(values['dss']),
        'safe_margin': float(values['safe_margin']),
        'alpha_max_rad': math.radians(float(values['alpha_max_deg'])),
    }
