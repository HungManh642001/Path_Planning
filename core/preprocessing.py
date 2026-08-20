"""
Preprocessing & Boundary Calculation Module
Handles obstacle inflation, start/goal state calculation
"""

import math
import config
import core.spatial_utils as su


def inflation_ring(safe_margin=config.SAFE_MARGIN):
    """Obstacle boundary offset for display: exactly what inflate_obstacles applies.

    There is a SINGLE ring, `safe_margin`. This used to return a PAIR because a
    second ring added a `R*(1/cos(alpha_max/2)-1)` turn term reserving the
    worst-case fillet bulge; that term is gone (the search checks each arc
    exactly instead), so the two values had become identical and the only caller
    was already discarding the second.
    """
    return safe_margin


def inflate_obstacles(obstacles, safe_margin=config.SAFE_MARGIN):
    """
    Inflate obstacle boundaries by SAFE_MARGIN.
    Keeps obstacles independent (no early Convex Hull).

    Args:
        obstacles: List of obstacle dicts with 'type', 'polygon' or 'center'/'radius'
        safe_margin: The operator's minimum stand-off distance (m)

    Returns:
        List of inflated obstacles
    """
    # SAFE_MARGIN only. The old `R*(1/cos(alpha_max/2)-1)` turn term covered the
    # worst-case bulge of a fillet arc into the corner it cuts; it is gone
    # because the search now checks that bulge EXACTLY, per corner, with the
    # real turn angle (`_corner_arc_clear`). Sized for alpha_max and applied to
    # every obstacle, the term closed 49% of all corridors between obstacle
    # pairs (measured, 1536 pairs) — a straight transit paid the same 3.3 km as
    # a 90-degree corner. Inflation is now purely the operator's minimum
    # stand-off distance. See docs/superpowers/specs/2026-08-08-obstacle-
    # inflation-safe-margin-design.md.
    inflation = safe_margin
    inflated = []
    
    for obstacle in obstacles:
        inflated_obs = obstacle.copy()
        
        if obstacle['type'] == 'circle':
            radius = obstacle['radius']
            inflated_obs['radius'] = radius + inflation
            
        elif obstacle['type'] == 'polygon':
            polygon = obstacle['polygon']
            inflated_obs['polygon'] = su.inflate_polygon(polygon, inflation)
        
        inflated.append(inflated_obs)
    
    return inflated


def calculate_start_state(origin, init_heading, L0=config.L0, R=config.R, alpha_max_rad=config.ALPHA_MAX_RAD):
    """
    Calculate the first waypoint W_1 and its heading after takeoff.
    
    From dynamics: d_1 = l_1 + R * tan(α_1 / 2)
    With constraint: l_1 ≥ L_0
    
    For simplicity, we place W_1 at distance d_1 in direction init_heading.
    
    Args:
        origin: (x, y) takeoff point O
        init_heading: Initial heading angle (radians)
        L0: Minimum distance for level flight stabilization
        R: Turn radius
        alpha_max_rad: Maximum turn angle allowed (radians)
    
    Returns:
        Dict with:
            - 'waypoint': (x, y) position of W_1
            - 'heading': heading angle at W_1
            - 'straight_length': l_1
            - 'distance_from_origin': d_1
    """
    
    # Assume we go straight for distance L0
    l_1 = L0

    # Conservative: reserve tangent length for the worst-case first turn α₁ = α_max,
    # so d₁ = L0 + R*tan(α_max/2) and l₁ = L0 exactly (l₁ ≥ L0 holds).
    d_1 = l_1 + R * math.tan(alpha_max_rad / 2)
    
    # Calculate W_1 position
    w1_x = origin[0] + d_1 * math.cos(init_heading)
    w1_y = origin[1] + d_1 * math.sin(init_heading)
    
    return {
        'waypoint': (w1_x, w1_y),
        'heading': init_heading,
        'straight_length': l_1,
        'distance_from_origin': d_1,
    }


def calculate_end_state(target, target_heading, dss=config.DSS, R=config.R, alpha_max_rad=config.ALPHA_MAX_RAD):
    """
    Calculate the final waypoint W_{n-1} before terminal camera sensor lock.
    
    From dynamics: d_n = l_n + d_ss + R * tan(α_{n-1} / 2)
    With l_n = 0 (we reach W_{n-1} directly at goal), so d_n = d_ss + R * tan(α_{n-1} / 2)
    
    Args:
        target: (x, y) goal position T
        target_heading: Final approach heading (radians)
        dss: Distance for terminal camera sensor lock
        R: Turn radius
        alpha_max_rad: Maximum turn angle allowed (radians)
    
    Returns:
        Dict with:
            - 'waypoint': (x, y) position of W_{n-1}
            - 'heading': heading angle at W_{n-1}
            - 'engagement_distance': d_ss
            - 'distance_to_target': d_n
    """

    d_n = dss + R * math.tan(alpha_max_rad / 2)
    
    # Work backwards from target
    # Position W_{n-1} at distance d_ss before target
    w_n_minus_1_x = target[0] - d_n * math.cos(target_heading)
    w_n_minus_1_y = target[1] - d_n * math.sin(target_heading)
    
    return {
        'waypoint': (w_n_minus_1_x, w_n_minus_1_y),
        'heading': target_heading,
        'engagement_distance': dss,
        'distance_to_target': d_n,
    }


def compute_inflated_obstacles(obstacles, safe_margin=config.SAFE_MARGIN):
    """
    Pre-process all obstacles: inflate them and create buffer zones.
    
    Args:
        obstacles: List of raw obstacles
        safe_margin: The operator's minimum stand-off distance (m)

    Returns:
        Dict with:
            - 'inflated_obstacles': inflated obstacle list
            - 'circle_obstacles': list of (center, radius) for circles
            - 'polygon_obstacles': list of polygon coordinates
    """
    
    inflated = inflate_obstacles(obstacles, safe_margin)
    
    circle_obstacles = []
    polygon_obstacles = []
    
    for obs in inflated:
        if obs['type'] == 'circle':
            circle_obstacles.append((obs['center'], obs['radius']))
        elif obs['type'] == 'polygon':
            polygon_obstacles.append(obs['polygon'])
    
    return {
        'inflated_obstacles': inflated,
        'circle_obstacles': circle_obstacles,
        'polygon_obstacles': polygon_obstacles,
    }


def prepare_scenario(scenario, R=config.R, L0=config.L0, DSS=config.DSS, safe_margin=config.SAFE_MARGIN, alpha_max_rad=config.ALPHA_MAX_RAD):
    """
    Full preprocessing of a scenario: inflate obstacles, calculate states.
    
    Args:
        scenario: Scenario dict from map_generator
        R: Turn radius
        L0: Minimum stabilization distance
        DSS: Distance for terminal camera sensor lock
        safe_margin: Safety margin buffer (m) - distance to expand obstacle boundaries
        alpha_max_rad: Maximum turn angle allowed (radians)
    
    Returns:
        Dict with:
            - 'start_state': Dict with waypoint, heading, straight_length
            - 'goal_state': Dict with waypoint, heading
            - 'start_pos': Original start position O
            - 'goal_pos': Original target position T
            - 'obstacles': Inflated obstacles
            - 'circle_obstacles': List of circle obstacles
            - 'polygon_obstacles': List of polygon obstacles
    """
    
    # Calculate start and goal waypoints
    start_state = calculate_start_state(scenario['start'], scenario['start_heading'], L0, R, alpha_max_rad)
    if scenario['goal_heading'] is None:
        # Free terminal approach direction: there is no fixed goal_heading to
        # offset W_{n-1} along, so the search targets T itself and the final
        # searched edge becomes the straight seeker run-in (>= DSS, any
        # direction). heading=None flags free mode for the planner.
        goal_state = {
            'waypoint': scenario['goal'],
            'heading': None,
            'engagement_distance': DSS,
            'distance_to_target': DSS,
        }
    else:
        goal_state = calculate_end_state(scenario['goal'], scenario['goal_heading'], DSS, R, alpha_max_rad)
    
    # Process obstacles
    inflated_data = compute_inflated_obstacles(scenario['obstacles'], safe_margin)

    # Raw (uninflated) obstacle sets, threaded through for callers that want to
    # measure or draw the true obstacle. They are NO LONGER the arc-clearance
    # reference: straight legs and turn arcs both clear the inflated set
    # (raw + SAFE_MARGIN) now that inflation carries no turn term — see
    # path_validation.path_is_valid.
    raw_circle_obstacles = [(o['center'], o['radius'])
                            for o in scenario['obstacles'] if o['type'] == 'circle']
    raw_polygon_obstacles = [o['polygon']
                             for o in scenario['obstacles'] if o['type'] == 'polygon']

    return {
        'start_state': start_state,
        'goal_state': goal_state,
        'start_pos': scenario['start'],
        'goal_pos': scenario['goal'],
        'start_heading': scenario['start_heading'],
        'goal_heading': scenario['goal_heading'],
        'turn_radius': R,
        'alpha_max_rad': alpha_max_rad,
        'safe_margin': safe_margin,
        'obstacles': inflated_data['inflated_obstacles'],
        'circle_obstacles': inflated_data['circle_obstacles'],
        'polygon_obstacles': inflated_data['polygon_obstacles'],
        'raw_circle_obstacles': raw_circle_obstacles,
        'raw_polygon_obstacles': raw_polygon_obstacles,
        'islands': scenario.get('islands', []),
        'dynamic_obstacles': scenario.get('dynamic_obstacles', []),
        # Per-scenario operating area / bounds. `safezones` is an optional list
        # of polygons (the aircraft must stay inside their union); `map_bounds`
        # is the legacy (width, height) rectangle. Both are threaded through so
        # the planner can constrain the search to them instead of the global
        # config.MAP_WIDTH/HEIGHT.
        'safezones': scenario.get('safezones'),
        'map_bounds': scenario.get('map_bounds'),
    }
