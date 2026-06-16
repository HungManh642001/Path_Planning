"""
Preprocessing & Boundary Calculation Module
Handles obstacle inflation, start/goal state calculation
"""

import math
import config
import spatial_utils as su


def inflate_obstacles(obstacles, R=config.R, safe_margin=config.SAFE_MARGIN, alpha_max_rad=config.ALPHA_MAX_RAD):
    """
    Inflate obstacle boundaries by R + SAFE_MARGIN.
    Keeps obstacles independent (no early Convex Hull).
    
    Args:
        obstacles: List of obstacle dicts with 'type', 'polygon' or 'center'/'radius'
        R: Turn radius
        safe_margin: Additional safety buffer
        alpha_max_rad: Maximum turn angle allowed (radians)
    
    Returns:
        List of inflated obstacles
    """
    inflation = R * (1 / math.cos(alpha_max_rad / 2) - 1) + safe_margin
    inflated = []
    
    for obstacle in obstacles:
        inflated_obs = obstacle.copy()
        
        if obstacle['type'] == 'circle':
            center = obstacle['center']
            radius = obstacle['radius']
            inflated_obs['radius'] = radius + inflation
            
        elif obstacle['type'] == 'polygon':
            polygon = obstacle['polygon']
            inflated_obs['polygon'] = su.inflate_polygon(polygon, inflation)
        
        inflated.append(inflated_obs)
    
    return inflated


def calculate_start_state(origin, init_heading, L0=config.L0, R=config.R, alpha_max_rad=config.ALPHA_MAX_RAD):
    """
    Calculate the first waypoint W_1 and its heading after launch.
    
    From dynamics: d_1 = l_1 + R * tan(α_1 / 2)
    With constraint: l_1 ≥ L_0
    
    For simplicity, we place W_1 at distance d_1 in direction init_heading.
    
    Args:
        origin: (x, y) launch point O
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
    
    # With minimal turn angle α_1 ≈ 0, d_1 ≈ l_1
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
    Calculate the final waypoint W_{n-1} before seeker engagement.
    
    From dynamics: d_n = l_n + d_ss + R * tan(α_{n-1} / 2)
    With l_n = 0 (we reach W_{n-1} directly at target)
    
    Args:
        target: (x, y) target position T
        target_heading: Final approach heading (radians)
        dss: Distance for seeker lock-on and guidance
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


def validate_kinodynamics(w_i, heading_i, w_next, heading_next, w_next_next=None, 
                         heading_next_next=None, R=config.R, alpha_max=config.ALPHA_MAX_RAD):
    """
    Validate kinodynamic constraints for a segment.
    
    Checks:
    1. Turn angle α ≤ α_max
    2. Straight segment length l > 0
    
    Args:
        w_i: Current waypoint (x, y)
        heading_i: Current heading (radians)
        w_next: Next waypoint (x, y)
        heading_next: Next heading (radians)
        w_next_next: Following waypoint (optional)
        heading_next_next: Following heading (optional)
        R: Turn radius
        alpha_max: Maximum turn angle (radians)
    
    Returns:
        Tuple (is_valid, error_message)
    """
    
    # Check turn angle constraint: |Δheading| ≤ α_max
    delta_heading = heading_next - heading_i
    delta_heading = math.atan2(math.sin(delta_heading), math.cos(delta_heading))
    alpha = abs(delta_heading)
    
    if alpha > alpha_max:
        return False, f"Turn angle {math.degrees(alpha):.2f}° exceeds max {math.degrees(alpha_max):.2f}°"
    
    # Check straight segment length
    # d_{i+1} = l_{i+1} + R * (tan(α_i/2) + tan(α_{i+1}/2))
    # We need l_{i+1} > 0
    
    
    if alpha_max is not None:
        # delta_heading_next = heading_next_next - heading_next
        # delta_heading_next = math.atan2(math.sin(delta_heading_next), math.cos(delta_heading_next))
        alpha_next = alpha_max
        
        # Distance from w_i to w_next
        d_segment = math.sqrt((w_next[0] - w_i[0])**2 + (w_next[1] - w_i[1])**2)
        
        # Calculate required straight length
        l_required = d_segment - R * (math.tan(alpha / 2) + math.tan(alpha_next / 2))
        
        if l_required < 10.0:  # Small threshold for numerical stability
            return False, f"Straight segment length {l_required:.2f}m is too small (need > 10m)"
    
    return True, "OK"


def compute_inflated_obstacles(obstacles, R=config.R, safe_margin=config.SAFE_MARGIN, alpha_max_rad=config.ALPHA_MAX_RAD):
    """
    Pre-process all obstacles: inflate them and create buffer zones.
    
    Args:
        obstacles: List of raw obstacles
        R: Turn radius
        safe_margin: Safety buffer
        alpha_max_rad: Maximum turn angle allowed (radians)
    
    Returns:
        Dict with:
            - 'inflated_obstacles': inflated obstacle list
            - 'circle_obstacles': list of (center, radius) for circles
            - 'polygon_obstacles': list of polygon coordinates
    """
    
    inflated = inflate_obstacles(obstacles, R, safe_margin, alpha_max_rad)
    
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
        DSS: Seeker engagement distance
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
    goal_state = calculate_end_state(scenario['goal'], scenario['goal_heading'], DSS, R, alpha_max_rad)
    
    # Process obstacles
    inflated_data = compute_inflated_obstacles(scenario['obstacles'], R, safe_margin, alpha_max_rad)
    
    return {
        'start_state': start_state,
        'goal_state': goal_state,
        'start_pos': scenario['start'],
        'goal_pos': scenario['goal'],
        'start_heading': scenario['start_heading'],
        'goal_heading': scenario['goal_heading'],
        'turn_radius': R,
        'alpha_max_rad': alpha_max_rad,
        'obstacles': inflated_data['inflated_obstacles'],
        'circle_obstacles': inflated_data['circle_obstacles'],
        'polygon_obstacles': inflated_data['polygon_obstacles'],
        'islands': scenario.get('islands', []),
        'sam_sites': scenario.get('sam_sites', []),
    }
