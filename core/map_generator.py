"""
Mock Map Generator Module
Generates synthetic mission scenarios with islands and dynamic obstacles for testing path planning algorithms.
"""

import random
import math
from shapely import Point, Polygon
import numpy as np
from scipy.spatial import distance
import config


def generate_random_islands(num_islands, map_bounds, start=None, goal=None, topology=None, seed=None):
    """
    Generate random island polygons with irregular shapes.
    
    Args:
        num_islands: Number of islands to generate
        map_bounds: (width, height) of map
        start: (x, y) start position
        goal: (x, y) goal position
        topology: Topology information
        seed: Random seed for reproducibility
    
    Returns:
        List of islands, each as list of (x, y) coordinates
    """
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)
    
    islands = []
    placed = []          # Polygon form of `islands`, for the separation test
    width, height = map_bounds

    attempts = 0
    max_attempts = 1000  # Prevent infinite loops in case of tight spacing

    # Calculate the geometric parameters between start and goal 
    mx, my = (start[0] + goal[0]) / 2, (start[1] + goal[1]) / 2             # midpoint
    angle_start_goal = math.atan2(goal[1] - start[1], goal[0] - start[0])   # angle from start to goal
    angle_perp = angle_start_goal + math.pi / 2                             # perpendicular angle
    dist_sg = math.hypot(goal[0] - start[0], goal[1] - start[1])
    
    while len(islands) < num_islands and attempts < max_attempts:
        if topology == 'random':
            # Random center for island
            center_x = random.uniform(width * 0.1, width * 0.9)
            center_y = random.uniform(height * 0.1, height * 0.9)
        elif topology == 'center_cluster':
            # Place islands around the midpoint between start and goal
            center_x = random.gauss(mx, dist_sg / 3)  # Gaussian around midpoint
            center_y = random.gauss(my, dist_sg / 3)  # Gaussian around midpoint
        elif topology == 'wall_block':
            # Place islands along a line perpendicular to the start-goal line
            t = random.uniform(-150000, 150000)  # offset along the perpendicular
            noise = random.uniform(-dist_sg / 3, dist_sg / 3)
            center_x = mx + t * math.cos(angle_perp) + noise * math.cos(angle_start_goal)
            center_y = my + t * math.sin(angle_perp) + noise * math.sin(angle_start_goal)
        
        center_x = max(width * 0.1, min(center_x, width * 0.9))
        center_y = max(height * 0.1, min(center_y, height * 0.9))
        
        # Random size
        size = random.uniform(config.ISLAND_SIZE_MIN, config.ISLAND_SIZE_MAX)
        
        # Random number of vertices
        num_vertices = random.randint(config.ISLAND_VERTICES_MIN, config.ISLAND_VERTICES_MAX)
        
        # Generate irregular polygon (star-like pattern with random perturbation)
        island = []
        for i in range(num_vertices):
            angle = 2 * math.pi * i / num_vertices
            
            # Randomize radius for each vertex
            radius = size * random.uniform(0.6, 1.0)
            
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            island.append((x, y))
        
        island_polygon = Polygon(island)

        # Islands must not overlap each other, the same rule the circle
        # generator enforces. Compared as real polygon distance rather than a
        # centre-distance heuristic, so irregular shapes are handled exactly.
        valid = all(island_polygon.distance(p) >= config.ISLAND_MIN_SEPARATION_M
                    for p in placed)

        # Check start, goal located within obstacle-free zone (buffer zone)
        if valid and start and goal:
            start_pt = Point(start)
            goal_pt = Point(goal)
            if (island_polygon.distance(start_pt) < config.SPAWN_CLEARANCE_M
                    or island_polygon.distance(goal_pt) < config.SPAWN_CLEARANCE_M):
                valid = False

        if valid:
            islands.append(island)
            placed.append(island_polygon)
            attempts = 0
        else:
            attempts += 1

    return islands


def generate_dynamic_obstacles(num_sites, map_bounds, start=None, goal=None, topology=None, seed=None):
    """
    Generate dynamic obstacles as circles.
    
    Args:
        num_sites: Number of dynamic obstacles
        map_bounds: (width, height) of map
        start: (x, y) start position
        goal: (x, y) goal position
        topology: Topology information
        seed: Random seed for reproducibility
    
    Returns:
        List of dynamic obstacles, each as (center, radius) tuple
    """
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)
    
    
    min_radius, max_radius = config.OBSTACLE_RADIUS_MIN, config.OBSTACLE_RADIUS_MAX
    dynamic_obstacles = []
    width, height = map_bounds
    
    min_separation = max_radius * 2 + 500  # Minimum distance between dynamic obstacles
    
    attempts = 0
    max_attempts = 100

    # Calculate the geometric parameters between start and goal 
    mx, my = (start[0] + goal[0]) / 2, (start[1] + goal[1]) / 2             # midpoint
    angle_start_goal = math.atan2(goal[1] - start[1], goal[0] - start[0])   # angle from start to goal
    angle_perp = angle_start_goal + math.pi / 2                             # perpendicular angle
    dist_sg = math.hypot(goal[0] - start[0], goal[1] - start[1])
    
    while len(dynamic_obstacles) < num_sites and attempts < max_attempts:
        if topology == 'random':
            # Random center for dynamic obstacle
            center_x = random.uniform(width * 0.1, width * 0.9)
            center_y = random.uniform(height * 0.1, height * 0.9)
        elif topology == 'center_cluster':
            # Place dynamic obstacles around the midpoint between start and goal
            center_x = random.gauss(mx, dist_sg / 3)  # Gaussian around midpoint
            center_y = random.gauss(my, dist_sg / 3)  # Gaussian around midpoint
        elif topology == 'wall_block':
            # Place dynamic obstacles along a line perpendicular to the start-goal line
            t = random.uniform(-150000, 150000)  # offset along the perpendicular
            noise = random.uniform(-dist_sg / 3, dist_sg / 3)
            center_x = mx + t * math.cos(angle_perp) + noise * math.cos(angle_start_goal)
            center_y = my + t * math.sin(angle_perp) + noise * math.sin(angle_start_goal)
        
        center_x = max(width * 0.1, min(center_x, width * 0.9))
        center_y = max(height * 0.1, min(center_y, height * 0.9))
        center = (center_x, center_y)
        
        # Random radius
        radius = random.uniform(min_radius, max_radius)
        
        # Check minimum separation from existing sites
        valid = True
        for existing_center, existing_radius in dynamic_obstacles:
            dist = math.sqrt((center[0] - existing_center[0])**2 + 
                            (center[1] - existing_center[1])**2)
            if dist < min_separation:
                valid = False
                break
        
        # Check start, goal located within obstacle-free zone (buffer zone)
        if valid and start and goal:
            start_pt = Point(start)
            goal_pt = Point(goal)
            obstacle_circle = Point(center).buffer(radius)

            if (obstacle_circle.distance(start_pt) < config.SPAWN_CLEARANCE_M
                    or obstacle_circle.distance(goal_pt) < config.SPAWN_CLEARANCE_M):
                valid = False
        
        if valid:
            dynamic_obstacles.append((center, radius))
            attempts = 0
        else:
            attempts += 1
    
    return dynamic_obstacles


def create_scenario(scenario_config):
    """
    Create a complete scenario with start, goal, obstacles.
    
    Args:
        scenario_config: Dict with keys:
            - 'start': (x, y) start position
            - 'start_heading': heading angle in radians
            - 'goal': (x, y) goal position
            - 'goal_heading': heading angle in radians, or None for a free
              terminal approach direction (planner chooses it)
            - 'num_islands': number of islands
            - 'num_dynamic_obstacles': number of dynamic obstacles
            - 'map_bounds': (width, height)
    
    Returns:
        Dict with complete scenario data
    """
    
    scenario = {
        'start': scenario_config.get('start'),
        'start_heading': scenario_config.get('start_heading', 0),
        'goal': scenario_config.get('goal'),
        # None => free terminal approach direction (the planner chooses it).
        'goal_heading': scenario_config.get('goal_heading'),
        'map_bounds': scenario_config.get('map_bounds', (config.MAP_WIDTH, config.MAP_HEIGHT)),
        # Optional operating areas: a LIST of polygons, each a list of (x, y)
        # vertices. The aircraft must stay inside their union. None/empty =>
        # fall back to the config.MAP_WIDTH/HEIGHT rectangle.
        'safezones': scenario_config.get('safezones'),
    }
    
    # Generate obstacles
    num_islands = scenario_config.get('num_islands', 0)
    num_dynamic_obstacles = scenario_config.get('num_dynamic_obstacles', 0)
    topology = scenario_config.get('topology', 'random')
    seed = scenario_config.get('seed', None)
    
    scenario['islands'] = generate_random_islands(num_islands, scenario['map_bounds'], scenario['start'], scenario['goal'], topology, seed=seed)
    scenario['dynamic_obstacles'] = generate_dynamic_obstacles(num_dynamic_obstacles, scenario['map_bounds'], scenario['start'], scenario['goal'], topology, seed=seed)
    
    # Convert to obstacle format
    scenario['obstacles'] = []
    
    for island in scenario['islands']:
        scenario['obstacles'].append({
            'type': 'polygon',
            'polygon': island
        })
    
    for center, radius in scenario['dynamic_obstacles']:
        scenario['obstacles'].append({
            'type': 'circle',
            'center': center,
            'radius': radius
        })
    
    return scenario


# ============ PREDEFINED SCENARIOS ============

def scenario1_open_ocean():
    """Scenario 1: Open ocean - no obstacles"""
    return create_scenario({
        'start': (2000, 2000),
        'start_heading': math.pi / 4,  # 45 degrees
        'goal': (450000, 450000),
        'goal_heading': math.pi / 4,
        'num_islands': 0,
        'num_dynamic_obstacles': 0,
        'seed': 42
    })


def scenario2_single_obstacle():
    """Scenario 2: Single large obstacle in the way"""
    return create_scenario({
        'start': (2000, 2000),
        'start_heading': math.pi / 4,
        'goal': (450000, 450000),
        'goal_heading': math.pi / 4,
        'num_islands': 1,
        'num_dynamic_obstacles': 1,
        'seed': 42
    })


def scenario3_narrow_gap():
    """Scenario 3: Two obstacles very close together (narrow gap)"""
    scenario = create_scenario({
        'start': (2000, 2000),
        'start_heading': math.pi / 4,
        'goal': (450000, 450000),
        'goal_heading': math.pi / 4,
        'num_islands': 0,
        'num_dynamic_obstacles': 0,
        'seed': 99
    })
    
    # Manually add two close islands
    island1 = [
        (22000, 20000),
        (24000, 20000),
        (24000, 22000),
        (22000, 22000),
    ]
    island2 = [
        (26000, 20000),
        (28000, 20000),
        (28000, 22000),
        (26000, 22000),
    ]
    
    scenario['islands'] = [island1, island2]
    scenario['obstacles'] = [
        {'type': 'polygon', 'polygon': island1},
        {'type': 'polygon', 'polygon': island2},
    ]
    
    return scenario


def scenario4_complex_maze():
    """Scenario 4: Complex maze with many obstacles"""
    return create_scenario({
        'start': (1000, 1000),
        'start_heading': 0,
        'goal': (480000, 480000),
        'goal_heading': 0,
        'num_islands': 12,  # Reduced from 20 for better traversability
        'num_dynamic_obstacles': 6,       # Reduced from 10
        'seed': 12345
    })


# ============ EASY SCENARIOS (Few obstacles, simple paths) ============

def scenario5_sparse_islands():
    """Scenario 5: Easy - Sparse islands, plenty of open water"""
    return create_scenario({
        'start': (5000, 5000),
        'start_heading': math.pi / 4,
        'goal': (450000, 450000),
        'goal_heading': math.pi / 4,
        'num_islands': 3,
        'num_dynamic_obstacles': 1,
        'seed': 111
    })


def scenario6_coastal_path():
    """Scenario 6: Easy - Light coastal dynamic obstacles, open corridor"""
    return create_scenario({
        'start': (10000, 10000),
        'start_heading': 0,
        'goal': (480000, 480000),
        'goal_heading': 0,
        'num_islands': 2,
        'num_dynamic_obstacles': 2,
        'seed': 222
    })


def scenario7_diagonal_crossing():
    """Scenario 7: Easy - Minimal obstacles, diagonal crossing"""
    return create_scenario({
        'start': (20000, 20000),
        'start_heading': math.pi / 4,
        'goal': (470000, 470000),
        'goal_heading': math.pi / 4,
        'num_islands': 4,
        'num_dynamic_obstacles': 0,
        'seed': 333
    })


def scenario8_open_with_dynamic_obstacles():
    """Scenario 8: Easy - Open terrain with scattered dynamic obstacles"""
    return create_scenario({
        'start': (10000, 250000),
        'start_heading': 0,
        'goal': (480000, 250000),
        'goal_heading': 0,
        'num_islands': 1,
        'num_dynamic_obstacles': 3,
        'seed': 444
    })


# ============ MEDIUM SCENARIOS (Moderate complexity) ============

def scenario9_island_archipelago():
    """Scenario 9: Medium - Archipelago with multiple islands"""
    return create_scenario({
        'start': (5000, 250000),
        'start_heading': 0,
        'goal': (490000, 250000),
        'goal_heading': 0,
        'num_islands': 8,
        'num_dynamic_obstacles': 2,
        'seed': 555
    })


def scenario10_dense_dynamic_obstacles():
    """Scenario 10: Medium - Dense dynamic obstacle field with some islands"""
    return create_scenario({
        'start': (50000, 50000),
        'start_heading': math.pi / 4,
        'goal': (450000, 450000),
        'goal_heading': math.pi / 4,
        'num_islands': 3,
        'num_dynamic_obstacles': 8,
        'seed': 666
    })


def scenario11_serpentine_route():
    """Scenario 11: Medium - Serpentine path through obstacle field"""
    return create_scenario({
        'start': (50000, 100000),
        'start_heading': 0,
        'goal': (450000, 400000),
        'goal_heading': 0,
        'num_islands': 7,
        'num_dynamic_obstacles': 4,
        'seed': 777
    })


def scenario12_perimeter_dynamic_obstacles():
    """Scenario 12: Medium - Goal protected by perimeter dynamic obstacles"""
    return create_scenario({
        'start': (10000, 250000),
        'start_heading': 0,
        'goal': (480000, 250000),
        'goal_heading': 0,
        'num_islands': 6,
        'num_dynamic_obstacles': 5,
        'seed': 888
    })


# ============ HARD SCENARIOS (High complexity, many obstacles) ============

def scenario13_dense_island_field():
    """Scenario 13: Hard - Very dense island field"""
    return create_scenario({
        'start': (25000, 25000),
        'start_heading': math.pi / 3,
        'goal': (475000, 475000),
        'goal_heading': math.pi / 3,
        'num_islands': 18,
        'num_dynamic_obstacles': 3,
        'seed': 999
    })


def scenario14_combined_obstacles():
    """Scenario 14: Hard - Combined island and dynamic obstacle """
    return create_scenario({
        'start': (30000, 30000),
        'start_heading': 0,
        'goal': (470000, 470000),
        'goal_heading': 0,
        'num_islands': 12,
        'num_dynamic_obstacles': 10,
        'seed': 1111
    })


def scenario15_narrow_channel():
    """Scenario 15: Hard - Forced through narrow channels between obstacles"""
    return create_scenario({
        'start': (50000, 250000),
        'start_heading': 0,
        'goal': (450000, 250000),
        'goal_heading': 0,
        'num_islands': 15,
        'num_dynamic_obstacles': 4,
        'seed': 2222
    })


def scenario16_extreme_complexity():
    """Scenario 16: Very Hard - Extreme complexity test"""
    return create_scenario({
        'start': (10000, 10000),
        'start_heading': math.pi / 6,
        'goal': (490000, 490000),
        'goal_heading': math.pi / 6,
        'num_islands': 20,
        'num_dynamic_obstacles': 12,
        'seed': 3333
    })


def get_all_scenarios():
    """Return all 16 predefined scenarios organized by difficulty"""
    return {
        # Original scenarios
        'scenario_01_open_ocean': scenario1_open_ocean,
        'scenario_02_single_obstacle': scenario2_single_obstacle,
        'scenario_03_narrow_gap': scenario3_narrow_gap,
        'scenario_04_complex_maze': scenario4_complex_maze,
        
        # Easy scenarios
        'scenario_05_sparse_islands': scenario5_sparse_islands,
        'scenario_06_coastal_path': scenario6_coastal_path,
        'scenario_07_diagonal_crossing': scenario7_diagonal_crossing,
        'scenario_08_open_with_dynamic_obstacles': scenario8_open_with_dynamic_obstacles,
        
        # Medium scenarios
        'scenario_09_island_archipelago': scenario9_island_archipelago,
        'scenario_10_dense_dynamic_obstacles': scenario10_dense_dynamic_obstacles,
        'scenario_11_serpentine_route': scenario11_serpentine_route,
        'scenario_12_perimeter_dynamic_obstacles': scenario12_perimeter_dynamic_obstacles,
        
        # Hard scenarios
        'scenario_13_dense_island_field': scenario13_dense_island_field,
        'scenario_14_combined_obstacles': scenario14_combined_obstacles,
        'scenario_15_narrow_channel': scenario15_narrow_channel,
        'scenario_16_extreme_complexity': scenario16_extreme_complexity,
    }
