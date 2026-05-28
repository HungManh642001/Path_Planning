"""
Mock Map Generator Module
Generates synthetic mission scenarios with islands and SAM sites
"""

import random
import math
import numpy as np
from scipy.spatial import distance
import config


def generate_random_islands(num_islands, map_bounds, seed=None):
    """
    Generate random island polygons with irregular shapes.
    
    Args:
        num_islands: Number of islands to generate
        map_bounds: (width, height) of map
        seed: Random seed for reproducibility
    
    Returns:
        List of islands, each as list of (x, y) coordinates
    """
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)
    
    islands = []
    width, height = map_bounds
    
    for _ in range(num_islands):
        # Random center for island
        center_x = random.uniform(width * 0.1, width * 0.9)
        center_y = random.uniform(height * 0.1, height * 0.9)
        
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
        
        islands.append(island)
    
    return islands


def generate_sam_sites(num_sites, map_bounds, radius_range=None, seed=None):
    """
    Generate SAM (Surface-to-Air Missile) defense sites as circles.
    
    Args:
        num_sites: Number of SAM sites
        map_bounds: (width, height) of map
        radius_range: (min_radius, max_radius) for SAM coverage
        seed: Random seed for reproducibility
    
    Returns:
        List of SAM sites, each as (center, radius) tuple
    """
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)
    
    if radius_range is None:
        radius_range = (config.SAM_RADIUS_MIN, config.SAM_RADIUS_MAX)
    
    min_radius, max_radius = radius_range
    sam_sites = []
    width, height = map_bounds
    
    min_separation = max_radius * 2 + 500  # Minimum distance between SAM sites
    
    attempts = 0
    max_attempts = 100
    
    while len(sam_sites) < num_sites and attempts < max_attempts:
        # Random center
        center_x = random.uniform(width * 0.1, width * 0.9)
        center_y = random.uniform(height * 0.1, height * 0.9)
        center = (center_x, center_y)
        
        # Random radius
        radius = random.uniform(min_radius, max_radius)
        
        # Check minimum separation from existing sites
        valid = True
        for existing_center, existing_radius in sam_sites:
            dist = math.sqrt((center[0] - existing_center[0])**2 + 
                            (center[1] - existing_center[1])**2)
            if dist < min_separation:
                valid = False
                break
        
        if valid:
            sam_sites.append((center, radius))
            attempts = 0
        else:
            attempts += 1
    
    return sam_sites


def create_scenario(scenario_config):
    """
    Create a complete scenario with start, goal, obstacles.
    
    Args:
        scenario_config: Dict with keys:
            - 'start': (x, y) start position
            - 'start_heading': heading angle in radians
            - 'goal': (x, y) goal position
            - 'goal_heading': heading angle in radians
            - 'num_islands': number of islands
            - 'num_sam': number of SAM sites
            - 'map_bounds': (width, height)
    
    Returns:
        Dict with complete scenario data
    """
    
    scenario = {
        'start': scenario_config.get('start'),
        'start_heading': scenario_config.get('start_heading', 0),
        'goal': scenario_config.get('goal'),
        'goal_heading': scenario_config.get('goal_heading', 0),
        'map_bounds': scenario_config.get('map_bounds', (config.MAP_WIDTH, config.MAP_HEIGHT)),
    }
    
    # Generate obstacles
    num_islands = scenario_config.get('num_islands', 0)
    num_sam = scenario_config.get('num_sam', 0)
    seed = scenario_config.get('seed', None)
    
    scenario['islands'] = generate_random_islands(num_islands, scenario['map_bounds'], seed=seed)
    scenario['sam_sites'] = generate_sam_sites(num_sam, scenario['map_bounds'], seed=seed)
    
    # Convert to obstacle format
    scenario['obstacles'] = []
    
    for island in scenario['islands']:
        scenario['obstacles'].append({
            'type': 'polygon',
            'polygon': island
        })
    
    for center, radius in scenario['sam_sites']:
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
        'num_sam': 0,
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
        'num_sam': 1,
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
        'num_sam': 0,
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
        'num_sam': 6,       # Reduced from 10
        'seed': 12345
    })


def get_all_scenarios():
    """Return all predefined scenarios"""
    return {
        'open_ocean': scenario1_open_ocean,
        'single_obstacle': scenario2_single_obstacle,
        'narrow_gap': scenario3_narrow_gap,
        'complex_maze': scenario4_complex_maze,
    }
