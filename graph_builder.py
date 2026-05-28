"""
Tangent Graph Generation Module
Computes bitangent lines between obstacles for navigation graph
"""

import math
from itertools import combinations
import spatial_utils as su
import config


class TangentGraph:
    """Graph of tangent lines connecting obstacles"""
    
    def __init__(self):
        self.nodes = []  # (x, y) positions
        self.edges = []  # ((x1, y1), (x2, y2)) tangent segments
        self.node_map = {}  # Map from position to node index
    
    def add_node(self, position):
        """Add a node to the graph"""
        key = (round(position[0], 1), round(position[1], 1))
        if key not in self.node_map:
            self.node_map[key] = len(self.nodes)
            self.nodes.append(position)
        return self.node_map[key]
    
    def add_edge(self, p1, p2):
        """Add an edge (tangent line) to the graph"""
        edge = (p1, p2)
        self.edges.append(edge)
    
    def get_edges_near_point(self, point, threshold=100.0):
        """Get all edges within threshold distance of a point"""
        nearby = []
        for edge in self.edges:
            dist = su.point_to_line_distance(point, edge[0], edge[1])
            if dist < threshold:
                nearby.append(edge)
        return nearby


def generate_bitangents(circle_obstacles, polygon_obstacles, filter_los=True):
    """
    Generate bitangent lines between obstacles.
    
    Args:
        circle_obstacles: List of (center, radius) tuples
        polygon_obstacles: List of polygon coordinates
        filter_los: If True, filter out tangents that cross obstacles
    
    Returns:
        TangentGraph object with all valid bitangents
    """
    
    graph = TangentGraph()
    
    # Add circle obstacles as nodes (centers)
    circle_indices = {}
    for i, (center, radius) in enumerate(circle_obstacles):
        idx = graph.add_node(center)
        circle_indices[i] = idx
    
    # Convert polygon obstacles to circles (using centroid and max distance)
    polygon_indices = {}
    for i, polygon in enumerate(polygon_obstacles):
        # Compute centroid
        centroid_x = sum(p[0] for p in polygon) / len(polygon)
        centroid_y = sum(p[1] for p in polygon) / len(polygon)
        centroid = (centroid_x, centroid_y)
        
        # Compute radius as max distance from centroid
        radius = max(math.sqrt((p[0] - centroid_x)**2 + (p[1] - centroid_y)**2) 
                    for p in polygon)
        
        idx = graph.add_node(centroid)
        polygon_indices[i] = idx
    
    all_obstacles = [(c, r) for c, r in circle_obstacles] + \
                    [((sum(p[0] for p in poly) / len(poly), 
                      sum(p[1] for p in poly) / len(poly)),
                      max(math.sqrt((p[0] - sum(pp[0] for pp in poly) / len(poly))**2 +
                                   (p[1] - sum(pp[1] for pp in poly) / len(poly))**2)
                         for p in poly))
                     for poly in polygon_obstacles]
    
    # Generate bitangents between all pairs of obstacles
    for i in range(len(all_obstacles)):
        for j in range(i + 1, len(all_obstacles)):
            circle1 = all_obstacles[i]
            circle2 = all_obstacles[j]
            
            tangents = su.compute_tangent_lines(circle1, circle2)
            
            for tangent in tangents:
                p1, p2 = tangent
                
                # Check line-of-sight if required
                if filter_los:
                    # Check if tangent crosses any other obstacle
                    crosses = False
                    for k, circle in enumerate(all_obstacles):
                        if k != i and k != j:
                            # Check if tangent is blocked by this obstacle
                            dist = su.point_to_line_distance(circle[0], p1, p2)
                            if dist < circle[1]:
                                crosses = True
                                break
                    
                    if crosses:
                        continue
                
                graph.add_edge(p1, p2)
    
    return graph


def extend_tangent_graph_with_start_goal(graph, start_pos, start_heading, goal_pos, goal_heading,
                                        circle_obstacles, polygon_obstacles):
    """
    Extend tangent graph with edges from start and goal to the graph nodes.
    
    Args:
        graph: TangentGraph object
        start_pos: Starting waypoint (x, y)
        start_heading: Starting heading (radians)
        goal_pos: Goal waypoint (x, y)
        goal_heading: Goal heading (radians)
        circle_obstacles: List of (center, radius)
        polygon_obstacles: List of polygons
    
    Returns:
        Extended TangentGraph
    """
    
    # Add start and goal as nodes
    start_idx = graph.add_node(start_pos)
    goal_idx = graph.add_node(goal_pos)
    
    all_obstacles = [(c, r) for c, r in circle_obstacles] + \
                    [((sum(p[0] for p in poly) / len(poly), 
                      sum(p[1] for p in poly) / len(poly)),
                      max(math.sqrt((p[0] - sum(pp[0] for pp in poly) / len(poly))**2 +
                                   (p[1] - sum(pp[1] for pp in poly) / len(poly))**2)
                         for p in poly))
                     for poly in polygon_obstacles]
    
    # Connect start to all graph nodes with better LOS checking
    num_connections = 0
    for node_pos in graph.nodes:
        if node_pos == start_pos:
            continue
        
        # Check line-of-sight from start to node
        los_clear = True
        for circle in all_obstacles:
            dist = su.point_to_line_distance(circle[0], start_pos, node_pos)
            # More lenient LOS check (0.9 factor allows some margin)
            if dist < circle[1] * 0.8:
                los_clear = False
                break
        
        if los_clear:
            graph.add_edge(start_pos, node_pos)
            num_connections += 1
    
    # Connect goal to all graph nodes
    num_goal_connections = 0
    for node_pos in graph.nodes:
        if node_pos == goal_pos:
            continue
        
        # Check line-of-sight from node to goal
        los_clear = True
        for circle in all_obstacles:
            dist = su.point_to_line_distance(circle[0], node_pos, goal_pos)
            if dist < circle[1] * 0.8:
                los_clear = False
                break
        
        if los_clear:
            graph.add_edge(node_pos, goal_pos)
            num_goal_connections += 1
    
    # Direct connection start to goal (if possible)
    direct_los = True
    for circle in all_obstacles:
        dist = su.point_to_line_distance(circle[0], start_pos, goal_pos)
        if dist < circle[1] * 0.8:
            direct_los = False
            break
    
    if direct_los:
        graph.add_edge(start_pos, goal_pos)
    
    return graph


def filter_blocked_lines(lines, obstacles):
    """
    Filter out tangent lines that cross through obstacle regions.
    
    Args:
        lines: List of ((x1, y1), (x2, y2)) line segments
        obstacles: List of inflated obstacles
    
    Returns:
        List of unblocked lines
    """
    
    unblocked = []
    
    for line in lines:
        p1, p2 = line
        
        blocked = False
        for obstacle in obstacles:
            if obstacle['type'] == 'circle':
                center = obstacle['center']
                radius = obstacle['radius']
                
                # Check if line intersects circle
                dist = su.point_to_line_distance(center, p1, p2)
                if dist < radius:
                    blocked = True
                    break
            
            elif obstacle['type'] == 'polygon':
                polygon = obstacle['polygon']
                if su.line_polygon_intersection(p1, p2, polygon):
                    blocked = True
                    break
        
        if not blocked:
            unblocked.append(line)
    
    return unblocked


def compute_connectivity_radius(obstacles, min_radius=500.0):
    """
    Compute a suitable connectivity radius for connecting start/goal to graph.
    Based on obstacle density and spacing.
    
    Args:
        obstacles: List of obstacles
        min_radius: Minimum connectivity radius
    
    Returns:
        Connectivity radius (meters)
    """
    
    if not obstacles:
        return 5000.0
    
    # Simple heuristic: use a radius that scales with number of obstacles
    radius = min_radius * (1 + len(obstacles) * 0.1)
    
    return min(radius, 10000.0)  # Cap at 10km
