"""
Tangent Graph Generation Module
Computes bitangent lines between obstacles for navigation graph
"""

import math
from collections import defaultdict
import spatial_utils as su
from shapely import Polygon, LineString
import config


class TangentGraph:
    """Graph of tangent lines connecting obstacles"""
    
    def __init__(self):
        self.nodes = []  # (x, y) positions
        self.edges = []  # ((x1, y1), (x2, y2)) tangent segments
        self.node_map = {}  # Map from position to node index
        self.adjacency = defaultdict(list)  # node index -> [(neighbor_idx, cost)]
        self.edge_map = set()  # Deduplicate undirected edges by node indices
    
    def _key(self, position):
        return (round(position[0], 1), round(position[1], 1))
       
    def add_node(self, position):
        """Add a node to the graph"""
        key = self._key(position)
        if key not in self.node_map:
            self.node_map[key] = len(self.nodes)
            self.nodes.append(position)
        return self.node_map[key]
    
    def find_node_index(self, position):
        """Return the index for a position already in the graph, or None."""
        return self.node_map.get(self._key(position))
    
    def add_edge(self, p1, p2, cost=None):
        """Add an edge (tangent line) to the graph"""
        if su.distance(p1, p2) < config.EPS:
            return None
        
        idx1 = self.add_node(p1)
        idx2 = self.add_node(p2)
        edge_key = tuple(sorted((idx1, idx2)))
        if edge_key in self.edge_map:
            return edge_key
        
        self.edge_map.add(edge_key)
        if cost is None:
            cost = su.distance(p1, p2)
        edge = (p1, p2)
        self.edges.append(edge)
        self.adjacency[idx1].append((idx2, cost))
        self.adjacency[idx2].append((idx1, cost))
        return edge_key
    
    def get_neighbors(self, position):
        """Return neighbor positions and edge costs for a graph node position."""
        idx = self.find_node_index(position)
        if idx is None:
            return []
        return [(self.nodes[neighbor_idx], cost)
                for neighbor_idx, cost in self.adjacency.get(idx, [])]
    
    def get_edges_near_point(self, point, threshold=100.0):
        """Get all edges within threshold distance of a point"""
        nearby = []
        for edge in self.edges:
            dist = su.point_to_line_distance(point, edge[0], edge[1])
            if dist < threshold:
                nearby.append(edge)
        return nearby

def _circle_obstacle(center, radius, obstacle_id):
    return {
        'id': obstacle_id,
        'type': 'circle',
        'center': center,
        'radius': radius,
    }

def _navigation_vertices_for_polygon(polygon):
    """Return a compact set of convex boundary vertices for graph search."""
    shape = Polygon(polygon)
    hull = shape.convex_hull
    simplified = hull.simplify(config.SAFE_MARGIN, preserve_topology=True)
    if simplified.geom_type != 'Polygon' or len(simplified.exterior.coords) < 4:
        simplified = hull
    return list(simplified.exterior.coords[:-1])

def _polygon_obstacle(polygon, obstacle_id):
    shape = Polygon(polygon)
    return {
        'id': obstacle_id,
        'type': 'polygon',
        'polygon': polygon,
        'visibility_vertices': _navigation_vertices_for_polygon(polygon),
        'shape': shape
    }

def _build_obstacle_models(circle_obstacles, polygon_obstacles):
    obstacles = []
    for i, (center, radius) in enumerate(circle_obstacles):
        obstacles.append(_circle_obstacle(center, radius, f'circle:{i}'))
    for i, polygon in enumerate(polygon_obstacles):
        obstacles.append(_polygon_obstacle(polygon, f'polygon:{i}'))
    return obstacles

def _circle_circle_tangents(circle1, circle2):
    """Return external and internal common tangents between two circles."""
    c1, r1 = circle1
    c2, r2 = circle2
    x1, y1 = c1
    x2, y2 = c2
    dx = x2 - x1
    dy = y2 - y1
    d = math.hypot(dx, dy)
    if d < config.EPS:
        return []
    
    vx = dx / d
    vy = dy / d
    tangents = []

    # signed_r2 = +r2 gives external tangents; -r2 gives internal tangents.
    for signed_r2 in (r2, -r2):
        c = (r1 - signed_r2) / d
        if c * c > 1.0 + config.EPS:
            continue
        h = math.sqrt(max(0.0, 1.0 - c * c))
        for side in (1.0, -1.0):
            nx = vx * c - side * h * vy
            ny = vy * c + side * h * vx
            p1 = (x1 + r1 * nx, y1 + r1 * ny)
            p2 = (x2 + signed_r2 * nx, y2 + signed_r2 * ny)
            tangents.append((p1, p2))
    
    return tangents

def _point_circle_tangents(point, circle):
    """Return tangent points on a circle from an external point."""
    center, radius = circle
    d = su.distance(point, center)
    if d <= radius + config.EPS:
        return []
    
    theta = math.atan2(point[1] - center[1], point[0] - center[0])
    alpha = math.acos(radius / d)
    tangent_points = []
    for angle in (theta + alpha, theta - alpha):
        tangent_points.append((
            center[0] + radius * math.cos(angle),
            center[1] + radius * math.sin(angle),
        ))
    return tangent_points

def _line_blocks_circle(line, center, radius, allow_touch):
    dist = su.point_to_line_distance(center, line.coords[0], line.coords[-1])
    if allow_touch:
        return dist < radius - 1e-5
    return dist < radius - 1e-5


def _ring_nodes(center, radius, n):
    pts = []
    for k in range(n):
        ang = 2.0 * math.pi * k / n
        pts.append((center[0] + radius * math.cos(ang),
                    center[1] + radius * math.sin(ang)))
    return pts


def _add_visibility_nodes(graph, circle_obstacles, polygon_obstacles):
    """Add boundary support nodes per obstacle and connect mutually-visible
    pairs whose connecting segment clears all obstacles."""
    nodes = []
    for center, radius in circle_obstacles:
        nodes.extend(_ring_nodes(center, radius, config.OBSTACLE_RING_SAMPLES))
    for poly in polygon_obstacles:
        nodes.extend(Polygon(poly).convex_hull.exterior.coords[:-1])

    poly_shapes = [Polygon(poly) for poly in polygon_obstacles]

    def clear(a, b):
        for center, radius in circle_obstacles:
            if su.point_to_line_distance(center, a, b) < radius - 1.0:
                return False
        line = LineString([a, b])
        for poly_shape in poly_shapes:
            if line.crosses(poly_shape):
                return False
        return True

    for n in nodes:
        graph.add_node(n)
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if clear(nodes[i], nodes[j]):
                graph.add_edge(nodes[i], nodes[j])

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
    
    # # Add circle obstacles as nodes (centers)
    # circle_indices = {}
    # for i, (center, radius) in enumerate(circle_obstacles):
    #     idx = graph.add_node(center)
    #     circle_indices[i] = idx
    
    # # Convert polygon obstacles to circles (using centroid and max distance)
    # polygon_indices = {}
    # for i, polygon in enumerate(polygon_obstacles):
    #     # Compute centroid
    #     centroid_x = sum(p[0] for p in polygon) / len(polygon)
    #     centroid_y = sum(p[1] for p in polygon) / len(polygon)
    #     centroid = (centroid_x, centroid_y)
        
    #     # Compute radius as max distance from centroid
    #     radius = max(math.sqrt((p[0] - centroid_x)**2 + (p[1] - centroid_y)**2) 
    #                 for p in polygon)
        
    #     idx = graph.add_node(centroid)
    #     polygon_indices[i] = idx
    
    all_obstacles = [(c, r) for c, r in circle_obstacles]
    # Polygons are represented solely by their hull vertices added in
    # _add_visibility_nodes; the old bounding-circle approximation is dropped.

    # Generate bitangents between all pairs of obstacles
    for i in range(len(all_obstacles)):
        for j in range(i + 1, len(all_obstacles)):
            circle1 = all_obstacles[i]
            circle2 = all_obstacles[j]
            
            tangents = _circle_circle_tangents(circle1, circle2)
            
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

    _add_visibility_nodes(graph, circle_obstacles, polygon_obstacles)
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

    poly_shapes = [Polygon(poly) for poly in polygon_obstacles]

    def clear(a, b):
        for center, radius in circle_obstacles:
            if su.point_to_line_distance(center, a, b) < radius:
                return False
        line = LineString([a, b])
        for poly_shape in poly_shapes:
            if line.intersects(poly_shape):
                return False
        return True

    # Connect start to all graph nodes with exact LOS checking
    num_connections = 0
    for node_pos in graph.nodes:
        if node_pos == start_pos:
            continue

        if clear(start_pos, node_pos):
            graph.add_edge(start_pos, node_pos)
            num_connections += 1

    # Connect goal to all graph nodes
    num_goal_connections = 0
    for node_pos in graph.nodes:
        if node_pos == goal_pos:
            continue

        if clear(node_pos, goal_pos):
            graph.add_edge(node_pos, goal_pos)
            num_goal_connections += 1

    # Direct connection start to goal (if possible)
    if clear(start_pos, goal_pos):
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
