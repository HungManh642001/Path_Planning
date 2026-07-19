"""High-level entry point for the focal planner variant.

Mirrors core.kinodynamic_astar.plan_trajectory but drives
FocalKinodynamicAstar. Does not modify the base module.
"""

import math

from ml_planner.focal_astar import FocalKinodynamicAstar
from ml_planner.guidance import make_guidance_secondary


def path_length(path):
    """Total polyline length (meters) of a [(waypoint, heading), ...] path."""
    total = 0.0
    for (a, _), (b, _) in zip(path, path[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def plan_trajectory_focal(preprocessed_scenario, focal_eps=None, secondary=None, verbose=False):
    """Plan a trajectory with focal (A*epsilon) search.

    Returns a dict with 'path', 'success', 'stats', 'planner'. Success means
    the fixed takeoff/approach legs are clear AND a body path was found, the
    same contract as the base plan_trajectory.
    """
    # 'guidance' selects the learned CNN secondary when a model is available,
    # else falls back to the hand-crafted secondary (Phase-1 behavior). Any
    # other value (None or a callable) is passed through unchanged.
    resolved_secondary = secondary
    if secondary == 'guidance':
        cb, _available = make_guidance_secondary(preprocessed_scenario)
        resolved_secondary = cb            # None when unavailable -> hand-crafted

    planner = FocalKinodynamicAstar(preprocessed_scenario, focal_eps=focal_eps, secondary=resolved_secondary)

    legs_ok = planner._check_fixed_legs()
    path = None
    if legs_ok:
        if verbose:
            print("Starting focal A* search...")
        path = planner.search()
        if verbose:
            stats = planner.get_search_stats()
            print(f"Focal search: {stats['iterations']}/{stats['max_iterations']} iterations")
            print("Path found" if path else "No path found")

    if path:
        path = planner.smooth_path(path)

    stats = planner.get_search_stats()
    stats['collision_checks'] = planner.collision_checks
    return {
        'path': path,
        'success': path is not None and legs_ok,
        'stats': stats,
        'planner': planner,
    }
