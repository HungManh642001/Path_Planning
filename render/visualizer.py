"""
Visualization Module
Plots mission scenarios, obstacles, and planned trajectories
"""

import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon as MplPolygon, Circle as MplCircle, Rectangle
import numpy as np

import config
import core.spatial_utils as su
import render.trajectory as tr

from shapely.geometry import Polygon as _ShPoly, Point as _ShPoint


def _plot_extents(scenario, pad=2000.0):
    """Axis limits for a scenario (legacy 'map' view).

    Uses the bounding box of all safezone polygons (plus padding) when present,
    else falls back to the global config.MAP_WIDTH/HEIGHT rectangle (legacy
    behaviour). Returns ((xmin, xmax), (ymin, ymax)).
    """
    safezones = scenario.get('safezones') if scenario else None
    if safezones:
        xs = [p[0] for poly in safezones for p in poly]
        ys = [p[1] for poly in safezones for p in poly]
        return (min(xs) - pad, max(xs) + pad), (min(ys) - pad, max(ys) + pad)
    return (-pad, config.MAP_WIDTH + pad), (-pad, config.MAP_HEIGHT + pad)


def _content_extents(scenario, preprocessed=None, result=None, pad_frac=0.08, min_pad=1000.0):
    """Axis limits framed to the mission ('content'/auto-fit view).

    Bounding box of the flown path + start/goal + obstacles, expanded to include
    the SMALLEST safezone polygon that contains both start and goal (the
    operating corridor). Larger enclosing safezones are excluded from the frame
    (they are still drawn, just clipped). Falls back to the config.MAP_WIDTH/
    HEIGHT rectangle when no content is available. Returns ((xmin,xmax),(ymin,ymax)).
    """
    xs, ys = [], []

    def add(p):
        xs.append(p[0])
        ys.append(p[1])

    def add_box(cx, cy, r):
        add((cx - r, cy - r))
        add((cx + r, cy + r))

    # Mission endpoints and interior waypoints.
    if preprocessed:
        for k in ('start_pos', 'goal_pos'):
            if preprocessed.get(k) is not None:
                add(preprocessed[k])
        for k in ('start_state', 'goal_state'):
            st = preprocessed.get(k) or {}
            if st.get('waypoint') is not None:
                add(st['waypoint'])
        # Inflated obstacles so drawn buffer zones are not clipped.
        for obs in preprocessed.get('obstacles', []):
            if obs.get('type') == 'circle':
                (cx, cy) = obs['center']
                add_box(cx, cy, obs['radius'])
            elif obs.get('type') == 'polygon':
                for p in obs['polygon']:
                    add(p)

    # The flown trajectory.
    if result and result.get('path'):
        for wp, _h in result['path']:
            add(wp)

    # Raw obstacles.
    if scenario:
        for isl in scenario.get('islands', []):
            for p in isl:
                add(p)
        for (cx, cy), r in scenario.get('dynamic_obstacles', []):
            add_box(cx, cy, r)

    # Operating corridor: smallest safezone covering BOTH start and goal.
    safezones = (scenario or {}).get('safezones')
    sp = (preprocessed or {}).get('start_pos') if preprocessed else (scenario or {}).get('start')
    gp = (preprocessed or {}).get('goal_pos') if preprocessed else (scenario or {}).get('goal')
    if safezones and sp is not None and gp is not None:
        best = None
        for sz in safezones:
            try:
                poly = _ShPoly(sz)
                if not poly.is_valid:
                    poly = poly.buffer(0)
                if poly.covers(_ShPoint(sp)) and poly.covers(_ShPoint(gp)):
                    if best is None or poly.area < best[0]:
                        best = (poly.area, sz)
            except Exception:
                continue
        if best is not None:
            for p in best[1]:
                add(p)

    if not xs:
        return (-min_pad, config.MAP_WIDTH + min_pad), (-min_pad, config.MAP_HEIGHT + min_pad)
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    span = max(maxx - minx, maxy - miny, 1.0)
    pad = max(min_pad, pad_frac * span)
    return (minx - pad, maxx + pad), (miny - pad, maxy + pad)


def plot_scenario(scenario, preprocessed, result=None, title="Mission Scenario",
                 save_path=None, figsize=(14, 12), trajectory_mode='dubins', fit='map'):
    """
    Create comprehensive visualization of mission scenario and trajectory.

    Args:
        scenario: Original scenario from map_generator
        preprocessed: Preprocessed scenario from preprocessing.prepare_scenario()
        result: Result dict from kinodynamic_astar.plan_trajectory()
        title: Figure title
        save_path: Path to save figure (optional)
        figsize: Figure size
        fit: View framing. 'map' (default) keeps the legacy full-map / safezone-bbox
             view; 'content' auto-fits the axes to the flown path + start/goal +
             obstacles (+ the operating-corridor safezone) so a small mission is
             easy to follow inside a large map.

    Returns:
        Matplotlib figure object
    """

    fig, ax = plt.subplots(figsize=figsize, dpi=config.FIGURE_DPI)

    # Set up map background
    if fit == 'content':
        (xlim, ylim) = _content_extents(scenario, preprocessed, result)
    else:
        (xlim, ylim) = _plot_extents(scenario)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # Draw operating areas: each safezone polygon when supplied, else the full
    # config.MAP_WIDTH/HEIGHT rectangle (legacy behaviour).
    safezones = scenario.get('safezones')
    if safezones:
        for sz in safezones:
            ax.add_patch(MplPolygon(sz, closed=True, fill=True,
                                    facecolor='lightblue', edgecolor='blue',
                                    linewidth=2, alpha=0.3))
    else:
        ax.add_patch(Rectangle((0, 0), config.MAP_WIDTH, config.MAP_HEIGHT,
                               fill=True, facecolor='lightblue', edgecolor='blue',
                               linewidth=2, alpha=0.3))
    
    # ====== DRAW OBSTACLES ======
    
    # Draw islands (polygons)
    for island in scenario.get('islands', []):
        island_patch = MplPolygon(island, fill=True, facecolor='saddlebrown', 
                                 edgecolor='darkred', linewidth=1.5, alpha=0.7)
        ax.add_patch(island_patch)
    
    # Draw SAM sites (circles)
    for center, radius in scenario.get('sam_sites', []):
        sam_patch = MplCircle(center, radius, fill=True, facecolor='red', 
                             edgecolor='darkred', linewidth=1.5, alpha=0.5)
        ax.add_patch(sam_patch)
    
    # Draw inflated obstacles (buffer zones) as dashed lines
    if config.PLOT_BUFFER_ZONES:
        for obstacle in preprocessed['obstacles']:
            if obstacle['type'] == 'circle':
                center = obstacle['center']
                radius = obstacle['radius']
                buffer_patch = MplCircle(center, radius, fill=False, 
                                        edgecolor='darkred', linewidth=1, 
                                        linestyle='--', alpha=0.5)
                ax.add_patch(buffer_patch)
            
            elif obstacle['type'] == 'polygon':
                polygon = obstacle['polygon']
                buffer_patch = MplPolygon(polygon, fill=False, 
                                         edgecolor='darkred', linewidth=1, 
                                         linestyle='--', alpha=0.5)
                ax.add_patch(buffer_patch)

    # ====== DRAW START & GOAL ======
    if config.PLOT_START_END_MARKERS:
        # Original takeoff point O
        O = preprocessed['start_pos']
        ax.plot(O[0], O[1], 'go', markersize=12, label='Takeoff Point O', zorder=5)
        
        # W1 (first waypoint after stabilization)
        W1 = preprocessed['start_state']['waypoint']
        ax.plot(W1[0], W1[1], 'g^', markersize=10, label='W1 (Stabilized)', zorder=5)
        ax.arrow(O[0], O[1], W1[0]-O[0], W1[1]-O[1], 
                head_width=500, head_length=500, fc='green', ec='green', alpha=0.3)
        
        # Original goal T
        T = preprocessed['goal_pos']
        ax.plot(T[0], T[1], 'r*', markersize=20, label='Goal T', zorder=5)
        
        # W_{n-1} (final waypoint before engagement)
        W_n_minus_1 = preprocessed['goal_state']['waypoint']
        ax.plot(W_n_minus_1[0], W_n_minus_1[1], 'rs', markersize=10, 
               label='W_{n-1} (Engage)', zorder=5)
        ax.arrow(W_n_minus_1[0], W_n_minus_1[1], T[0]-W_n_minus_1[0], T[1]-W_n_minus_1[1],
                head_width=500, head_length=500, fc='red', ec='red', alpha=0.3)
        
        # Annotate segments
        l0 = preprocessed['start_state']['straight_length']
        dss = preprocessed['goal_state']['engagement_distance']
        
        # Add text annotations
        ax.text(O[0]-500, O[1]-500, f"L₀={l0/1000:.1f}km", fontsize=9, 
               bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))
        ax.text(W_n_minus_1[0]-500, W_n_minus_1[1]+500, f"d_ss={dss/1000:.1f}km", 
               fontsize=9, bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.7))
    
    # ====== DRAW PLANNED TRAJECTORY ======
    if result and result.get('path'):
        path = result['path']
        waypoints = [wp for wp, heading in path]
        
        # Draw the flown trajectory: straight legs + radius-R turn arcs at each
        # corner (the planner's actual kinodynamic model). This replaces the legacy
        # Dubins renderer, whose placeholder sampler dropped whole segments (LRL/RRL
        # -> no samples) so the line appeared to jump between waypoints.
        try:
            R = preprocessed.get('turn_radius', config.R)
            # Span the full mission O..T (the planner path covers only W_1..W_{n-1}).
            full = tr.build_full_path(path, preprocessed)
            samples = [(x, y) for (x, y) in tr.sample_trajectory(full, R, mode=trajectory_mode)]

            if samples and len(samples) >= 2:
                traj_xs = [s[0] for s in samples]
                traj_ys = [s[1] for s in samples]

                # Draw smooth trajectory (thick blue line)
                _label = 'Dubins Trajectory' if trajectory_mode == 'dubins' else 'Straight Segments'
                ax.plot(traj_xs, traj_ys, 'b-', linewidth=3.0, label=_label,
                       alpha=0.9, zorder=3)

                # Draw waypoint markers
                for i, wp in enumerate(waypoints):
                    ax.plot(wp[0], wp[1], 'bo', markersize=8, alpha=0.7, zorder=4)
                    if i % max(1, len(waypoints)//5) == 0:  # Label every 5th or fewer
                        ax.text(wp[0]+300, wp[1]+300, f'W{i}', fontsize=9, alpha=0.6)

                # Mark where each turn arc begins and ends (small dots).
                if trajectory_mode == 'dubins':
                    turns = tr.turn_markers(full, R)
                    for j, t in enumerate(turns):
                        ax.plot(*t['start'], 'o', color='lime', markersize=4, zorder=5,
                                label='Turn start' if j == 0 else None)
                        ax.plot(*t['end'], 'o', color='magenta', markersize=4, zorder=5,
                                label='Turn end' if j == 0 else None)
            else:
                # Fallback: draw straight lines if trajectory sampling fails
                for i in range(len(waypoints) - 1):
                    wp1 = waypoints[i]
                    wp2 = waypoints[i + 1]
                    ax.plot([wp1[0], wp2[0]], [wp1[1], wp2[1]], 'b-', linewidth=2.5,
                           label='Trajectory' if i == 0 else '')

        except Exception as e:
            # Fallback to straight line segments
            for i in range(len(waypoints) - 1):
                wp1 = waypoints[i]
                wp2 = waypoints[i + 1]
                ax.plot([wp1[0], wp2[0]], [wp1[1], wp2[1]], 'b-', linewidth=2.5, 
                       label='Trajectory' if i == 0 else '')
            
            # Mark waypoints
            for i, wp in enumerate(waypoints):
                ax.plot(wp[0], wp[1], 'bo', markersize=6, alpha=0.7)

    
    # ====== CONFIGURE & DISPLAY ======
    ax.set_xlabel('East (m)', fontsize=11)
    ax.set_ylabel('North (m)', fontsize=11)
    ax.set_title(title, fontsize=13, fontweight='bold')
    
    # Legend
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='upper left', fontsize=10)
    
    # Add info box
    info_text = f"""Autonomous Aircraft Path Planning System
    R = {config.R}m | α_max = {config.ALPHA_MAX}° | L₀ = {config.L0}m
    Dynamic Obstacles: {len(scenario.get('dynamic_obstacles', []))} | Islands: {len(scenario.get('islands', []))}"""
        
    if result:
        stats = result.get('stats', {})
        success = result.get('success', False)
        status = "✓ SUCCESS" if success else "✗ FAILED"
        info_text += f"\n{status} | Iter: {stats.get('iterations', 0)}/{config.MAX_ITERATIONS}"
        if result.get('path'):
            info_text += f" | Waypoints: {len(result['path'])}"
    
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=9,
           verticalalignment='top', bbox=dict(boxstyle='round', 
                                             facecolor='wheat', alpha=0.8),
           family='monospace')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=config.FIGURE_DPI, bbox_inches='tight')
        print(f"Figure saved to {save_path}")
        plt.close()
    
    return fig


def plot_trajectory_details(result, preprocessed, save_path=None):
    """
    Create detailed trajectory analysis plots.
    
    Args:
        result: Result dict from plan_trajectory()
        preprocessed: Preprocessed scenario
        save_path: Path to save figure
    
    Returns:
        Matplotlib figure object
    """
    
    if not result.get('path'):
        return None
    
    path = result['path']
    waypoints = [wp for wp, _ in path]
    headings = [h for _, h in path]
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Subplot 1: X-Y trajectory
    ax = axes[0, 0]
    xs = [wp[0] for wp in waypoints]
    ys = [wp[1] for wp in waypoints]
    ax.plot(xs, ys, 'b-', linewidth=2, label='Trajectory')
    ax.plot(xs[0], ys[0], 'go', markersize=10, label='Start')
    ax.plot(xs[-1], ys[-1], 'r*', markersize=15, label='Goal')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Trajectory in XY Plane')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_aspect('equal')
    
    # Subplot 2: Heading over distance
    ax = axes[0, 1]
    distances = [0]
    for i in range(1, len(waypoints)):
        dx = waypoints[i][0] - waypoints[i-1][0]
        dy = waypoints[i][1] - waypoints[i-1][1]
        dist = math.sqrt(dx**2 + dy**2)
        distances.append(distances[-1] + dist)
    
    headings_deg = [math.degrees(h) for h in headings]
    ax.plot(distances, headings_deg, 'b-', linewidth=2)
    ax.set_xlabel('Distance (m)')
    ax.set_ylabel('Heading (degrees)')
    ax.set_title('Heading vs Distance')
    ax.grid(True, alpha=0.3)
    
    # Subplot 3: Turn angles
    ax = axes[1, 0]
    if len(headings) > 1:
        turn_angles = []
        for i in range(1, len(headings)):
            delta = headings[i] - headings[i-1]
            delta = math.atan2(math.sin(delta), math.cos(delta))
            turn_angles.append(math.degrees(abs(delta)))
        
        ax.bar(range(len(turn_angles)), turn_angles, color='steelblue', alpha=0.7)
        ax.axhline(y=config.ALPHA_MAX, color='r', linestyle='--', label=f'α_max = {config.ALPHA_MAX}°')
        ax.set_xlabel('Segment')
        ax.set_ylabel('Turn Angle (degrees)')
        ax.set_title('Turn Angles per Segment')
        ax.grid(True, alpha=0.3, axis='y')
        ax.legend()
    
    # Subplot 4: Segment distances
    ax = axes[1, 1]
    segment_distances = []
    for i in range(len(waypoints) - 1):
        dx = waypoints[i+1][0] - waypoints[i][0]
        dy = waypoints[i+1][1] - waypoints[i][1]
        dist = math.sqrt(dx**2 + dy**2)
        segment_distances.append(dist / 1000)  # Convert to km
    
    ax.bar(range(len(segment_distances)), segment_distances, color='forestgreen', alpha=0.7)
    ax.set_xlabel('Segment')
    ax.set_ylabel('Distance (km)')
    ax.set_title('Segment Distances')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Add total path stats
    total_distance = sum(segment_distances)
    stats_text = f"Total Distance: {total_distance:.2f} km\nWaypoints: {len(waypoints)}\nTurns: {len(turn_angles) if len(headings) > 1 else 0}"
    fig.text(0.98, 0.02, stats_text, ha='right', va='bottom', fontsize=10,
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8),
            family='monospace')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=config.FIGURE_DPI, bbox_inches='tight')
        print(f"Trajectory details saved to {save_path}")
    
    return fig


def plot_obstacles_comparison(scenario, preprocessed, title="Obstacle Inflation", save_path=None):
    """
    Compare original and inflated obstacles side-by-side.
    
    Args:
        scenario: Original scenario
        preprocessed: Preprocessed scenario
        title: Figure title
        save_path: Path to save figure
    
    Returns:
        Matplotlib figure object
    """
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Left: Original obstacles
    for island in scenario.get('islands', []):
        patch = MplPolygon(island, fill=True, facecolor='saddlebrown', 
                          edgecolor='darkred', linewidth=1.5, alpha=0.7)
        ax1.add_patch(patch)
    
    for center, radius in scenario.get('dynamic_obstacles', []):
        patch = MplCircle(center, radius, fill=True, facecolor='red', 
                         edgecolor='darkred', linewidth=1.5, alpha=0.5)
        ax1.add_patch(patch)
    
    (cmp_xlim, cmp_ylim) = _plot_extents(scenario, pad=0.0)
    ax1.set_xlim(*cmp_xlim)
    ax1.set_ylim(*cmp_ylim)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Original Obstacles')
    ax1.set_xlabel('East (m)')
    ax1.set_ylabel('North (m)')
    
    # Right: Inflated obstacles
    for obstacle in preprocessed['obstacles']:
        if obstacle['type'] == 'circle':
            center = obstacle['center']
            radius = obstacle['radius']
            patch = MplCircle(center, radius, fill=True, facecolor='lightsalmon', 
                             edgecolor='darkred', linewidth=1.5, alpha=0.6)
            ax2.add_patch(patch)
        
        elif obstacle['type'] == 'polygon':
            polygon = obstacle['polygon']
            patch = MplPolygon(polygon, fill=True, facecolor='lightsalmon', 
                              edgecolor='darkred', linewidth=1.5, alpha=0.6)
            ax2.add_patch(patch)
    
    ax2.set_xlim(*cmp_xlim)
    ax2.set_ylim(*cmp_ylim)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.set_title(f'Inflated Obstacles (R={config.R}m + buffer={config.SAFE_MARGIN}m)')
    ax2.set_xlabel('East (m)')
    ax2.set_ylabel('North (m)')
    
    fig.suptitle(title, fontsize=13, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=config.FIGURE_DPI, bbox_inches='tight')
        print(f"Obstacle comparison saved to {save_path}")
    
    return fig
