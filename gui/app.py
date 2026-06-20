"""Planner GUI orchestrator: 3-column layout (config | map | results) and the
scenario->plan pipeline. run_pipeline is Tk-free so it can be tested headlessly."""

import math
import time
import tkinter as tk
from tkinter import ttk, filedialog

import config
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import gui.params as gp
import gui.summary as gsummary
import gui.scenario_io as sio
import gui.interaction as gi
import render.trajectory as tr
from gui.config_panel import ConfigPanel
from gui.map_canvas import MapCanvas
from gui.results_panel import ResultsPanel

# Minimum drag distances (map metres) distinguishing an aim/size from a click.
_AIM_MIN_M = config.MAP_WIDTH * 0.01      # ~5 km on the 500 km map
_RADIUS_MIN_M = 1000.0
_POLY_CLOSE_TOL_M = config.MAP_WIDTH * 0.02


def _build_scenario_dict(state):
    obstacles = []
    for o in state['obstacles']:
        if o['type'] == 'circle':
            obstacles.append({'type': 'circle', 'center': o['center'], 'radius': o['radius']})
        else:
            obstacles.append({'type': 'polygon', 'polygon': o['polygon']})
    islands = [o['polygon'] for o in state['obstacles'] if o['type'] == 'polygon']
    sams = [(o['center'], o['radius']) for o in state['obstacles'] if o['type'] == 'circle']
    return {
        'start': state['start'], 'start_heading': state['start_heading'],
        'goal': state['goal'], 'goal_heading': state['goal_heading'],
        'obstacles': obstacles, 'islands': islands, 'sam_sites': sams,
    }


def run_pipeline(state, values):
    """Apply params, preprocess, plan. Returns (result, preprocessed,
    raw_circles, raw_polys, runtime_s)."""
    kwargs = gp.apply_overrides(values)
    scenario = _build_scenario_dict(state)
    preprocessed = prep.prepare_scenario(scenario, **kwargs)
    raw_circles = [(o['center'], o['radius']) for o in scenario['obstacles'] if o['type'] == 'circle']
    raw_polys = [o['polygon'] for o in scenario['obstacles'] if o['type'] == 'polygon']
    t0 = time.perf_counter()
    result = astar.plan_trajectory(preprocessed, verbose=False)
    runtime_s = time.perf_counter() - t0
    return result, preprocessed, raw_circles, raw_polys, runtime_s


class PlannerApp:
    def __init__(self, root):
        self.root = root
        root.title('Missile Path Planner')
        self.state = {'start': None, 'start_heading': 0.0,
                      'goal': None, 'goal_heading': 0.0, 'obstacles': []}
        self.result = None
        self.preprocessed = None
        self.raw_circles = []
        self.raw_polys = []
        self.mode = 'idle'          # idle | start | goal | polygon | circle
        self._poly_pts = []
        self._circle_center = None
        self._aim_anchor = None     # press point for start/goal aim & circle centre
        self._drag_xy = None        # current drag point (for live preview)

        container = ttk.Frame(root); container.pack(fill=tk.BOTH, expand=True)
        self.config_panel = ConfigPanel(
            container, on_run=self.on_run, on_set_start=lambda: self._set_mode('start'),
            on_set_goal=lambda: self._set_mode('goal'),
            on_draw_polygon=lambda: self._set_mode('polygon'),
            on_draw_circle=lambda: self._set_mode('circle'),
            on_clear=self.on_clear, on_load=self.on_load, on_save=self.on_save)
        self.config_panel.widget().pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=4)

        self.canvas = MapCanvas(container, on_press=self.on_map_press,
                                on_drag=self.on_map_drag, on_release=self.on_map_release)
        self.canvas.widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.results = ResultsPanel(container, on_render_mode_change=self.on_mode_change)
        self.results.widget().pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=4)

        self._redraw()

    def _set_mode(self, mode):
        self.mode = mode
        self._poly_pts = []
        self._circle_center = None
        self._aim_anchor = None
        self._drag_xy = None
        hints = {
            'start': 'Launch: click to place, drag to aim heading',
            'goal': 'Target: click to place, drag to aim heading',
            'circle': 'SAM: press centre, drag out to the radius, release',
            'polygon': 'Island: click each vertex, click near the first to close',
        }
        self.results.log(hints.get(mode, f'Mode: {mode}'))

    def _current_overlay(self):
        """In-progress drawing overlay for the canvas (live preview)."""
        ov = {}
        if self.mode == 'polygon':
            ov['poly_pts'] = list(self._poly_pts)
            if self._poly_pts and self._drag_xy is not None:
                ov['rubber'] = self._drag_xy
        elif self.mode == 'circle' and self._aim_anchor is not None:
            r = 0.0 if self._drag_xy is None else math.hypot(
                self._drag_xy[0] - self._aim_anchor[0], self._drag_xy[1] - self._aim_anchor[1])
            ov['circle_preview'] = (self._aim_anchor, r)
        elif self.mode in ('start', 'goal') and self._aim_anchor is not None and self._drag_xy is not None:
            a, d = self._aim_anchor, self._drag_xy
            if self.mode == 'goal':
                # Approach arrow points INTO the target: same drag direction, but
                # the head sits at T so it lines up with the incoming flight path.
                ov['aim'] = ((2 * a[0] - d[0], 2 * a[1] - d[1]), a)
            else:
                ov['aim'] = (a, d)               # departure: head at the drag point
        return ov

    def on_map_press(self, x, y):
        if self.mode in ('start', 'goal'):
            self._aim_anchor = (x, y)
            self._drag_xy = (x, y)
            self.state[self.mode] = (x, y)          # place immediately; aim on drag
        elif self.mode == 'circle':
            self._aim_anchor = (x, y)
            self._drag_xy = (x, y)
        elif self.mode == 'polygon':
            if (len(self._poly_pts) >= 3 and
                    math.hypot(x - self._poly_pts[0][0], y - self._poly_pts[0][1]) < _POLY_CLOSE_TOL_M):
                self.state['obstacles'].append({'type': 'polygon', 'polygon': list(self._poly_pts)})
                self._poly_pts = []
                self.mode = 'idle'
                self.results.log('Polygon closed')
            else:
                self._poly_pts.append((x, y))
                self.results.log(f'Vertex {len(self._poly_pts)} (click near the first to close)')
        self._redraw()

    def on_map_drag(self, x, y):
        if self.mode in ('start', 'goal', 'circle', 'polygon'):
            self._drag_xy = (x, y)
            if self.mode in ('start', 'goal') and self._aim_anchor is not None:
                h = gi.heading_from_drag(self._aim_anchor, (x, y), _AIM_MIN_M)
                if h is not None:
                    self.state[self.mode + '_heading'] = h
            self._redraw()

    def on_map_release(self, x, y):
        if self.mode in ('start', 'goal') and self._aim_anchor is not None:
            h = gi.heading_from_drag(self._aim_anchor, (x, y), _AIM_MIN_M)
            if h is not None:
                self.state[self.mode + '_heading'] = h
            self._aim_anchor = None
            self._drag_xy = None
            self.mode = 'idle'
        elif self.mode == 'circle' and self._aim_anchor is not None:
            r = math.hypot(x - self._aim_anchor[0], y - self._aim_anchor[1])
            if r >= _RADIUS_MIN_M:
                self.state['obstacles'].append(
                    {'type': 'circle', 'center': self._aim_anchor, 'radius': r})
                self.results.log(f'SAM added (r={r/1000:.1f} km)')
            self._aim_anchor = None
            self._drag_xy = None
            self.mode = 'idle'
        # polygon: vertices are committed on press; release is a no-op
        self._redraw()

    def on_clear(self):
        self.state['obstacles'] = []
        self._poly_pts = []
        self._aim_anchor = None
        self._drag_xy = None
        self.result = None
        self.preprocessed = None
        self._redraw()

    def on_save(self):
        path = filedialog.asksaveasfilename(defaultextension='.json')
        if path:
            with open(path, 'w') as f:
                f.write(sio.scenario_to_json(self.state))
            self.results.log(f'Saved {path}')

    def on_load(self):
        path = filedialog.askopenfilename(filetypes=[('JSON', '*.json')])
        if path:
            with open(path) as f:
                self.state = sio.scenario_from_json(f.read())
            self.result = None
            self._redraw()
            self.results.log(f'Loaded {path}')

    def _apply_numeric_entries(self):
        sg = self.config_panel.numeric_start_goal()
        if sg['start'] is not None:
            self.state['start'] = sg['start']
            self.state['start_heading'] = math.radians(sg['start_heading_deg'])
        if sg['goal'] is not None:
            self.state['goal'] = sg['goal']
            self.state['goal_heading'] = math.radians(sg['goal_heading_deg'])

    def on_run(self):
        self._apply_numeric_entries()
        if self.state['start'] is None or self.state['goal'] is None:
            self.results.log('ERROR: set launch and target first')
            return
        self.results.log('Planning...')
        try:
            (self.result, self.preprocessed, self.raw_circles,
             self.raw_polys, runtime_s) = run_pipeline(self.state, self.config_panel.values())
        except Exception as e:
            self.results.log(f'ERROR: {e}')
            return
        summary = gsummary.compute_summary(
            self.result, self.preprocessed, self.raw_circles, self.raw_polys,
            self.results.render_mode(), runtime_s)
        self.results.show_summary(summary)
        if summary['success']:
            self.results.log('Done')
            self._log_turn_angles()
        else:
            self.results.log('No path found')
        self._redraw()

    def _log_turn_angles(self):
        """Log the turn angle at every turning waypoint (full path O..T)."""
        R = self.preprocessed.get('turn_radius', config.R)
        full = tr.build_full_path(self.result['path'], self.preprocessed)
        turns = tr.turn_markers(full, R)
        if not turns:
            self.results.log('Turn angles: none (straight run)')
            return
        self.results.log('Turn angles per waypoint:')
        for i, t in enumerate(turns, start=1):
            a = t['angle_deg']
            side = 'L' if a > 0 else 'R'
            mx, my = t['mid']
            self.results.log(f'  W{i}: {abs(a):5.1f} deg {side}  @({mx/1000:.1f}, {my/1000:.1f}) km')

    def on_mode_change(self, _mode):
        self._redraw()              # re-render only; do NOT re-plan

    def _redraw(self):
        self.canvas.render(self.state, self.result, self.preprocessed,
                           self.results.render_mode(), overlay=self._current_overlay())
