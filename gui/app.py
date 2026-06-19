"""Planner GUI orchestrator: 3-column layout (config | map | results) and the
scenario->plan pipeline. run_pipeline is Tk-free so it can be tested headlessly."""

import math
import time
import tkinter as tk
from tkinter import ttk, filedialog

import config
import preprocessing as prep
import kinodynamic_astar as astar
import gui.params as gp
import gui.summary as gsummary
import gui.scenario_io as sio
from gui.config_panel import ConfigPanel
from gui.map_canvas import MapCanvas
from gui.results_panel import ResultsPanel


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

        container = ttk.Frame(root); container.pack(fill=tk.BOTH, expand=True)
        self.config_panel = ConfigPanel(
            container, on_run=self.on_run, on_set_start=lambda: self._set_mode('start'),
            on_set_goal=lambda: self._set_mode('goal'),
            on_draw_polygon=lambda: self._set_mode('polygon'),
            on_draw_circle=lambda: self._set_mode('circle'),
            on_clear=self.on_clear, on_load=self.on_load, on_save=self.on_save)
        self.config_panel.widget().pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=4)

        self.canvas = MapCanvas(container, on_map_click=self.on_map_click)
        self.canvas.widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.results = ResultsPanel(container, on_render_mode_change=self.on_mode_change)
        self.results.widget().pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=4)

        self._redraw()

    def _set_mode(self, mode):
        self.mode = mode
        self._poly_pts = []
        self._circle_center = None
        self.results.log(f'Mode: {mode} (click on the map)')

    def on_map_click(self, x, y):
        if self.mode == 'start':
            self.state['start'] = (x, y); self.mode = 'idle'
        elif self.mode == 'goal':
            self.state['goal'] = (x, y); self.mode = 'idle'
        elif self.mode == 'circle':
            if self._circle_center is None:
                self._circle_center = (x, y)
                self.results.log('Click again to set radius')
            else:
                r = math.hypot(x - self._circle_center[0], y - self._circle_center[1])
                self.state['obstacles'].append({'type': 'circle', 'center': self._circle_center, 'radius': r})
                self._circle_center = None; self.mode = 'idle'
        elif self.mode == 'polygon':
            close_tol = config.MAP_WIDTH * 0.02
            if (len(self._poly_pts) >= 3 and
                    math.hypot(x - self._poly_pts[0][0], y - self._poly_pts[0][1]) < close_tol):
                self.state['obstacles'].append({'type': 'polygon', 'polygon': list(self._poly_pts)})
                self._poly_pts = []
                self.mode = 'idle'
                self.results.log('Polygon closed')
            else:
                self._poly_pts.append((x, y))
                self.results.log(f'Polygon point {len(self._poly_pts)} (click near first point to close)')
        self._redraw()

    def on_clear(self):
        self.state['obstacles'] = []
        self._poly_pts = []
        self.result = None
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
        self.results.log('Done' if summary['success'] else 'No path found')
        self._redraw()

    def on_mode_change(self, _mode):
        self._redraw()              # re-render only; do NOT re-plan

    def _redraw(self):
        self.canvas.render(self.state, self.result, self.preprocessed,
                           self.results.render_mode())
