"""Matplotlib map canvas for the planner GUI.

Draws obstacles, start/goal (with heading arrows), the planned trajectory, and a
live "in-progress" overlay while the user is drawing (polygon vertices, a circle
preview, or a heading aim arrow). Reports mouse press/drag/release in map
coordinates so the app can implement click-to-place + drag-to-aim.
"""

import math

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Circle as MplCircle, Polygon as MplPolygon

import config
import trajectory as tr


class MapCanvas:
    def __init__(self, parent, on_press=None, on_drag=None, on_release=None):
        self._on_press = on_press or (lambda x, y: None)
        self._on_drag = on_drag or (lambda x, y: None)
        self._on_release = on_release or (lambda x, y: None)
        self._pressed = False
        self._last_xy = None

        self.fig = Figure(figsize=(7, 7), dpi=100, layout='tight')
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.mpl_connect('button_press_event', self._handle_press)
        self.canvas.mpl_connect('motion_notify_event', self._handle_motion)
        self.canvas.mpl_connect('button_release_event', self._handle_release)
        self._draw_empty()

    def widget(self):
        return self.canvas.get_tk_widget()

    # -- mouse plumbing --------------------------------------------------------
    def _handle_press(self, event):
        if event.button != 1 or event.inaxes != self.ax or event.xdata is None:
            return
        self._pressed = True
        self._last_xy = (float(event.xdata), float(event.ydata))
        self._on_press(*self._last_xy)

    def _handle_motion(self, event):
        if not self._pressed or event.inaxes != self.ax or event.xdata is None:
            return
        self._last_xy = (float(event.xdata), float(event.ydata))
        self._on_drag(*self._last_xy)

    def _handle_release(self, event):
        if event.button != 1 or not self._pressed:
            return
        self._pressed = False
        if event.xdata is not None and event.inaxes == self.ax:
            self._last_xy = (float(event.xdata), float(event.ydata))
        if self._last_xy is not None:
            self._on_release(*self._last_xy)

    # -- rendering -------------------------------------------------------------
    def _draw_empty(self):
        self.ax.clear()
        self.ax.set_xlim(0, config.MAP_WIDTH)
        self.ax.set_ylim(0, config.MAP_HEIGHT)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('East (m)')
        self.ax.set_ylabel('North (m)')
        self.ax.grid(True, alpha=0.3)

    def _draw_heading_arrow(self, pos, heading, color):
        length = config.MAP_WIDTH * 0.06
        dx = length * math.cos(heading)
        dy = length * math.sin(heading)
        self.ax.annotate('', xy=(pos[0] + dx, pos[1] + dy), xytext=(pos[0], pos[1]),
                         arrowprops=dict(arrowstyle='-|>', color=color, lw=2))

    def render(self, state, result, preprocessed, render_mode, overlay=None):
        self._draw_empty()
        overlay = overlay or {}

        for o in state['obstacles']:
            if o['type'] == 'circle':
                self.ax.add_patch(MplCircle(o['center'], o['radius'], color='salmon', alpha=0.5))
            else:
                self.ax.add_patch(MplPolygon(o['polygon'], color='saddlebrown', alpha=0.6))

        if state['start'] is not None:
            self.ax.plot(*state['start'], 'go', markersize=10, label='Launch O')
            self._draw_heading_arrow(state['start'], state.get('start_heading', 0.0), 'green')
        if state['goal'] is not None:
            self.ax.plot(*state['goal'], 'r*', markersize=16, label='Target T')
            self._draw_heading_arrow(state['goal'], state.get('goal_heading', 0.0), 'red')

        # in-progress drawing overlay
        poly_pts = overlay.get('poly_pts') or []
        if poly_pts:
            xs = [p[0] for p in poly_pts]
            ys = [p[1] for p in poly_pts]
            self.ax.plot(xs, ys, 'o--', color='orange', markersize=6, linewidth=1.5)
            rubber = overlay.get('rubber')
            if rubber is not None:
                self.ax.plot([xs[-1], rubber[0]], [ys[-1], rubber[1]], ':', color='orange', linewidth=1)
        circ = overlay.get('circle_preview')
        if circ is not None:
            center, radius = circ
            self.ax.add_patch(MplCircle(center, radius, fill=False, edgecolor='orange',
                                        linestyle='--', linewidth=1.5))
            self.ax.plot(*center, 'o', color='orange', markersize=5)
        aim = overlay.get('aim')
        if aim is not None:
            p0, p1 = aim
            self.ax.annotate('', xy=p1, xytext=p0,
                             arrowprops=dict(arrowstyle='-|>', color='blue', lw=2, linestyle='--'))

        if result and result.get('path'):
            R = (preprocessed or {}).get('turn_radius', config.R)
            full = tr.build_full_path(result['path'], preprocessed or {})
            pts = tr.sample_trajectory(full, R, mode=render_mode)
            if len(pts) >= 2:
                self.ax.plot([p[0] for p in pts], [p[1] for p in pts], 'b-', linewidth=2.5,
                             label='Dubins' if render_mode == 'dubins' else 'Straight')
            for wp, _ in result['path']:
                self.ax.plot(wp[0], wp[1], 'bo', markersize=5, alpha=0.7)
            if render_mode == 'dubins':
                turns = tr.turn_markers(full, R)
                for j, t in enumerate(turns):
                    self.ax.plot(*t['start'], '^', color='darkgreen', markersize=8,
                                 label='Turn start' if j == 0 else None)
                    self.ax.plot(*t['end'], 'v', color='purple', markersize=8,
                                 label='Turn end' if j == 0 else None)

        handles, labels = self.ax.get_legend_handles_labels()
        if labels:
            self.ax.legend(loc='upper left', fontsize=8)
        self.canvas.draw()
