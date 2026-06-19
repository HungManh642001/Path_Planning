"""Matplotlib map canvas for the planner GUI: draws obstacles, start/goal, and the
planned trajectory, and reports left-clicks in map coordinates."""

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Circle as MplCircle, Polygon as MplPolygon

import config
import trajectory as tr


class MapCanvas:
    def __init__(self, parent, on_map_click):
        self._on_map_click = on_map_click
        self.fig = Figure(figsize=(7, 7), dpi=100, layout='tight')
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.mpl_connect('button_press_event', self._handle_click)
        self._draw_empty()

    def widget(self):
        return self.canvas.get_tk_widget()

    def _handle_click(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        if event.button == 1:
            self._on_map_click(float(event.xdata), float(event.ydata))

    def _draw_empty(self):
        self.ax.clear()
        self.ax.set_xlim(0, config.MAP_WIDTH)
        self.ax.set_ylim(0, config.MAP_HEIGHT)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('East (m)')
        self.ax.set_ylabel('North (m)')
        self.ax.grid(True, alpha=0.3)

    def render(self, state, result, preprocessed, render_mode):
        self._draw_empty()
        for o in state['obstacles']:
            if o['type'] == 'circle':
                self.ax.add_patch(MplCircle(o['center'], o['radius'],
                                            color='salmon', alpha=0.5))
            else:
                self.ax.add_patch(MplPolygon(o['polygon'], color='saddlebrown', alpha=0.6))
        if state['start'] is not None:
            self.ax.plot(*state['start'], 'go', markersize=10, label='Launch O')
        if state['goal'] is not None:
            self.ax.plot(*state['goal'], 'r*', markersize=16, label='Target T')

        if result and result.get('path'):
            R = (preprocessed or {}).get('turn_radius', config.R)
            pts = tr.sample_trajectory(result['path'], R, mode=render_mode)
            if len(pts) >= 2:
                self.ax.plot([p[0] for p in pts], [p[1] for p in pts],
                             'b-', linewidth=2.5,
                             label='Dubins' if render_mode == 'dubins' else 'Straight')
            for wp, _ in result['path']:
                self.ax.plot(wp[0], wp[1], 'bo', markersize=5, alpha=0.7)

        handles, labels = self.ax.get_legend_handles_labels()
        if labels:
            self.ax.legend(loc='upper left', fontsize=8)
        self.canvas.draw()
