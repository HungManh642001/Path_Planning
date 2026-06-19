"""Right results column: metric summary, render-mode toggle, and status log."""

import tkinter as tk
from tkinter import ttk

_FIELDS = [
    ('success', 'Success'),
    ('distance_km', 'Distance (km)'),
    ('num_waypoints', 'Waypoints'),
    ('num_turns', 'Turns'),
    ('max_turn_deg', 'Max turn (deg)'),
    ('runtime_ms', 'Runtime (ms)'),
    ('iterations', 'Iterations'),
    ('valid', 'Collision-free'),
]


class ResultsPanel:
    def __init__(self, parent, on_render_mode_change):
        self.frame = ttk.Frame(parent)
        self._on_mode = on_render_mode_change

        ttk.Label(self.frame, text='Results', font=('Arial', 11, 'bold')).pack(anchor=tk.W, pady=(4, 2))
        grid = ttk.Frame(self.frame); grid.pack(fill=tk.X)
        self._value_vars = {}
        for r, (key, label) in enumerate(_FIELDS):
            ttk.Label(grid, text=label, width=16, font=('Arial', 9)).grid(row=r, column=0, sticky='w')
            var = tk.StringVar(value='--')
            self._value_vars[key] = var
            ttk.Label(grid, textvariable=var, font=('Arial', 9, 'bold')).grid(row=r, column=1, sticky='w')

        ttk.Separator(self.frame).pack(fill=tk.X, pady=6)
        ttk.Label(self.frame, text='Render mode', font=('Arial', 10, 'bold')).pack(anchor=tk.W)
        self._mode = tk.StringVar(value='dubins')
        for val, text in (('dubins', 'Dubins (real)'), ('straight', 'Straight (waypoints)')):
            ttk.Radiobutton(self.frame, text=text, value=val, variable=self._mode,
                            command=lambda: self._on_mode(self._mode.get())).pack(anchor=tk.W)

        ttk.Separator(self.frame).pack(fill=tk.X, pady=6)
        ttk.Label(self.frame, text='Log', font=('Arial', 10, 'bold')).pack(anchor=tk.W)
        self._log = tk.Text(self.frame, height=14, width=30, font=('Courier', 8))
        self._log.pack(fill=tk.BOTH, expand=True)

    def widget(self):
        return self.frame

    def _fmt(self, key, value):
        if key in ('success', 'valid'):
            return 'yes' if value else 'no'
        if isinstance(value, float):
            return f'{value:.2f}'
        return str(value)

    def show_summary(self, summary):
        for key, _ in _FIELDS:
            self._value_vars[key].set(self._fmt(key, summary.get(key)))

    def log(self, message):
        self._log.insert(tk.END, message + '\n')
        self._log.see(tk.END)

    def render_mode(self):
        return self._mode.get()
