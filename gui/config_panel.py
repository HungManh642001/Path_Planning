"""Left configuration column: scenario inputs, tactical params, collapsible Advanced,
and the RUN button. Builds parameter widgets from gui.params.PARAM_SPECS."""

import tkinter as tk
from tkinter import ttk

import gui.params as gp


class ConfigPanel:
    def __init__(self, parent, on_run, on_set_start, on_set_goal,
                 on_draw_polygon, on_draw_circle, on_clear, on_load, on_save):
        self.frame = ttk.Frame(parent)
        self._vars = {}

        ttk.Label(self.frame, text='Scenario', font=('Arial', 11, 'bold')).pack(anchor=tk.W, pady=(4, 2))
        sc = ttk.Frame(self.frame); sc.pack(fill=tk.X)
        ttk.Button(sc, text='Set Launch', command=on_set_start).grid(row=0, column=0, sticky='ew', padx=2, pady=2)
        ttk.Button(sc, text='Set Target', command=on_set_goal).grid(row=0, column=1, sticky='ew', padx=2, pady=2)
        ttk.Button(sc, text='Draw Island', command=on_draw_polygon).grid(row=1, column=0, sticky='ew', padx=2, pady=2)
        ttk.Button(sc, text='Draw SAM', command=on_draw_circle).grid(row=1, column=1, sticky='ew', padx=2, pady=2)
        ttk.Button(sc, text='Clear All', command=on_clear).grid(row=2, column=0, sticky='ew', padx=2, pady=2)
        sc.columnconfigure(0, weight=1); sc.columnconfigure(1, weight=1)

        # Numeric start/goal entry
        ttk.Label(self.frame, text='Numeric (x, y, heading deg)', font=('Arial', 9)).pack(anchor=tk.W, pady=(6, 0))
        self._sg = {}
        for name in ('start_x', 'start_y', 'start_h', 'goal_x', 'goal_y', 'goal_h'):
            self._sg[name] = tk.StringVar(value='')
        grid = ttk.Frame(self.frame); grid.pack(fill=tk.X)
        ttk.Label(grid, text='Launch').grid(row=0, column=0)
        for c, n in enumerate(('start_x', 'start_y', 'start_h')):
            ttk.Entry(grid, textvariable=self._sg[n], width=8).grid(row=0, column=c + 1, padx=1)
        ttk.Label(grid, text='Target').grid(row=1, column=0)
        for c, n in enumerate(('goal_x', 'goal_y', 'goal_h')):
            ttk.Entry(grid, textvariable=self._sg[n], width=8).grid(row=1, column=c + 1, padx=1)

        io = ttk.Frame(self.frame); io.pack(fill=tk.X, pady=2)
        ttk.Button(io, text='Load JSON', command=on_load).grid(row=0, column=0, sticky='ew', padx=2)
        ttk.Button(io, text='Save JSON', command=on_save).grid(row=0, column=1, sticky='ew', padx=2)
        io.columnconfigure(0, weight=1); io.columnconfigure(1, weight=1)

        ttk.Separator(self.frame).pack(fill=tk.X, pady=4)
        ttk.Label(self.frame, text='Parameters', font=('Arial', 11, 'bold')).pack(anchor=tk.W)
        self._build_param_group(self.frame, ('tactical', 'run'))

        self._adv_shown = tk.BooleanVar(value=False)
        ttk.Checkbutton(self.frame, text='Advanced ▸', variable=self._adv_shown,
                        command=self._toggle_advanced).pack(anchor=tk.W, pady=(6, 0))
        self._adv_frame = ttk.Frame(self.frame)
        self._build_param_group(self._adv_frame, ('advanced',))

        ttk.Separator(self.frame).pack(fill=tk.X, pady=6)
        ttk.Button(self.frame, text='▶ RUN', command=on_run).pack(fill=tk.X, ipady=6)

    def _build_param_group(self, parent, groups):
        for spec in gp.PARAM_SPECS:
            if spec['group'] not in groups:
                continue
            var = tk.DoubleVar(value=spec['default'])
            self._vars[spec['key']] = var
            row = ttk.Frame(parent); row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=spec['label'], width=20, font=('Arial', 8)).pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=var, width=10).pack(side=tk.RIGHT)

    def _toggle_advanced(self):
        if self._adv_shown.get():
            self._adv_frame.pack(fill=tk.X)
        else:
            self._adv_frame.pack_forget()

    def widget(self):
        return self.frame

    def values(self):
        return {k: v.get() for k, v in self._vars.items()}

    def _opt_float(self, name):
        text = self._sg[name].get().strip()
        try:
            return float(text)
        except ValueError:
            return None

    def numeric_start_goal(self):
        sx, sy = self._opt_float('start_x'), self._opt_float('start_y')
        gx, gy = self._opt_float('goal_x'), self._opt_float('goal_y')
        return {
            'start': (sx, sy) if sx is not None and sy is not None else None,
            'start_heading_deg': self._opt_float('start_h') or 0.0,
            'goal': (gx, gy) if gx is not None and gy is not None else None,
            'goal_heading_deg': self._opt_float('goal_h') or 0.0,
        }
