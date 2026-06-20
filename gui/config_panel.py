"""Left configuration column: scenario inputs, tactical params, collapsible
Advanced, and the RUN button. Builds parameter widgets from gui.params.PARAM_SPECS.

The scenario/parameter area scrolls vertically so it never pushes RUN off-screen;
RUN is pinned to the bottom of the panel and always visible. Each parameter has a
slider and a numeric entry kept in sync. values() parses entries defensively
(empty/garbage falls back to the spec default) so RUN never crashes on a
transiently-invalid field.
"""

import tkinter as tk
from tkinter import ttk

import gui.params as gp
import gui.interaction as gi


def _fmt(value):
    """Compact display: integers without a trailing .0, floats trimmed."""
    if float(value).is_integer():
        return str(int(value))
    return f'{value:g}'


class ConfigPanel:
    def __init__(self, parent, on_run, on_set_start, on_set_goal,
                 on_draw_polygon, on_draw_circle, on_clear, on_load, on_save):
        self.frame = ttk.Frame(parent)
        self._vars = {}                       # key -> StringVar (source of truth)
        self._defaults = {s['key']: s['default'] for s in gp.PARAM_SPECS}
        self._scale_vars = {}                 # key -> DoubleVar for the slider
        self._syncing = set()                 # keys currently syncing (loop guard)

        # --- scrollable body ---------------------------------------------------
        body = ttk.Frame(self.frame)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self._scroll_canvas = tk.Canvas(body, width=260, highlightthickness=0)
        self._vbar = ttk.Scrollbar(body, orient=tk.VERTICAL, command=self._scroll_canvas.yview)
        self._scroll_canvas.configure(yscrollcommand=self._vbar.set)
        self._scroll_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self._scrollable = False        # only true once content overflows
        self._vbar_shown = False
        inner = ttk.Frame(self._scroll_canvas)
        self._inner_id = self._scroll_canvas.create_window((0, 0), window=inner, anchor='nw')
        inner.bind('<Configure>', lambda e: self._update_scrollregion())
        self._scroll_canvas.bind('<Configure>', self._on_canvas_configure)
        self._scroll_canvas.bind_all('<MouseWheel>', self._on_mousewheel)
        self._scroll_canvas.bind_all('<Button-4>', self._on_mousewheel)
        self._scroll_canvas.bind_all('<Button-5>', self._on_mousewheel)

        # --- scenario buttons --------------------------------------------------
        ttk.Label(inner, text='Scenario', font=('Arial', 11, 'bold')).pack(anchor=tk.W, pady=(4, 2))
        ttk.Label(inner, text='(click to place; drag to aim heading)',
                  font=('Arial', 8), foreground='#555').pack(anchor=tk.W)
        sc = ttk.Frame(inner); sc.pack(fill=tk.X)
        ttk.Button(sc, text='Set Launch', command=on_set_start).grid(row=0, column=0, sticky='ew', padx=2, pady=2)
        ttk.Button(sc, text='Set Target', command=on_set_goal).grid(row=0, column=1, sticky='ew', padx=2, pady=2)
        ttk.Button(sc, text='Draw Island', command=on_draw_polygon).grid(row=1, column=0, sticky='ew', padx=2, pady=2)
        ttk.Button(sc, text='Draw SAM', command=on_draw_circle).grid(row=1, column=1, sticky='ew', padx=2, pady=2)
        ttk.Button(sc, text='Clear All', command=on_clear).grid(row=2, column=0, columnspan=2, sticky='ew', padx=2, pady=2)
        sc.columnconfigure(0, weight=1); sc.columnconfigure(1, weight=1)

        # --- numeric start/goal entry -----------------------------------------
        ttk.Label(inner, text='Numeric (x, y, heading deg)', font=('Arial', 9)).pack(anchor=tk.W, pady=(6, 0))
        self._sg = {}
        for name in ('start_x', 'start_y', 'start_h', 'goal_x', 'goal_y', 'goal_h'):
            self._sg[name] = tk.StringVar(value='')
        grid = ttk.Frame(inner); grid.pack(fill=tk.X)
        ttk.Label(grid, text='Launch').grid(row=0, column=0)
        for c, n in enumerate(('start_x', 'start_y', 'start_h')):
            ttk.Entry(grid, textvariable=self._sg[n], width=8).grid(row=0, column=c + 1, padx=1)
        ttk.Label(grid, text='Target').grid(row=1, column=0)
        for c, n in enumerate(('goal_x', 'goal_y', 'goal_h')):
            ttk.Entry(grid, textvariable=self._sg[n], width=8).grid(row=1, column=c + 1, padx=1)

        io = ttk.Frame(inner); io.pack(fill=tk.X, pady=2)
        ttk.Button(io, text='Load JSON', command=on_load).grid(row=0, column=0, sticky='ew', padx=2)
        ttk.Button(io, text='Save JSON', command=on_save).grid(row=0, column=1, sticky='ew', padx=2)
        io.columnconfigure(0, weight=1); io.columnconfigure(1, weight=1)

        # --- parameters --------------------------------------------------------
        ttk.Separator(inner).pack(fill=tk.X, pady=4)
        ttk.Label(inner, text='Parameters', font=('Arial', 11, 'bold')).pack(anchor=tk.W)
        self._build_param_group(inner, ('tactical', 'run'))

        self._adv_shown = tk.BooleanVar(value=False)
        ttk.Checkbutton(inner, text='Advanced', variable=self._adv_shown,
                        command=self._toggle_advanced).pack(anchor=tk.W, pady=(6, 0))
        self._adv_frame = ttk.Frame(inner)
        self._build_param_group(self._adv_frame, ('advanced',))

        # --- RUN (pinned, always visible) -------------------------------------
        ttk.Separator(self.frame).pack(side=tk.TOP, fill=tk.X, pady=4)
        ttk.Button(self.frame, text='RUN', command=on_run).pack(side=tk.BOTTOM, fill=tk.X, ipady=6, padx=2, pady=2)

    # -- parameter rows: slider + entry kept in sync ---------------------------
    def _build_param_group(self, parent, groups):
        for spec in gp.PARAM_SPECS:
            if spec['group'] not in groups:
                continue
            key = spec['key']
            var = tk.StringVar(value=_fmt(spec['default']))
            self._vars[key] = var
            scale_var = tk.DoubleVar(value=float(spec['default']))
            self._scale_vars[key] = scale_var

            row = ttk.Frame(parent); row.pack(fill=tk.X, pady=(2, 0))
            ttk.Label(row, text=spec['label'], font=('Arial', 8)).pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=var, width=9).pack(side=tk.RIGHT)
            scale = ttk.Scale(parent, from_=spec['min'], to=spec['max'],
                              variable=scale_var,
                              command=lambda v, k=key: self._on_scale(k))
            scale.pack(fill=tk.X, pady=(0, 2))
            var.trace_add('write', lambda *a, k=key: self._on_entry(k))

    def _on_scale(self, key):
        if key in self._syncing:
            return
        self._syncing.add(key)
        try:
            self._vars[key].set(_fmt(self._scale_vars[key].get()))
        finally:
            self._syncing.discard(key)

    def _on_entry(self, key):
        if key in self._syncing:
            return
        text = self._vars[key].get()
        try:
            value = float(text.strip())
        except (ValueError, AttributeError):
            return                              # leave slider; values() will fall back
        self._syncing.add(key)
        try:
            self._scale_vars[key].set(value)
        finally:
            self._syncing.discard(key)

    def _toggle_advanced(self):
        if self._adv_shown.get():
            self._adv_frame.pack(fill=tk.X)
        else:
            self._adv_frame.pack_forget()

    def _on_canvas_configure(self, event):
        self._scroll_canvas.itemconfigure(self._inner_id, width=event.width)
        self._update_scrollregion()

    def _update_scrollregion(self):
        """Recompute scrollability: only scroll/show the bar once content overflows."""
        bbox = self._scroll_canvas.bbox('all')
        if bbox is None:
            return
        self._scroll_canvas.configure(scrollregion=bbox)
        content_h = bbox[3] - bbox[1]
        view_h = self._scroll_canvas.winfo_height()
        self._scrollable = content_h > view_h + 1
        if self._scrollable and not self._vbar_shown:
            self._vbar.pack(side=tk.RIGHT, fill=tk.Y)
            self._vbar_shown = True
        elif not self._scrollable and self._vbar_shown:
            self._vbar.pack_forget()
            self._vbar_shown = False
            self._scroll_canvas.yview_moveto(0.0)        # snap back to top

    def _pointer_over_panel(self, event):
        w = self.frame.winfo_containing(event.x_root, event.y_root)
        while w is not None:
            if w == self.frame:
                return True
            w = getattr(w, 'master', None)
        return False

    def _on_mousewheel(self, event):
        # No scrolling until the panel content overflows, and only when the
        # pointer is actually over the left panel.
        if not self._scrollable or not self._pointer_over_panel(event):
            return
        if event.num == 4:
            delta = -1
        elif event.num == 5:
            delta = 1
        else:
            delta = -1 if event.delta > 0 else 1
        self._scroll_canvas.yview_scroll(delta, 'units')

    def widget(self):
        return self.frame

    def set_field(self, key, text):
        """Set a parameter entry's text (used by tests and programmatic edits)."""
        self._vars[key].set(text)

    def values(self):
        return {k: gi.parse_param(v.get(), self._defaults[k]) for k, v in self._vars.items()}

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
