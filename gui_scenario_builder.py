"""
Interactive GUI Scenario Builder with Integrated Results Display
- Configurable parameters (turn radius, max angle, safe margin, heuristic weight)
- Smaller markers for cleaner visualization
- Integrated result display (left=scenario, right=results, no separate windows)
"""

import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import math
import time
import threading

import config
import preprocessing as prep
import kinodynamic_astar as astar
import dubins_curves as dc


class ScenarioBuilder:
    """Interactive scenario builder with integrated results display"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Missile Path Planning - Interactive Scenario Builder")
        self.root.geometry("1900x1050")
        
        # Scenario data
        self.start_point = None
        self.goal_point = None
        self.obstacles = []
        self.current_polygon = []
        self.current_circle_center = None
        self.last_result = None
        self.preprocessed = None
        
        # UI State
        self.mode = "idle"
        self.drawing_radius = 0
        
        # Parameters (configurable on GUI)
        self.params = {
            'turn_radius': config.TURN_RADIUS,
            'max_turn_angle': math.degrees(config.ALPHA_MAX_RAD),
            'safe_margin': config.SAFE_MARGIN,
            'h_weight': config.HEURISTIC_WEIGHT,
            'launch_angle': config.LAUNCH_ANGLE_DEFAULT,
            'approach_angle': config.APPROACH_ANGLE_DEFAULT,
            'dss': config.DSS,
            'l0': config.L0,
        }
        
        self.create_ui()
        
    def create_ui(self):
        """Create the user interface with parameter controls"""
        
        # Main container
        main_container = ttk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # === LEFT PANEL - Scrollable Controls ===
        left_panel_container = ttk.Frame(main_container, width=320)
        left_panel_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=5, pady=5)
        left_panel_container.pack_propagate(False)
        
        # Title
        title_label = ttk.Label(left_panel_container, text="Scenario Builder", 
                               font=("Arial", 12, "bold"))
        title_label.pack(pady=8)
        
        # Create scrollable canvas for all left panel content
        left_canvas = tk.Canvas(left_panel_container, bg="white", highlightthickness=0)
        left_scrollbar = ttk.Scrollbar(left_panel_container, orient=tk.VERTICAL, command=left_canvas.yview)
        left_panel = ttk.Frame(left_canvas)
        
        left_panel.bind(
            "<Configure>",
            lambda e: left_canvas.configure(scrollregion=left_canvas.bbox("all"))
        )
        
        left_canvas.create_window((0, 0), window=left_panel, anchor="nw")
        left_canvas.configure(yscrollcommand=left_scrollbar.set)
        
        # Bind mousewheel scrolling for left panel
        def _on_left_mousewheel(event):
            left_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        left_canvas.bind_all("<MouseWheel>", _on_left_mousewheel)
        
        left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # === PARAMETERS SECTION ===
        params_frame = ttk.LabelFrame(left_panel, text="⚙️ Parameters", padding=5)
        params_frame.pack(fill=tk.X, pady=8, padx=3)
        
        # ---- PARAMETERS CONTENT ----
        # Turn Radius
        ttk.Label(params_frame, text="Turn Radius (m):", font=("Arial", 9)).pack(anchor=tk.W, padx=5, pady=(5,0))
        self.turn_radius_var = tk.DoubleVar(value=self.params['turn_radius'])
        self.turn_radius_var.trace('w', self.update_params)
        ttk.Scale(params_frame, from_=1000, to=10000, orient=tk.HORIZONTAL, 
                 variable=self.turn_radius_var).pack(fill=tk.X, padx=5)
        self.turn_radius_label = ttk.Label(params_frame, text=f"{self.params['turn_radius']:.0f} m", 
                                          font=("Arial", 8), foreground="blue")
        self.turn_radius_label.pack(anchor=tk.W, padx=5, pady=(0,3))
        
        # Max Turn Angle
        ttk.Label(params_frame, text="Max Turn Angle (°):", font=("Arial", 9)).pack(anchor=tk.W, padx=5, pady=(5,0))
        self.max_angle_var = tk.DoubleVar(value=self.params['max_turn_angle'])
        self.max_angle_var.trace('w', self.update_params)
        ttk.Scale(params_frame, from_=5, to=45, orient=tk.HORIZONTAL, 
                 variable=self.max_angle_var).pack(fill=tk.X, padx=5)
        self.max_angle_label = ttk.Label(params_frame, text=f"{self.params['max_turn_angle']:.1f}°",
                                        font=("Arial", 8), foreground="blue")
        self.max_angle_label.pack(anchor=tk.W, padx=5, pady=(0,3))
        
        # Safe Margin
        ttk.Label(params_frame, text="Safe Margin (m):", font=("Arial", 9)).pack(anchor=tk.W, padx=5, pady=(5,0))
        self.safe_margin_var = tk.DoubleVar(value=self.params['safe_margin'])
        self.safe_margin_var.trace('w', self.update_params)
        ttk.Scale(params_frame, from_=0, to=5000, orient=tk.HORIZONTAL, 
                 variable=self.safe_margin_var).pack(fill=tk.X, padx=5)
        self.safe_margin_label = ttk.Label(params_frame, text=f"{self.params['safe_margin']:.0f} m",
                                          font=("Arial", 8), foreground="blue")
        self.safe_margin_label.pack(anchor=tk.W, padx=5, pady=(0,3))
        
        # Heuristic Weight
        ttk.Label(params_frame, text="Heuristic Weight:", font=("Arial", 9)).pack(anchor=tk.W, padx=5, pady=(5,0))
        self.h_weight_var = tk.DoubleVar(value=self.params['h_weight'])
        self.h_weight_var.trace('w', self.update_params)
        ttk.Scale(params_frame, from_=0.5, to=3.0, orient=tk.HORIZONTAL, 
                 variable=self.h_weight_var).pack(fill=tk.X, padx=5)
        self.h_weight_label = ttk.Label(params_frame, text=f"{self.params['h_weight']:.2f}",
                                       font=("Arial", 8), foreground="blue")
        self.h_weight_label.pack(anchor=tk.W, padx=5, pady=(0,3))
        
        # Launch Angle
        ttk.Label(params_frame, text="Launch Angle (°):", font=("Arial", 9)).pack(anchor=tk.W, padx=5, pady=(5,0))
        self.launch_angle_var = tk.DoubleVar(value=self.params['launch_angle'])
        self.launch_angle_var.trace('w', self.update_params)
        ttk.Scale(params_frame, from_=config.LAUNCH_ANGLE_MIN, to=config.LAUNCH_ANGLE_MAX, 
                 orient=tk.HORIZONTAL, variable=self.launch_angle_var).pack(fill=tk.X, padx=5)
        self.launch_angle_label = ttk.Label(params_frame, text=f"{self.params['launch_angle']:.1f}°",
                                           font=("Arial", 8), foreground="blue")
        self.launch_angle_label.pack(anchor=tk.W, padx=5, pady=(0,3))
        
        # Approach Angle
        ttk.Label(params_frame, text="Approach Angle (°):", font=("Arial", 9)).pack(anchor=tk.W, padx=5, pady=(5,0))
        self.approach_angle_var = tk.DoubleVar(value=self.params['approach_angle'])
        self.approach_angle_var.trace('w', self.update_params)
        ttk.Scale(params_frame, from_=config.APPROACH_ANGLE_MIN, to=config.APPROACH_ANGLE_MAX, 
                 orient=tk.HORIZONTAL, variable=self.approach_angle_var).pack(fill=tk.X, padx=5)
        self.approach_angle_label = ttk.Label(params_frame, text=f"{self.params['approach_angle']:.1f}°",
                                             font=("Arial", 8), foreground="blue")
        self.approach_angle_label.pack(anchor=tk.W, padx=5, pady=(0,3))
        
        # DSS (Seeker Engagement Distance)
        ttk.Label(params_frame, text="Seeker Distance (m):", font=("Arial", 9)).pack(anchor=tk.W, padx=5, pady=(5,0))
        self.dss_var = tk.DoubleVar(value=self.params['dss'])
        self.dss_var.trace('w', self.update_params)
        ttk.Scale(params_frame, from_=5000, to=50000, orient=tk.HORIZONTAL, 
                 variable=self.dss_var).pack(fill=tk.X, padx=5)
        self.dss_label = ttk.Label(params_frame, text=f"{self.params['dss']:.0f} m",
                                  font=("Arial", 8), foreground="blue")
        self.dss_label.pack(anchor=tk.W, padx=5, pady=(0,3))
        
        # L0 (Stabilization Distance)
        ttk.Label(params_frame, text="Stab. Distance (m):", font=("Arial", 9)).pack(anchor=tk.W, padx=5, pady=(5,0))
        self.l0_var = tk.DoubleVar(value=self.params['l0'])
        self.l0_var.trace('w', self.update_params)
        ttk.Scale(params_frame, from_=1000, to=15000, orient=tk.HORIZONTAL, 
                 variable=self.l0_var).pack(fill=tk.X, padx=5)
        self.l0_label = ttk.Label(params_frame, text=f"{self.params['l0']:.0f} m",
                                 font=("Arial", 8), foreground="blue")
        self.l0_label.pack(anchor=tk.W, padx=5, pady=(0,5))
        
        # === POINT SELECTION ===
        points_frame = ttk.LabelFrame(left_panel, text="📍 Points", padding=8)
        points_frame.pack(fill=tk.X, pady=8)
        
        self.start_btn = ttk.Button(points_frame, text="🎯 Launch Point",
                                   command=self.select_start_point)
        self.start_btn.pack(fill=tk.X, pady=3)
        self.start_label = ttk.Label(points_frame, text="Not set", foreground="red", font=("Arial", 8))
        self.start_label.pack(fill=tk.X, padx=5)
        
        self.goal_btn = ttk.Button(points_frame, text="🎯 Target Point",
                                  command=self.select_goal_point)
        self.goal_btn.pack(fill=tk.X, pady=3)
        self.goal_label = ttk.Label(points_frame, text="Not set", foreground="red", font=("Arial", 8))
        self.goal_label.pack(fill=tk.X, padx=5)
        
        # === OBSTACLES ===
        obstacles_frame = ttk.LabelFrame(left_panel, text="🏝️ Obstacles", padding=8)
        obstacles_frame.pack(fill=tk.X, pady=8)
        
        self.polygon_btn = ttk.Button(obstacles_frame, text="Draw Island",
                                     command=self.start_draw_polygon)
        self.polygon_btn.pack(fill=tk.X, pady=2)
        
        self.circle_btn = ttk.Button(obstacles_frame, text="Draw SAM Site",
                                    command=self.start_draw_circle)
        self.circle_btn.pack(fill=tk.X, pady=2)
        
        self.polygon_label = ttk.Label(obstacles_frame, text="Islands: 0", font=("Arial", 8))
        self.polygon_label.pack(fill=tk.X, padx=5)
        
        self.circle_label = ttk.Label(obstacles_frame, text="SAM Sites: 0", font=("Arial", 8))
        self.circle_label.pack(fill=tk.X, padx=5)
        
        # === CONTROL ===
        control_frame = ttk.LabelFrame(left_panel, text="🎮 Control", padding=8)
        control_frame.pack(fill=tk.X, pady=8)
        
        ttk.Button(control_frame, text="Clear Last",
                  command=self.clear_last_obstacle).pack(fill=tk.X, pady=2)
        ttk.Button(control_frame, text="Clear All",
                  command=self.clear_all).pack(fill=tk.X, pady=2)
        ttk.Button(control_frame, text="Reset All",
                  command=self.reset_scenario).pack(fill=tk.X, pady=2)
        
        # === EXECUTE ===
        exec_frame = ttk.LabelFrame(left_panel, text="▶️ Execute", padding=8)
        exec_frame.pack(fill=tk.X, pady=8)
        
        self.run_btn = ttk.Button(exec_frame, text="Run Planning",
                                 command=self.run_planning, state=tk.DISABLED)
        self.run_btn.pack(fill=tk.X, pady=5)
        
        # === STATUS ===
        status_frame = ttk.LabelFrame(left_panel, text="📋 Status", padding=8)
        status_frame.pack(fill=tk.BOTH, expand=True, pady=8)
        
        self.status_text = tk.Text(status_frame, height=15, width=32, 
                                  font=("Courier", 8), state=tk.DISABLED)
        self.status_text.pack(fill=tk.BOTH, expand=True)
        
        # === MIDDLE PANEL - Scenario Map ===
        middle_panel = ttk.LabelFrame(main_container, text="🗺️ Scenario Map", padding=5)
        middle_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.fig_scenario = Figure(figsize=(7.0, 9.5), dpi=100, tight_layout=True)
        self.ax_scenario = self.fig_scenario.add_subplot(111)
        self.fig_scenario.subplots_adjust(left=0.08, right=0.95, top=0.95, bottom=0.08)
        self.canvas_scenario = FigureCanvasTkAgg(self.fig_scenario, middle_panel)
        self.canvas_scenario.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.canvas_scenario.mpl_connect('button_press_event', self.on_canvas_click)
        self.canvas_scenario.mpl_connect('motion_notify_event', self.on_canvas_motion)
        
        # Initialize - only scenario map now
        self.redraw_scenario()
        
    def update_params(self, *args):
        """Update parameters from sliders"""
        self.params['turn_radius'] = self.turn_radius_var.get()
        self.params['max_turn_angle'] = self.max_angle_var.get()
        self.params['safe_margin'] = self.safe_margin_var.get()
        self.params['h_weight'] = self.h_weight_var.get()
        self.params['launch_angle'] = self.launch_angle_var.get()
        self.params['approach_angle'] = self.approach_angle_var.get()
        self.params['dss'] = self.dss_var.get()
        self.params['l0'] = self.l0_var.get()
        
        self.turn_radius_label.config(text=f"{self.params['turn_radius']:.0f} m")
        self.max_angle_label.config(text=f"{self.params['max_turn_angle']:.1f}°")
        self.safe_margin_label.config(text=f"{self.params['safe_margin']:.0f} m")
        self.h_weight_label.config(text=f"{self.params['h_weight']:.2f}")
        self.launch_angle_label.config(text=f"{self.params['launch_angle']:.1f}°")
        self.approach_angle_label.config(text=f"{self.params['approach_angle']:.1f}°")
        self.dss_label.config(text=f"{self.params['dss']:.0f} m")
        self.l0_label.config(text=f"{self.params['l0']:.0f} m")
        
    def log_status(self, message):
        """Log message to status area"""
        self.status_text.config(state=tk.NORMAL)
        self.status_text.insert(tk.END, message + "\n")
        self.status_text.see(tk.END)
        self.status_text.config(state=tk.DISABLED)
        self.root.update()
    
    def clear_status(self):
        """Clear status area"""
        self.status_text.config(state=tk.NORMAL)
        self.status_text.delete(1.0, tk.END)
        self.status_text.config(state=tk.DISABLED)
    
    def select_start_point(self):
        self.mode = "draw_start"
        self.log_status("🎯 Click on map to select launch point...")
    
    def select_goal_point(self):
        self.mode = "draw_goal"
        self.log_status("🎯 Click on map to select target point...")
    
    def start_draw_polygon(self):
        if not self.start_point or not self.goal_point:
            messagebox.showwarning("Warning", "Please select launch and target points first!")
            return
        self.mode = "draw_polygon"
        self.current_polygon = []
        self.log_status("🏝️ Click vertices. Close by clicking near first point.")
    
    def start_draw_circle(self):
        if not self.start_point or not self.goal_point:
            messagebox.showwarning("Warning", "Please select launch and target points first!")
            return
        self.mode = "draw_circle"
        self.current_circle_center = None
        self.drawing_radius = 0
        self.log_status("🛡️ Click center, move mouse, click to confirm.")
    
    def on_canvas_click(self, event):
        if event.xdata is None or event.ydata is None:
            return
        
        x, y = event.xdata, event.ydata
        
        if self.mode == "draw_start":
            self.start_point = (x, y)
            self.start_label.config(text=f"({x:.0f}, {y:.0f})", foreground="green")
            self.mode = "idle"
            self.log_status(f"✅ Launch: ({x:.0f}, {y:.0f})")
            self.update_buttons()
            self.redraw_scenario()
            
        elif self.mode == "draw_goal":
            self.goal_point = (x, y)
            self.goal_label.config(text=f"({x:.0f}, {y:.0f})", foreground="green")
            self.mode = "idle"
            self.log_status(f"✅ Target: ({x:.0f}, {y:.0f})")
            self.update_buttons()
            self.redraw_scenario()
            
        elif self.mode == "draw_polygon":
            if len(self.current_polygon) > 0:
                last_pt = self.current_polygon[-1]
                dist = math.sqrt((x - last_pt[0])**2 + (y - last_pt[1])**2)
                if dist < 5000 and len(self.current_polygon) >= 3:
                    self.obstacles.append({
                        'type': 'polygon',
                        'data': self.current_polygon
                    })
                    self.current_polygon = []
                    self.polygon_label.config(
                        text=f"Islands: {sum(1 for o in self.obstacles if o['type'] == 'polygon')}"
                    )
                    self.log_status(f"✅ Island added ({len(self.obstacles)} total)")
                    self.mode = "idle"
                    self.redraw_scenario()
                    return
            
            self.current_polygon.append((x, y))
            self.log_status(f"  Vertex {len(self.current_polygon)}: ({x:.0f}, {y:.0f})")
            self.redraw_scenario()
            
        elif self.mode == "draw_circle":
            if self.current_circle_center is None:
                self.current_circle_center = (x, y)
                self.log_status(f"  Center: ({x:.0f}, {y:.0f}). Move mouse to set radius.")
            else:
                radius = math.sqrt((x - self.current_circle_center[0])**2 + 
                                 (y - self.current_circle_center[1])**2)
                self.obstacles.append({
                    'type': 'circle',
                    'data': {'center': self.current_circle_center, 'radius': radius}
                })
                self.current_circle_center = None
                self.circle_label.config(
                    text=f"SAM Sites: {sum(1 for o in self.obstacles if o['type'] == 'circle')}"
                )
                self.log_status(f"✅ SAM site added ({len(self.obstacles)} total)")
                self.mode = "idle"
                self.redraw_scenario()
    
    def on_canvas_motion(self, event):
        if event.xdata is None or event.ydata is None:
            return
        if self.mode == "draw_circle" and self.current_circle_center:
            self.drawing_radius = math.sqrt((event.xdata - self.current_circle_center[0])**2 + 
                                           (event.ydata - self.current_circle_center[1])**2)
            self.redraw_scenario()
    
    def clear_last_obstacle(self):
        if self.obstacles:
            self.obstacles.pop()
            self.polygon_label.config(
                text=f"Islands: {sum(1 for o in self.obstacles if o['type'] == 'polygon')}"
            )
            self.circle_label.config(
                text=f"SAM Sites: {sum(1 for o in self.obstacles if o['type'] == 'circle')}"
            )
            self.log_status("🗑️ Last obstacle removed")
            self.redraw_scenario()
    
    def clear_all(self):
        self.obstacles = []
        self.current_polygon = []
        self.current_circle_center = None
        self.polygon_label.config(text="Islands: 0")
        self.circle_label.config(text="SAM Sites: 0")
        self.log_status("🗑️ All obstacles cleared")
        self.redraw_scenario()
    
    def reset_scenario(self):
        self.start_point = None
        self.goal_point = None
        self.obstacles = []
        self.current_polygon = []
        self.current_circle_center = None
        self.mode = "idle"
        self.last_result = None
        
        self.start_label.config(text="Not set", foreground="red")
        self.goal_label.config(text="Not set", foreground="red")
        self.polygon_label.config(text="Islands: 0")
        self.circle_label.config(text="SAM Sites: 0")
        
        self.clear_status()
        self.log_status("🔄 Scenario reset.")
        self.update_buttons()
        self.redraw_scenario()
    
    def update_buttons(self):
        if self.start_point and self.goal_point:
            self.polygon_btn.config(state=tk.NORMAL)
            self.circle_btn.config(state=tk.NORMAL)
            self.run_btn.config(state=tk.NORMAL)
        else:
            self.polygon_btn.config(state=tk.DISABLED)
            self.circle_btn.config(state=tk.DISABLED)
            self.run_btn.config(state=tk.DISABLED)
    
    def redraw_scenario(self):
        """Redraw scenario map with trajectory result overlaid"""
        self.ax_scenario.clear()
        
        # Set axis FIRST - before drawing anything to prevent auto-scaling
        self.ax_scenario.set_xlim(0, config.MAP_WIDTH)
        self.ax_scenario.set_ylim(0, config.MAP_HEIGHT)
        self.ax_scenario.set_aspect('equal')
        self.ax_scenario.autoscale_view(scalex=False, scaley=False)
        self.ax_scenario.set_xlabel("X (m)", fontsize=9)
        self.ax_scenario.set_ylabel("Y (m)", fontsize=9)
        
        # Set title based on whether we have results
        if self.last_result and self.last_result['success']:
            self.ax_scenario.set_title("Scenario Map - ✅ Trajectory Planned", fontsize=10, fontweight='bold')
        else:
            self.ax_scenario.set_title("Scenario Map", fontsize=10, fontweight='bold')
        
        self.ax_scenario.grid(True, alpha=0.2, linestyle='--')
        
        # Draw trajectory if available (before obstacles so they appear on top)
        if self.last_result and self.last_result['success']:
            if self.last_result.get('dubins_path'):
                dubins_points = self.last_result['dubins_path']
                xs = [p[0] for p in dubins_points]
                ys = [p[1] for p in dubins_points]
                self.ax_scenario.plot(xs, ys, 'b-', linewidth=2.0, label='Trajectory', zorder=3)
            else:
                path = self.last_result['path']
                waypoints = [wp for wp, heading in path]
                
                # Draw connecting lines: start → waypoint[0] → ... → waypoint[-1] → goal
                if waypoints and self.start_point and self.goal_point:
                    # Line from start point to first waypoint
                    self.ax_scenario.plot([self.start_point[0], waypoints[0][0]], 
                                        [self.start_point[1], waypoints[0][1]], 
                                        'b--', linewidth=1.5, alpha=0.7, zorder=2)
                    
                    # Waypoints trajectory
                    xs = [p[0] for p in waypoints]
                    ys = [p[1] for p in waypoints]
                    self.ax_scenario.plot(xs, ys, 'b-', linewidth=2.0, label='Trajectory', zorder=3)
                    self.ax_scenario.plot(xs, ys, 'bo', markersize=3, zorder=4)
                    
                    # Line from last waypoint to goal point
                    self.ax_scenario.plot([waypoints[-1][0], self.goal_point[0]], 
                                        [waypoints[-1][1], self.goal_point[1]], 
                                        'b--', linewidth=1.5, alpha=0.7, zorder=2)
        
        # Draw obstacles
        for obs in self.obstacles:
            if obs['type'] == 'polygon':
                polygon = obs['data']
                if polygon:
                    xs = [p[0] for p in polygon] + [polygon[0][0]]
                    ys = [p[1] for p in polygon] + [polygon[0][1]]
                    self.ax_scenario.fill(xs, ys, 'brown', alpha=0.3, edgecolor='brown', linewidth=1.5)
                    
            elif obs['type'] == 'circle':
                center = obs['data']['center']
                radius = obs['data']['radius']
                circle = plt.Circle(center, radius, fill=False, edgecolor='red', linewidth=1.5, linestyle='--')
                self.ax_scenario.add_patch(circle)
                self.ax_scenario.plot(center[0], center[1], 'r+', markersize=5, markeredgewidth=0.8)
        
        # Draw inflated obstacles if available (from preprocessing)
        if self.preprocessed and 'obstacles' in self.preprocessed:
            for obs in self.preprocessed['obstacles']:
                if obs['type'] == 'polygon':
                    polygon = obs['polygon']
                    if polygon:
                        xs = [p[0] for p in polygon] + [polygon[0][0]]
                        ys = [p[1] for p in polygon] + [polygon[0][1]]
                        self.ax_scenario.fill(xs, ys, 'red', alpha=0.1, edgecolor='red', linewidth=1.0, linestyle=':', label='Inflated Obstacle')
                        
                elif obs['type'] == 'circle':
                    center = obs['center']
                    radius = obs['radius']
                    circle = plt.Circle(center, radius, fill=False, edgecolor='red', linewidth=1.0, linestyle=':', alpha=0.6)
                    self.ax_scenario.add_patch(circle)
        
        # Current polygon being drawn
        if self.current_polygon:
            xs = [p[0] for p in self.current_polygon]
            ys = [p[1] for p in self.current_polygon]
            self.ax_scenario.plot(xs, ys, 'b-', linewidth=1.5, alpha=0.7)
            self.ax_scenario.plot(xs, ys, 'bo', markersize=3)
        
        # Circle being drawn
        if self.mode == "draw_circle" and self.current_circle_center and self.drawing_radius > 0:
            circle = plt.Circle(self.current_circle_center, self.drawing_radius, 
                              fill=False, edgecolor='blue', linewidth=1.5, linestyle=':')
            self.ax_scenario.add_patch(circle)
            self.ax_scenario.plot(self.current_circle_center[0], self.current_circle_center[1], 
                        'b+', markersize=5, markeredgewidth=0.8)
        
        # Points (smaller markers)
        legend_elements = []
        if self.start_point:
            self.ax_scenario.plot(self.start_point[0], self.start_point[1], 'g^', markersize=7, zorder=5)
            legend_elements.append(plt.Line2D([0], [0], marker='^', color='w', markerfacecolor='g', markersize=7, label='Launch'))
        
        if self.goal_point:
            self.ax_scenario.plot(self.goal_point[0], self.goal_point[1], 'r*', markersize=9, zorder=5)
            legend_elements.append(plt.Line2D([0], [0], marker='*', color='w', markerfacecolor='r', markersize=9, label='Target'))
        
        if self.last_result and self.last_result['success']:
            legend_elements.insert(0, plt.Line2D([0], [0], color='b', linewidth=2, label='Trajectory'))
        
        if legend_elements:
            self.ax_scenario.legend(handles=legend_elements, loc='upper left', fontsize=8)
        
        # Don't call tight_layout() every redraw - it causes size flickering
        # Use fixed margins instead
        self.fig_scenario.subplots_adjust(left=0.08, right=0.95, top=0.95, bottom=0.08)
        self.canvas_scenario.draw()
    
    
    def run_planning(self):
        """Spawn background thread for path planning"""
        if not self.start_point or not self.goal_point:
            messagebox.showerror("Error", "Please set launch and target points!")
            return
        
        # Disable run button during planning
        self.run_btn.config(state=tk.DISABLED, text="⏳ Planning...")
        
        # Start planning in background thread
        planning_thread = threading.Thread(target=self._planning_worker, daemon=True)
        planning_thread.start()
    
    def _planning_worker(self):
        """Worker thread for path planning (runs in background)"""
        self.log_status("\n" + "="*35)
        self.log_status("🚀 Running path planning...")
        
        try:
            # Create scenario
            scenario = {
                'start': self.start_point,
                'start_heading': math.pi / 4,
                'goal': self.goal_point,
                'goal_heading': math.pi / 4,
                'islands': [obs['data'] for obs in self.obstacles if obs['type'] == 'polygon'],
                'sam_sites': [(obs['data']['center'], obs['data']['radius']) 
                             for obs in self.obstacles if obs['type'] == 'circle'],
                'obstacles': []
            }
            
            # Convert to obstacle format
            for obs in self.obstacles:
                if obs['type'] == 'polygon':
                    scenario['obstacles'].append({
                        'type': 'polygon',
                        'polygon': obs['data']
                    })
                elif obs['type'] == 'circle':
                    scenario['obstacles'].append({
                        'type': 'circle',
                        'center': obs['data']['center'],
                        'radius': obs['data']['radius']
                    })
            
            self.log_status(f"📊 Obstacles: {len(scenario['obstacles'])}")
            self.log_status(f"⚙️ R={self.params['turn_radius']:.0f}m, α_max={self.params['max_turn_angle']:.1f}°")
            
            # Preprocess with current parameters from GUI
            self.log_status("⚙️ Preprocessing...")
            self.preprocessed = prep.prepare_scenario(
                scenario, 
                R=self.params['turn_radius'],
                L0=self.params['l0'],
                DSS=self.params['dss'],
                launch_angle=self.params['launch_angle'],
                approach_angle=self.params['approach_angle']
            )
            
            # Plan
            self.log_status("🔍 Planning...")
            start_time = time.time()
            result = astar.plan_trajectory(self.preprocessed, verbose=False)
            elapsed_time = time.time() - start_time
            
            if result['success']:
                path = result['path']
                waypoints = [wp for wp, heading in path]
                
                # Compute Dubins curves
                self.log_status("🎯 Dubins curves...")
                try:
                    dubins_points = dc.sample_all_dubins_paths(path, 30)
                    result['dubins_path'] = dubins_points
                except Exception as e:
                    self.log_status(f"  ⚠️ Dubins error")
                
                # Calculate distance
                total_dist = sum(math.sqrt((waypoints[i+1][0] - waypoints[i][0])**2 + 
                                          (waypoints[i+1][1] - waypoints[i][1])**2)
                               for i in range(len(waypoints) - 1))
                
                self.log_status(f"\n✅ SUCCESS!")
                self.log_status(f"⏱️  {elapsed_time:.3f}s")
                self.log_status(f"📍 {len(waypoints)} waypoints")
                self.log_status(f"📏 {total_dist/1000:.2f} km")
                self.log_status(f"🔍 {result['stats']['iterations']} iterations")
                
                self.last_result = result
                
            else:
                self.log_status(f"\n❌ FAILED")
                self.log_status(f"⏱️  {elapsed_time:.3f}s")
                self.log_status(f"🔍 {result['stats']['iterations']} iterations")
                self.last_result = result
                
        except Exception as e:
            self.log_status(f"\n❌ ERROR: {str(e)[:80]}")
            self.log_status(f"Traceback: {str(e)}")
            self.last_result = {'success': False}
        
        finally:
            # Schedule GUI update on main thread
            self.root.after(0, self._planning_complete)


    def _planning_complete(self):
        """Callback after planning completes (runs on main thread)"""
        # Re-enable run button
        self.run_btn.config(state=tk.NORMAL, text="▶️ Run Planning")
        
        # Update scenario map with results
        self.redraw_scenario()


def main():
    root = tk.Tk()
    app = ScenarioBuilder(root)
    root.mainloop()


if __name__ == "__main__":
    main()
