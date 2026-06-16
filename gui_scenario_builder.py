"""
Interactive GUI Scenario Builder
Allows interactive creation of scenarios with point-and-click interface
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
    """Interactive scenario builder with GUI"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Missile Path Planning - Interactive Scenario Builder")
        self.root.geometry("1600x900")
        
        # Scenario data
        self.start_point = None
        self.goal_point = None
        self.obstacles = []  # List of {'type': 'polygon'/'circle', 'data': [...]}
        self.current_polygon = []  # Points for current polygon being drawn
        self.current_circle_center = None
        self.last_result = None
        self.preprocessed = None
        
        # UI State
        self.mode = "idle"  # idle, draw_start, draw_goal, draw_polygon, draw_circle
        self.drawing_radius = 0

        # Paramerters (configurable on GUI)
        self.params = {
            'turn_radius': config.R,
            'max_turn_angle': config.ALPHA_MAX,
            'safe_margin': config.SAFE_MARGIN,
            'h_weight': config.HEURISTIC_WEIGHT,
            'launch_angle': config.LAUNCH_ANGLE_DEFAULT,
            'approach_angle': config.APPROACH_ANGLE_DEFAULT,
            'dss': config.DSS,
            'l0': config.L0
        }
        
        # Create UI
        self.create_ui()
        
    def create_ui(self):
        """Create the user interface"""
        
        # Left panel - Controls
        left_panel_container = ttk.Frame(self.root, width=280)
        left_panel_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=10, pady=10)
        left_panel_container.pack_propagate(False)
        
        # Title
        title_label = ttk.Label(left_panel_container, text="Scenario Builder", 
                               font=("Arial", 14, "bold"))
        title_label.pack(pady=10)

        # Create scrollable canvas for all left panel content
        left_canvas = tk.Canvas(left_panel_container, bg='white', highlightthickness=0)
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
        params_frame = ttk.LabelFrame(left_panel, text="Parameters", padding=10)
        params_frame.pack(fill=tk.X, pady=10, padx=5)

        # --- PARAMETERS CONTENT ---
        # Turn Radius
        ttk.Label(params_frame, text="Turn Radius (m):", font=("Arial", 9)).pack(anchor=tk.W, padx=5, pady=(5, 0))
        self.turn_radius_var = tk.DoubleVar(value=self.params['turn_radius'])
        self.turn_radius_var.trace_add('write', self.update_params)
        ttk.Scale(params_frame, from_=3000, to=10000, orient=tk.HORIZONTAL, variable=self.turn_radius_var).pack(fill=tk.X, padx=5)
        self.turn_radius_label = ttk.Label(params_frame, text=f"{self.params['turn_radius']:.0f} m", 
                                           font=("Arial", 8), foreground="blue")
        self.turn_radius_label.pack(anchor=tk.W, padx=5, pady=(0, 3))

        # Max Turn Angle
        ttk.Label(params_frame, text="Max Turn Angle:", font=("Arial", 8)).pack(anchor=tk.W, padx=5, pady=(5, 0))
        self.max_angle_var = tk.DoubleVar(value=self.params['max_turn_angle'])
        self.max_angle_var.trace_add('write', self.update_params)
        ttk.Scale(params_frame, from_=30, to=90, orient=tk.HORIZONTAL, variable=self.max_angle_var).pack(fill=tk.X, padx=5)
        self.max_angle_label = ttk.Label(params_frame, text=f"{self.params['max_turn_angle']:.1f}",
                                         font=("Arial", 8), foreground="blue")
        self.max_angle_label.pack(anchor=tk.W, padx=5, pady=(0, 3))

        # Safe Margin
        ttk.Label(params_frame, text="Safe Margin (m):", font=("Arial", 9)).pack(anchor=tk.W, padx=5, pady=(5, 0))
        self.safe_margin_var = tk.DoubleVar(value=self.params['safe_margin'])
        self.safe_margin_var.trace_add("write", self.update_params)
        ttk.Scale(params_frame, from_=1000, to=10000, orient=tk.HORIZONTAL, variable=self.safe_margin_var).pack(fill=tk.X, padx=5)
        self.safe_margin_label = ttk.Label(params_frame, text=f"{self.params['safe_margin']:.0f} m",
                                         font=("Arial", 8), foreground="blue")
        self.safe_margin_label.pack(anchor=tk.W, padx=5, pady=(0, 3))

        # Launch Angle
        ttk.Label(params_frame, text="Launch Angle:", font=("Arial", 9)).pack(anchor=tk.W, padx=5, pady=(5, 0))
        self.launch_angle_var = tk.DoubleVar(value=self.params['launch_angle'])
        self.launch_angle_var.trace_add("write", self.update_params)
        ttk.Scale(params_frame, from_=config.LAUNCH_ANGLE_MIN, to=config.LAUNCH_ANGLE_MAX,
                  orient=tk.HORIZONTAL, variable=self.launch_angle_var).pack(fill=tk.X, padx=5)
        self.launch_angle_label = ttk.Label(params_frame, text=f"{self.params['launch_angle']:.1f}",
                                            font=("Arial", 8), foreground="blue")
        self.launch_angle_label.pack(anchor=tk.W, padx=5, pady=(0, 3))

        # Approach Angle
        ttk.Label(params_frame, text="Approach Angle:", font=("Arial", 9)).pack(anchor=tk.W, padx=5, pady=(5, 0))
        self.approach_angle_var = tk.DoubleVar(value=self.params['approach_angle'])
        self.approach_angle_var.trace_add("write", self.update_params)
        ttk.Scale(params_frame, from_=config.APPROACH_ANGLE_MIN, to=config.APPROACH_ANGLE_MAX,
                  orient=tk.HORIZONTAL, variable=self.approach_angle_var).pack(fill=tk.X, padx=5)
        self.approach_angle_label = ttk.Label(params_frame, text=f"{self.params['approach_angle']:.1f}",
                                              font=("Arial", 8), foreground="blue")
        self.approach_angle_label.pack(anchor=tk.W, padx=5, pady=(0, 3))

        # DSS
        ttk.Label(params_frame, text="Seeker Distance (m):", font=("Arial", 9)).pack(anchor=tk.W, padx=5, pady=(5, 0))
        self.dss_var = tk.DoubleVar(value=self.params['dss'])
        self.dss_var.trace_add("write", self.update_params)
        ttk.Scale(params_frame, from_=5000, to=50000, orient=tk.HORIZONTAL, variable=self.dss_var).pack(fill=tk.X, padx=5)
        self.dss_label = ttk.Label(params_frame, text=f"{self.params['dss']:.0f} m",
                                   font=("Arial", 8), foreground="blue")
        self.dss_label.pack(anchor=tk.W, padx=5, pady=(0, 3))

        # L0
        ttk.Label(params_frame, text="L0 (m):", font=("Arial", 9)).pack(anchor=tk.W, padx=5, pady=(5, 0))
        self.l0_var = tk.DoubleVar(value=self.params['l0'])
        self.l0_var.trace_add("write", self.update_params)
        ttk.Scale(params_frame, from_=1000, to=10000, orient=tk.HORIZONTAL, variable=self.l0_var).pack(fill=tk.X, padx=5)
        self.l0_label = ttk.Label(params_frame, text=f"{self.params['l0']:.0f} m", 
                                  font=("Arial", 8), foreground="blue")
        self.l0_label.pack(anchor=tk.W, padx=5, pady=(0, 5))
        
        # Point selection
        points_frame = ttk.LabelFrame(left_panel, text="Points", padding=10)
        points_frame.pack(fill=tk.X, padx=5, pady=10)
        
        self.start_btn = ttk.Button(points_frame, text="Select Launch Point",
                                   command=self.select_start_point)
        self.start_btn.pack(fill=tk.X, pady=5)
        self.start_label = ttk.Label(points_frame, text="Launch: Not set", 
                                    foreground="red")
        self.start_label.pack(fill=tk.X, pady=2)
        
        self.goal_btn = ttk.Button(points_frame, text="Select Target Point",
                                  command=self.select_goal_point)
        self.goal_btn.pack(fill=tk.X, pady=5)
        self.goal_label = ttk.Label(points_frame, text="Target: Not set",
                                   foreground="red")
        self.goal_label.pack(fill=tk.X, pady=2)
        
        # Obstacles
        obstacles_frame = ttk.LabelFrame(left_panel, text="Obstacles", padding=10)
        obstacles_frame.pack(fill=tk.X, padx=5, pady=10)
        
        self.polygon_btn = ttk.Button(obstacles_frame, text="Draw Island (Polygon)",
                                     command=self.start_draw_polygon)
        self.polygon_btn.pack(fill=tk.X, pady=5)
        
        self.circle_btn = ttk.Button(obstacles_frame, text="Draw SAM Site (Circle)",
                                    command=self.start_draw_circle)
        self.circle_btn.pack(fill=tk.X, pady=5)
        
        self.polygon_label = ttk.Label(obstacles_frame, text="Polygons: 0")
        self.polygon_label.pack(fill=tk.X, pady=2)
        
        self.circle_label = ttk.Label(obstacles_frame, text="Circles: 0")
        self.circle_label.pack(fill=tk.X, pady=2)
        
        # === CONTROL ===
        control_frame = ttk.LabelFrame(left_panel, text="Control", padding=10)
        control_frame.pack(fill=tk.X, padx=5, pady=10)
        
        ttk.Button(control_frame, text="Clear Last Obstacle",
                  command=self.clear_last_obstacle).pack(fill=tk.X, pady=3)
        ttk.Button(control_frame, text="Clear All",
                  command=self.clear_all).pack(fill=tk.X, pady=3)
        ttk.Button(control_frame, text="Reset Scenario",
                  command=self.reset_scenario).pack(fill=tk.X, pady=3)
        
        # === Execute ===
        exec_frame = ttk.LabelFrame(left_panel, text="Execute", padding=10)
        exec_frame.pack(fill=tk.X, padx=5, pady=10)
        
        self.run_btn = ttk.Button(exec_frame, text="Run Path Planning",
                                 command=self.run_planning, state=tk.DISABLED)
        self.run_btn.pack(fill=tk.X, pady=5)
        
        # === Status ===
        status_frame = ttk.LabelFrame(left_panel, text="Status", padding=10)
        status_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=10)
        
        self.status_text = tk.Text(status_frame, height=20, width=35, 
                                  font=("Courier", 8), state=tk.DISABLED)
        self.status_text.pack(fill=tk.BOTH, expand=True)
        
        # scrollbar = ttk.Scrollbar(status_frame, orient=tk.VERTICAL,
        #                          command=self.status_text.yview)
        # self.status_text.config(yscroll=scrollbar.set)
        
        # Right panel - Canvas
        right_panel = ttk.Frame(self.root)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=30, pady=10)
        
        # Create matplotlib figure
        self.fig = Figure(layout="tight", dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, right_panel)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Bind events
        self.canvas.mpl_connect('button_press_event', self.on_canvas_click)
        self.canvas.mpl_connect('motion_notify_event', self.on_canvas_motion)
        
        # Initialize plot
        self.redraw_canvas()

    def update_params(self, *args):
        """Update parameters from sliders"""
        self.params['turn_radius'] = self.turn_radius_var.get()
        self.params['max_turn_angle'] = self.max_angle_var.get()
        self.params['safe_margin'] = self.safe_margin_var.get()
        self.params['launch_angle'] = self.launch_angle_var.get()
        self.params['approach_angle'] = self.approach_angle_var.get()
        self.params['dss'] = self.dss_var.get()
        self.params['l0'] = self.l0_var.get()
        
        self.turn_radius_label.config(text=f"{self.params['turn_radius']:.0f} m")
        self.max_angle_label.config(text=f"{self.params['max_turn_angle']:.1f}")
        self.safe_margin_label.config(text=f"{self.params['safe_margin']:.0f} m")
        self.launch_angle_label.config(text=f"{self.params['launch_angle']:.1f}")
        self.approach_angle_label.config(text=f"{self.params['approach_angle']:.1f}")
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
        """Start selection of launch point"""
        self.mode = "draw_start"
        self.log_status("🎯 Click on map to select launch point...")
    
    def select_goal_point(self):
        """Start selection of target point"""
        self.mode = "draw_goal"
        self.log_status("🎯 Click on map to select target point...")
    
    def start_draw_polygon(self):
        """Start drawing a polygon"""
        if not self.start_point or not self.goal_point:
            messagebox.showwarning("Warning", "Please select launch and target points first!")
            return
        
        self.mode = "draw_polygon"
        self.current_polygon = []
        self.log_status("Click to add polygon vertices. Double-click to finish.")
    
    def start_draw_circle(self):
        """Start drawing a circle"""
        if not self.start_point or not self.goal_point:
            messagebox.showwarning("Warning", "Please select launch and target points first!")
            return
        
        self.mode = "draw_circle"
        self.current_circle_center = None
        self.drawing_radius = 0
        self.log_status("Click center, then move mouse to set radius. Click again to confirm.")
    
    def on_canvas_click(self, event):
        """Handle canvas click events"""
        if event.xdata is None or event.ydata is None:
            return
        
        x, y = event.xdata, event.ydata
        
        if self.mode == "draw_start":
            self.start_point = (x, y)
            self.start_label.config(text=f"Launch: ({x:.0f}, {y:.0f})", foreground="green")
            self.mode = "idle"
            self.log_status(f"Launch point set: ({x:.0f}, {y:.0f})")
            self.update_buttons()
            self.redraw_canvas()
            
        elif self.mode == "draw_goal":
            self.goal_point = (x, y)
            self.goal_label.config(text=f"Target: ({x:.0f}, {y:.0f})", foreground="green")
            self.mode = "idle"
            self.log_status(f"Target point set: ({x:.0f}, {y:.0f})")
            self.update_buttons()
            self.redraw_canvas()
            
        elif self.mode == "draw_polygon":
            # Check for double-click to finish
            if len(self.current_polygon) > 0:
                last_pt = self.current_polygon[-1]
                dist = math.sqrt((x - last_pt[0])**2 + (y - last_pt[1])**2)
                if dist < 5000:  # Close to first point
                    if len(self.current_polygon) >= 3:
                        self.obstacles.append({
                            'type': 'polygon',
                            'data': self.current_polygon
                        })
                        self.current_polygon = []
                        self.polygon_label.config(
                            text=f"Islands: {sum(1 for o in self.obstacles if o['type'] == 'polygon')}"
                        )
                        self.log_status(f"Island added ({len(self.obstacles)} total)")
                        self.mode = "idle"
                        self.redraw_canvas()
                        return
            
            self.current_polygon.append((x, y))
            self.log_status(f"  Vertex added: ({x:.0f}, {y:.0f}) - {len(self.current_polygon)} vertices")
            self.redraw_canvas()
            
        elif self.mode == "draw_circle":
            if self.current_circle_center is None:
                self.current_circle_center = (x, y)
                self.log_status(f"  Circle center: ({x:.0f}, {y:.0f}). Move mouse to set radius.")
            else:
                # Finish circle
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
                self.log_status(f"SAM site added ({len(self.obstacles)} total)")
                self.mode = "idle"
                self.redraw_canvas()
    
    def on_canvas_motion(self, event):
        """Handle mouse motion for circle drawing"""
        if event.xdata is None or event.ydata is None:
            return
        
        if self.mode == "draw_circle" and self.current_circle_center:
            self.drawing_radius = math.sqrt((event.xdata - self.current_circle_center[0])**2 + 
                                           (event.ydata - self.current_circle_center[1])**2)
            self.redraw_canvas()
    
    def clear_last_obstacle(self):
        """Remove last obstacle"""
        if self.obstacles:
            self.obstacles.pop()
            self.polygon_label.config(
                text=f"Islands: {sum(1 for o in self.obstacles if o['type'] == 'polygon')}"
            )
            self.circle_label.config(
                text=f"SAM Sites: {sum(1 for o in self.obstacles if o['type'] == 'circle')}"
            )
            self.log_status("Last obstacle removed")
            self.redraw_canvas()
    
    def clear_all(self):
        """Clear all obstacles"""
        self.obstacles = []
        self.current_polygon = []
        self.current_circle_center = None
        self.last_result = None
        self.preprocessed = None
        self.polygon_label.config(text="Islands: 0")
        self.circle_label.config(text="SAM Sites: 0")
        self.log_status("All obstacles cleared")
        self.redraw_canvas()
    
    def reset_scenario(self):
        """Reset everything"""
        self.start_point = None
        self.goal_point = None
        self.obstacles = []
        self.current_polygon = []
        self.current_circle_center = None
        self.mode = "idle"
        self.last_result = None
        self.preprocessed = None

        self.start_label.config(text="Launch: Not set", foreground="red")
        self.goal_label.config(text="Target: Not set", foreground="red")
        self.polygon_label.config(text="Islands: 0")
        self.circle_label.config(text="SAM Sites: 0")
        
        self.clear_status()
        self.log_status("Scenario reset. Ready for new scenario.")
        self.update_buttons()
        self.redraw_canvas()
    
    def update_buttons(self):
        """Update button states"""
        if self.start_point and self.goal_point:
            self.polygon_btn.config(state=tk.NORMAL)
            self.circle_btn.config(state=tk.NORMAL)
            self.run_btn.config(state=tk.NORMAL)
        else:
            self.polygon_btn.config(state=tk.DISABLED)
            self.circle_btn.config(state=tk.DISABLED)
            self.run_btn.config(state=tk.DISABLED)
    
    def redraw_canvas(self):
        """Redraw scenario map with trajectory result overlaid"""
        self.ax.clear()
        
        # Set map bounds
        self.ax.set_xlim(0, config.MAP_WIDTH)
        self.ax.set_ylim(0, config.MAP_HEIGHT)
        self.ax.set_aspect('equal')
        self.ax.autoscale_view(scalex=False, scaley=False)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")

        # Set title based on whether we have results
        if self.last_result and self.last_result['success']:
            self.ax.set_title("Scenario Map - Trajectory Planned", fontsize=10, fontweight='bold')
        else:
            self.ax.set_title("Scenario Map", fontsize=10, fontweight='bold')
        
        self.ax.grid(True, alpha=0.2, linestyle='--')

        # Draw trajectory if available (before obstacles so they appear on top)
        if self.last_result and self.last_result['success']:
            if self.last_result.get('dubins_path'):
                dubins_points = self.last_result['dubins_path']
                xs = [p[0] for p in dubins_points]
                ys = [p[1] for p in dubins_points]
                self.ax.plot(xs, ys, 'b-', linewidth=2.0, label='Trajectory', zorder=3)
            else:
                path = self.last_result['path']
                waypoints = [wp for wp, heading in path]

                if waypoints and self.start_point and self.goal_point:
                    # Line form start point to first watypoint
                    self.ax.plot([self.start_point[0], waypoints[0][0]], 
                                 [self.start_point[1], waypoints[0][1]],
                                 'b--', linewidth=1.5, alpha=0.7, zorder=2)

                    # Waypoints trajectory
                    xs = [p[0] for p in waypoints]
                    ys = [p[1] for p in waypoints]
                    self.ax.plot(xs, ys, 'b-', linewidth=2.0, label='Trajectory', zorder=3)
                    self.ax.plot(xs, ys, 'bo', markersize=3, zorder=4)

                    # Line from last waypoint to goal point
                    self.ax.plot([waypoints[-1][0], self.goal_point[0]],
                                 [waypoints[-1][1], self.goal_point[1]],
                                 'b--', linewidth=1.5, alpha=0.7, zorder=2) 

        # Draw obstacles
        for obs in self.obstacles:
            if obs['type'] == 'polygon':
                polygon = obs['data']
                if polygon:
                    xs = [p[0] for p in polygon] + [polygon[0][0]]
                    ys = [p[1] for p in polygon] + [polygon[0][1]]
                    self.ax.fill(xs, ys, 'brown', alpha=0.3, edgecolor='brown', linewidth=2)
                    # self.ax.plot(xs, ys, 'brown', linewidth=2, label='Island' if obs == self.obstacles[0] else '')
                    
            elif obs['type'] == 'circle':
                center = obs['data']['center']
                radius = obs['data']['radius']
                circle = plt.Circle(center, radius, fill=False, edgecolor='red', linewidth=2, linestyle='--')
                self.ax.add_patch(circle)
                self.ax.plot(center[0], center[1], 'r+', markersize=10, markeredgewidth=2)
        
        # Draw inflated obstacles if available (from preprocessing)
        if self.preprocessed and 'obstacles' in self.preprocessed:
            for obs in self.preprocessed['obstacles']:
                if obs['type'] == 'polygon':
                    polygon = obs['polygon']
                    if polygon:
                        xs = [p[0] for p in polygon] + [polygon[0][0]]
                        ys = [p[1] for p in polygon] + [polygon[0][1]]
                        self.ax.fill(xs, ys, 'red', alpha=0.1, edgecolor='red', linewidth=1.0, linestyle=':', label='Inflated Obstacle')
                elif obs['type'] == 'circle':
                    center = obs['center']
                    radius = obs['radius']
                    circle = plt.Circle(center, radius, fill=False, edgecolor='red', linewidth=1.0, linestyle=':', alpha=0.6)
                    self.ax.add_patch(circle)
        
        # Draw current polygon being drawn
        if self.current_polygon:
            xs = [p[0] for p in self.current_polygon]
            ys = [p[1] for p in self.current_polygon]
            self.ax.plot(xs, ys, 'b-', linewidth=2, alpha=0.7)
            self.ax.plot(xs, ys, 'bo', markersize=6)
        
        # Draw circle being drawn
        if self.mode == "draw_circle" and self.current_circle_center and self.drawing_radius > 0:
            circle = plt.Circle(self.current_circle_center, self.drawing_radius, 
                              fill=False, edgecolor='blue', linewidth=2, linestyle=':')
            self.ax.add_patch(circle)
            self.ax.plot(self.current_circle_center[0], self.current_circle_center[1], 
                        'b+', markersize=10, markeredgewidth=2)
        
        # Draw start point
        if self.start_point:
            self.ax.plot(self.start_point[0], self.start_point[1], 'g^', markersize=8, 
                        label='Launch Point', zorder=5)
            self.ax.text(self.start_point[0], self.start_point[1] - 20000, 'START',
                        ha='center', fontsize=10, color='green', fontweight='bold')
        
        # Draw goal point
        if self.goal_point:
            self.ax.plot(self.goal_point[0], self.goal_point[1], 'r*', markersize=8,
                        label='Target', zorder=5)
            self.ax.text(self.goal_point[0], self.goal_point[1] + 20000, 'TARGET',
                        ha='center', fontsize=10, color='red', fontweight='bold')
        
        self.ax.legend(loc='upper left')
        # self.fig.tight_layout()
        self.canvas.draw()
    
    def run_planning(self):
        """Run path planning with current scenario"""
        if not self.start_point or not self.goal_point:
            messagebox.showerror("Error", "Please set launch and target points!")
            return
        
        # Disable run button during planning
        self.run_btn.config(state=tk.DISABLED, text="Planning...")

        # Start planning in background thread

        planning_thread = threading.Thread(target=self._planning_worker, daemon=True)
        planning_thread.start()
    
    def _planning_worker(self):
        """Worker thread for path planning (run in backgroud)"""
        self.log_status("\n" + "="*40)
        self.log_status("Running path planning...")
        
        try:
            # Create scenario
            scenario = {
                'start': self.start_point,
                'start_heading': config.deg_to_rad(self.params['launch_angle']),
                'goal': self.goal_point,
                'goal_heading': config.deg_to_rad(self.params['approach_angle']),
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
            
            self.log_status(f"Obstacles: {len(scenario['obstacles'])}")
            self.log_status(f"R={self.params['turn_radius']:.0f} m, alpha_max={self.params['max_turn_angle']:.1f}")
            
            # Preprocess
            self.log_status("Preprocessing...")
            self.preprocessed = prep.prepare_scenario(
                scenario,
                R=self.params['turn_radius'],
                L0=self.params['l0'],
                DSS=self.params['dss'],
                safe_margin=self.params['safe_margin'],
                alpha_max_rad=config.deg_to_rad(self.params['max_turn_angle'])
            )

            print(self.params)
            
            # Plan
            self.log_status("Planning trajectory...")
            import time
            start_time = time.time()
            result = astar.plan_trajectory(self.preprocessed, verbose=True)
            elapsed_time = time.time() - start_time
            
            # Show results
            if result['success']:
                path = result['path']
                waypoints = [wp for wp, heading in path]

                # Compute Dubins curves
                self.log_status("Dubins curves...")
                try: 
                    dubins_points = dc.sample_all_dubins_paths(path, 30)
                    result['dubins_path'] = dubins_points
                except Exception as e:
                    self.log_status(f"Dubins error: {e}")
                
                # Calculate distance
                total_dist = 0
                for i in range(len(waypoints) - 1):
                    dx = waypoints[i+1][0] - waypoints[i][0]
                    dy = waypoints[i+1][1] - waypoints[i][1]
                    total_dist += math.sqrt(dx**2 + dy**2)
                
                self.log_status(f"\n✅ SUCCESS!")
                self.log_status(f"Planning time: {elapsed_time:.3f}s")
                self.log_status(f"Waypoints: {len(waypoints)}")
                self.log_status(f"Total distance: {total_dist/1000:.2f} km")
                self.log_status(f"Iterations: {result['stats']['iterations']}")
                
                self.last_result = result

                
            else:
                self.log_status(f"\n❌ FAILED - No path found")
                self.log_status(f"Planning time: {elapsed_time:.3f}s")
                self.log_status(f"Iterations: {result['stats']['iterations']}")
                
                self.last_result = result
                
        except Exception as e:
            self.log_status(f"\n❌ ERROR: {str(e)}")
            import traceback
            self.log_status(traceback.format_exc())
            self.last_result = {'success': False}
        
        finally:
            # Schedule GUI update on main thread
            self.root.after(0, self._planning_complete)
    
    def _planning_complete(self):
        """Callback after planning completes (runs on main thread)"""
        # Re-enable run button
        self.run_btn.config(state=tk.NORMAL, text="Run Path Planning")

        # Update scenario map with results
        self.redraw_canvas()


def main():
    """Main entry point"""
    root = tk.Tk()
    app = ScenarioBuilder(root)
    root.mainloop()


if __name__ == "__main__":
    main()
