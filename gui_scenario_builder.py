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

import config
import preprocessing as prep
import kinodynamic_astar as astar
import visualizer as viz


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
        
        # UI State
        self.mode = "idle"  # idle, draw_start, draw_goal, draw_polygon, draw_circle
        self.drawing_radius = 0
        
        # Create UI
        self.create_ui()
        
    def create_ui(self):
        """Create the user interface"""
        
        # Left panel - Controls
        left_panel = ttk.Frame(self.root, width=300)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, padx=10, pady=10)
        
        # Title
        title_label = ttk.Label(left_panel, text="Scenario Builder", 
                               font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        # Point selection
        points_frame = ttk.LabelFrame(left_panel, text="Points", padding=10)
        points_frame.pack(fill=tk.X, pady=10)
        
        self.start_btn = ttk.Button(points_frame, text="🎯 Select Launch Point",
                                   command=self.select_start_point)
        self.start_btn.pack(fill=tk.X, pady=5)
        self.start_label = ttk.Label(points_frame, text="Launch: Not set", 
                                    foreground="red")
        self.start_label.pack(fill=tk.X, pady=2)
        
        self.goal_btn = ttk.Button(points_frame, text="🎯 Select Target Point",
                                  command=self.select_goal_point)
        self.goal_btn.pack(fill=tk.X, pady=5)
        self.goal_label = ttk.Label(points_frame, text="Target: Not set",
                                   foreground="red")
        self.goal_label.pack(fill=tk.X, pady=2)
        
        # Obstacles
        obstacles_frame = ttk.LabelFrame(left_panel, text="Obstacles", padding=10)
        obstacles_frame.pack(fill=tk.X, pady=10)
        
        self.polygon_btn = ttk.Button(obstacles_frame, text="🏝️ Draw Island (Polygon)",
                                     command=self.start_draw_polygon)
        self.polygon_btn.pack(fill=tk.X, pady=5)
        
        self.circle_btn = ttk.Button(obstacles_frame, text="🛡️ Draw SAM Site (Circle)",
                                    command=self.start_draw_circle)
        self.circle_btn.pack(fill=tk.X, pady=5)
        
        self.polygon_label = ttk.Label(obstacles_frame, text="Polygons: 0")
        self.polygon_label.pack(fill=tk.X, pady=2)
        
        self.circle_label = ttk.Label(obstacles_frame, text="Circles: 0")
        self.circle_label.pack(fill=tk.X, pady=2)
        
        # Clear buttons
        clear_frame = ttk.LabelFrame(left_panel, text="Clear", padding=10)
        clear_frame.pack(fill=tk.X, pady=10)
        
        ttk.Button(clear_frame, text="Clear Last Obstacle",
                  command=self.clear_last_obstacle).pack(fill=tk.X, pady=3)
        ttk.Button(clear_frame, text="Clear All",
                  command=self.clear_all).pack(fill=tk.X, pady=3)
        ttk.Button(clear_frame, text="Reset Scenario",
                  command=self.reset_scenario).pack(fill=tk.X, pady=3)
        
        # Execute
        exec_frame = ttk.LabelFrame(left_panel, text="Execute", padding=10)
        exec_frame.pack(fill=tk.X, pady=10)
        
        self.run_btn = ttk.Button(exec_frame, text="▶️ Run Path Planning",
                                 command=self.run_planning, state=tk.DISABLED)
        self.run_btn.pack(fill=tk.X, pady=5)
        
        # Status
        status_frame = ttk.LabelFrame(left_panel, text="Status", padding=10)
        status_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        self.status_text = tk.Text(status_frame, height=20, width=35, 
                                  font=("Courier", 9), state=tk.DISABLED)
        self.status_text.pack(fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(status_frame, orient=tk.VERTICAL,
                                 command=self.status_text.yview)
        self.status_text.config(yscroll=scrollbar.set)
        
        # Right panel - Canvas
        right_panel = ttk.Frame(self.root)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, right_panel)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Bind events
        self.canvas.mpl_connect('button_press_event', self.on_canvas_click)
        self.canvas.mpl_connect('motion_notify_event', self.on_canvas_motion)
        
        # Initialize plot
        self.redraw_canvas()
        
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
        self.log_status("🏝️ Click to add polygon vertices. Double-click to finish.")
    
    def start_draw_circle(self):
        """Start drawing a circle"""
        if not self.start_point or not self.goal_point:
            messagebox.showwarning("Warning", "Please select launch and target points first!")
            return
        
        self.mode = "draw_circle"
        self.current_circle_center = None
        self.drawing_radius = 0
        self.log_status("🛡️ Click center, then move mouse to set radius. Click again to confirm.")
    
    def on_canvas_click(self, event):
        """Handle canvas click events"""
        if event.xdata is None or event.ydata is None:
            return
        
        x, y = event.xdata, event.ydata
        
        if self.mode == "draw_start":
            self.start_point = (x, y)
            self.start_label.config(text=f"Launch: ({x:.0f}, {y:.0f})", foreground="green")
            self.mode = "idle"
            self.log_status(f"✅ Launch point set: ({x:.0f}, {y:.0f})")
            self.update_buttons()
            self.redraw_canvas()
            
        elif self.mode == "draw_goal":
            self.goal_point = (x, y)
            self.goal_label.config(text=f"Target: ({x:.0f}, {y:.0f})", foreground="green")
            self.mode = "idle"
            self.log_status(f"✅ Target point set: ({x:.0f}, {y:.0f})")
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
                            text=f"Polygons: {sum(1 for o in self.obstacles if o['type'] == 'polygon')}"
                        )
                        self.log_status(f"✅ Polygon added ({len(self.obstacles)} obstacles)")
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
                    text=f"Circles: {sum(1 for o in self.obstacles if o['type'] == 'circle')}"
                )
                self.log_status(f"✅ Circle added ({len(self.obstacles)} obstacles)")
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
                text=f"Polygons: {sum(1 for o in self.obstacles if o['type'] == 'polygon')}"
            )
            self.circle_label.config(
                text=f"Circles: {sum(1 for o in self.obstacles if o['type'] == 'circle')}"
            )
            self.log_status("🗑️ Last obstacle removed")
            self.redraw_canvas()
    
    def clear_all(self):
        """Clear all obstacles"""
        self.obstacles = []
        self.current_polygon = []
        self.current_circle_center = None
        self.polygon_label.config(text="Polygons: 0")
        self.circle_label.config(text="Circles: 0")
        self.log_status("🗑️ All obstacles cleared")
        self.redraw_canvas()
    
    def reset_scenario(self):
        """Reset everything"""
        self.start_point = None
        self.goal_point = None
        self.obstacles = []
        self.current_polygon = []
        self.current_circle_center = None
        self.mode = "idle"
        
        self.start_label.config(text="Launch: Not set", foreground="red")
        self.goal_label.config(text="Target: Not set", foreground="red")
        self.polygon_label.config(text="Polygons: 0")
        self.circle_label.config(text="Circles: 0")
        
        self.clear_status()
        self.log_status("🔄 Scenario reset. Ready for new scenario.")
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
        """Redraw the canvas"""
        self.ax.clear()
        
        # Set map bounds
        self.ax.set_xlim(0, config.MAP_WIDTH)
        self.ax.set_ylim(0, config.MAP_HEIGHT)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.set_title("Scenario Map (Click to add points/obstacles)")
        self.ax.grid(True, alpha=0.3)
        
        # Draw obstacles
        for obs in self.obstacles:
            if obs['type'] == 'polygon':
                polygon = obs['data']
                if polygon:
                    xs = [p[0] for p in polygon] + [polygon[0][0]]
                    ys = [p[1] for p in polygon] + [polygon[0][1]]
                    self.ax.fill(xs, ys, 'brown', alpha=0.3, edgecolor='brown', linewidth=2)
                    self.ax.plot(xs, ys, 'brown', linewidth=2, label='Island' if obs == self.obstacles[0] else '')
                    
            elif obs['type'] == 'circle':
                center = obs['data']['center']
                radius = obs['data']['radius']
                circle = plt.Circle(center, radius, fill=False, edgecolor='red', linewidth=2, linestyle='--')
                self.ax.add_patch(circle)
                self.ax.plot(center[0], center[1], 'r+', markersize=10, markeredgewidth=2)
        
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
            self.ax.plot(self.start_point[0], self.start_point[1], 'g^', markersize=15, 
                        label='Launch Point', zorder=5)
            self.ax.text(self.start_point[0], self.start_point[1] - 20000, 'START',
                        ha='center', fontsize=10, color='green', fontweight='bold')
        
        # Draw goal point
        if self.goal_point:
            self.ax.plot(self.goal_point[0], self.goal_point[1], 'r*', markersize=20,
                        label='Target', zorder=5)
            self.ax.text(self.goal_point[0], self.goal_point[1] + 20000, 'TARGET',
                        ha='center', fontsize=10, color='red', fontweight='bold')
        
        self.ax.legend(loc='upper left')
        self.fig.tight_layout()
        self.canvas.draw()
    
    def run_planning(self):
        """Run path planning with current scenario"""
        if not self.start_point or not self.goal_point:
            messagebox.showerror("Error", "Please set launch and target points!")
            return
        
        self.log_status("\n" + "="*40)
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
            
            self.log_status(f"📊 Scenario: {len(scenario['obstacles'])} obstacles")
            
            # Preprocess
            self.log_status("⚙️ Preprocessing...")
            preprocessed = prep.prepare_scenario(scenario)
            
            # Plan
            self.log_status("🔍 Planning trajectory...")
            import time
            start_time = time.time()
            result = astar.plan_trajectory(preprocessed, verbose=False)
            elapsed_time = time.time() - start_time
            
            # Show results
            if result['success']:
                path = result['path']
                waypoints = [wp for wp, heading in path]
                
                # Calculate distance
                total_dist = 0
                for i in range(len(waypoints) - 1):
                    dx = waypoints[i+1][0] - waypoints[i][0]
                    dy = waypoints[i+1][1] - waypoints[i][1]
                    total_dist += math.sqrt(dx**2 + dy**2)
                
                self.log_status(f"\n✅ SUCCESS!")
                self.log_status(f"⏱️  Planning time: {elapsed_time:.3f}s")
                self.log_status(f"📍 Waypoints: {len(waypoints)}")
                self.log_status(f"📏 Total distance: {total_dist/1000:.2f} km")
                self.log_status(f"🔍 Iterations: {result['stats']['iterations']}")
                
                # Draw result
                self.draw_result(preprocessed, result)
                
            else:
                self.log_status(f"\n❌ FAILED - No path found")
                self.log_status(f"⏱️  Planning time: {elapsed_time:.3f}s")
                self.log_status(f"🔍 Iterations: {result['stats']['iterations']}")
                
        except Exception as e:
            self.log_status(f"\n❌ ERROR: {str(e)}")
            import traceback
            self.log_status(traceback.format_exc())
    
    def draw_result(self, preprocessed, result):
        """Draw the planning result on canvas"""
        # Create new figure for result
        fig, ax = plt.subplots(figsize=(12, 10))
        
        try:
            viz.plot_scenario(
                {'islands': [obs['data'] for obs in self.obstacles if obs['type'] == 'polygon'],
                 'sam_sites': [(obs['data']['center'], obs['data']['radius']) 
                              for obs in self.obstacles if obs['type'] == 'circle']},
                preprocessed,
                result,
                title="Path Planning Result",
                save_path=None
            )
            plt.show()
        except Exception as e:
            messagebox.showerror("Error", f"Could not display result: {str(e)}")


def main():
    """Main entry point"""
    root = tk.Tk()
    app = ScenarioBuilder(root)
    root.mainloop()


if __name__ == "__main__":
    main()
