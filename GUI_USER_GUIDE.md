# 🎨 Interactive GUI Scenario Builder - User Guide

## Overview

The Interactive GUI Scenario Builder allows you to create custom scenarios for path planning by:
- ✅ Clicking to select launch and target points
- ✅ Drawing polygons for islands/obstacles  
- ✅ Drawing circles for SAM defense sites
- ✅ Running path planning in real-time
- ✅ Viewing results immediately

---

## 🚀 How to Run

### Start the GUI
```bash
cd d:\Workspace\VTX\VCM_Path_Planning
python gui_scenario_builder.py
```

The GUI will open with a map on the right and control panel on the left.

---

## 📋 Step-by-Step Usage

### 1️⃣ Select Launch Point
- Click **"🎯 Select Launch Point"** button
- Click anywhere on the map to set your launch position
- The point will appear as a **green triangle (▲)** labeled "START"

### 2️⃣ Select Target Point  
- Click **"🎯 Select Target Point"** button
- Click on the map to set your target position
- The point will appear as a **red star (★)** labeled "TARGET"

### 3️⃣ Draw Islands (Polygons)
- Click **"🏝️ Draw Island (Polygon)"** button
- Click multiple points on the map to define vertices
- Each click adds a vertex (shown as blue dots)
- When done, click near the first point to close the polygon
- The polygon will turn brown and be added as an obstacle

**Example: Drawing a triangle island**
```
1. Click point 1 → Blue dot appears
2. Click point 2 → Line appears connecting points
3. Click point 3 → Triangle outline appears
4. Click near point 1 → Triangle filled, polygon added ✅
```

### 4️⃣ Draw SAM Sites (Circles)
- Click **"🛡️ Draw SAM Site (Circle)"** button
- Click to set the circle center
- Move mouse to set radius (circle outline follows cursor)
- Click again to confirm
- The circle will appear as a red dashed circle

**Example: Drawing a SAM defense zone**
```
1. Click for center → Blue + appears at center
2. Move mouse away → Red dashed circle grows/shrinks
3. Click to confirm → Circle becomes red, added as obstacle ✅
```

---

## 🎮 Map Interaction

### Map Features
- **Size:** 500km × 500km (500,000m × 500,000m)
- **Grid:** Shows coordinate system
- **Zoom:** Use mouse wheel to zoom in/out
- **Pan:** Click and drag to move around map

### Legend
- 🟢 **Green triangle (▲)** = Launch point (START)
- 🔴 **Red star (★)** = Target point (TARGET)
- 🟤 **Brown polygon** = Island obstacle
- 🔴 **Red dashed circle** = SAM defense zone

---

## 🎯 Control Panel Features

### Status Panel (Bottom)
Shows real-time status messages:
- ✅ "Launch point set"
- ✅ "Vertex added"
- ✅ "Polygon added"
- ✅ "Circle added"
- ✅ "SUCCESS! Path found"

### Obstacle Management
- **"Clear Last Obstacle"** - Remove the most recently added obstacle
- **"Clear All"** - Remove all obstacles (keeps start/goal)
- **"Reset Scenario"** - Clear everything and start fresh

### Path Planning
- **"▶️ Run Path Planning"** - Execute algorithm with current scenario
  - Enabled only when both launch and target points are set
  - Shows planning time, waypoints, distance, iterations
  - Displays result in new window

---

## 💡 Usage Examples

### Example 1: Simple Corridor Test
```
1. Set launch point: (50km, 50km)
2. Set target point: (450km, 450km)
3. Draw 2 islands with 20km gap between them
4. Click "Run Path Planning"
Expected: Path should navigate through the gap
```

### Example 2: Dense Obstacle Field
```
1. Set launch point: (10km, 10km)
2. Set target point: (490km, 490km)
3. Draw 15-20 small islands randomly
4. Draw 8-10 SAM circles scattered
5. Click "Run Path Planning"
Expected: Complex path with multiple waypoints
```

### Example 3: Coastal Defense
```
1. Set launch point: (10km, 250km)
2. Set target point: (490km, 250km)
3. Draw several large islands along the coast
4. Draw 5-7 SAM sites forming a defensive ring
5. Click "Run Path Planning"
Expected: Path that evades coastal defense network
```

---

## ⚙️ Keyboard Shortcuts

**Current UI Controls:**
- **Left Click:** Select points / add vertices / set radius
- **Double-Click near first vertex:** Finish polygon
- **Right-Click:** Not used (reserved for future)

---

## 🔍 Understanding Results

After running path planning, the result window shows:

### Trajectory Plot
- **Blue line:** Planned trajectory path
- **Blue dots:** Waypoints along the path
- **Circles at waypoints:** Turn radius indicators
- **Brown areas:** Inflated obstacle buffers

### Status Log (Example)
```
========================================
🚀 Running path planning...
📊 Scenario: 5 obstacles
⚙️ Preprocessing...
🔍 Planning trajectory...

✅ SUCCESS!
⏱️  Planning time: 0.0456s
📍 Waypoints: 4
📏 Total distance: 485.32 km
🔍 Iterations: 15
```

---

## 📊 Performance Indicators

| Metric | Good | Acceptable | Slow |
|--------|------|-----------|------|
| Planning Time | <0.05s | <0.5s | >0.5s |
| Waypoints | 2-5 | 5-10 | >10 |
| Iterations | <10 | 10-50 | >50 |

---

## 🐛 Troubleshooting

### "Run Path Planning" button is disabled
- ✅ Make sure you selected BOTH launch and target points
- ✅ The button enables automatically once both points are set

### Path planning fails ("No path found")
- Obstacles might block all possible paths
- Try removing some obstacles
- Check that launch and target are not inside obstacles

### Polygon not closing
- You need at least 3 vertices
- Click very close to the first point to close it
- The first point should turn red/highlighted when you're close

### Circle radius too small/large
- You can redraw by adding a new circle
- Use "Clear Last" to remove it if needed
- The next circle will work better

### Map too zoomed/far
- Double-click on the map to reset zoom
- Use scroll wheel to zoom in/out
- Click and drag to pan around

---

## 📁 Output Files

When you run path planning:
- **Console output:** Real-time status messages in the GUI
- **Result window:** Matplotlib figure showing the planned path
- **No files saved:** Results are displayed only (use "Save" in matplotlib to save PNG)

---

## 🎓 Tips & Tricks

### For Testing
1. **Start Simple:** Begin with open ocean (no obstacles)
2. **Add Gradually:** Add obstacles one by one
3. **Test Edges:** Try placing obstacles near the direct line
4. **Stress Test:** Try 15-20 obstacles to see limits

### For Validation
- **Straight Line Test:** No obstacles → should be straight path
- **Single Obstacle:** One island → should go around it  
- **Multiple Obstacles:** Should navigate intelligently
- **Dense Field:** Should find optimal multi-waypoint path

### For Performance
- Use "Clear Last" instead of "Clear All" to preserve points
- Draw simpler polygons (fewer vertices) for faster processing
- Larger obstacles (with radius/vertices) don't significantly slow algorithm

---

## 🎨 Visualization Colors

| Element | Color | Shape |
|---------|-------|-------|
| Launch Point | Green | Triangle ▲ |
| Target Point | Red | Star ★ |
| Island | Brown | Filled Polygon |
| Island Outline | Brown | Solid Line |
| SAM Site | Red | Dashed Circle |
| Current Polygon | Blue | Solid Line |
| Current Circle | Blue | Dotted Circle |
| Waypoints | Blue | Dots |
| Trajectory | Blue | Thick Line |

---

## 📱 Keyboard Reference

| Action | How |
|--------|-----|
| Select Launch | Click button, then click map |
| Select Target | Click button, then click map |
| Add Polygon | Click button, then click vertices, close polygon |
| Add Circle | Click button, click center, move mouse, click again |
| Clear Last | Click "Clear Last Obstacle" |
| Clear All | Click "Clear All" |
| Reset All | Click "Reset Scenario" |
| Run Planning | Click "▶️ Run Path Planning" |

---

## ✅ Quick Checklist Before Running

- [ ] Launch point selected (green triangle visible)
- [ ] Target point selected (red star visible)  
- [ ] At least one obstacle added (or intentionally testing open space)
- [ ] "Run Path Planning" button is enabled
- [ ] You want to see the result

---

## 🔗 Integration with Test Suite

After creating a scenario in GUI:
1. Note the obstacle positions and types
2. Create a custom scenario function in `map_generator.py`
3. Add it to the 16-scenario test suite
4. Run `python main.py` for batch testing

---

## 📞 Support

For issues:
1. Check the Status log in the GUI
2. Read error messages carefully
3. Try the troubleshooting section above
4. Simplify the scenario and try again

---

## 🚀 Next Steps

- Create custom scenarios for testing
- Export scenario data for batch testing
- Integrate with test suite for validation
- Build more complex test cases

**Status:** ✅ Ready to use for interactive scenario testing

