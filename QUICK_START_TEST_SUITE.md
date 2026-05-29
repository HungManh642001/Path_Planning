# Quick Start Guide - 16 Scenario Test Suite

## Running the Complete Test Suite

### Command
```bash
cd d:\Workspace\VTX\VCM_Path_Planning
python main.py
```

### Output
- Runs all 16 scenarios sequentially
- Generates 60 PNG visualization files
- Prints detailed console report with metrics
- Execution time: ~67 seconds

### Console Output Includes
- ✅ Overall success rate (100%)
- ✅ Success rate by difficulty level
- ✅ Detailed timing breakdown
- ✅ Performance comparison table
- ✅ Path statistics
- ✅ Search efficiency metrics

---

## Running Individual Scenarios

### Example: Run Scenario 16 (Extreme Complexity)

```python
#!/usr/bin/env python3
import map_generator as mg
import preprocessing as prep
import kinodynamic_astar as astar
import visualizer as viz
import config

# Get all scenarios
scenarios = mg.get_all_scenarios()

# Get Scenario 16
scenario_func = scenarios['scenario_16_extreme_complexity']
print("Running: Extreme Complexity Scenario")
print(f"  Islands: 20, SAM Sites: 12")

# Generate scenario
scenario = scenario_func()

# Preprocess
preprocessed = prep.prepare_scenario(scenario)

# Plan trajectory
result = astar.plan_trajectory(preprocessed)

# Check success
if result['success']:
    path = result['path']
    waypoints = [wp for wp, heading in path]
    
    print(f"✓ SUCCESS")
    print(f"  Waypoints: {len(waypoints)}")
    print(f"  Iterations: {result['stats']['iterations']}/50000")
    
    # Visualize
    fig, ax = viz.create_figure()
    viz.plot_scenario(
        scenario, 
        preprocessed, 
        result,
        title="Scenario 16 - Extreme Complexity",
        save_path="results/scenario_16.png"
    )
else:
    print("✗ FAILED - No path found")
```

---

## Accessing Individual Test Results

### Get Scenario Information

```python
import map_generator as mg

# List all scenarios
scenarios = mg.get_all_scenarios()

for name, func in scenarios.items():
    scenario = func()
    islands = len(scenario.get('islands', []))
    sams = len(scenario.get('sam_sites', []))
    print(f"{name:40} | Islands: {islands:2} | SAM: {sams:2}")
```

### Expected Output
```
scenario_01_open_ocean                   | Islands:  0 | SAM:  0
scenario_02_single_obstacle              | Islands:  1 | SAM:  1
scenario_03_narrow_gap                   | Islands:  2 | SAM:  0
scenario_04_complex_maze                 | Islands: 12 | SAM:  6
scenario_05_sparse_islands               | Islands:  3 | SAM:  1
scenario_06_coastal_path                 | Islands:  2 | SAM:  2
scenario_07_diagonal_crossing            | Islands:  4 | SAM:  0
scenario_08_open_with_sam                | Islands:  1 | SAM:  3
scenario_09_island_archipelago           | Islands:  8 | SAM:  2
scenario_10_dense_defense                | Islands:  3 | SAM:  8
scenario_11_serpentine_route             | Islands:  7 | SAM:  4
scenario_12_perimeter_defense            | Islands:  6 | SAM:  5
scenario_13_dense_island_field           | Islands: 18 | SAM:  3
scenario_14_combined_threat              | Islands: 12 | SAM: 10
scenario_15_narrow_channel               | Islands: 15 | SAM:  4
scenario_16_extreme_complexity           | Islands: 20 | SAM: 12
```

---

## Performance Comparison

### View Planning Time by Scenario

```python
import map_generator as mg
import preprocessing as prep
import kinodynamic_astar as astar
import time

scenarios = mg.get_all_scenarios()
results = {}

print(f"{'Scenario':<40} | {'Planning Time':>12} | {'Obstacles':>10}")
print("-" * 65)

for name, scenario_func in scenarios.items():
    scenario = scenario_func()
    islands = len(scenario.get('islands', []))
    sams = len(scenario.get('sam_sites', []))
    
    preprocessed = prep.prepare_scenario(scenario)
    
    start_time = time.time()
    result = astar.plan_trajectory(preprocessed)
    planning_time = time.time() - start_time
    
    print(f"{name:<40} | {planning_time:>10.3f}s | {islands}I+{sams}S")
```

---

## Scenario Categories

### Baseline Scenarios (1-4)
Used for regression testing

```python
baseline = {
    'scenario_01_open_ocean': mg.scenario1_open_ocean,
    'scenario_02_single_obstacle': mg.scenario2_single_obstacle,
    'scenario_03_narrow_gap': mg.scenario3_narrow_gap,
    'scenario_04_complex_maze': mg.scenario4_complex_maze,
}
```

### Easy Scenarios (5-8)
For performance baseline

```python
easy = {
    'scenario_05_sparse_islands': mg.scenario5_sparse_islands,
    'scenario_06_coastal_path': mg.scenario6_coastal_path,
    'scenario_07_diagonal_crossing': mg.scenario7_diagonal_crossing,
    'scenario_08_open_with_sam': mg.scenario8_open_with_sam,
}
```

### Medium Scenarios (9-12)
For moderate complexity testing

```python
medium = {
    'scenario_09_island_archipelago': mg.scenario9_island_archipelago,
    'scenario_10_dense_defense': mg.scenario10_dense_defense,
    'scenario_11_serpentine_route': mg.scenario11_serpentine_route,
    'scenario_12_perimeter_defense': mg.scenario12_perimeter_defense,
}
```

### Hard Scenarios (13-16)
For stress testing

```python
hard = {
    'scenario_13_dense_island_field': mg.scenario13_dense_island_field,
    'scenario_14_combined_threat': mg.scenario14_combined_threat,
    'scenario_15_narrow_channel': mg.scenario15_narrow_channel,
    'scenario_16_extreme_complexity': mg.scenario16_extreme_complexity,
}
```

---

## Extract Detailed Metrics

### Get Path Statistics

```python
import math

def analyze_path(path):
    """Analyze path for statistics"""
    waypoints = [wp for wp, heading in path]
    
    # Calculate distances
    total_distance = 0
    segment_distances = []
    
    for i in range(len(waypoints) - 1):
        dx = waypoints[i+1][0] - waypoints[i][0]
        dy = waypoints[i+1][1] - waypoints[i][1]
        dist = math.sqrt(dx**2 + dy**2)
        segment_distances.append(dist)
        total_distance += dist
    
    # Calculate turn angles
    turn_angles = []
    for i in range(len(path) - 1):
        h1 = path[i][1]
        h2 = path[i+1][1]
        delta = h2 - h1
        delta = math.atan2(math.sin(delta), math.cos(delta))
        turn_angles.append(abs(delta))
    
    return {
        'waypoints': len(waypoints),
        'segments': len(segment_distances),
        'total_distance': total_distance,
        'avg_segment': total_distance / len(segment_distances) if segment_distances else 0,
        'min_segment': min(segment_distances) if segment_distances else 0,
        'max_segment': max(segment_distances) if segment_distances else 0,
        'max_turn': math.degrees(max(turn_angles)) if turn_angles else 0,
        'avg_turn': math.degrees(sum(turn_angles) / len(turn_angles)) if turn_angles else 0,
    }

# Usage
scenario = mg.scenario16_extreme_complexity()
preprocessed = prep.prepare_scenario(scenario)
result = astar.plan_trajectory(preprocessed)

if result['success']:
    stats = analyze_path(result['path'])
    print(f"Waypoints: {stats['waypoints']}")
    print(f"Total Distance: {stats['total_distance']/1000:.2f} km")
    print(f"Avg Segment: {stats['avg_segment']:.0f} m")
    print(f"Max Turn Angle: {stats['max_turn']:.2f}°")
```

---

## Performance Benchmarking

### Compare Multiple Scenarios

```python
import time
import statistics

def benchmark_scenarios(scenario_names):
    """Benchmark multiple scenarios"""
    scenarios = mg.get_all_scenarios()
    timings = {}
    
    for name in scenario_names:
        if name in scenarios:
            times = []
            
            for run in range(3):  # Run 3 times
                scenario = scenarios[name]()
                preprocessed = prep.prepare_scenario(scenario)
                
                start_time = time.time()
                result = astar.plan_trajectory(preprocessed)
                elapsed = time.time() - start_time
                times.append(elapsed)
            
            timings[name] = {
                'mean': statistics.mean(times),
                'median': statistics.median(times),
                'min': min(times),
                'max': max(times),
            }
    
    return timings

# Benchmark hard scenarios
hard_scenarios = [
    'scenario_13_dense_island_field',
    'scenario_14_combined_threat',
    'scenario_15_narrow_channel',
    'scenario_16_extreme_complexity',
]

results = benchmark_scenarios(hard_scenarios)

for name, timing in results.items():
    print(f"\n{name}:")
    print(f"  Mean:   {timing['mean']:.3f}s")
    print(f"  Median: {timing['median']:.3f}s")
    print(f"  Range:  {timing['min']:.3f}s - {timing['max']:.3f}s")
```

---

## Visualization Options

### Save Custom Visualization

```python
import matplotlib.pyplot as plt

scenario = mg.scenario16_extreme_complexity()
preprocessed = prep.prepare_scenario(scenario)
result = astar.plan_trajectory(preprocessed)

# Create figure
fig, ax = viz.create_figure(figsize=(12, 10))

# Plot scenario
viz.plot_scenario(
    scenario,
    preprocessed,
    result,
    title="Custom: Extreme Complexity Test",
    save_path="results/custom_scenario_16.png"
)

# Also save to PDF
fig.savefig("results/custom_scenario_16.pdf", dpi=300)

plt.close()
```

---

## Create Custom Scenario

### Add New Test Case

```python
def scenario_17_custom():
    """Custom scenario for testing"""
    return mg.create_scenario({
        'start': (10000, 10000),
        'start_heading': math.pi / 4,
        'goal': (490000, 490000),
        'goal_heading': math.pi / 4,
        'num_islands': 25,      # More islands
        'num_sam': 15,          # More SAM
        'seed': 9999            # Custom seed
    })

# Test custom scenario
scenario = scenario_17_custom()
preprocessed = prep.prepare_scenario(scenario)
result = astar.plan_trajectory(preprocessed)

if result['success']:
    print(f"✓ Custom scenario passed!")
else:
    print(f"✗ Custom scenario failed")
```

---

## Troubleshooting

### Debug Scenario Issues

```python
import logging

# Enable detailed logging
logging.basicConfig(level=logging.DEBUG)

scenario = mg.scenario16_extreme_complexity()
print(f"Islands: {len(scenario.get('islands', []))}")
print(f"SAM Sites: {len(scenario.get('sam_sites', []))}")

# Trace preprocessing
preprocessed = prep.prepare_scenario(scenario)
print(f"Inflated obstacles: {len(preprocessed['inflated_obstacles'])}")

# Trace planning
result = astar.plan_trajectory(preprocessed, verbose=True)
```

---

## Documentation References

For more information, see:
- `TEST_EVALUATION_REPORT.md` - Comprehensive evaluation
- `TEST_SCENARIOS_GUIDE.md` - Detailed scenario descriptions
- `PROJECT_SUMMARY.md` - System architecture
- `README.md` - System overview

---

## Summary

The 16-scenario test suite provides:

✅ **Comprehensive Coverage** - All difficulty levels  
✅ **Easy Integration** - Simple Python API  
✅ **Detailed Metrics** - Performance tracking  
✅ **Visualization** - 60 PNG output files  
✅ **Documentation** - Complete guides and reports  

**Status:** Production-ready for deployment
