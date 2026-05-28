"""
Main Test Suite
Runs 4 scenario tests and generates visualizations
"""

import os
import sys
import time
import math
import matplotlib.pyplot as plt

import config
import map_generator as mg
import preprocessing as prep
import kinodynamic_astar as astar
import visualizer as viz
import performance_eval as perf


def print_header(text):
    """Print formatted header"""
    print("\n" + "="*70)
    print(f"  {text}")
    print("="*70)


def print_result(scenario_name, result, elapsed_time):
    """Print scenario result summary"""
    print(f"\n{'─'*70}")
    print(f"Scenario: {scenario_name}")
    print(f"{'─'*70}")
    
    if result['success']:
        print(f"✓ SUCCESS - Path found in {elapsed_time:.2f}s")
        path = result['path']
        print(f"  Waypoints: {len(path)}")
        
        # Calculate path statistics
        total_dist = 0
        max_turn = 0
        for i in range(len(path) - 1):
            wp1, h1 = path[i]
            wp2, h2 = path[i + 1]
            
            dx = wp2[0] - wp1[0]
            dy = wp2[1] - wp1[1]
            dist = math.sqrt(dx**2 + dy**2)
            total_dist += dist
            
            if i < len(path) - 2:
                h3 = path[i + 2][1]
                delta = h2 - h1
                delta = math.atan2(math.sin(delta), math.cos(delta))
                turn_angle = abs(delta)
                max_turn = max(max_turn, turn_angle)
        
        print(f"  Total Path Distance: {total_dist/1000:.2f} km")
        print(f"  Max Turn Angle: {math.degrees(max_turn):.2f}°")
        
        stats = result.get('stats', {})
        print(f"  Iterations: {stats.get('iterations', 0)}/{config.MAX_ITERATIONS}")
    else:
        print(f"✗ FAILED - No path found after {elapsed_time:.2f}s")
        stats = result.get('stats', {})
        print(f"  Iterations: {stats.get('iterations', 0)}/{config.MAX_ITERATIONS}")
        print(f"  Open Set: {stats.get('open_set_size', 0)}")
        print(f"  Closed Set: {stats.get('closed_set_size', 0)}")


def run_scenario(scenario_func, scenario_name, output_dir="results"):
    """
    Run a single scenario.
    
    Args:
        scenario_func: Function that returns a scenario dict
        scenario_name: Human-readable scenario name
        output_dir: Directory to save results
    
    Returns:
        Dict with result and scenario data
    """
    
    print_header(f"Running Scenario: {scenario_name}")
    
    # Create output directory if needed
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate scenario
    print("  Generating scenario...")
    scenario = scenario_func()
    print(f"    Islands: {len(scenario.get('islands', []))}")
    print(f"    SAM Sites: {len(scenario.get('sam_sites', []))}")
    
    # Preprocess
    print("  Preprocessing...")
    preprocessed = prep.prepare_scenario(scenario)
    
    # Plan trajectory
    print("  Planning trajectory...")
    start_time = time.time()
    result = astar.plan_trajectory(preprocessed, verbose=True)
    elapsed_time = time.time() - start_time
    
    # Print results
    print_result(scenario_name, result, elapsed_time)
    
    # Visualize
    print("  Creating visualizations...")
    
    # Main trajectory plot
    main_fig = viz.plot_scenario(scenario, preprocessed, result, 
                                title=f"{scenario_name} - Missile Trajectory",
                                save_path=os.path.join(output_dir, f"01_scenario_{scenario_name.lower().replace(' ', '_')}.png"))
    
    # Detailed trajectory analysis
    if result['success']:
        detail_fig = viz.plot_trajectory_details(result, preprocessed,
                                                save_path=os.path.join(output_dir, f"02_trajectory_details_{scenario_name.lower().replace(' ', '_')}.png"))
    
    # Obstacle inflation comparison
    obs_fig = viz.plot_obstacles_comparison(scenario, preprocessed,
                                           save_path=os.path.join(output_dir, f"03_obstacles_{scenario_name.lower().replace(' ', '_')}.png"))
    
    return {
        'scenario_name': scenario_name,
        'scenario': scenario,
        'preprocessed': preprocessed,
        'result': result,
        'elapsed_time': elapsed_time,
        'success': result['success'],
    }


def run_all_scenarios(output_dir="results"):
    """
    Run all 4 scenarios and generate report.
    
    Args:
        output_dir: Directory to save results
    
    Returns:
        List of scenario results
    """
    
    print_header("MISSILE PATH PLANNING SYSTEM - TEST SUITE")
    
    print("\nConfiguration:")
    print(f"  R (turn radius): {config.R} m")
    print(f"  α_max: {config.ALPHA_MAX}°")
    print(f"  L₀ (stabilization distance): {config.L0} m")
    print(f"  d_ss (engagement distance): {config.DSS} m")
    print(f"  Safe margin: {config.SAFE_MARGIN} m")
    print(f"  Map bounds: {config.MAP_WIDTH} x {config.MAP_HEIGHT} m")
    
    results = []
    
    # Scenario 1: Open ocean
    result1 = run_scenario(mg.scenario1_open_ocean, "Open Ocean")
    results.append(result1)
    
    # Scenario 2: Single obstacle
    result2 = run_scenario(mg.scenario2_single_obstacle, "Single Obstacle")
    results.append(result2)
    
    # Scenario 3: Narrow gap
    result3 = run_scenario(mg.scenario3_narrow_gap, "Narrow Gap")
    results.append(result3)
    
    # Scenario 4: Complex maze
    result4 = run_scenario(mg.scenario4_complex_maze, "Complex Maze")
    results.append(result4)
    
    # ===== SUMMARY =====
    print_header("TEST SUMMARY")
    
    successful = sum(1 for r in results if r['success'])
    total_time = sum(r['elapsed_time'] for r in results)
    
    print(f"\nResults:")
    print(f"  Total Scenarios: {len(results)}")
    print(f"  Successful: {successful}/{len(results)}")
    print(f"  Success Rate: {100*successful/len(results):.1f}%")
    print(f"  Total Time: {total_time:.2f}s")
    
    print(f"\nDetailed Results:")
    for i, result in enumerate(results, 1):
        status = "✓" if result['success'] else "✗"
        print(f"  {i}. {result['scenario_name']:20} [{status}] {result['elapsed_time']:6.2f}s")
    
    print(f"\nOutput files saved to: {output_dir}/")
    print("  - 01_scenario_*.png: Main trajectory visualizations")
    print("  - 02_trajectory_details_*.png: Detailed trajectory analysis")
    print("  - 03_obstacles_*.png: Original vs inflated obstacles")
    
    # Generate summary statistics
    print("\n" + "="*70)
    print("  SYSTEM STATISTICS")
    print("="*70)
    
    for result in results:
        if result['success']:
            path_length = len(result['result']['path'])
            scenario = result['scenario']
            print(f"\n{result['scenario_name']}:")
            print(f"  Obstacles: {len(scenario.get('islands', []))} islands + {len(scenario.get('sam_sites', []))} SAM sites")
            print(f"  Waypoints: {path_length}")
            
            # Calculate total distance
            path = result['result']['path']
            total_dist = 0
            for i in range(len(path) - 1):
                wp1, _ = path[i]
                wp2, _ = path[i + 1]
                dx = wp2[0] - wp1[0]
                dy = wp2[1] - wp1[1]
                total_dist += math.sqrt(dx**2 + dy**2)
            
            print(f"  Total Distance: {total_dist/1000:.2f} km")
    
    print("\n" + "="*70)
    
    return results


def main():
    """Main entry point"""
    
    # Ensure matplotlib is set up for non-interactive mode
    plt.switch_backend('Agg')
    
    # Run all scenarios
    results = run_all_scenarios(output_dir="results")
    
    # Close all figures to free memory
    plt.close('all')
    
    print("\n✓ Test suite completed!")
    
    return results


if __name__ == "__main__":
    main()
