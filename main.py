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
    Run a single scenario with performance instrumentation.
    
    Args:
        scenario_func: Function that returns a scenario dict
        scenario_name: Human-readable scenario name
        output_dir: Directory to save results
    
    Returns:
        Dict with result, scenario data, and performance metrics
    """
    
    print_header(f"Running Scenario: {scenario_name}")
    
    # Create performance metrics tracker
    metrics = perf.PerformanceMetrics(scenario_name)
    
    # Create output directory if needed
    os.makedirs(output_dir, exist_ok=True)
    
    try:
        # Generate scenario
        print("  Generating scenario...")
        metrics.start_timer('generation')
        scenario = scenario_func()
        metrics.end_timer('generation')
        print(f"    Islands: {len(scenario.get('islands', []))}")
        print(f"    SAM Sites: {len(scenario.get('sam_sites', []))}")
        
        # Preprocess
        print("  Preprocessing...")
        metrics.start_timer('preprocessing')
        preprocessed = prep.prepare_scenario(scenario)
        metrics.end_timer('preprocessing')
        
        # Plan trajectory
        print("  Planning trajectory...")
        metrics.start_timer('planning')
        start_time = time.time()
        result = astar.plan_trajectory(preprocessed, verbose=False)
        elapsed_time = time.time() - start_time
        metrics.end_timer('planning')
        
        # Record search statistics
        if result.get('stats'):
            metrics.record_search_stats(result)
        
        # Record path statistics
        if result.get('success') and result.get('path'):
            metrics.record_path_stats(result['path'], preprocessed)
        
        # Print results
        print_result(scenario_name, result, elapsed_time)
        
        # Visualize
        print("  Creating visualizations...")
        metrics.start_timer('visualization')
        
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
        
        metrics.end_timer('visualization')
        
        return {
            'scenario_name': scenario_name,
            'scenario': scenario,
            'preprocessed': preprocessed,
            'result': result,
            'elapsed_time': elapsed_time,
            'success': result['success'],
            'metrics': metrics
        }
        
    except Exception as e:
        print(f"\n❌ {scenario_name}: FAILED")
        print(f"   Error: {str(e)}")
        return {
            'scenario_name': scenario_name,
            'success': False,
            'elapsed_time': 0,
            'metrics': metrics
        }


def run_all_scenarios(output_dir="results"):
    """
    Run all 16 scenarios and generate comprehensive evaluation report.
    
    Args:
        output_dir: Directory to save results
    
    Returns:
        List of scenario results
    """
    
    print_header("MISSILE PATH PLANNING SYSTEM - COMPREHENSIVE TEST SUITE (16 SCENARIOS)")
    
    print("\nConfiguration:")
    print(f"  R (turn radius): {config.R} m")
    print(f"  α_max: {config.ALPHA_MAX}°")
    print(f"  L₀ (stabilization distance): {config.L0} m")
    print(f"  d_ss (engagement distance): {config.DSS} m")
    print(f"  Safe margin: {config.SAFE_MARGIN} m")
    print(f"  Map bounds: {config.MAP_WIDTH/1000:.0f}km x {config.MAP_HEIGHT/1000:.0f}km")
    
    results = []
    performance_comparator = perf.PerformanceComparator()
    
    # Get all scenarios from map_generator
    all_scenarios = mg.get_all_scenarios()
    scenario_list = list(all_scenarios.items())
    
    print(f"\n📋 Running {len(scenario_list)} scenarios...")
    print(f"   - Scenarios 1-4: Baseline tests")
    print(f"   - Scenarios 5-8: Easy (sparse obstacles)")
    print(f"   - Scenarios 9-12: Medium (moderate complexity)")
    print(f"   - Scenarios 13-16: Hard (high complexity)")
    
    # Execute all scenarios
    for idx, (scenario_key, scenario_func) in enumerate(scenario_list, 1):
        scenario_name = scenario_key.replace('_', ' ').title()
        try:
            result = run_scenario(scenario_func, scenario_name)
            results.append(result)
            if result.get('metrics'):
                performance_comparator.add_result(result['metrics'])
        except Exception as e:
            print(f"⚠️  Scenario {idx}: {scenario_name} failed - {str(e)}")
    
    # ===== COMPREHENSIVE SUMMARY =====
    print_header("TEST RESULTS SUMMARY")
    
    successful = sum(1 for r in results if r['success'])
    total_time = sum(r['elapsed_time'] for r in results)
    
    print(f"\n📊 Overall Performance:")
    print(f"  Total Scenarios: {len(results)}")
    print(f"  Successful: {successful}/{len(results)}")
    print(f"  Success Rate: {100*successful/len(results):.1f}%")
    print(f"  Total Time: {total_time:.2f}s")
    print(f"  Average Time per Scenario: {total_time/len(results):.2f}s")
    
    # Categorize by difficulty
    baseline = results[:4]
    easy = results[4:8]
    medium = results[8:12]
    hard = results[12:16]
    
    baseline_success = sum(1 for r in baseline if r['success'])
    easy_success = sum(1 for r in easy if r['success'])
    medium_success = sum(1 for r in medium if r['success'])
    hard_success = sum(1 for r in hard if r['success'])
    
    print(f"\n📈 Success Rate by Difficulty:")
    print(f"  Baseline (1-4):    {baseline_success}/4 ({100*baseline_success/4:.0f}%)")
    print(f"  Easy (5-8):        {easy_success}/4 ({100*easy_success/4:.0f}%)")
    print(f"  Medium (9-12):     {medium_success}/4 ({100*medium_success/4:.0f}%)")
    print(f"  Hard (13-16):      {hard_success}/4 ({100*hard_success/4:.0f}%)")
    
    print(f"\n📋 Detailed Results:")
    print(f"{'Idx':<4} {'Scenario':<35} {'Status':<8} {'Time':<8} {'Waypts':<8} {'Obstacles':<12}")
    print("─" * 75)
    
    for i, result in enumerate(results, 1):
        status = "✓ PASS" if result['success'] else "✗ FAIL"
        scenario_name = result['scenario_name'][:33]
        elapsed = result['elapsed_time']
        waypts = len(result['result']['path']) if result['success'] and result.get('result', {}).get('path') else 0
        
        if result.get('scenario'):
            num_islands = len(result['scenario'].get('islands', []))
            num_sam = len(result['scenario'].get('sam_sites', []))
            obstacles = f"{num_islands}I+{num_sam}S"
        else:
            obstacles = "N/A"
        
        print(f"{i:<4} {scenario_name:<35} {status:<8} {elapsed:>6.2f}s {waypts:>8} {obstacles:>12}")
    
    print(f"\nOutput files saved to: {output_dir}/")
    print("  - 01_scenario_*.png: Main trajectory visualizations (with Dubins curves)")
    print("  - 02_trajectory_details_*.png: Detailed trajectory analysis")
    print("  - 03_obstacles_*.png: Original vs inflated obstacles")
    
    # ===== DETAILED STATISTICS BY DIFFICULTY =====
    print_header("DETAILED STATISTICS BY DIFFICULTY LEVEL")
    
    # Baseline statistics
    print("\n🔵 BASELINE SCENARIOS (1-4):")
    for result in baseline:
        if result['success']:
            path_length = len(result['result']['path'])
            scenario = result['scenario']
            islands = len(scenario.get('islands', []))
            sams = len(scenario.get('sam_sites', []))
            print(f"  {result['scenario_name']:<30} | Islands: {islands:2} | SAM: {sams:2} | Waypoints: {path_length:2}")
    
    # Easy statistics
    print("\n🟢 EASY SCENARIOS (5-8):")
    for result in easy:
        if result['success']:
            path_length = len(result['result']['path'])
            scenario = result['scenario']
            islands = len(scenario.get('islands', []))
            sams = len(scenario.get('sam_sites', []))
            print(f"  {result['scenario_name']:<30} | Islands: {islands:2} | SAM: {sams:2} | Waypoints: {path_length:2}")
    
    # Medium statistics
    print("\n🟡 MEDIUM SCENARIOS (9-12):")
    for result in medium:
        if result['success']:
            path_length = len(result['result']['path'])
            scenario = result['scenario']
            islands = len(scenario.get('islands', []))
            sams = len(scenario.get('sam_sites', []))
            print(f"  {result['scenario_name']:<30} | Islands: {islands:2} | SAM: {sams:2} | Waypoints: {path_length:2}")
    
    # Hard statistics
    print("\n🔴 HARD SCENARIOS (13-16):")
    for result in hard:
        if result['success']:
            path_length = len(result['result']['path'])
            scenario = result['scenario']
            islands = len(scenario.get('islands', []))
            sams = len(scenario.get('sam_sites', []))
            print(f"  {result['scenario_name']:<30} | Islands: {islands:2} | SAM: {sams:2} | Waypoints: {path_length:2}")
    
    # ===== PERFORMANCE METRICS =====
    print_header("PERFORMANCE EVALUATION")
    performance_comparator.print_comparison()
    
    # ===== TIMING BREAKDOWN =====
    print_header("TIMING ANALYSIS")
    
    total_planning_time = 0
    total_visualization_time = 0
    
    for result in results:
        if result.get('metrics'):
            metrics = result['metrics']
            if 'planning' in metrics.timings and 'elapsed' in metrics.timings['planning']:
                total_planning_time += metrics.timings['planning']['elapsed']
            if 'visualization' in metrics.timings and 'elapsed' in metrics.timings['visualization']:
                total_visualization_time += metrics.timings['visualization']['elapsed']
    
    print(f"\n⏱️ Total Timing Breakdown:")
    print(f"  Total Planning Time: {total_planning_time:.3f}s")
    print(f"  Total Visualization Time: {total_visualization_time:.3f}s")
    print(f"  Average Planning per Scenario: {total_planning_time/len(results):.3f}s")
    
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
