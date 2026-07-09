"""
Run tests with random scenarios and visualize the results.
"""
import math
import random
import json
import os
import matplotlib.pyplot as plt
from logger_config import setup_logging

import config
import core.map_generator as mg 
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import core.spatial_utils as su
import render.visualizer as viz
import render.trajectory as tr
import performance_eval as perf


def print_header(text):
    """
    Print a formatted header for test output.
    
    Args:
        text: Header text to print
    """
    print("\n" + "=" * 80)
    print(text)
    print("=" * 80 + "\n")  


def run_scenario(scenario, scenario_name, kwargs, output_dir="results"):
    """
    Run a single scenario with performance instrumentation.
    
    Args:
        scenario_func: Function that returns a scenario dict
        scenario_name: Human-readable scenario name
        seed: Random seed for reproducibility
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
        
        # Preprocess
        print("  Preprocessing...")
        metrics.start_timer('preprocessing')
        preprocessed = prep.prepare_scenario(scenario, **kwargs)
        logger.info(preprocessed)
        preprocessed_time = metrics.end_timer('preprocessing')
        
        # Plan trajectory
        print("  Planning trajectory...")
        metrics.start_timer('planning')
        result = astar.plan_trajectory(preprocessed, verbose=False)
        planning_time = metrics.end_timer('planning')
        
        # Record search statistics
        if result.get('stats'):
            metrics.record_search_stats(result)
        
        # Record path statistics
        if result.get('success') and result.get('path'):
            metrics.record_path_stats(result['path'], preprocessed)
        
        # Visualize
        print("  Creating visualizations...")
        metrics.start_timer('visualization')
        
        # Main trajectory plot (legacy full-map view)
        base_name = scenario_name.lower().replace(' ', '_')
        main_fig = viz.plot_scenario(scenario, preprocessed, result,
                                    title=f"{scenario_name} - Autonomous Aircraft Trajectory",
                                    save_path=os.path.join(output_dir, f"01_scenario_{base_name}.png"))

        # Auto-fit view: framed to the mission so the flight is easy to follow.
        autofit_fig = viz.plot_scenario(scenario, preprocessed, result,
                                    title=f"{scenario_name} - Autonomous Aircraft Trajectory (auto-fit)",
                                    save_path=os.path.join(output_dir, f"02_scenario_{base_name}_autofit.png"),
                                    fit='content')

        metrics.end_timer('visualization')
        
        return {
            'scenario_name': scenario_name,
            'scenario': scenario,
            'preprocessed': preprocessed,
            'result': result,
            'elapsed_time': preprocessed_time + planning_time,
            'preprocessing_time': preprocessed_time,
            'planning_time': planning_time,
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
            'preprocessing_time': 0,
            'planning_time': 0,
            'metrics': metrics
        }


def run_test(output_dir="results1"):
    """
    Run a batch of random scenarios and save results.
    
    Args:
        num_tests: Number of random scenarios to run
        output_dir: Directory to save results
    """

    print_header("AUTONOMOUS AIRCRAFT PATH PLANNING SYSTEM - COMPREHENSIVE TEST SUITE (16 SCENARIOS)")
    
    print("\nConfiguration:")
    print(f"  R (turn radius): {config.R} m")
    print(f"  alpha_max: {config.ALPHA_MAX}°")
    print(f"  L_0 (stabilization distance): {config.L0} m")
    print(f"  d_ss (engagement distance): {config.DSS} m")
    print(f"  Safe margin: {config.SAFE_MARGIN} m")
    print(f"  Map bounds: {config.MAP_WIDTH/1000:.0f}km x {config.MAP_HEIGHT/1000:.0f}km")

    perf_metrics = perf.PerformanceComparator()
    

    scenario_name = f"Scenario Test"
    scenario = {
        'start': (449446.4583, 1188023.5911),
        'start_heading': 0.0,
        'goal': (521214.3377, 1164069.7764),
        'goal_heading': None,
        'obstacles': [],
        'islands': [],
        'dynamic_obstacles': [],
        'safezones': [
                        [
                            (444644.39, 1193895.31),
                            (458768.76, 1205669.96),
                            (479719.30, 1205774.77),
                            (534653.91, 1170269.46),
                            (534786.43, 1155488.26),
                            (459316.41, 1155175.60),
                            (444716.15, 1176742.81),
                            (444172.21, 1187300.65)
                        ],
                        [
                            (26454.20, 1663527.05),
                            (1669424.07, 1717332.52),
                            (1773367.51, 21327.40),
                            (0.0, 0.0)
                        ]
                    ],
    }
    kwargs = {
        'R': 10000,
        'L0': 4000,
        'alpha_max_rad': math.pi / 2,
        'DSS': 20000,
    }
    try:
        result = run_scenario(scenario, scenario_name, kwargs, output_dir=output_dir)
        
        if result.get('metrics'):
            perf_metrics.add_result(result['metrics'])
    except Exception as e:
        logger.exception(f"Scenario: {scenario_name} failed with error: {e}")
       
    if result['success']:
        logger.info(f"{'Scenario Name':<30} {'Status':<10} {'Total Time (s)':<15} {'Preprocessing (s)':<20} {'Planning (s)':<15} {'Waypoints':<10} {'Distance (m)':<12} {'Iterations':<10} {'Obstacles':<10}")
        logger.info("-" * 150)


        status = "SUCCESS" if result['success'] else "FAILED"
        scenario_name = result['scenario_name']
        total_time = result['elapsed_time']
        preprocessing_time = result['preprocessing_time']
        planning_time = result['planning_time']
        waypoints = len(result['result']['path']) if result['success'] and result['result'].get('path') else 0
        iterations = result['result']['stats']['iterations'] if result['result'].get('stats') else 0
        dist = result['metrics'].path_stats.get('total_distance', 0)
        turns = result['metrics'].path_stats.get('turn_angles', [])
        
        if result.get('scenario'):
            num_islands = len(result['scenario'].get('islands', []))
            num_dynamic_obstacles = len(result['scenario'].get('dynamic_obstacles', []))
            obstacles = f"{num_islands} I, {num_dynamic_obstacles} D"
        else:
            obstacles = "N/A"

        logger.info(f"{result['scenario_name']:<30} {status:<10} {result['elapsed_time']:<13.2f}s | {result['preprocessing_time']:<18.2f}s | {result['planning_time']:<13.2f}s | {waypoints:<10} | {dist:<10.2f}m | {iterations:<10} | {obstacles:<10}")

    return result


def main():
    """
    Main function to run the batch random tests.
    """

    # Ensure matplotlib is set up for non-interactive use (no GUI)
    plt.switch_backend('Agg')

    # Run all scenarios in batch mode
    output_dir = "results_test"  # Directory to save results
    run_test(output_dir=output_dir)

    # Close all matplotlib figures to free memory
    plt.close('all')

    logger.info(f"\nAll tests completed. Results saved in the {output_dir} directory.")


if __name__ == "__main__":
    # Configure logging
    logger = setup_logging("RUNTEST", log_file="logs/run_test.log")
    main()
