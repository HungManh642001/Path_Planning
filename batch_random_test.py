"""
Run tests with random scenarios and visualize the results.
"""

import json
import math
import os
import random

import matplotlib.pyplot as plt
import performance_eval as perf

from path_planning import config
from path_planning.core import (
    kinodynamic_astar as astar,
    map_generator as mg,
    preprocessing as prep,
    spatial_utils as su,
)
from path_planning.logger_config import setup_logging
from path_planning.render import visualizer as viz


def generate_random_scenario(seed=42):
    """
    Generate a random scenario with islands and dynamic obstacles.

    Args:
        seed: Random seed for reproducibility

    Returns:
        scenario: Dictionary containing map bounds, start/goal, islands, and dynamic obstacles
    """
    random.seed(seed)

    # Map bounds
    map_bounds = (config.MAP_WIDTH, config.MAP_HEIGHT)
    width, height = map_bounds

    # Random start and goal positions within the map bounds
    while True:
        start = (
            random.uniform(width * 0.1, width * 0.9),
            random.uniform(height * 0.1, height * 0.9),
        )
        goal = (
            random.uniform(width * 0.1, width * 0.9),
            random.uniform(height * 0.1, height * 0.9),
        )
        if su.distance(start, goal) > 400000:  # Ensure start and goal are not too close
            break

    heading_start_to_goal = su.angle_to_heading(start, goal)

    topology = random.choices(
        ["random", "center_cluster", "wall_block"], weights=[0.1, 0.45, 0.45]
    )[0]  # Randomly choose a topology for obstacle placement

    return mg.create_scenario(
        {
            "map_bounds": map_bounds,
            "start": start,
            "start_heading": heading_start_to_goal
            + random.uniform(
                -math.pi / 2, math.pi / 2
            ),  # Add some randomness to the start heading
            "goal": goal,
            "goal_heading": heading_start_to_goal
            + random.uniform(
                -math.pi / 2, math.pi / 2
            ),  # Add some randomness to the goal heading
            "num_islands": random.randint(0, 20),
            "num_dynamic_obstacles": random.randint(0, 20),
            "topology": topology,
            "seed": seed,
        }
    )


def print_header(text):
    """
    Print a formatted header for test output.

    Args:
        text: Header text to print
    """
    print("\n" + "=" * 80)
    print(text)
    print("=" * 80 + "\n")


def run_scenario(scenario_func, scenario_name, seed=42, output_dir="results"):
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
        # Generate scenario
        print("  Generating scenario...")
        metrics.start_timer("generation")
        scenario = scenario_func(seed=seed)
        metrics.end_timer("generation")
        print(f"    Islands: {len(scenario.get('islands', []))}")
        print(f"    Dynamic Obstacles: {len(scenario.get('dynamic_obstacles', []))}")

        # Preprocess
        print("  Preprocessing...")
        metrics.start_timer("preprocessing")
        preprocessed = prep.prepare_scenario(scenario)
        preprocessed_time = metrics.end_timer("preprocessing")

        # Plan trajectory
        print("  Planning trajectory...")
        metrics.start_timer("planning")
        result = astar.plan_trajectory(preprocessed, verbose=False)
        planning_time = metrics.end_timer("planning")

        # Record search statistics
        if result.get("stats"):
            metrics.record_search_stats(result)

        # Record path statistics
        if result.get("success") and result.get("path"):
            metrics.record_path_stats(result["path"], preprocessed)

        # Visualize
        print("  Creating visualizations...")
        metrics.start_timer("visualization")

        # Main trajectory plot
        main_fig = viz.plot_scenario(
            scenario,
            preprocessed,
            result,
            title=f"{scenario_name} - Autonomous Aircraft Trajectory",
            save_path=os.path.join(
                output_dir, f"01_scenario_{scenario_name.lower().replace(' ', '_')}.png"
            ),
        )

        metrics.end_timer("visualization")

        return {
            "scenario_name": scenario_name,
            "scenario": scenario,
            "preprocessed": preprocessed,
            "result": result,
            "elapsed_time": preprocessed_time + planning_time,
            "preprocessing_time": preprocessed_time,
            "planning_time": planning_time,
            "success": result["success"],
            "metrics": metrics,
        }

    except Exception as e:
        print(f"\n❌ {scenario_name}: FAILED")
        print(f"   Error: {str(e)}")
        return {
            "scenario_name": scenario_name,
            "success": False,
            "elapsed_time": 0,
            "preprocessing_time": 0,
            "planning_time": 0,
            "metrics": metrics,
        }


def run_batch_random_tests(num_tests=1000, output_dir="results1"):
    """
    Run a batch of random scenarios and save results.

    Args:
        num_tests: Number of random scenarios to run
        output_dir: Directory to save results
    """

    print_header(
        "AUTONOMOUS AIRCRAFT PATH PLANNING SYSTEM - COMPREHENSIVE TEST SUITE (16 SCENARIOS)"
    )

    print("\nConfiguration:")
    print(f"  R (turn radius): {config.R} m")
    print(f"  alpha_max: {config.ALPHA_MAX}°")
    print(f"  L_0 (stabilization distance): {config.L0} m")
    print(f"  d_ss (engagement distance): {config.DSS} m")
    print(f"  Safe margin: {config.SAFE_MARGIN} m")
    print(
        f"  Map bounds: {config.MAP_WIDTH / 1000:.0f}km x {config.MAP_HEIGHT / 1000:.0f}km"
    )

    all_results = []
    perf_metrics = perf.PerformanceComparator()

    # for i in range(num_tests):
    # for i in [125, 319, 338, 426, 485, 532, 544, 581, 625, 641, 674, 686, 904, 923, 963, 981, 996, 998]:
    # for i in [125, 319, 338, 426, 532, 544, 581, 641, 674, 686, 904, 923, 963, 981, 998]:
    for i in [86, 125, 366, 485]:
        seed = i  # Different seed for each test
        scenario_name = f"Random Scenario {i + 1}"

        try:
            result = run_scenario(
                generate_random_scenario,
                scenario_name,
                seed=seed,
                output_dir=output_dir,
            )
            all_results.append(result)

            if result.get("metrics"):
                perf_metrics.add_result(result["metrics"])
        except Exception as e:
            logger.exception(
                f"Scenario {i + 1}: {scenario_name} failed with error: {e}"
            )

    # Save summary of all results
    print_header("Batch Random Test Summary")

    logger.info(f"Total Scenarios Run: {len(all_results)}")
    logger.info(
        f"{'Idx':<4} {'Scenario Name':<30} {'Status':<10} {'Total Time (s)':<15} {'Preprocessing (s)':<15} {'Planning (s)':<15} {'Waypoints':<10} {'Distance (m)':<12} {'Iterations':<10} {'Obstacles':<10}"
    )
    logger.info("-" * 150)

    summary_results = []
    for i, res in enumerate(all_results):
        status = "SUCCESS" if res["success"] else "FAILED"
        scenario_name = res["scenario_name"]
        total_time = res["elapsed_time"]
        preprocessing_time = res["preprocessing_time"]
        planning_time = res["planning_time"]
        waypoints = (
            len(res["result"]["path"])
            if res["success"] and res["result"].get("path")
            else 0
        )
        iterations = (
            res["result"]["stats"]["iterations"] if res["result"].get("stats") else 0
        )
        dist = res["metrics"].path_stats.get("total_distance", 0)
        turns = res["metrics"].path_stats.get("turn_angles", [])

        if res.get("scenario"):
            num_islands = len(res["scenario"].get("islands", []))
            num_dynamic_obstacles = len(res["scenario"].get("dynamic_obstacles", []))
            obstacles = f"{num_islands} I, {num_dynamic_obstacles} D"
        else:
            obstacles = "N/A"

        logger.info(
            f"{i + 1:<4} {res['scenario_name']:<30} {status:<10} {res['elapsed_time']:<13.2f}s | {res['preprocessing_time']:<13.2f}s | {res['planning_time']:<13.2f}s | {waypoints:<10} | {dist:<10.2f}| {iterations:<10} | {obstacles:>10}"
        )
        summary_results.append(
            {
                "scenario_name": scenario_name,
                "status": status,
                "total_time": total_time,
                "preprocessing_time": preprocessing_time,
                "planning_time": planning_time,
                "waypoints": waypoints,
                "distance_m": dist,
                "turns": turns,
                "iterations": iterations,
                "obstacles": obstacles,
            }
        )

    # Save summary to JSON
    summary_path = os.path.join(output_dir, "batch_random_test_summary.json")
    with open(summary_path, "w") as f:
        json.dump(summary_results, f, indent=4)
    return all_results


def main():
    """
    Main function to run the batch random tests.
    """

    # Ensure matplotlib is set up for non-interactive use (no GUI)
    plt.switch_backend("Agg")

    # Run all scenarios in batch mode
    output_dir = "results1_v1"  # Directory to save results
    num_tests = 1000  # Number of random scenarios to run
    run_batch_random_tests(num_tests=num_tests, output_dir=output_dir)

    # Close all matplotlib figures to free memory
    plt.close("all")

    logger.info(f"\nAll tests completed. Results saved in the {output_dir} directory.")


if __name__ == "__main__":
    # Configure logging
    logger = setup_logging("BatchRandomTest", log_file="logs/batch_random_test.log")
    main()
