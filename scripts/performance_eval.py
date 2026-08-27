"""
Performance Evaluation Module
Measures and analyzes algorithm runtime, path quality, and system efficiency
"""

import logging
import math
import time


logger = logging.getLogger(__name__)


class PerformanceMetrics:
    """Tracks performance metrics for path planning"""

    def __init__(self, scenario_name):
        self.scenario_name = scenario_name
        self.timings = {}
        self.path_stats = {}
        self.search_stats = {}

    def start_timer(self, phase_name):
        """Start timing a phase"""
        self.timings[phase_name] = {"start": time.time()}

    def end_timer(self, phase_name):
        """End timing a phase"""
        if phase_name in self.timings:
            elapsed = time.time() - self.timings[phase_name]["start"]
            self.timings[phase_name]["elapsed"] = elapsed
            return elapsed
        return None

    def record_path_stats(self, path, preprocessed):
        """Record statistics about the planned path"""
        if not path:
            return

        waypoints = [wp for wp, _ in path]
        headings = [h for _, h in path]

        # Distances are measured over the FULL flown mission O -> W1 -> ... -> T,
        # not just the search path. The search path starts at W1 (the seeded
        # takeoff corner) and, in fixed-goal mode, ends at W_{n-1}, so it omits
        # the takeoff leg O->W1 and the terminal leg W_{n-1}->T. Prepend O and
        # append T (mirroring render.trajectory.build_full_path) so
        # total_distance is the true flown length.
        full_wps = list(waypoints)
        orig_origin = preprocessed.get("start_pos") if preprocessed else None
        orig_target = preprocessed.get("goal_pos") if preprocessed else None
        if orig_origin is not None and (
            not full_wps or math.dist(orig_origin, full_wps[0]) > 1.0
        ):
            full_wps = [tuple(orig_origin), *full_wps]
        if orig_target is not None and (
            not full_wps or math.dist(orig_target, full_wps[-1]) > 1.0
        ):
            full_wps = [*full_wps, tuple(orig_target)]

        # Calculate distances
        segment_distances = []
        total_distance = 0

        for i in range(len(full_wps) - 1):
            dist = math.dist(full_wps[i], full_wps[i + 1])
            segment_distances.append(dist)
            total_distance += dist
            # logger.info(f"Segment {i}: {full_wps[i]} -> {full_wps[i + 1]}, Distance: {dist:.2f} m")

        # Calculate turn angles
        turn_angles = []
        max_turn = 0

        for i in range(len(headings) - 1):
            delta = headings[i + 1] - headings[i]
            delta = math.atan2(math.sin(delta), math.cos(delta))
            turn_angle = abs(delta)
            turn_angles.append(turn_angle)
            max_turn = max(max_turn, turn_angle)

        self.path_stats = {
            "waypoints": len(waypoints),
            "segments": len(segment_distances),
            "total_distance": total_distance,
            "avg_segment": total_distance / len(segment_distances)
            if segment_distances
            else 0,
            "min_segment": min(segment_distances) if segment_distances else 0,
            "max_segment": max(segment_distances) if segment_distances else 0,
            "max_turn_angle": max_turn,
            "avg_turn_angle": sum(turn_angles) / len(turn_angles) if turn_angles else 0,
            "turns_count": len(turn_angles),
        }

    def record_search_stats(self, search_result):
        """Record A* search statistics"""
        if search_result and search_result.get("stats"):
            self.search_stats = search_result["stats"].copy()

    def get_summary(self):
        """Get summary report"""
        return {
            "scenario": self.scenario_name,
            "timings": self.timings,
            "path": self.path_stats,
            "search": self.search_stats,
        }

    def print_report(self):
        """Print formatted performance report"""
        logger.info(f"\n{'─' * 70}")
        logger.info(f"Performance Report: {self.scenario_name}")
        logger.info(f"{'─' * 70}")

        # Timing breakdown
        logger.info("\n⏱️  Timing Breakdown:")
        total_time = 0
        for phase, data in self.timings.items():
            if "elapsed" in data:
                elapsed = data["elapsed"]
                total_time += elapsed
                percentage = (elapsed / (total_time + 0.0001)) * 100
                logger.info(f"  {phase:25} {elapsed:8.4f}s ({percentage:5.1f}%)")

        logger.info(f"  {'Total':25} {total_time:8.4f}s (100.0%)")

        # Search statistics
        if self.search_stats:
            logger.info("\n🔍 A* Search Statistics:")
            logger.info(f"  Iterations: {self.search_stats.get('iterations', 0):6}")
            logger.info(
                f"  Open Set Size: {self.search_stats.get('open_set_size', 0):6}"
            )
            logger.info(
                f"  Closed Set Size: {self.search_stats.get('closed_set_size', 0):6}"
            )

            budget_s = self.search_stats.get("time_budget_s", 0.0)
            if budget_s:
                cut = " (exhausted)" if self.search_stats.get("is_budget_bound") else ""
                logger.info(f"  Time Budget:  {budget_s:6.2f}s{cut}")

        # Path statistics
        if self.path_stats:
            logger.info("\n📍 Path Statistics:")
            logger.info(
                f"  Total Distance: {self.path_stats.get('total_distance', 0) / 1000:8.2f} km"
            )
            logger.info(f"  Waypoints: {self.path_stats.get('waypoints', 0):6}")
            logger.info(f"  Segments: {self.path_stats.get('segments', 0):6}")

            if self.path_stats.get("segments", 0) > 0:
                logger.info(
                    f"  Avg Segment: {self.path_stats.get('avg_segment', 0):8.1f} m"
                )
                logger.info(
                    f"  Min Segment: {self.path_stats.get('min_segment', 0):8.1f} m"
                )
                logger.info(
                    f"  Max Segment: {self.path_stats.get('max_segment', 0):8.1f} m"
                )

            logger.info(
                f"  Max Turn Angle: {math.degrees(self.path_stats.get('max_turn_angle', 0)):8.2f}°"
            )

            if self.path_stats.get("turns_count", 0) > 0:
                logger.info(
                    f"  Avg Turn Angle: {math.degrees(self.path_stats.get('avg_turn_angle', 0)):8.2f}°"
                )


class PerformanceComparator:
    """Compares performance across multiple scenarios"""

    def __init__(self):
        self.results = []

    def add_result(self, metrics):
        """Add a performance metrics result"""
        self.results.append(metrics.get_summary())

    def print_comparison(self):
        """Print comparison table"""
        if not self.results:
            return

        logger.info("\n" + "=" * 100)
        logger.info("  PERFORMANCE COMPARISON ACROSS ALL SCENARIOS")
        logger.info("=" * 100)

        # Summary table
        logger.info(
            f"\n{'Scenario':<20} {'Status':<10} {'Time (s)':<12} {'Distance (km)':<15} {'Waypoints':<12} {'Iterations':<12}"
        )
        logger.info("─" * 100)

        total_time = 0
        total_distance = 0
        successful = 0

        for result in self.results:
            scenario = result["scenario"]

            # Get timing
            timing_sum = sum(
                data.get("elapsed", 0)
                for data in result["timings"].values()
                if isinstance(data, dict)
            )
            total_time += timing_sum

            # Get path distance
            distance = result["path"].get("total_distance", 0) / 1000
            total_distance += distance

            # Get waypoints
            waypoints = result["path"].get("waypoints", 0)

            # Get iterations
            iterations = result["search"].get("iterations", 0)

            # Status
            status = (
                "✓ SUCCESS" if result["path"].get("waypoints", 0) > 0 else "✗ FAILED"
            )

            logger.info(
                f"{scenario:<20} {status:<10} {timing_sum:>10.4f}s {distance:>13.2f} {waypoints:>11} {iterations:>11}"
            )

            if result["path"].get("waypoints", 0) > 0:
                successful += 1

        logger.info("─" * 100)
        logger.info(
            f"{'TOTAL':<20} {f'{successful}/{len(self.results)}':<10} {total_time:>10.4f}s {total_distance:>13.2f}"
        )
        logger.info("=" * 100)

        # Detailed statistics
        logger.info("\n📊 DETAILED STATISTICS:\n")

        for result in self.results:
            scenario = result["scenario"]
            logger.info(f"\n{scenario}:")

            timing_sum = sum(
                data.get("elapsed", 0)
                for data in result["timings"].values()
                if isinstance(data, dict)
            )
            logger.info(f"  Total Runtime: {timing_sum:.4f}s")

            if result["path"].get("waypoints", 0) > 0:
                logger.info(
                    f"  Distance: {result['path'].get('total_distance', 0) / 1000:.2f} km"
                )
                logger.info(f"  Waypoints: {result['path'].get('waypoints', 0)}")
                logger.info(
                    f"  Max Turn: {math.degrees(result['path'].get('max_turn_angle', 0)):.2f}°"
                )

            logger.info(f"  Iterations: {result['search'].get('iterations', 0)}")
