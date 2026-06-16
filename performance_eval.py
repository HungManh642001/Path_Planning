"""
Performance Evaluation Module
Measures and analyzes algorithm runtime, path quality, and system efficiency
"""

import time
import math
from collections import defaultdict


class PerformanceMetrics:
    """Tracks performance metrics for path planning"""
    
    def __init__(self, scenario_name):
        self.scenario_name = scenario_name
        self.timings = {}
        self.path_stats = {}
        self.search_stats = {}
    
    def start_timer(self, phase_name):
        """Start timing a phase"""
        self.timings[phase_name] = {'start': time.time()}
    
    def end_timer(self, phase_name):
        """End timing a phase"""
        if phase_name in self.timings:
            elapsed = time.time() - self.timings[phase_name]['start']
            self.timings[phase_name]['elapsed'] = elapsed
            return elapsed
        return None
    
    def record_path_stats(self, path, preprocessed):
        """Record statistics about the planned path"""
        if not path:
            return
        
        waypoints = [wp for wp, _ in path]
        headings = [h for _, h in path]
        
        # Calculate distances
        segment_distances = []
        total_distance = 0
        
        for i in range(len(waypoints) - 1):
            wp1 = waypoints[i]
            wp2 = waypoints[i + 1]
            dist = math.sqrt((wp2[0] - wp1[0])**2 + (wp2[1] - wp1[1])**2)
            segment_distances.append(dist)
            total_distance += dist
        
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
            'waypoints': len(waypoints),
            'segments': len(segment_distances),
            'total_distance': total_distance,
            'avg_segment': total_distance / len(segment_distances) if segment_distances else 0,
            'min_segment': min(segment_distances) if segment_distances else 0,
            'max_segment': max(segment_distances) if segment_distances else 0,
            'max_turn_angle': max_turn,
            'avg_turn_angle': sum(turn_angles) / len(turn_angles) if turn_angles else 0,
            'turns_count': len(turn_angles),
        }
    
    def record_search_stats(self, search_result):
        """Record A* search statistics"""
        if search_result and search_result.get('stats'):
            self.search_stats = search_result['stats'].copy()
    
    def get_summary(self):
        """Get summary report"""
        return {
            'scenario': self.scenario_name,
            'timings': self.timings,
            'path': self.path_stats,
            'search': self.search_stats,
        }
    
    def print_report(self):
        """Print formatted performance report"""
        print(f"\n{'─' * 70}")
        print(f"Performance Report: {self.scenario_name}")
        print(f"{'─' * 70}")
        
        # Timing breakdown
        print("\n⏱️  Timing Breakdown:")
        total_time = 0
        for phase, data in self.timings.items():
            if 'elapsed' in data:
                elapsed = data['elapsed']
                total_time += elapsed
                percentage = (elapsed / (total_time + 0.0001)) * 100
                print(f"  {phase:25} {elapsed:8.4f}s ({percentage:5.1f}%)")
        
        print(f"  {'Total':25} {total_time:8.4f}s (100.0%)")
        
        # Search statistics
        if self.search_stats:
            print("\n🔍 A* Search Statistics:")
            print(f"  Iterations: {self.search_stats.get('iterations', 0):6}/{self.search_stats.get('max_iterations', 0)}")
            print(f"  Open Set Size: {self.search_stats.get('open_set_size', 0):6}")
            print(f"  Closed Set Size: {self.search_stats.get('closed_set_size', 0):6}")
            
            if self.search_stats.get('iterations', 0) > 0 and self.search_stats.get('max_iterations', 0) > 0:
                efficiency = self.search_stats['iterations'] / self.search_stats['max_iterations']
                print(f"  Efficiency: {efficiency*100:6.2f}%")
        
        # Path statistics
        if self.path_stats:
            print("\n📍 Path Statistics:")
            print(f"  Total Distance: {self.path_stats.get('total_distance', 0)/1000:8.2f} km")
            print(f"  Waypoints: {self.path_stats.get('waypoints', 0):6}")
            print(f"  Segments: {self.path_stats.get('segments', 0):6}")
            
            if self.path_stats.get('segments', 0) > 0:
                print(f"  Avg Segment: {self.path_stats.get('avg_segment', 0):8.1f} m")
                print(f"  Min Segment: {self.path_stats.get('min_segment', 0):8.1f} m")
                print(f"  Max Segment: {self.path_stats.get('max_segment', 0):8.1f} m")
            
            print(f"  Max Turn Angle: {math.degrees(self.path_stats.get('max_turn_angle', 0)):8.2f}°")
            
            if self.path_stats.get('turns_count', 0) > 0:
                print(f"  Avg Turn Angle: {math.degrees(self.path_stats.get('avg_turn_angle', 0)):8.2f}°")


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
        
        print("\n" + "=" * 100)
        print("  PERFORMANCE COMPARISON ACROSS ALL SCENARIOS")
        print("=" * 100)
        
        # Summary table
        print(f"\n{'Scenario':<20} {'Status':<10} {'Time (s)':<12} {'Distance (km)':<15} {'Waypoints':<12} {'Iterations':<12}")
        print("─" * 100)
        
        total_time = 0
        total_distance = 0
        successful = 0
        
        for result in self.results:
            scenario = result['scenario']
            
            # Get timing
            timing_sum = sum(
                data.get('elapsed', 0) 
                for data in result['timings'].values() 
                if isinstance(data, dict)
            )
            total_time += timing_sum
            
            # Get path distance
            distance = result['path'].get('total_distance', 0) / 1000
            total_distance += distance
            
            # Get waypoints
            waypoints = result['path'].get('waypoints', 0)
            
            # Get iterations
            iterations = result['search'].get('iterations', 0)
            
            # Status
            status = "✓ SUCCESS" if result['path'].get('waypoints', 0) > 0 else "✗ FAILED"
            
            print(f"{scenario:<20} {status:<10} {timing_sum:>10.4f}s {distance:>13.2f} {waypoints:>11} {iterations:>11}")
            
            if result['path'].get('waypoints', 0) > 0:
                successful += 1
        
        print("─" * 100)
        print(f"{'TOTAL':<20} {f'{successful}/{len(self.results)}':<10} {total_time:>10.4f}s {total_distance:>13.2f}")
        print("=" * 100)
        
        # Detailed statistics
        print("\n📊 DETAILED STATISTICS:\n")
        
        for result in self.results:
            scenario = result['scenario']
            print(f"\n{scenario}:")
            
            timing_sum = sum(
                data.get('elapsed', 0) 
                for data in result['timings'].values() 
                if isinstance(data, dict)
            )
            print(f"  Total Runtime: {timing_sum:.4f}s")
            
            if result['path'].get('waypoints', 0) > 0:
                print(f"  Distance: {result['path'].get('total_distance', 0)/1000:.2f} km")
                print(f"  Waypoints: {result['path'].get('waypoints', 0)}")
                print(f"  Max Turn: {math.degrees(result['path'].get('max_turn_angle', 0)):.2f}°")
            
            print(f"  Iterations: {result['search'].get('iterations', 0)}")


def format_time_breakdown(timings):
    """Format timing breakdown as string"""
    output = []
    total = sum(t.get('elapsed', 0) for t in timings.values() if isinstance(t, dict))
    
    for phase, data in timings.items():
        if isinstance(data, dict) and 'elapsed' in data:
            elapsed = data['elapsed']
            pct = (elapsed / total * 100) if total > 0 else 0
            output.append(f"{phase}: {elapsed:.4f}s ({pct:.1f}%)")
    
    return " | ".join(output)


def estimate_flight_time(path_distance, cruise_speed=600):
    """
    Estimate actual flight time for missile.
    
    Args:
        path_distance: Total path distance in meters
        cruise_speed: Cruise speed in m/s (default ~2160 km/h)
    
    Returns:
        Flight time in seconds
    """
    if cruise_speed <= 0:
        return 0
    
    return path_distance / cruise_speed


def calculate_path_efficiency(direct_distance, actual_distance):
    """
    Calculate path efficiency ratio.
    
    Args:
        direct_distance: Straight-line distance from start to goal
        actual_distance: Actual planned path distance
    
    Returns:
        Efficiency ratio (0 to 1, where 1 is optimal)
    """
    if direct_distance <= 0 or actual_distance <= 0:
        return 0
    
    return direct_distance / actual_distance
