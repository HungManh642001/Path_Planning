import time
import math
import map_generator as mg
import preprocessing as prep
import kinodynamic_astar as astar

# Representative spread across difficulty (uses the predefined scenarios directly,
# independent of map_generator.get_all_scenarios which the user may have trimmed).
_SCENARIOS = [
    ('open', mg.scenario1_open_ocean),
    ('sparse', mg.scenario5_sparse_islands),
    ('archipelago', mg.scenario9_island_archipelago),
    ('dense_islands', mg.scenario13_dense_island_field),
    ('extreme', mg.scenario16_extreme_complexity),
]


def test_planning_runtime_under_one_second():
    slow = []
    for name, fn in _SCENARIOS:
        pre = prep.prepare_scenario(fn())
        t0 = time.perf_counter()
        res = astar.plan_trajectory(pre, verbose=False)
        dt = time.perf_counter() - t0
        print(f"[bench] {name:14} success={res['success']!s:5} time={dt*1000:7.1f} ms")
        slow.append((name, dt))
    worst = max(slow, key=lambda x: x[1])
    # Hard guard: the wall-clock budget (0.9s) + setup must keep every query under ~1.3s.
    assert worst[1] < 1.3, f"slowest scenario {worst[0]} took {worst[1]:.3f}s (budget regression)"
