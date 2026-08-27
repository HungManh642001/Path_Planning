"""smooth_path must be able to reproduce its own input.

The DP searches SUBSEQUENCES of the path the search produced, so the identity
subsequence is always available to it — unless one of its gates rejects a corner
the search itself built. That happened: the turn gate used `_alpha_build`
(alpha_max - GEOM_EPS_RAD, the CONSTRUCTION reserve) while re-deriving the turn
from waypoint coordinates. A corner built at the limit reads back as
`_alpha_build + ~3e-15 rad`, so it was rejected, every continuation out of it
died, and the DP returned its "found nothing" fallback — smoothing silently did
nothing, and the delivered path kept every pivot-slide and fan waypoint the
aircraft flies straight through.

The gate belongs at the true limit here because nothing in the DP is
constructed from an angle: it measures corners between existing waypoints with
the oracle's own formula, bit for bit.
"""

import math

from path_planning import config, planner as astar, planner as astar_v0
from path_planning.scenario import preprocessing as prep, presets as mg


PLANNERS = (astar.KinodynamicAstar,)

# A corner at exactly alpha_max, followed by waypoints the aircraft flies
# straight through. l1 = 20 km - R*tan(45 deg) = 12 km, well over L0.
CORNER = (20000.0, 0.0)
GOAL = (20000.0, 200000.0)
PASS_THROUGH = [(20000.0, 40000.0), (20000.0, 80000.0), (20000.0, 120000.0)]


def _planner(cls):
    scenario = mg.create_scenario(
        {
            "map_bounds": (config.MAP_WIDTH, config.MAP_HEIGHT),
            "start": (0.0, 0.0),
            "start_heading": 0.0,
            "goal": GOAL,
            "goal_heading": None,
            "num_islands": 0,
            "num_dynamic_obstacles": 0,
            "seed": 1,
        }
    )
    return cls(prep.prepare_scenario(scenario))


def _path():
    return (
        [(CORNER, math.pi / 2)]
        + [(w, math.pi / 2) for w in PASS_THROUGH]
        + [(GOAL, math.pi / 2)]
    )


def test_a_corner_at_exactly_alpha_max_does_not_disable_smoothing():
    for cls in PLANNERS:
        planner = _planner(cls)
        turn = abs(
            astar_v0._angle_diff(
                math.atan2(
                    PASS_THROUGH[0][1] - CORNER[1], PASS_THROUGH[0][0] - CORNER[0]
                ),
                math.atan2(CORNER[1], CORNER[0]),
            )
        )
        assert turn == planner.alpha_max_rad, (
            "test geometry no longer sits ON the limit"
        )
        assert turn > planner._alpha_build, "test no longer exercises the build reserve"

        path = _path()
        out = planner.smooth_path(path)
        assert out is not path, (
            f"{cls.__module__}: DP fell back to the unsmoothed input"
        )


def test_waypoints_flown_straight_through_are_dropped():
    """A waypoint with no turn costs exactly zero path length, so length alone
    leaves the DP indifferent to it; SMOOTH_NODE_PENALTY_M breaks the tie."""
    for cls in PLANNERS:
        planner = _planner(cls)
        out = planner.smooth_path(_path())
        kept = [w for w, _ in out]
        for w in PASS_THROUGH:
            assert w not in kept, f"{cls.__module__}: kept pass-through waypoint {w}"
        assert CORNER in kept, f"{cls.__module__}: dropped the real corner"
