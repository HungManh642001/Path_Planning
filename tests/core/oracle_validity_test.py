"""The invariant the old suite lacked: every plan the planner reports as a
is_success must be accepted by the independent oracle (with the shared circle
tolerance) over the FULL O..T path, including the fixed legs."""

import pytest
from batch_random_test import generate_random_scenario

from path_planning import config
from path_planning.core import (
    kinodynamic_astar as astar,
    path_validation as pv,
    preprocessing as prep,
)
from path_planning.render import trajectory as tr


SEEDS = list(range(60))  # fast subset; the full 1000-seed sweep is Step 4


@pytest.mark.parametrize("seed", SEEDS)
def test_successful_plan_is_oracle_valid(seed):
    scn = generate_random_scenario(seed=seed)
    pre = prep.prepare_scenario(scn)
    result = astar.plan_trajectory(pre)
    if not result["is_success"]:
        # A reported failure carries a reason and is not asserted for validity.
        assert result["failure_reason"] in (
            "no_path",
            "start_leg_blocked",
            "goal_leg_blocked",
            "path_self_collision",
        )
        return
    full = tr.build_full_path(result["path"], pre)
    rawc = [
        (o["center"], o["radius"]) for o in scn["obstacles"] if o["type"] == "circle"
    ]
    rawp = [o["polygon"] for o in scn["obstacles"] if o["type"] == "polygon"]
    assert pv.path_is_valid(
        full,
        pre["circle_obstacles"],
        pre["polygon_obstacles"],
        config.R,
        config.ALPHA_MAX_RAD,
        config.L0,
        config.DSS,
        raw_circle_obstacles=rawc,
        raw_polygon_obstacles=rawp,
    ), f"seed {seed}: reported is_success but oracle rejected the full path"
