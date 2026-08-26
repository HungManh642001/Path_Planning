"""The wall-clock budget is the search's ONLY cap, and it is injectable.

``MAX_ITERATIONS`` is gone: a cap counted in iterations says nothing an operator
can act on, and having two independent stop conditions meant a search could end
for a reason the reply never named. What remains is one number with a unit —
``time_budget_s`` — passed per call, falling back to ``config.TIME_BUDGET_S``
only when the caller supplies nothing.
"""

import math

import pytest

from path_planning import config
from path_planning.core import (
    kinodynamic_astar as astar_main,
    kinodynamic_astar_v0 as astar_v0,
    map_generator as mg,
    preprocessing as prep,
)


PLANNERS = pytest.mark.parametrize(
    "planner_module", [astar_main, astar_v0], ids=["main", "v0"]
)


@pytest.fixture(scope="module")
def preprocessed():
    return prep.prepare_scenario(mg.scenario2_single_obstacle())


@PLANNERS
def test_injected_budget_overrides_config(planner_module, preprocessed, monkeypatch):
    monkeypatch.setattr(config, "TIME_BUDGET_S", 15.0)
    planner = planner_module.KinodynamicAstar(preprocessed, time_budget_s=2.5)
    assert planner.time_budget_s == 2.5


@PLANNERS
def test_budget_falls_back_to_config(planner_module, preprocessed, monkeypatch):
    monkeypatch.setattr(config, "TIME_BUDGET_S", 7.5)
    planner = planner_module.KinodynamicAstar(preprocessed)
    assert planner.time_budget_s == 7.5


@PLANNERS
@pytest.mark.parametrize("bad", [0.0, -1.0, math.inf, math.nan])
def test_unusable_budget_is_refused(planner_module, preprocessed, bad):
    with pytest.raises(ValueError):
        planner_module.KinodynamicAstar(preprocessed, time_budget_s=bad)


@PLANNERS
def test_unusable_config_budget_is_refused(planner_module, preprocessed, monkeypatch):
    """``None`` no longer means "unlimited" — it is simply not a budget."""
    monkeypatch.setattr(config, "TIME_BUDGET_S", None)
    with pytest.raises(ValueError):
        planner_module.KinodynamicAstar(preprocessed)


@PLANNERS
def test_stats_report_the_budget_not_an_iteration_cap(planner_module, preprocessed):
    planner = planner_module.KinodynamicAstar(preprocessed, time_budget_s=15.0)
    planner.plan()
    stats = planner.get_search_stats()
    assert "max_iterations" not in stats
    assert stats["time_budget_s"] == 15.0
    assert stats["is_budget_bound"] is False


@PLANNERS
def test_exhausted_budget_is_reported_as_budget_bound(planner_module, preprocessed):
    """A search cut off by the clock must SAY so, not look like an empty map."""
    planner = planner_module.KinodynamicAstar(preprocessed, time_budget_s=1e-9)
    assert planner.search() is None
    stats = planner.get_search_stats()
    assert stats["is_budget_bound"] is True
    # is_search_failed says "ended without a path"; is_budget_bound is what tells the
    # two ways of ending apart.
    assert stats["is_search_failed"] is True


@PLANNERS
def test_plan_trajectory_forwards_the_budget(planner_module, preprocessed):
    result = planner_module.plan_trajectory(preprocessed, time_budget_s=1e-9)
    assert result["is_success"] is False
    assert result["failure_reason"] == "no_path"
    assert result["stats"]["is_budget_bound"] is True
    assert result["stats"]["time_budget_s"] == 1e-9


@PLANNERS
def test_iteration_count_is_not_capped_by_a_constant(planner_module):
    assert not hasattr(config, "MAX_ITERATIONS")
    assert not hasattr(planner_module.KinodynamicAstar, "max_iterations")
