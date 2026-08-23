from __future__ import annotations

import math
from pathlib import Path

from vtx_service import plan
from vtx_service.map_file import PreloadedMap
from vtx_service.messages import (
    Circle,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    VehicleLimits,
)

LIMITS = VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0)


def _request(**overrides: object) -> PlanRequest:
    base: dict[str, object] = dict(
        request_id=b"\x02" * 16,
        idl_version=1,
        start=(50000.0, 50000.0),
        start_heading_deg=45.0,
        goal=(300000.0, 250000.0),
        goal_heading_deg=45.0,
        goal_heading_free=True,
        islands=(),
        dynamic_obstacles=(),
        safezones=(),
        use_preloaded_map=False,
        limits=LIMITS,
        budget=SearchBudget(15.0, 50000),
    )
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


def test_open_water_mission_succeeds() -> None:
    reply = plan(_request())
    assert reply.status is PlanStatus.OK
    assert reply.detail == ""
    assert len(reply.waypoints) >= 2
    assert reply.path_length_m > 0.0


def test_reply_echoes_the_request_id_and_idl_version() -> None:
    request = _request(request_id=b"\x07" * 16)
    reply = plan(request)
    assert reply.request_id == request.request_id
    assert reply.idl_version == request.idl_version


def test_path_starts_at_takeoff_and_ends_at_the_target() -> None:
    request = _request()
    reply = plan(request)
    assert reply.waypoints[0].position == request.start
    assert reply.waypoints[-1].position == request.goal


def test_first_waypoint_keeps_the_requested_takeoff_bearing() -> None:
    reply = plan(_request(start_heading_deg=45.0))
    assert math.isclose(reply.waypoints[0].heading_deg, 45.0, abs_tol=1e-6)


def test_reply_carries_version_and_config_identity() -> None:
    reply = plan(_request())
    assert reply.planner_version
    assert len(reply.config_hash) == 16


def test_reply_reports_the_budget_it_used_not_the_one_requested() -> None:
    """Mục 4.3: đề nghị của client CHƯA được tôn trọng, và reply nói thật."""
    import config

    reply = plan(_request(budget=SearchBudget(time_budget_s=0.001, max_iterations=7)))
    assert reply.applied_time_budget_s == float(config.TIME_BUDGET_S or 0.0)
    assert reply.stats.max_iterations == config.MAX_ITERATIONS
    assert reply.status is PlanStatus.OK  # ngân sách 1 ms KHÔNG được áp dụng


def test_a_goal_buried_in_an_obstacle_fails_honestly() -> None:
    reply = plan(
        _request(dynamic_obstacles=(Circle(center=(300000.0, 250000.0), radius_m=40000.0),))
    )
    assert reply.status is not PlanStatus.OK
    assert reply.detail != ""


def test_a_wrong_idl_version_is_refused_without_searching() -> None:
    reply = plan(_request(idl_version=999))
    assert reply.status is PlanStatus.INVALID_REQUEST
    assert "idl_version" in reply.detail
    assert reply.stats.iterations == 0
    # Hardcoding 0.0 here would be wrong: a refusal still burns real wall
    # time in the caller's process, and the operator's first check on a
    # refusal is how long it took.
    assert reply.plan_wall_time_s >= 0.0


def test_asking_for_a_map_the_service_does_not_have_is_refused() -> None:
    reply = plan(_request(use_preloaded_map=True), preloaded=None)
    assert reply.status is PlanStatus.INVALID_REQUEST
    assert "preloaded" in reply.detail


def test_the_preloaded_map_actually_changes_the_route(tmp_path: Path) -> None:
    path = tmp_path / "m.xml"
    path.write_text(
        '<vtx-map version="1"><safezones/><obstacles>'
        '<polygon><point x="150000" y="120000"/><point x="200000" y="120000"/>'
        '<point x="175000" y="200000"/></polygon>'
        '<circle cx="220000" cy="180000" r="15000"/>'
        "</obstacles></vtx-map>",
        encoding="utf-8",
    )
    loaded = PreloadedMap.load(path)
    open_water = plan(_request())
    with_basemap = plan(_request(use_preloaded_map=True), preloaded=loaded)
    assert with_basemap.status is PlanStatus.OK
    assert with_basemap.path_length_m > open_water.path_length_m


def test_wall_time_is_measured_and_positive() -> None:
    assert plan(_request()).plan_wall_time_s > 0.0


def test_the_shipped_planner_is_v0_not_main() -> None:
    """Kept separate from ``equivalence_test.py`` on purpose: that test's
    ``test_adapter_is_transparent`` only diverges on 5/18 presets when the
    planner module is swapped (measured Task 8) - alone it is not a reliable
    guard against a planner swap. This one-line identity check is: it fails
    on ANY swap, regardless of which presets happen to agree.
    """
    import core.kinodynamic_astar_v0 as v0
    import vtx_service.planner as planner_module

    assert planner_module.astar is v0
