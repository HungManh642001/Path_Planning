"""Cơ chế cưỡng chế số 1: adapter phải điền đủ mọi khoá `Scenario` khai báo.

Nếu `core.types.Scenario` mọc thêm một khoá bắt buộc, test này đỏ ngay thay vì
để một `KeyError` nổ ra giữa lúc chạy thật.
"""

from __future__ import annotations

import math
from typing import get_type_hints

from core.types import Scenario

from vtx_service.angles import bearing_deg_to_math_rad
from vtx_service.messages import (
    IDL_VERSION,
    Circle,
    PlanRequest,
    SearchBudget,
    VehicleLimits,
)
from vtx_service.scenario_builder import build_scenario

LIMITS = VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0)


def _request(**overrides: object) -> PlanRequest:
    base: dict[str, object] = dict(
        request_id=b"\x01" * 16,
        idl_version=IDL_VERSION,
        start=(50000.0, 50000.0),
        start_heading_deg=90.0,
        goal=(300000.0, 200000.0),
        goal_heading_deg=45.0,
        goal_heading_free=False,
        islands=(((100000.0, 100000.0), (120000.0, 100000.0), (110000.0, 130000.0)),),
        dynamic_obstacles=(Circle(center=(200000.0, 150000.0), radius_m=12000.0),),
        safezones=(),
        use_preloaded_map=False,
        limits=LIMITS,
        budget=SearchBudget(15.0),
    )
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


def test_builder_fills_every_key_the_scenario_type_declares() -> None:
    assert set(build_scenario(_request())) == set(get_type_hints(Scenario))


def test_coordinates_pass_through_bit_identically() -> None:
    """Không có phép chiếu nào; test tương đương dựa vào điều này."""
    request = _request()
    built = build_scenario(request)
    assert built["start"] == request.start
    assert built["goal"] == request.goal
    assert built["islands"][0][0] == request.islands[0][0]


def test_headings_are_converted_to_the_planner_convention() -> None:
    # phương vị 90 = đông = +x = 0 rad
    assert math.isclose(build_scenario(_request())["start_heading"], 0.0, abs_tol=1e-12)


def test_goal_heading_is_converted_to_the_planner_convention_when_fixed() -> None:
    # goal_heading_deg=45.0 trong fixture; so với angles.py, không tự suy công thức.
    built = build_scenario(_request())
    assert built["goal_heading"] == bearing_deg_to_math_rad(45.0)


def test_free_goal_becomes_none_not_a_sentinel_number() -> None:
    assert build_scenario(_request(goal_heading_free=True))["goal_heading"] is None


def test_map_bounds_is_deliberately_none() -> None:
    """Spec mục 4.2: map_bounds neo tại gốc toạ độ; safezones mạnh hơn."""
    assert build_scenario(_request())["map_bounds"] is None


def test_empty_safezones_becomes_none_so_the_planner_stays_permissive() -> None:
    assert build_scenario(_request(safezones=()))["safezones"] is None


def test_safezones_are_passed_through_when_present() -> None:
    zone = ((0.0, 0.0), (400000.0, 0.0), (400000.0, 400000.0))
    assert build_scenario(_request(safezones=(zone,)))["safezones"] == [list(zone)]


def test_obstacles_is_the_tagged_union_the_pipeline_consumes() -> None:
    built = build_scenario(_request())
    assert sorted(o["type"] for o in built["obstacles"]) == ["circle", "polygon"]
    circle = next(o for o in built["obstacles"] if o["type"] == "circle")
    assert circle["center"] == (200000.0, 150000.0)
    assert circle["radius"] == 12000.0


def test_the_built_scenario_actually_runs_through_the_pipeline() -> None:
    import core.kinodynamic_astar_v0 as astar
    import core.preprocessing as prep

    preprocessed = prep.prepare_scenario(
        build_scenario(_request()),
        turn_radius=LIMITS.turn_radius_m,
        l0=LIMITS.l0_m,
        dss=LIMITS.dss_m,
        safe_margin=LIMITS.safe_margin_m,
        alpha_max_rad=math.radians(LIMITS.alpha_max_deg),
    )
    assert astar.plan_trajectory(preprocessed)["success"] is True
