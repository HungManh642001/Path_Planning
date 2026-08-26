"""Cơ chế cưỡng chế số 2: adapter không được làm sai lệch bất cứ điều gì.

Hai khẳng định tách bạch, và việc tách chúng ra là có lý do.

`test_adapter_is_transparent` so đường bay qua service với đường bay khi gọi
thẳng thuật toán TRÊN CÙNG MỘT dict Scenario. Yêu cầu là bit-identical. Cả hai
vế đều gọi thuật toán HIỆN HÀNH, nên test không bao giờ lỗi thời: thuật toán đổi
thì hai vế đổi cùng nhau và test vẫn xanh; adapter lệch đi thì đỏ ngay.

`test_every_preset_still_solves_through_the_service` KHÔNG đòi bit-identical, vì
có một khác biệt ngữ nghĩa cố ý: preset mang `map_bounds = (500000, 500000)`
còn IDL bỏ trường đó (spec mục 4.2), nên service chạy ở chế độ không giới hạn.
Đòi bit-identical ở đây là ép hai thứ khác nhau phải giống nhau.
"""

from __future__ import annotations

import math

from path_planning import config
from path_planning.core import kinodynamic_astar_v0 as astar
from path_planning.core import map_generator as mg
from path_planning.core import mission as mission
from path_planning.core import preprocessing as prep
import pytest

from service.vtx_service import plan
from service.vtx_service.angles import math_rad_to_bearing_deg
from service.vtx_service.messages import (
    IDL_VERSION,
    Circle,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    VehicleLimits,
)
from service.vtx_service.scenario_builder import build_scenario

LIMITS = VehicleLimits(
    turn_radius_m=config.R,
    l0_m=config.L0,
    dss_m=config.DSS,
    safe_margin_m=config.SAFE_MARGIN,
    alpha_max_deg=config.ALPHA_MAX,
)
# Đúng mặc định của service, nên đường đi qua service phải TRÙNG KHỚP đường
# đi gọi thẳng planner - đó là điều bộ test này kiểm tra.
BUDGET = SearchBudget(time_budget_s=float(config.TIME_BUDGET_S))
SCENARIOS = sorted(mg.get_all_scenarios())


def _request_from_scenario(name: str) -> PlanRequest:
    """Dựng một request tương đương với một preset."""
    scenario = mg.get_all_scenarios()[name]()
    goal_heading = scenario["goal_heading"]
    return PlanRequest(
        request_id=name.encode("utf-8")[:16].ljust(16, b"\x00"),
        idl_version=IDL_VERSION,
        start=scenario["start"],
        start_heading_deg=math_rad_to_bearing_deg(scenario["start_heading"]),
        goal=scenario["goal"],
        goal_heading_deg=0.0 if goal_heading is None else math_rad_to_bearing_deg(goal_heading),
        goal_heading_free=goal_heading is None,
        islands=tuple(tuple(tuple(v) for v in poly) for poly in scenario["islands"]),
        dynamic_obstacles=tuple(
            Circle(center=tuple(center), radius_m=radius)
            for center, radius in scenario["dynamic_obstacles"]
        ),
        safezones=(),
        use_preloaded_map=False,
        limits=LIMITS,
        budget=BUDGET,
    )


def _direct_plan(request: PlanRequest):
    """Gọi thẳng thuật toán trên CHÍNH dict Scenario mà adapter dựng ra."""
    preprocessed = prep.prepare_scenario(
        build_scenario(request),
        turn_radius=request.limits.turn_radius_m,
        l0=request.limits.l0_m,
        dss=request.limits.dss_m,
        safe_margin=request.limits.safe_margin_m,
        alpha_max_rad=math.radians(request.limits.alpha_max_deg),
    )
    result = astar.plan_trajectory(preprocessed)
    full = mission.full_mission_path(result["path"], preprocessed) if result["path"] else []
    return result, full


@pytest.mark.parametrize("name", SCENARIOS)
def test_adapter_is_transparent(name: str) -> None:
    request = _request_from_scenario(name)
    result, expected_full = _direct_plan(request)
    reply = plan(request)

    assert (reply.status is PlanStatus.OK) == result["success"]
    assert len(reply.waypoints) == len(expected_full)
    for got, (position, heading) in zip(reply.waypoints, expected_full, strict=True):
        # Bit-identical: không có phép toán nào chạm vào toạ độ.
        assert got.position == position
        assert got.heading_deg == pytest.approx(math_rad_to_bearing_deg(heading), abs=1e-9)

    expected_length = sum(
        math.dist(expected_full[i][0], expected_full[i + 1][0])
        for i in range(len(expected_full) - 1)
    )
    assert reply.path_length_m == pytest.approx(expected_length, rel=0.0, abs=1e-9)
    assert reply.stats.iterations == result["stats"]["iterations"]


def test_every_preset_still_solves_through_the_service() -> None:
    failures = [
        name
        for name in SCENARIOS
        if plan(_request_from_scenario(name)).status is not PlanStatus.OK
    ]
    assert failures == [], f"service làm mất mission: {failures}"


@pytest.mark.parametrize("name", SCENARIOS)
def test_service_does_not_lengthen_the_route_against_the_preset(name: str) -> None:
    """So với preset NGUYÊN BẢN (còn map_bounds), không phải dict của adapter."""
    scenario = mg.get_all_scenarios()[name]()
    preprocessed = prep.prepare_scenario(scenario)
    result = astar.plan_trajectory(preprocessed)
    full = mission.full_mission_path(result["path"], preprocessed)
    baseline = sum(math.dist(full[i][0], full[i + 1][0]) for i in range(len(full) - 1))

    reply = plan(_request_from_scenario(name))
    assert reply.status is PlanStatus.OK
    assert reply.path_length_m <= baseline * 1.005
