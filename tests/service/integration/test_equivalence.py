"""Kiểm thử tích hợp tính tương đương và trong suốt giữa service adapter và planner domain."""

from __future__ import annotations

import math

import pytest

from path_planning import config
from path_planning.geometry import spatial
from path_planning.planner import plan_trajectory
from path_planning.scenario.preprocessing import prepare_scenario
from path_planning.scenario.presets import get_all_scenarios
from path_planning.trajectory.mission_path import full_mission_path
from service.vtx_service import plan
from service.vtx_service.angles import math_rad_to_bearing_deg
from service.vtx_service.messages import (
    IDL_VERSION,
    Circle,
    PlanReply,
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
BUDGET = SearchBudget(time_budget_s=float(config.TIME_BUDGET_S))
SCENARIOS = sorted(get_all_scenarios().keys())


def _request_from_scenario(name: str) -> PlanRequest:
    """Dựng một PlanRequest tương đương với một preset benchmark."""
    scenario = get_all_scenarios()[name]()
    goal_heading = scenario["goal_heading"]
    return PlanRequest(
        request_id=name.encode("utf-8")[:16].ljust(16, b"\x00"),
        idl_version=IDL_VERSION,
        start=scenario["start"],
        start_heading_deg=math_rad_to_bearing_deg(scenario["start_heading"]),
        goal=scenario["goal"],
        goal_heading_deg=0.0
        if goal_heading is None
        else math_rad_to_bearing_deg(goal_heading),
        is_goal_heading_free=goal_heading is None,
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


def _direct_plan(request: PlanRequest) -> tuple[dict, list]:
    """Gọi thẳng thuật toán trên chính dict Scenario do adapter dựng ra."""
    preprocessed = prepare_scenario(
        build_scenario(request),
        turn_radius=request.limits.turn_radius_m,
        l0=request.limits.l0_m,
        dss=request.limits.dss_m,
        safe_margin=request.limits.safe_margin_m,
        alpha_max_rad=math.radians(request.limits.alpha_max_deg),
    )
    result = plan_trajectory(preprocessed)
    full = full_mission_path(result["path"], preprocessed) if result["path"] else []
    return result, full


@pytest.mark.parametrize("name", SCENARIOS)
def test_adapter_is_transparent_and_bit_identical(name: str) -> None:
    """Kiểm tra gọi qua service cho kết quả bit-identical so với gọi trực tiếp core planner."""
    # Arrange
    request = _request_from_scenario(name)
    result, expected_full = _direct_plan(request)

    # Act
    reply: PlanReply = plan(request)

    # Assert
    assert (reply.status is PlanStatus.OK) == result["is_success"]
    assert len(reply.waypoints) == len(expected_full)
    for got, (position, heading) in zip(reply.waypoints, expected_full, strict=True):
        assert got.position == position
        assert got.heading_deg == pytest.approx(
            math_rad_to_bearing_deg(heading), abs=1e-9
        )

    expected_length = spatial.calculate_dubins_path_length(
        expected_full, request.limits.turn_radius_m
    )
    assert reply.path_length_m == pytest.approx(expected_length, rel=0.0, abs=1e-9)
    assert reply.stats.iterations == result["stats"]["iterations"]


def test_every_preset_still_solves_through_the_service() -> None:
    """Kiểm tra tất cả 16 preset đều giải thành công với status OK qua service."""
    # Arrange & Act
    failures = [
        name
        for name in SCENARIOS
        if plan(_request_from_scenario(name)).status is not PlanStatus.OK
    ]

    # Assert
    assert failures == [], f"Service thất bại trên các kịch bản: {failures}"
