"""Kiểm thử đơn vị cho module chuyển đổi kịch bản service.vtx_service.scenario_builder."""

from __future__ import annotations

import math
from typing import get_type_hints

from path_planning.planner import plan_trajectory
from path_planning.scenario.preprocessing import prepare_scenario
from path_planning.types import Scenario
from service.vtx_service.angles import bearing_deg_to_math_rad
from service.vtx_service.messages import (
    IDL_VERSION,
    Circle,
    PlanRequest,
    SearchBudget,
    VehicleLimits,
)
from service.vtx_service.scenario_builder import build_scenario

LIMITS = VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0)


def _build_request(**overrides: object) -> PlanRequest:
    """Khởi tạo PlanRequest mặc định cho test chuyển đổi kịch bản."""
    base: dict[str, object] = {
        "request_id": b"\x01" * 16,
        "idl_version": IDL_VERSION,
        "start": (50000.0, 50000.0),
        "start_heading_deg": 90.0,
        "goal": (300000.0, 200000.0),
        "goal_heading_deg": 45.0,
        "is_goal_heading_free": False,
        "islands": (((100000.0, 100000.0), (120000.0, 100000.0), (110000.0, 130000.0)),),
        "dynamic_obstacles": (Circle(center=(200000.0, 150000.0), radius_m=12000.0),),
        "safezones": (),
        "use_preloaded_map": False,
        "limits": LIMITS,
        "budget": SearchBudget(15.0),
    }
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


def test_builder_fills_every_key_in_scenario_typeddict() -> None:
    """Kiểm tra build_scenario điền đầy đủ 100% tất cả các trường được khai báo trong Scenario TypedDict."""
    # Arrange & Act
    built = build_scenario(_build_request())

    # Assert
    assert set(built) == set(get_type_hints(Scenario))


def test_coordinates_pass_through_identically() -> None:
    """Kiểm tra tọa độ điểm start, goal và đỉnh đảo được giữ nguyên giá trị."""
    # Arrange
    req = _build_request()

    # Act
    built = build_scenario(req)

    # Assert
    assert built["start"] == req.start
    assert built["goal"] == req.goal
    assert built["islands"][0][0] == req.islands[0][0]


def test_headings_are_converted_to_mathematical_convention() -> None:
    """Kiểm tra góc phương vị (90 độ đông) được chuyển đổi sang radian toán học (+x = 0 rad)."""
    # Arrange & Act
    built = build_scenario(_build_request())

    # Assert
    assert math.isclose(built["start_heading"], 0.0, abs_tol=1e-12)


def test_goal_heading_is_converted_when_fixed() -> None:
    """Kiểm tra hướng đích cố định (45 độ) được chuyển đổi chính xác."""
    # Arrange & Act
    built = build_scenario(_build_request())

    # Assert
    assert built["goal_heading"] == bearing_deg_to_math_rad(45.0)


def test_free_goal_heading_becomes_none() -> None:
    """Kiểm tra cờ is_goal_heading_free=True chuyển goal_heading thành None."""
    # Arrange & Act
    built = build_scenario(_build_request(is_goal_heading_free=True))

    # Assert
    assert built["goal_heading"] is None


def test_obstacles_list_contains_tagged_union_circles_and_polygons() -> None:
    """Kiểm tra danh sách obstacles chứa đúng định dạng tagged union."""
    # Arrange & Act
    built = build_scenario(_build_request())

    # Assert
    assert sorted(o["type"] for o in built["obstacles"]) == ["circle", "polygon"]
    circle = next(o for o in built["obstacles"] if o["type"] == "circle")
    assert circle["center"] == (200000.0, 150000.0)
    assert circle["radius"] == 12000.0


def test_built_scenario_executes_through_planner_pipeline() -> None:
    """Kiểm tra kịch bản được build chạy thành công qua pipeline quy hoạch đường bay."""
    # Arrange
    built = build_scenario(_build_request())
    preprocessed = prepare_scenario(
        built,
        turn_radius=LIMITS.turn_radius_m,
        l0=LIMITS.l0_m,
        dss=LIMITS.dss_m,
        safe_margin=LIMITS.safe_margin_m,
        alpha_max_rad=math.radians(LIMITS.alpha_max_deg),
    )

    # Act
    result = plan_trajectory(preprocessed)

    # Assert
    assert result["is_success"] is True
