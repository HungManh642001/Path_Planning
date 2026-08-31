"""Kiểm thử tích hợp cho dịch vụ lập kế hoạch đường bay service.vtx_service.planner."""

from __future__ import annotations

import math
from pathlib import Path

from path_planning import config
from path_planning import planner as path_planning_planner
from service.vtx_service import plan
from service.vtx_service import planner as service_planner_module
from service.vtx_service.map_file import PreloadedMap
from service.vtx_service.messages import (
    IDL_VERSION,
    Circle,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    VehicleLimits,
)

LIMITS = VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0)


def _build_request(**overrides: object) -> PlanRequest:
    """Khởi tạo PlanRequest chuẩn cho test planner service."""
    base: dict[str, object] = {
        "request_id": b"\x02" * 16,
        "idl_version": IDL_VERSION,
        "start": (50000.0, 50000.0),
        "start_heading_deg": 45.0,
        "goal": (300000.0, 250000.0),
        "goal_heading_deg": 45.0,
        "is_goal_heading_free": True,
        "islands": (),
        "dynamic_obstacles": (),
        "safezones": (),
        "use_preloaded_map": False,
        "limits": LIMITS,
        "budget": SearchBudget(15.0),
    }
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


def test_open_water_mission_succeeds_with_ok_status() -> None:
    """Kiểm tra nhiệm vụ vùng nước trống lập kế hoạch thành công với status OK."""
    # Arrange
    req = _build_request()

    # Act
    reply = plan(req)

    # Assert
    assert reply.status is PlanStatus.OK
    assert reply.detail == ""
    assert len(reply.waypoints) >= 2
    assert reply.path_length_m > 0.0


def test_reply_echoes_request_id_and_idl_version() -> None:
    """Kiểm tra phản hồi giữ nguyên request_id và idl_version của client."""
    # Arrange
    req = _build_request(request_id=b"\x07" * 16)

    # Act
    reply = plan(req)

    # Assert
    assert reply.request_id == req.request_id
    assert reply.idl_version == req.idl_version


def test_path_starts_at_takeoff_and_ends_at_target() -> None:
    """Kiểm tra waypoint đầu tiên là điểm cất cánh và waypoint cuối cùng là đích."""
    # Arrange
    req = _build_request()

    # Act
    reply = plan(req)

    # Assert
    assert reply.waypoints[0].position == req.start
    assert reply.waypoints[-1].position == req.goal


def test_first_waypoint_keeps_requested_takeoff_bearing() -> None:
    """Kiểm tra waypoint đầu tiên giữ nguyên góc phương vị cất cánh được yêu cầu."""
    # Arrange
    req = _build_request(start_heading_deg=45.0)

    # Act
    reply = plan(req)

    # Assert
    assert math.isclose(reply.waypoints[0].heading_deg, 45.0, abs_tol=1e-6)


def test_tiny_budget_triggers_budget_bound_no_path() -> None:
    """Kiểm tra ngân sách siêu nhỏ dẫn đến trạng thái NO_PATH do chạm ngưỡng thời gian."""
    # Arrange
    req = _build_request(budget=SearchBudget(time_budget_s=1e-9))

    # Act
    reply = plan(req)

    # Assert
    assert reply.applied_time_budget_s == 1e-9
    assert reply.status is PlanStatus.NO_PATH
    assert reply.stats.is_budget_bound is True


def test_empty_budget_falls_back_to_service_default() -> None:
    """Kiểm tra ngân sách rỗng (0.0s) tự động rơi về giá trị mặc định của hệ thống."""
    # Arrange
    req = _build_request(budget=SearchBudget(time_budget_s=0.0))

    # Act
    reply = plan(req)

    # Assert
    assert reply.applied_time_budget_s == float(config.TIME_BUDGET_S)
    assert reply.status is PlanStatus.OK


def test_unbuildable_geometry_returns_invalid_request() -> None:
    """Kiểm tra hình học không hợp lệ (đa giác 2 đỉnh) trả về INVALID_REQUEST mà không làm sập process."""
    # Arrange
    req = _build_request(islands=(((0.0, 0.0), (1000.0, 1000.0)),))

    # Act
    reply = plan(req)

    # Assert
    assert reply.status is PlanStatus.INVALID_REQUEST
    assert reply.detail != ""


def test_goal_buried_in_obstacle_fails_cleanly() -> None:
    """Kiểm tra đích nằm chìm trong vật cản báo lỗi chính xác."""
    # Arrange
    req = _build_request(
        dynamic_obstacles=(Circle(center=(300000.0, 250000.0), radius_m=40000.0),)
    )

    # Act
    reply = plan(req)

    # Assert
    assert reply.status is not PlanStatus.OK
    assert reply.detail != ""


def test_wrong_idl_version_is_rejected_without_search() -> None:
    """Kiểm tra phiên bản IDL không khớp bị từ chối ngay lập tức với 0 vòng lặp tìm kiếm."""
    # Arrange
    req = _build_request(idl_version=999)

    # Act
    reply = plan(req)

    # Assert
    assert reply.status is PlanStatus.INVALID_REQUEST
    assert "idl_version" in reply.detail
    assert reply.stats.iterations == 0


def test_preloaded_map_changes_planned_trajectory(tmp_path: Path) -> None:
    """Kiểm tra nạp bản đồ nền thay đổi đường bay khi có vật cản bổ sung."""
    # Arrange
    map_file = tmp_path / "basemap.xml"
    map_file.write_text(
        '<vtx-map version="1"><safezones/><obstacles>'
        '<polygon><point x="150000" y="120000"/><point x="200000" y="120000"/>'
        '<point x="175000" y="200000"/></polygon>'
        '<circle cx="220000" cy="180000" r="15000"/>'
        "</obstacles></vtx-map>",
        encoding="utf-8",
    )
    loaded = PreloadedMap.load(map_file)
    open_water = plan(_build_request())

    # Act
    with_basemap = plan(_build_request(use_preloaded_map=True), preloaded=loaded)

    # Assert
    assert with_basemap.status is PlanStatus.OK
    assert with_basemap.path_length_m > open_water.path_length_m


def test_shipped_planner_is_canonical_module() -> None:
    """Kiểm tra service liên kết trực tiếp với module path_planning.planner chuẩn."""
    # Arrange & Act & Assert
    assert service_planner_module.astar is path_planning_planner
