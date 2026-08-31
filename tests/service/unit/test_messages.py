"""Kiểm thử đơn vị cho module định nghĩa thông điệp DDS service.vtx_service.messages."""

from __future__ import annotations

import dataclasses

import pytest

from service.vtx_service.messages import (
    IDL_VERSION,
    Circle,
    PlanReply,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    SearchStats,
    VehicleLimits,
    Waypoint,
)


def _build_request(**overrides: object) -> PlanRequest:
    """Hàm trợ giúp khởi tạo đối tượng PlanRequest với các tham số mặc định hợp lệ."""
    base: dict[str, object] = {
        "request_id": b"\x00" * 16,
        "idl_version": IDL_VERSION,
        "start": (0.0, 0.0),
        "start_heading_deg": 0.0,
        "goal": (100000.0, 0.0),
        "goal_heading_deg": 90.0,
        "is_goal_heading_free": False,
        "islands": (),
        "dynamic_obstacles": (),
        "safezones": (),
        "use_preloaded_map": False,
        "limits": VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0),
        "budget": SearchBudget(15.0),
    }
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


def test_plan_request_is_immutable_frozen_dataclass() -> None:
    """Kiểm tra PlanRequest là immutable và không thể sửa đổi sau khi khởi tạo."""
    # Arrange
    req = _build_request()

    # Act & Assert
    with pytest.raises(dataclasses.FrozenInstanceError):
        req.start = (1.0, 1.0)  # type: ignore[misc]


def test_plan_request_requires_exact_sixteen_byte_request_id() -> None:
    """Kiểm tra request_id bắt buộc phải có độ dài đúng 16 bytes (UUID)."""
    # Arrange & Act & Assert
    with pytest.raises(ValueError, match="16 byte"):
        _build_request(request_id=b"\x00" * 8)


def test_plan_status_enum_values_match_wire_contract() -> None:
    """Kiểm tra giá trị nguyên của PlanStatus khớp hoàn toàn với đặc tả IDL."""
    # Arrange & Act
    statuses = [int(member) for member in PlanStatus]

    # Assert
    assert statuses == list(range(9))
    assert PlanStatus.OK == 0
    assert PlanStatus.ORACLE_REJECTED == 4
    assert PlanStatus.BUSY == 8


def test_plan_reply_reports_actual_applied_time_budget() -> None:
    """Kiểm tra phản hồi PlanReply chứa trường applied_time_budget_s."""
    # Arrange & Act
    fields = {f.name for f in dataclasses.fields(PlanReply)}

    # Assert
    assert "applied_time_budget_s" in fields


def test_search_budget_contains_only_time_budget_cap() -> None:
    """Kiểm tra SearchBudget chỉ chứa time_budget_s và không chứa iteration cap."""
    # Arrange & Act
    budget_fields = {f.name for f in dataclasses.fields(SearchBudget)}
    stats_fields = {f.name for f in dataclasses.fields(SearchStats)}

    # Assert
    assert budget_fields == {"time_budget_s"}
    assert "max_iterations" not in stats_fields


def test_circle_geometry_rejects_non_positive_radius() -> None:
    """Kiểm tra Circle từ chối bán kính không dương (<= 0)."""
    # Arrange & Act & Assert
    with pytest.raises(ValueError, match="radius"):
        Circle(center=(0.0, 0.0), radius_m=0.0)


def test_vehicle_limits_rejects_negative_safe_margin() -> None:
    """Kiểm tra VehicleLimits từ chối khoảng cách an toàn safe_margin_m âm."""
    # Arrange & Act & Assert
    with pytest.raises(ValueError, match="safe_margin_m"):
        VehicleLimits(8000.0, 8000.0, 15000.0, -1.0, 90.0)


def test_plan_reply_round_trips_through_dataclasses_replace() -> None:
    """Kiểm tra tính toàn vẹn của PlanReply khi sử dụng dataclasses.replace."""
    # Arrange
    reply = PlanReply(
        request_id=b"\x00" * 16,
        idl_version=IDL_VERSION,
        status=PlanStatus.OK,
        detail="",
        waypoints=(Waypoint((0.0, 0.0), 12.5),),
        path_length_m=1.0,
        plan_wall_time_s=0.5,
        applied_time_budget_s=15.0,
        stats=SearchStats(3, 7, False, False),
        planner_version="abc1234",
        config_hash="0123456789abcdef",
    )

    # Act
    modified = dataclasses.replace(reply, detail="modified detail")

    # Assert
    assert modified.detail == "modified detail"
    assert modified.request_id == reply.request_id
