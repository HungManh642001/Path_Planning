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


def _request(**overrides: object) -> PlanRequest:
    base: dict[str, object] = dict(
        request_id=b"\x00" * 16,
        idl_version=IDL_VERSION,
        start=(0.0, 0.0),
        start_heading_deg=0.0,
        goal=(100000.0, 0.0),
        goal_heading_deg=90.0,
        is_goal_heading_free=False,
        islands=(),
        dynamic_obstacles=(),
        safezones=(),
        use_preloaded_map=False,
        limits=VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0),
        budget=SearchBudget(15.0),
    )
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


def test_request_is_frozen() -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        _request().start = (1.0, 1.0)  # type: ignore[misc]


def test_request_id_must_be_16_bytes() -> None:
    with pytest.raises(ValueError, match="16 byte"):
        _request(request_id=b"\x00" * 8)


def test_status_values_match_the_wire_contract() -> None:
    # Các số này khớp enum trong IDL. Đổi ở đây mà không đổi IDL là một thay đổi
    # phá vỡ hợp đồng đi qua không tiếng động.
    assert [int(member) for member in PlanStatus] == list(range(9))
    assert PlanStatus.OK == 0
    assert PlanStatus.ORACLE_REJECTED == 4
    assert PlanStatus.BUSY == 8


def test_there_is_no_frame_field() -> None:
    """Chỉ có một hệ toạ độ. Thêm WGS84 sau là một lần tăng idl_version."""
    assert "frame" not in {f.name for f in dataclasses.fields(PlanRequest)}


def test_reply_reports_the_budget_it_actually_used() -> None:
    """Đề nghị của client có thể bị thay (trống, hoặc quá trần); reply nói thật."""
    fields = {f.name for f in dataclasses.fields(PlanReply)}
    assert "applied_time_budget_s" in fields


def test_the_budget_on_the_wire_is_time_only() -> None:
    """Không còn trần theo số vòng lặp nào để đề nghị, hay để báo cáo lại."""
    assert {f.name for f in dataclasses.fields(SearchBudget)} == {"time_budget_s"}
    assert "max_iterations" not in {f.name for f in dataclasses.fields(SearchStats)}


def test_circle_rejects_a_non_positive_radius() -> None:
    with pytest.raises(ValueError, match="radius"):
        Circle(center=(0.0, 0.0), radius_m=0.0)


def test_limits_reject_a_negative_margin() -> None:
    with pytest.raises(ValueError, match="safe_margin_m"):
        VehicleLimits(8000.0, 8000.0, 15000.0, -1.0, 90.0)


def test_reply_round_trips_through_dataclasses_replace() -> None:
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
    assert dataclasses.replace(reply, detail="x").detail == "x"
