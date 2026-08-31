"""Kiểm thử tích hợp tầng truyền thông DDS service.vtx_service.transport."""

from __future__ import annotations

import dataclasses
import re
import threading
import uuid
from pathlib import Path

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

pytest.importorskip(
    "cyclonedds", reason="Chưa cài đặt thư viện cyclonedds binding"
)

from cyclonedds.idl.types import uint32

from service.vtx_service.transport import (
    DdsTransport,
    WireReply,
)

IDL_PATH = (
    Path(__file__).resolve().parents[3]
    / "src"
    / "service"
    / "idl"
    / "vtx_path_planning.idl"
)
DOMAIN = 92


def _build_request(request_id: bytes) -> PlanRequest:
    """Khởi tạo PlanRequest chuẩn cho test transport DDS."""
    return PlanRequest(
        request_id=request_id,
        idl_version=IDL_VERSION,
        start=(50000.0, 50000.0),
        start_heading_deg=45.0,
        goal=(300000.0, 250000.0),
        goal_heading_deg=137.5,
        is_goal_heading_free=False,
        islands=(((1e5, 1e5), (1.2e5, 1e5), (1.1e5, 1.3e5)),),
        dynamic_obstacles=(Circle(center=(2e5, 1.5e5), radius_m=12000.0),),
        safezones=(((0.0, 0.0), (5e5, 0.0), (5e5, 5e5)),),
        use_preloaded_map=False,
        limits=VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0),
        budget=SearchBudget(15.0),
    )


def _build_reply(request: PlanRequest) -> PlanReply:
    """Khởi tạo PlanReply mẫu phản hồi cho request tương ứng."""
    return PlanReply(
        request_id=request.request_id,
        idl_version=IDL_VERSION,
        status=PlanStatus.ORACLE_REJECTED,
        detail="first W1..W2 l=7421.3 < L0=8000",
        waypoints=(Waypoint((1.5, -2.5), 137.5), Waypoint((3e5, 2.5e5), 42.0)),
        path_length_m=123456.78901234567,
        plan_wall_time_s=0.0421,
        applied_time_budget_s=15.0,
        stats=SearchStats(1234, 56, True, False),
        planner_version="v1.0-3-gabc1234-dirty",
        config_hash="0123456789abcdef",
    )


def test_idl_status_enum_matches_python_plan_status() -> None:
    """Kiểm tra các giá trị enum PlanStatus trong file IDL khớp 100% với Python."""
    # Arrange
    text = IDL_PATH.read_text(encoding="utf-8")
    match = re.search(r"enum\s+PlanStatus\s*\{(.*?)\}", text, re.DOTALL)

    # Act & Assert
    assert match
    members = [item.strip() for item in match.group(1).split(",") if item.strip()]
    assert members == [f"PLAN_{member.name}" for member in PlanStatus]


def test_idl_status_field_is_uint32_in_wire_reply() -> None:
    """Kiểm tra trường status trong WireReply dùng kiểu uint32 tương thích cyclonedds."""
    # Arrange
    text = IDL_PATH.read_text(encoding="utf-8")

    # Act & Assert
    assert re.search(r"\bPlanStatus\s+status\s*;", text)
    status_field = next(f for f in dataclasses.fields(WireReply) if f.name == "status")
    assert status_field.type is uint32


def test_request_survives_dds_round_trip() -> None:
    """Kiểm tra gửi và nhận PlanRequest / PlanReply qua DDS transport thật trên domain test."""
    # Arrange
    request_id = uuid.uuid4().bytes
    seen: list[PlanRequest] = []
    done = threading.Event()

    def handler(incoming: PlanRequest) -> PlanReply:
        seen.append(incoming)
        done.set()
        return _build_reply(incoming)

    service = DdsTransport(domain_id=DOMAIN)
    client = DdsTransport(domain_id=DOMAIN)
    thread = threading.Thread(target=service.serve, args=(handler,), daemon=True)
    thread.start()

    # Act
    try:
        assert client.wait_for_service(timeout_s=20.0)
        reply = client.request(_build_request(request_id), timeout_s=30.0)
        assert done.wait(timeout=5.0)

        # Assert
        got = seen[0]
        assert got.request_id == request_id
        assert got.start == (50000.0, 50000.0)
        assert got.goal_heading_deg == 137.5
        assert got.is_goal_heading_free is False
        assert len(got.islands[0]) == 3
        assert got.dynamic_obstacles[0].radius_m == 12000.0

        assert reply.request_id == request_id
        assert reply.status is PlanStatus.ORACLE_REJECTED
        assert reply.path_length_m == 123456.78901234567
    finally:
        service.close()
        client.close()


def test_correlated_replies_match_correct_request_id() -> None:
    """Kiểm tra cơ chế tương quan ánh xạ đúng request_id khi có nhiều yêu cầu gửi liên tiếp."""
    # Arrange
    def handler(incoming: PlanRequest) -> PlanReply:
        return _build_reply(incoming)

    service = DdsTransport(domain_id=DOMAIN + 3)
    client = DdsTransport(domain_id=DOMAIN + 3)
    threading.Thread(target=service.serve, args=(handler,), daemon=True).start()

    # Act & Assert
    try:
        assert client.wait_for_service(timeout_s=20.0)
        first = uuid.uuid4().bytes
        second = uuid.uuid4().bytes
        assert client.request(_build_request(first), timeout_s=30.0).request_id == first
        assert client.request(_build_request(second), timeout_s=30.0).request_id == second
    finally:
        service.close()
        client.close()
