"""Kiểm thử đơn vị cho module service.vtx_service.codec (mã hóa/giải mã Protobuf)."""

from __future__ import annotations

import uuid

from service.vtx_service import codec
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


def _sample_request() -> PlanRequest:
    return PlanRequest(
        request_id=uuid.uuid4().bytes,
        idl_version=IDL_VERSION,
        start=(10000.0, 20000.0),
        start_heading_deg=45.0,
        goal=(80000.0, 90000.0),
        goal_heading_deg=180.0,
        is_goal_heading_free=False,
        islands=(
            ((15000.0, 15000.0), (25000.0, 15000.0), (25000.0, 25000.0)),
            ((35000.0, 35000.0), (45000.0, 35000.0), (45000.0, 45000.0)),
        ),
        dynamic_obstacles=(
            Circle(center=(50000.0, 50000.0), radius_m=3000.0),
            Circle(center=(60000.0, 60000.0), radius_m=4000.0),
        ),
        safezones=(
            ((0.0, 0.0), (100000.0, 0.0), (100000.0, 100000.0), (0.0, 100000.0)),
        ),
        use_preloaded_map=True,
        limits=VehicleLimits(
            turn_radius_m=8000.0,
            l0_m=4000.0,
            dss_m=23000.0,
            safe_margin_m=500.0,
            alpha_max_deg=45.0,
        ),
        budget=SearchBudget(time_budget_s=4.5),
    )


def _sample_reply() -> PlanReply:
    return PlanReply(
        request_id=uuid.uuid4().bytes,
        idl_version=IDL_VERSION,
        status=PlanStatus.OK,
        detail="thành công",
        waypoints=(
            Waypoint(position=(10000.0, 20000.0), heading_deg=45.0),
            Waypoint(position=(30000.0, 40000.0), heading_deg=60.0),
            Waypoint(position=(80000.0, 90000.0), heading_deg=180.0),
        ),
        path_length_m=123456.78,
        plan_wall_time_s=0.456,
        applied_time_budget_s=4.5,
        stats=SearchStats(
            iterations=150,
            open_set_size=25,
            is_search_failed=False,
            is_budget_bound=False,
        ),
        planner_version="vtx-0.1.0",
        config_hash="a1b2c3d4e5f6",
    )


def test_request_codec_round_trip_preserves_all_fields() -> None:
    """Kiểm tra mã hóa và giải mã PlanRequest qua Protobuf không làm mất mát trường dữ liệu."""
    # Arrange
    req = _sample_request()

    # Act
    encoded = codec.encode_request(req)
    decoded = codec.decode_request(encoded)

    # Assert
    assert isinstance(encoded, bytes)
    assert decoded.request_id == req.request_id
    assert decoded.idl_version == req.idl_version
    assert decoded.start == req.start
    assert decoded.start_heading_deg == req.start_heading_deg
    assert decoded.goal == req.goal
    assert decoded.goal_heading_deg == req.goal_heading_deg
    assert decoded.is_goal_heading_free == req.is_goal_heading_free
    assert decoded.islands == req.islands
    assert decoded.dynamic_obstacles == req.dynamic_obstacles
    assert decoded.safezones == req.safezones
    assert decoded.use_preloaded_map == req.use_preloaded_map
    assert decoded.limits == req.limits
    assert decoded.budget == req.budget
    assert decoded == req


def test_reply_codec_round_trip_preserves_all_fields() -> None:
    """Kiểm tra mã hóa và giải mã PlanReply qua Protobuf không làm mất mát trường dữ liệu."""
    # Arrange
    reply = _sample_reply()

    # Act
    encoded = codec.encode_reply(reply)
    decoded = codec.decode_reply(encoded)

    # Assert
    assert isinstance(encoded, bytes)
    assert decoded.request_id == reply.request_id
    assert decoded.idl_version == reply.idl_version
    assert decoded.status == reply.status
    assert decoded.detail == reply.detail
    assert decoded.waypoints == reply.waypoints
    assert decoded.path_length_m == reply.path_length_m
    assert decoded.plan_wall_time_s == reply.plan_wall_time_s
    assert decoded.applied_time_budget_s == reply.applied_time_budget_s
    assert decoded.stats == reply.stats
    assert decoded.planner_version == reply.planner_version
    assert decoded.config_hash == reply.config_hash
    assert decoded == reply


def test_plan_status_enum_values_match_proto_definition() -> None:
    """Kiểm tra ánh xạ PlanStatus enum giữa Python dataclass và Protobuf enum."""
    # Arrange
    from service.vtx_service.proto import vtx_path_planning_pb2 as pb

    # Assert
    assert pb.PlanStatus.PLAN_STATUS_OK == PlanStatus.OK.value
    assert pb.PlanStatus.PLAN_STATUS_NO_PATH == PlanStatus.NO_PATH.value
    assert (
        pb.PlanStatus.PLAN_STATUS_START_LEG_BLOCKED
        == PlanStatus.START_LEG_BLOCKED.value
    )
    assert (
        pb.PlanStatus.PLAN_STATUS_GOAL_LEG_BLOCKED == PlanStatus.GOAL_LEG_BLOCKED.value
    )
    assert pb.PlanStatus.PLAN_STATUS_ORACLE_REJECTED == PlanStatus.ORACLE_REJECTED.value
    assert pb.PlanStatus.PLAN_STATUS_INVALID_REQUEST == PlanStatus.INVALID_REQUEST.value
    assert pb.PlanStatus.PLAN_STATUS_TIMEOUT == PlanStatus.TIMEOUT.value
    assert pb.PlanStatus.PLAN_STATUS_INTERNAL_ERROR == PlanStatus.INTERNAL_ERROR.value
    assert pb.PlanStatus.PLAN_STATUS_BUSY == PlanStatus.BUSY.value
