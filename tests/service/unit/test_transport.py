"""Kiểm thử đơn vị cho module service.vtx_service.transport (giao vận NATS)."""

from __future__ import annotations

import asyncio
import uuid
from unittest.mock import AsyncMock, MagicMock

import pytest

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
from service.vtx_service.transport import (
    DEFAULT_NATS_SERVER,
    DEFAULT_QUEUE_GROUP,
    DEFAULT_SUBJECT,
    NatsClient,
    NatsTransport,
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
        islands=(),
        dynamic_obstacles=(),
        safezones=(),
        use_preloaded_map=False,
        limits=VehicleLimits(
            turn_radius_m=8000.0,
            l0_m=4000.0,
            dss_m=23000.0,
            safe_margin_m=500.0,
            alpha_max_deg=45.0,
        ),
        budget=SearchBudget(time_budget_s=4.5),
    )


def _sample_reply(req: PlanRequest) -> PlanReply:
    return PlanReply(
        request_id=req.request_id,
        idl_version=req.idl_version,
        status=PlanStatus.OK,
        detail="thành công",
        waypoints=(Waypoint(position=(10000.0, 20000.0), heading_deg=45.0),),
        path_length_m=50000.0,
        plan_wall_time_s=0.1,
        applied_time_budget_s=4.5,
        stats=SearchStats(
            iterations=10,
            open_set_size=5,
            is_search_failed=False,
            is_budget_bound=False,
        ),
        planner_version="vtx-0.1.0",
        config_hash="test",
    )


def test_transport_default_constants() -> None:
    """Kiểm tra các hằng số mặc định cho NATS subject và queue group."""
    assert DEFAULT_SUBJECT == "vtx.algorithms.path_planning.plan"
    assert DEFAULT_QUEUE_GROUP == "vtx.algorithms.path_planning"
    assert DEFAULT_NATS_SERVER == "nats://localhost:4222"


def test_transport_initialization() -> None:
    """Kiểm tra khởi tạo đối tượng NatsTransport."""
    transport = NatsTransport(
        server_url="nats://127.0.0.1:4222",
        subject="custom.subject",
        queue="custom.queue",
    )
    assert transport.server_url == "nats://127.0.0.1:4222"
    assert transport.subject == "custom.subject"
    assert transport.queue == "custom.queue"
    assert transport.nc is None


def test_client_initialization() -> None:
    """Kiểm tra khởi tạo đối tượng NatsClient."""
    client = NatsClient(
        server_url="nats://127.0.0.1:4222",
        subject="custom.subject",
    )
    assert client.server_url == "nats://127.0.0.1:4222"
    assert client.subject == "custom.subject"
    assert client.nc is None


@pytest.mark.asyncio
async def test_transport_message_handling_normal_flow() -> None:
    """Kiểm tra luồng xử lý message bình thường của NatsTransport."""
    # Arrange
    transport = NatsTransport()
    mock_nc = AsyncMock()
    transport.nc = mock_nc

    req = _sample_request()
    expected_reply = _sample_reply(req)

    def mock_handler(request: PlanRequest) -> PlanReply:
        return expected_reply

    # Tạo tin nhắn giả lập
    mock_msg = MagicMock()
    mock_msg.reply = "_INBOX.test1234"
    mock_msg.data = codec.encode_request(req)

    # Giả lập subscription callback
    captured_cb = None

    async def fake_subscribe(subject, queue, cb):
        nonlocal captured_cb
        captured_cb = cb
        return AsyncMock()

    mock_nc.subscribe.side_effect = fake_subscribe

    import nats
    original_connect = nats.connect
    nats.connect = AsyncMock(return_value=mock_nc)

    try:
        # Act
        await transport.start(mock_handler)
        assert captured_cb is not None
        await captured_cb(mock_msg)

        # Assert
        mock_nc.publish.assert_called_once()
        call_args = mock_nc.publish.call_args[0]
        assert call_args[0] == "_INBOX.test1234"
        sent_reply = codec.decode_reply(call_args[1])
        assert sent_reply.status == PlanStatus.OK
        assert sent_reply.request_id == req.request_id
    finally:
        nats.connect = original_connect
        await transport.close()


@pytest.mark.asyncio
async def test_transport_message_handling_decode_error_returns_invalid_request() -> (
    None
):
    """Kiểm tra khi nhận message rác, NatsTransport trả về INVALID_REQUEST."""
    # Arrange
    transport = NatsTransport()
    mock_nc = AsyncMock()
    transport.nc = mock_nc

    mock_msg = MagicMock()
    mock_msg.reply = "_INBOX.test_error"
    mock_msg.data = b"malformed_data_not_protobuf"

    captured_cb = None

    async def fake_subscribe(subject, queue, cb):
        nonlocal captured_cb
        captured_cb = cb
        return AsyncMock()

    mock_nc.subscribe.side_effect = fake_subscribe

    import nats
    original_connect = nats.connect
    nats.connect = AsyncMock(return_value=mock_nc)

    try:
        # Act
        await transport.start(lambda req: _sample_reply(req))
        assert captured_cb is not None
        await captured_cb(mock_msg)

        # Assert
        mock_nc.publish.assert_called_once()
        call_args = mock_nc.publish.call_args[0]
        assert call_args[0] == "_INBOX.test_error"
        sent_reply = codec.decode_reply(call_args[1])
        assert sent_reply.status == PlanStatus.INVALID_REQUEST
    finally:
        nats.connect = original_connect
        await transport.close()


@pytest.mark.asyncio
async def test_transport_message_handling_handler_exception_returns_internal_error() -> (
    None
):
    """Kiểm tra khi handler ném ngoại lệ, NatsTransport bắt lại và trả về INTERNAL_ERROR."""
    # Arrange
    transport = NatsTransport()
    mock_nc = AsyncMock()
    transport.nc = mock_nc

    req = _sample_request()
    mock_msg = MagicMock()
    mock_msg.reply = "_INBOX.test_exception"
    mock_msg.data = codec.encode_request(req)

    def crashing_handler(request: PlanRequest) -> PlanReply:
        raise RuntimeError("bộ nhớ cạn kiệt bất ngờ")

    captured_cb = None

    async def fake_subscribe(subject, queue, cb):
        nonlocal captured_cb
        captured_cb = cb
        return AsyncMock()

    mock_nc.subscribe.side_effect = fake_subscribe

    import nats
    original_connect = nats.connect
    nats.connect = AsyncMock(return_value=mock_nc)

    try:
        # Act
        await transport.start(crashing_handler)
        assert captured_cb is not None
        await captured_cb(mock_msg)

        # Assert
        mock_nc.publish.assert_called_once()
        call_args = mock_nc.publish.call_args[0]
        assert call_args[0] == "_INBOX.test_exception"
        sent_reply = codec.decode_reply(call_args[1])
        assert sent_reply.status == PlanStatus.INTERNAL_ERROR
        assert "bộ nhớ cạn kiệt" in sent_reply.detail
    finally:
        nats.connect = original_connect
        await transport.close()
