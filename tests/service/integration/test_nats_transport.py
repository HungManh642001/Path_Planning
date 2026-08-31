"""Kiểm thử tích hợp cho tầng giao vận NATS service.vtx_service.transport."""

from __future__ import annotations

import asyncio
import uuid
from unittest.mock import AsyncMock, MagicMock

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
from service.vtx_service.transport import (
    NatsClient,
    NatsTransport,
)


def _build_request(request_id: bytes) -> PlanRequest:
    """Khởi tạo PlanRequest chuẩn cho test transport NATS."""
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


@pytest.mark.asyncio
async def test_nats_transport_and_client_round_trip() -> None:
    """Kiểm tra luồng gửi nhận PlanRequest và PlanReply qua NATS client-transport."""
    # Arrange
    req_id = uuid.uuid4().bytes
    request = _build_request(req_id)
    expected_reply = _build_reply(request)

    subscriptions: dict[str, list] = {}

    class MockNatsConn:
        def __init__(self) -> None:
            self.is_closed = False

        async def subscribe(self, subject: str, queue: str = "", cb=None):
            sub = MagicMock()
            subscriptions.setdefault(subject, []).append((queue, cb))
            return sub

        async def publish(self, subject: str, data: bytes):
            for _, cb in subscriptions.get(subject, []):
                msg = MagicMock()
                msg.data = data
                msg.reply = ""
                await cb(msg)

        async def request(self, subject: str, data: bytes, timeout: float = 6.0):
            reply_inbox = f"_INBOX.{uuid.uuid4().hex}"
            reply_future = asyncio.get_running_loop().create_future()

            async def reply_cb(msg):
                if not reply_future.done():
                    reply_future.set_result(msg)

            subscriptions[reply_inbox] = [("", reply_cb)]

            handlers = subscriptions.get(subject, [])
            assert len(handlers) > 0, f"Không có subscriber nào cho subject {subject}"
            _, server_cb = handlers[0]
            req_msg = MagicMock()
            req_msg.data = data
            req_msg.reply = reply_inbox
            await server_cb(req_msg)

            return await asyncio.wait_for(reply_future, timeout=timeout)

        async def flush(self):
            pass

        async def drain(self):
            pass

        async def close(self):
            self.is_closed = True

    mock_nc = MockNatsConn()

    import nats

    original_connect = nats.connect
    nats.connect = AsyncMock(return_value=mock_nc)

    def service_handler(req: PlanRequest) -> PlanReply:
        return _build_reply(req)

    transport = NatsTransport()
    client = NatsClient()

    try:
        # Act
        await transport.start(service_handler)
        await client.connect()

        reply = await client.request_plan(request, timeout_s=5.0)

        # Assert
        assert reply.request_id == req_id
        assert reply.status is PlanStatus.ORACLE_REJECTED
        assert reply.path_length_m == expected_reply.path_length_m
        assert len(reply.waypoints) == 2
        assert reply.planner_version == "v1.0-3-gabc1234-dirty"
    finally:
        nats.connect = original_connect
        await client.close()
        await transport.close()


@pytest.mark.asyncio
async def test_nats_client_timeout_raises_timeout_error() -> None:
    """Kiểm tra khi service không phản hồi trong hạn mức, client ném TimeoutError."""

    # Arrange
    class MockTimeoutNatsConn:
        def __init__(self) -> None:
            self.is_closed = False

        async def request(self, subject: str, data: bytes, timeout: float = 6.0):
            await asyncio.sleep(timeout + 0.1)
            raise TimeoutError("NATS request timed out")

        async def close(self):
            self.is_closed = True

    mock_nc = MockTimeoutNatsConn()
    import nats

    original_connect = nats.connect
    nats.connect = AsyncMock(return_value=mock_nc)

    client = NatsClient()
    try:
        await client.connect()
        with pytest.raises(TimeoutError):
            await client.request_plan(_build_request(uuid.uuid4().bytes), timeout_s=0.1)
    finally:
        nats.connect = original_connect
        await client.close()
