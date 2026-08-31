"""Kiểm thử đơn vị cho module NatsStressTester và StressTestSummary của QA Suite."""

from __future__ import annotations

import asyncio
from unittest.mock import AsyncMock

import pytest

from path_planning.scenario.presets import get_all_scenarios
from service.vtx_service.messages import (
    IDL_VERSION,
    PlanReply,
    PlanStatus,
    SearchStats as ProtoSearchStats,
    Waypoint as ProtoWaypoint,
)
from tools.qa_suite.core.stress_tester import (
    NatsStressTester,
    StressTestSummary,
)


def _create_fake_reply(is_ok: bool = True) -> PlanReply:
    """Tạo đối tượng PlanReply giả định phục vụ kiểm thử."""
    status = PlanStatus.OK if is_ok else PlanStatus.NO_PATH
    waypoints = (
        (
            ProtoWaypoint(position=(100.0, 100.0), heading_deg=0.0),
            ProtoWaypoint(position=(100.0, 500.0), heading_deg=0.0),
        )
        if is_ok
        else ()
    )
    return PlanReply(
        request_id=b"\x01" * 16,
        idl_version=IDL_VERSION,
        status=status,
        detail="" if is_ok else "No path found",
        waypoints=waypoints,
        path_length_m=400.0 if is_ok else 0.0,
        plan_wall_time_s=0.02,
        applied_time_budget_s=5.0,
        stats=ProtoSearchStats(
            iterations=25,
            open_set_size=3,
            is_search_failed=not is_ok,
            is_budget_bound=False,
        ),
        planner_version="vtx-1.0.0",
        config_hash="test-hash",
    )


@pytest.mark.asyncio
async def test_nats_stress_tester_collects_concurrency_metrics() -> None:
    """Kiểm thử NatsStressTester đo tải thành công và tổng hợp đầy đủ các chỉ số."""
    scenario = get_all_scenarios()["scenario_01_open_ocean"]()
    fake_reply = _create_fake_reply(is_ok=True)

    mock_client = AsyncMock()
    mock_client.connect = AsyncMock()
    mock_client.close = AsyncMock()
    mock_client.request_plan = AsyncMock(return_value=fake_reply)

    tester = NatsStressTester()
    summary = await tester.run_stress_test(
        scenario=scenario,
        total_requests=20,
        concurrency=4,
        timeout_s=5.0,
        client=mock_client,
    )

    assert isinstance(summary, StressTestSummary)
    assert summary.total_requests == 20
    assert summary.concurrency == 4
    assert summary.success_count == 20
    assert summary.error_count == 0
    assert summary.timeout_count == 0
    assert summary.throughput_rps > 0.0
    assert summary.wall_time_s > 0.0
    assert len(summary.latencies) == 20
    assert summary.latency_p50_s >= 0.0
    assert summary.latency_p90_s >= summary.latency_p50_s
    assert summary.latency_p95_s >= summary.latency_p90_s
    assert summary.latency_p99_s >= summary.latency_p95_s
    assert mock_client.request_plan.call_count == 20


@pytest.mark.asyncio
async def test_nats_stress_tester_handles_errors_and_timeouts() -> None:
    """Kiểm thử NatsStressTester phân loại chính xác các lỗi trả về, timeout và exception."""
    ok_reply = _create_fake_reply(is_ok=True)
    err_reply = _create_fake_reply(is_ok=False)

    call_index = 0

    async def side_effect_request(*args, **kwargs) -> PlanReply:
        nonlocal call_index
        idx = call_index
        call_index += 1
        if idx % 4 == 0:
            return ok_reply
        elif idx % 4 == 1:
            return err_reply
        elif idx % 4 == 2:
            raise TimeoutError("NATS request timed out")
        else:
            raise RuntimeError("Connection dropped")

    mock_client = AsyncMock()
    mock_client.connect = AsyncMock()
    mock_client.close = AsyncMock()
    mock_client.request_plan = AsyncMock(side_effect=side_effect_request)

    tester = NatsStressTester()
    summary = await tester.run_stress_test(
        total_requests=8,
        concurrency=2,
        client=mock_client,
    )

    assert summary.total_requests == 8
    assert summary.success_count == 2  # idx 0, 4
    assert summary.timeout_count == 2  # idx 2, 6
    assert summary.error_count == 4  # idx 1, 5 (err_reply) + idx 3, 7 (RuntimeError)
    assert len(summary.latencies) == 8


@pytest.mark.asyncio
async def test_nats_stress_tester_concurrency_limiting() -> None:
    """Kiểm thử Semaphore giới hạn số lượng request đồng thời không vượt quá concurrency."""
    current_in_flight = 0
    max_in_flight = 0
    in_flight_lock = asyncio.Lock()

    async def delayed_request(*args, **kwargs) -> PlanReply:
        nonlocal current_in_flight, max_in_flight
        async with in_flight_lock:
            current_in_flight += 1
            if current_in_flight > max_in_flight:
                max_in_flight = current_in_flight
        await asyncio.sleep(0.02)
        async with in_flight_lock:
            current_in_flight -= 1
        return _create_fake_reply(is_ok=True)

    mock_client = AsyncMock()
    mock_client.connect = AsyncMock()
    mock_client.close = AsyncMock()
    mock_client.request_plan = AsyncMock(side_effect=delayed_request)

    concurrency_limit = 3
    tester = NatsStressTester()
    summary = await tester.run_stress_test(
        total_requests=12,
        concurrency=concurrency_limit,
        client=mock_client,
    )

    assert summary.total_requests == 12
    assert summary.success_count == 12
    assert max_in_flight <= concurrency_limit


@pytest.mark.asyncio
async def test_nats_stress_tester_progress_callback() -> None:
    """Kiểm thử NatsStressTester gọi progress_callback sau mỗi request hoàn thành."""
    fake_reply = _create_fake_reply(is_ok=True)
    mock_client = AsyncMock()
    mock_client.connect = AsyncMock()
    mock_client.close = AsyncMock()
    mock_client.request_plan = AsyncMock(return_value=fake_reply)

    progress_records: list[tuple[int, int]] = []

    def callback(completed: int, total: int) -> None:
        progress_records.append((completed, total))

    tester = NatsStressTester()
    summary = await tester.run_stress_test(
        total_requests=6,
        concurrency=2,
        progress_callback=callback,
        client=mock_client,
    )

    assert summary.total_requests == 6
    assert len(progress_records) == 6
    assert progress_records[-1] == (6, 6)


def test_run_stress_test_sync_wrapper() -> None:
    """Kiểm thử phương thức đồng bộ run_stress_test_sync hoạt động chính xác."""
    fake_reply = _create_fake_reply(is_ok=True)

    class FakeClient:
        def __init__(self) -> None:
            self.nc = None

        async def connect(self) -> None:
            pass

        async def close(self) -> None:
            pass

        async def request_plan(self, req: object, timeout_s: float = 6.0) -> PlanReply:
            return fake_reply

    tester = NatsStressTester(client=FakeClient())  # type: ignore[arg-type]
    summary = tester.run_stress_test_sync(total_requests=10, concurrency=2)

    assert isinstance(summary, StressTestSummary)
    assert summary.total_requests == 10
    assert summary.success_count == 10
    assert summary.error_count == 0


def test_nats_stress_tester_invalid_params() -> None:
    """Kiểm thử NatsStressTester ném ngoại lệ ValueError khi tham số đầu vào không hợp lệ."""
    tester = NatsStressTester()
    with pytest.raises(ValueError, match="total_requests"):
        tester.run_stress_test_sync(total_requests=0)

    with pytest.raises(ValueError, match="concurrency"):
        tester.run_stress_test_sync(total_requests=10, concurrency=0)
