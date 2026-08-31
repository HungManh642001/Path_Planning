"""Module đo tải đồng thời và độ trễ NATS Microservice (NatsStressTester).

Hỗ trợ gửi liên tục N requests tới NATS queue group với mức độ đồng thời (concurrency)
được kiểm soát qua asyncio.Semaphore, đồng thời thu thập các chỉ số thông lượng (RPS),
thời gian phản hồi (p50, p90, p95, p99) và phân loại lỗi.
"""

from __future__ import annotations

import asyncio
import dataclasses
import logging
import time
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from path_planning.scenario.presets import get_all_scenarios
from service.vtx_service.transport import (
    DEFAULT_NATS_SERVER,
    DEFAULT_SUBJECT,
    NatsClient,
)
from tools.qa_suite.core.runner import scenario_to_plan_request


logger = logging.getLogger("vtx-qa.stress_tester")


if TYPE_CHECKING:
    from path_planning.types import Scenario


@dataclass
class StressTestSummary:
    """Tổng hợp số liệu thống kê sau một phiên kiểm thử đo tải NATS Microservice.

    Attributes:
        total_requests: Tổng số requests đã gửi.
        concurrency: Số lượng requests đồng thời tối đa (in-flight concurrency limit).
        success_count: Số requests thành công (nhận PlanReply có status OK).
        error_count: Số requests thất bại do lỗi thuật toán hoặc lỗi server nội bộ.
        timeout_count: Số requests bị timeout (không nhận được phản hồi trong hạn định).
        throughput_rps: Thông lượng đạt được (requests trên giây).
        wall_time_s: Tổng thời gian thực tế để hoàn thành toàn bộ đợt kiểm thử (giây).
        latency_p50_s: Độ trễ phân vị 50 (median) tính bằng giây.
        latency_p90_s: Độ trễ phân vị 90 tính bằng giây.
        latency_p95_s: Độ trễ phân vị 95 tính bằng giây.
        latency_p99_s: Độ trễ phân vị 99 tính bằng giây.
        latencies: Danh sách độ trễ của từng request riêng lẻ (giây).
    """

    total_requests: int
    concurrency: int
    success_count: int
    error_count: int
    timeout_count: int
    throughput_rps: float
    wall_time_s: float
    latency_p50_s: float
    latency_p90_s: float
    latency_p95_s: float
    latency_p99_s: float
    latencies: list[float]


@dataclass
class _RequestOutcome:
    """Kết quả chi tiết của một request đơn lẻ trong phiên đo tải."""

    is_success: bool
    is_timeout: bool
    is_error: bool
    latency_s: float


class NatsStressTester:
    """Động cơ đo tải bất đồng bộ cho NATS Path Planning Microservice."""

    def __init__(self, client: NatsClient | None = None) -> None:
        """Khởi tạo NatsStressTester.

        Args:
            client: Tuỳ chọn truyền trước instance NatsClient (thường dùng cho
                mock test).
        """
        self._injected_client = client

    async def run_stress_test(
        self,
        server_url: str = DEFAULT_NATS_SERVER,
        subject: str = DEFAULT_SUBJECT,
        scenario: Scenario | None = None,
        total_requests: int = 50,
        concurrency: int = 5,
        timeout_s: float = 6.0,
        progress_callback: Callable[[int, int], None] | None = None,
        client: NatsClient | None = None,
    ) -> StressTestSummary:
        """Thực thi kiểm thử đo tải đồng thời bất đồng bộ trên NATS microservice.

        Args:
            server_url: Địa chỉ NATS server kết nối tới.
            subject: Subject NATS gửi yêu cầu lập lịch.
            scenario: Kịch bản kiểm thử (mặc định lấy scenario_01_open_ocean).
            total_requests: Tổng số requests cần gửi.
            concurrency: Số requests đồng thời tối đa.
            timeout_s: Thời gian chờ tối đa cho mỗi request (giây).
            progress_callback: Callback nhận (completed_count, total_requests).
            client: Tuỳ chọn truyền instance NatsClient cụ thể.

        Returns:
            StressTestSummary: Báo cáo tổng hợp số liệu đo tải.

        Raises:
            ValueError: Nếu total_requests <= 0 hoặc concurrency <= 0.
        """
        if total_requests <= 0:
            raise ValueError(
                f"total_requests phải lớn hơn 0, nhận được: {total_requests}"
            )
        if concurrency <= 0:
            raise ValueError(f"concurrency phải lớn hơn 0, nhận được: {concurrency}")

        if scenario is None:
            all_presets = get_all_scenarios()
            if "scenario_01_open_ocean" in all_presets:
                scenario = all_presets["scenario_01_open_ocean"]()
            elif "scenario_01_open_space" in all_presets:
                scenario = all_presets["scenario_01_open_space"]()
            else:
                scenario = next(iter(all_presets.values()))()

        base_request = scenario_to_plan_request(
            scenario, name="stress_req", time_budget_s=timeout_s
        )

        active_client = client or self._injected_client
        own_client = False
        if active_client is None:
            active_client = NatsClient(server_url=server_url, subject=subject)
            own_client = True

        if (
            own_client
            or active_client.nc is None
            or getattr(active_client.nc, "is_closed", True)
        ):
            await active_client.connect()

        sem = asyncio.Semaphore(concurrency)
        completed_count = 0

        async def _send_one(idx: int) -> _RequestOutcome:
            nonlocal completed_count
            req_id = f"stress_{idx:05d}".encode()[:16].ljust(16, b"\x00")
            req_msg = dataclasses.replace(base_request, request_id=req_id)

            async with sem:
                t0 = time.perf_counter()
                is_success = False
                is_timeout = False
                is_error = False
                try:
                    reply = await active_client.request_plan(
                        req_msg, timeout_s=timeout_s
                    )
                    t1 = time.perf_counter()
                    lat = t1 - t0
                    if reply.is_ok:
                        is_success = True
                    else:
                        is_error = True
                except (TimeoutError, asyncio.TimeoutError):
                    t1 = time.perf_counter()
                    lat = t1 - t0
                    is_timeout = True
                except Exception:
                    t1 = time.perf_counter()
                    lat = t1 - t0
                    is_error = True

                completed_count += 1
                if progress_callback is not None:
                    try:
                        progress_callback(completed_count, total_requests)
                    except Exception as exc:
                        logger.debug("Lỗi trong callback: %s", exc)

                return _RequestOutcome(
                    is_success=is_success,
                    is_timeout=is_timeout,
                    is_error=is_error,
                    latency_s=lat,
                )

        wall_start = time.perf_counter()
        try:
            tasks = [_send_one(i) for i in range(total_requests)]
            outcomes = await asyncio.gather(*tasks)
        finally:
            if own_client:
                await active_client.close()

        wall_time_s = time.perf_counter() - wall_start

        latencies = [o.latency_s for o in outcomes]
        success_count = sum(1 for o in outcomes if o.is_success)
        timeout_count = sum(1 for o in outcomes if o.is_timeout)
        error_count = sum(1 for o in outcomes if o.is_error)
        throughput_rps = (total_requests / wall_time_s) if wall_time_s > 0 else 0.0

        if latencies:
            arr = np.array(latencies, dtype=float)
            p50 = float(np.percentile(arr, 50))
            p90 = float(np.percentile(arr, 90))
            p95 = float(np.percentile(arr, 95))
            p99 = float(np.percentile(arr, 99))
        else:
            p50 = p90 = p95 = p99 = 0.0

        return StressTestSummary(
            total_requests=total_requests,
            concurrency=concurrency,
            success_count=success_count,
            error_count=error_count,
            timeout_count=timeout_count,
            throughput_rps=throughput_rps,
            wall_time_s=wall_time_s,
            latency_p50_s=p50,
            latency_p90_s=p90,
            latency_p95_s=p95,
            latency_p99_s=p99,
            latencies=latencies,
        )

    def run_stress_test_sync(
        self,
        server_url: str = DEFAULT_NATS_SERVER,
        subject: str = DEFAULT_SUBJECT,
        scenario: Scenario | None = None,
        total_requests: int = 50,
        concurrency: int = 5,
        timeout_s: float = 6.0,
        progress_callback: Callable[[int, int], None] | None = None,
        client: NatsClient | None = None,
    ) -> StressTestSummary:
        """Wrapper đồng bộ thực thi stress test trên NATS microservice."""
        coro = self.run_stress_test(
            server_url=server_url,
            subject=subject,
            scenario=scenario,
            total_requests=total_requests,
            concurrency=concurrency,
            timeout_s=timeout_s,
            progress_callback=progress_callback,
            client=client,
        )

        with ThreadPoolExecutor(max_workers=1) as executor:
            future = executor.submit(asyncio.run, coro)
            return future.result()
