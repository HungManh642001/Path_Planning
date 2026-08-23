"""Thời hạn cứng, và tính bền của service sau khi phải giết một tiến trình con.

Planner là Python thuần, CPU-bound, và chỉ kiểm tra ngân sách tại các điểm trong
vòng lặp search - nó KHÔNG hủy được từ bên ngoài một cách lịch sự. Tiến trình
con là cách duy nhất để có thời hạn cứng thật.
"""

from __future__ import annotations

import time

import pytest

from vtx_service.messages import (
    Circle,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    VehicleLimits,
)
from vtx_service.runner import PlanRunner

LIMITS = VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0)


def _request(**overrides: object) -> PlanRequest:
    base: dict[str, object] = dict(
        request_id=b"\x06" * 16,
        idl_version=1,
        start=(50000.0, 50000.0),
        start_heading_deg=45.0,
        goal=(300000.0, 250000.0),
        goal_heading_deg=45.0,
        goal_heading_free=True,
        islands=(),
        dynamic_obstacles=(),
        safezones=(),
        use_preloaded_map=False,
        limits=LIMITS,
        budget=SearchBudget(15.0, 50000),
    )
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


@pytest.fixture()
def runner():
    instance = PlanRunner(preloaded=None)
    instance.start()
    try:
        yield instance
    finally:
        instance.stop()


def test_a_mission_plans_in_a_child_process(runner: PlanRunner) -> None:
    reply = runner.submit(_request())
    assert reply.status is PlanStatus.OK
    assert len(reply.waypoints) >= 2


def test_the_reply_survives_the_process_boundary_intact(runner: PlanRunner) -> None:
    reply = runner.submit(_request())
    assert reply.request_id == b"\x06" * 16
    assert isinstance(reply.status, PlanStatus)
    # double phải qua pickle nguyên vẹn từng bit.
    assert reply.path_length_m == runner.submit(_request()).path_length_m


def test_child_start_cost_is_within_the_measured_envelope(runner: PlanRunner) -> None:
    """Spec ghi median 56 ms cho forkserver + preload. Nới rộng cho máy chậm."""
    runner.submit(_request())  # bỏ lần đầu (khởi động forkserver)
    started = time.perf_counter()
    runner.submit(_request())
    assert time.perf_counter() - started < 5.0


def test_a_hung_child_becomes_timeout_and_the_runner_keeps_working() -> None:
    """Thời hạn CỨNG: tiến trình con treo phải bị giết, không được treo service.

    Hạ `config.TIME_BUDGET_S` xuống trong lúc test, vì thời hạn cứng được tính
    là `effective_time_budget_s() + grace`. Với giá trị thật (15 s) test này sẽ
    chạy 15,5 giây và TRÔNG NHƯ TREO — đúng thứ nó sinh ra để phát hiện.
    """
    import config

    saved = config.TIME_BUDGET_S
    instance = PlanRunner(preloaded=None, grace_s=0.5)
    instance.start()
    try:
        config.TIME_BUDGET_S = 0.5
        instance.force_hang_next = True  # cửa hậu chỉ dùng cho test
        hung = instance.submit(_request())
        assert hung.status is PlanStatus.TIMEOUT
        assert hung.request_id == b"\x06" * 16
        assert hung.waypoints == ()
        # Và runner vẫn phục vụ được ngay sau đó.
        config.TIME_BUDGET_S = saved
        assert instance.submit(_request()).status is PlanStatus.OK
    finally:
        config.TIME_BUDGET_S = saved
        instance.stop()


def test_a_child_that_raises_becomes_internal_error_not_a_dead_runner() -> None:
    instance = PlanRunner(preloaded=None)
    instance.start()
    try:
        instance.force_raise_next = True  # cửa hậu chỉ dùng cho test
        broken = instance.submit(_request())
        assert broken.status is PlanStatus.INTERNAL_ERROR
        assert broken.detail
        assert instance.submit(_request()).status is PlanStatus.OK
    finally:
        instance.stop()


def test_config_mutation_in_a_child_cannot_leak_into_the_parent(runner: PlanRunner) -> None:
    """Cách ly 35 hằng số global là một trong hai lý do có tiến trình con."""
    import config

    before = config.NUM_START_CORNERS
    runner.submit(_request())
    assert config.NUM_START_CORNERS == before
