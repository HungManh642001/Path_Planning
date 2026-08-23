"""Thời hạn cứng, và tính bền của service sau khi phải giết một tiến trình con.

Planner là Python thuần, CPU-bound, và chỉ kiểm tra ngân sách tại các điểm trong
vòng lặp search - nó KHÔNG hủy được từ bên ngoài một cách lịch sự. Tiến trình
con là cách duy nhất để có thời hạn cứng thật.
"""

from __future__ import annotations

import time

import pytest

from vtx_service.messages import (
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
    """Spec ghi median 56 ms cho forkserver + preload trên máy phát triển gốc.

    Từng nới tới 5.0 s trên máy này vì `planner_version()` (gọi trên MỌI
    reply) trả phí `git describe` (3,5-4,9 s đo được) trong CHÍNH tiến trình
    con dùng-một-lần mỗi request - bất kể cache function-level nào, vì mỗi
    request là một tiến trình con MỚI không kế thừa cache đó (forkserver
    fork+exec một interpreter mới). Từ khi `_PLANNER_VERSION` chuyển sang
    tính lúc IMPORT MODULE (một lần, trong tiến trình forkserver, trước khi
    có tiến trình con nào - xem `runtime.py`), subprocess đó biến mất hoàn
    toàn khỏi đường request: đo lại 4 submit liên tiếp trên máy này ra
    1.03-1.09 s mỗi lần, gần như không dao động. 2.0 s giữ khoảng 2x biên độ
    an toàn cho máy chậm/dao động tải, và vẫn đủ chặt để bắt được đúng kiểu
    hồi quy test này sinh ra để bắt: lỡ ai "tối ưu" _PLANNER_VERSION trở lại
    thành lazy/`lru_cache`, chi phí subprocess-trên-mỗi-request quay lại và
    đẩy con số này lên 3.5 s+, thất bại rõ ràng trước ngưỡng 2.0 s.
    """
    runner.submit(_request())  # bỏ lần đầu (khởi động forkserver)
    started = time.perf_counter()
    runner.submit(_request())
    assert time.perf_counter() - started < 2.0


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
        instance._force_hang_next = True  # cửa hậu chỉ dùng cho test
        hung = instance.submit(_request())
        assert hung.status is PlanStatus.TIMEOUT
        assert hung.request_id == b"\x06" * 16
        assert hung.waypoints == ()
        # Elapsed time is essentially the deadline it burned - 0.0 would be
        # the first field an operator would question on a TIMEOUT reply.
        assert hung.plan_wall_time_s > 0.0
        assert hung.stats.budget_bound is True
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
        instance._force_raise_next = True  # cửa hậu chỉ dùng cho test
        broken = instance.submit(_request())
        assert broken.status is PlanStatus.INTERNAL_ERROR
        assert broken.detail
        assert broken.plan_wall_time_s > 0.0
        # A child that raised was never bound by the time budget - only
        # TIMEOUT is.
        assert broken.stats.budget_bound is False
        assert instance.submit(_request()).status is PlanStatus.OK
    finally:
        instance.stop()


def test_config_mutation_in_a_child_cannot_leak_into_the_parent(runner: PlanRunner) -> None:
    """Cách ly 35 hằng số global là một trong hai lý do có tiến trình con."""
    import config

    before = config.NUM_START_CORNERS
    runner.submit(_request())
    assert config.NUM_START_CORNERS == before


def test_unlimited_budget_does_not_get_killed_by_the_grace_period() -> None:
    """F1: ``config.TIME_BUDGET_S = None`` ("không giới hạn") must not become
    a SIGKILL after ``grace_s`` seconds.

    Bug: the deadline was computed as ``effective_time_budget_s() + grace_s``.
    ``effective_time_budget_s()`` returns ``0.0`` for "unlimited" (see
    ``runtime.py``), so the deadline collapsed to plain ``grace_s`` - a real
    mission got SIGKILLed on every request regardless of how little time it
    actually needed.

    ``grace_s`` is deliberately tiny (10 ms) here: under the OLD code that
    makes the deadline ~10 ms, which a real mission (forkserver fork +
    planning) cannot possibly beat, so the bug would fail this test with
    near-100% reliability rather than by luck. Under the FIXED code the
    deadline falls back to ``unlimited_deadline_s`` (default 300 s), so the
    mission has all the time it needs and must succeed.
    """
    import config

    saved = config.TIME_BUDGET_S
    instance = PlanRunner(preloaded=None, grace_s=0.01)
    instance.start()
    try:
        config.TIME_BUDGET_S = None
        reply = instance.submit(_request())
        assert reply.status is PlanStatus.OK
        assert reply.detail == ""
    finally:
        config.TIME_BUDGET_S = saved
        instance.stop()
