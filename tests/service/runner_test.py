"""Thời hạn cứng, và tính bền của service sau khi phải giết một tiến trình con.

Planner là Python thuần, CPU-bound, và chỉ kiểm tra ngân sách tại các điểm trong
vòng lặp search - nó KHÔNG hủy được từ bên ngoài một cách lịch sự. Tiến trình
con là cách duy nhất để có thời hạn cứng thật.
"""

from __future__ import annotations

import os
import time
from pathlib import Path

from path_planning import config
import pytest

from service.vtx_service.messages import (
    IDL_VERSION,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    VehicleLimits,
)
from service.vtx_service.runner import PlanRunner

LIMITS = VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0)


def _request(**overrides: object) -> PlanRequest:
    base: dict[str, object] = dict(
        request_id=b"\x06" * 16,
        idl_version=IDL_VERSION,
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
        budget=SearchBudget(15.0),
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
    toàn khỏi đường request.

    MỘT LẦN NỮA nới ngưỡng này trông như flake trước khi lộ ra là chính XÁC
    cùng một họ lỗi: `sys.path` bị `conftest.py` sửa LÚC CHẠY, mà forkserver
    là fork+exec - một interpreter MỚI đọc `PYTHONPATH` từ biến môi trường,
    KHÔNG thấy được `sys.path` cha thêm lúc chạy. Khi forkserver không import
    được `_PRELOAD`, `multiprocessing` NUỐT ImportError ÂM THẦM, quay về tự
    import mọi thứ trong tiến trình con - kể cả `git describe` mà đoạn trên
    tưởng đã loại bỏ. ĐO ĐƯỢC trên máy này, 3 submit mỗi cách: `sys.path` sửa
    lúc chạy 3.52 / 3.69 / 4.05 s; `PYTHONPATH` đặt qua biến môi trường 0.07 /
    0.07 / 0.08 s - khoảng 50x. Sửa ở `PlanRunner.start()`
    (`_ensure_pythonpath_for_forkserver`), không phải ở đây.

    Sau sửa, `--durations` đo được submit thứ hai ~0.2-0.3 s trên máy này.
    NGƯỠNG SIẾT từ 2.0 s xuống 1.0 s: 2.0 s không còn "đủ chặt" theo đúng
    nghĩa test này tồn tại để bắt - nó vẫn bắt được kiểu hồi quy CŨ (import
    thất bại quay về ~3.5 s+) nhưng sẽ bỏ lọt một hồi quy NHỎ hơn, ví dụ một
    chi phí ~500 ms-1 s mới bị thêm vào đường request mà không đủ lớn để chạm
    2.0 s. 1.0 s vẫn giữ ~3-5x biên độ an toàn trên số đo thực tế (~0.2-0.3 s)
    cho máy chậm/dao động tải, và thấp hơn MỘT BẬC ĐỘ LỚN so với sàn của cả
    hai kiểu hồi quy đã biết (~3.5 s).
    """
    runner.submit(_request())  # bỏ lần đầu (khởi động forkserver)
    started = time.perf_counter()
    runner.submit(_request())
    assert time.perf_counter() - started < 1.0


def test_a_hung_child_becomes_timeout_and_the_runner_keeps_working() -> None:
    """Thời hạn CỨNG: tiến trình con treo phải bị giết, không được treo service.

    Ngân sách 0,5 s đi trong CHÍNH REQUEST, vì thời hạn cứng được tính là
    `effective_time_budget_s(request.budget.time_budget_s) + grace`. Với mặc
    định thật của config (15 s) test này sẽ chạy 15,5 giây và TRÔNG NHƯ TREO —
    đúng thứ nó sinh ra để phát hiện.
    """
    instance = PlanRunner(preloaded=None, grace_s=0.5)
    instance.start()
    try:
        instance._force_hang_next = True  # cửa hậu chỉ dùng cho test
        started = time.perf_counter()
        hung = instance.submit(_request(budget=SearchBudget(0.5)))
        elapsed = time.perf_counter() - started
        assert hung.status is PlanStatus.TIMEOUT
        assert hung.request_id == b"\x06" * 16
        assert hung.waypoints == ()
        # Thời hạn bám ngân sách của REQUEST, không phải mặc định của config:
        # 0,5 + 0,5 s ân hạn, chứ không phải 15,5 s.
        assert elapsed < 5.0, f"thời hạn cứng không bám ngân sách request: {elapsed:.1f}s"
        # Elapsed time is essentially the deadline it burned - 0.0 would be
        # the first field an operator would question on a TIMEOUT reply.
        assert hung.plan_wall_time_s > 0.0
        assert hung.stats.budget_bound is True
        # Và runner vẫn phục vụ được ngay sau đó.
        assert instance.submit(_request()).status is PlanStatus.OK
    finally:
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
    from path_planning import config

    before = config.NUM_START_CORNERS
    runner.submit(_request())
    assert config.NUM_START_CORNERS == before


def test_an_empty_request_budget_does_not_collapse_the_deadline() -> None:
    """F1, dưới dạng còn lại sau khi "không giới hạn" bị bỏ.

    Ngân sách trống (``0.0``) trên dây phải rơi về mặc định của service TRƯỚC
    khi cộng ân hạn. Cộng thẳng ``0.0 + grace_s`` biến nó thành một SIGKILL sau
    đúng ``grace_s`` giây trên MỌI request - đó chính là lỗi F1, và một client
    không điền trường budget là cách dễ nhất để gặp lại nó.

    ``grace_s`` cố tình bé (10 ms): dưới lỗi đó thời hạn còn ~10 ms, thứ mà một
    mission thật (fork + lập kế hoạch) không thể nào kịp, nên test đỏ gần như
    chắc chắn thay vì đỏ theo may rủi.
    """
    instance = PlanRunner(preloaded=None, grace_s=0.01)
    instance.start()
    try:
        reply = instance.submit(_request(budget=SearchBudget(0.0)))
        assert reply.status is PlanStatus.OK
        assert reply.detail == ""
        assert reply.applied_time_budget_s == float(config.TIME_BUDGET_S)
    finally:
        instance.stop()


def test_start_ensures_pythonpath_for_the_forkserver(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Pins the MECHANISM, not the clock.

    ``forkserver`` is fork+exec - a fresh interpreter that reads
    ``PYTHONPATH`` from the environment, not the parent's runtime
    ``sys.path`` (exactly what pytest's ``conftest.py`` sets up: it inserts
    the repo root and ``service/`` into ``sys.path`` at import time, without
    touching the environment). When the forkserver cannot import
    ``_PRELOAD`` because of that, ``multiprocessing`` swallows the
    ``ImportError`` SILENTLY, and every child falls back to importing
    everything itself - including the import-time ``git describe`` R15 was
    supposed to remove from the request path.

    Measured on this machine, 3 submits each: ``sys.path`` edited at runtime
    (the broken case) 3.52 / 3.69 / 4.05 s; ``PYTHONPATH`` set as an
    environment variable (the fixed case) 0.07 / 0.07 / 0.08 s - roughly 50x,
    invisible anywhere except the wall clock.

    ``PYTHONPATH`` is pre-seeded with an unrelated sentinel entry here to
    pin the other half of the contract: ``start()`` must APPEND, never
    replace - a caller may have legitimate entries of its own.
    """
    sentinel = "/some/unrelated/caller/entry"
    monkeypatch.setenv("PYTHONPATH", sentinel)

    instance = PlanRunner(preloaded=None)
    instance.start()
    try:
        # Derived independently of runner._REPO_ROOT / _SERVICE_ROOT, so this
        # test can tell a correct value from a broken one instead of trusting
        # the same computation it means to check.
        repo_root = str(Path(__file__).resolve().parents[2])
        service_root = str(Path(__file__).resolve().parents[2] / "src" / "service")
        entries = os.environ["PYTHONPATH"].split(os.pathsep)
        assert repo_root in entries
        assert service_root in entries
        assert sentinel in entries
    finally:
        instance.stop()
