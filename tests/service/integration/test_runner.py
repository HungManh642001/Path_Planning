"""Kiểm thử tích hợp cho tiến trình thực thi cách ly PlanRunner trong service.vtx_service.runner."""

from __future__ import annotations

import os
import time
from collections.abc import Generator
from pathlib import Path

import pytest

from path_planning import config
from service.vtx_service.messages import (
    IDL_VERSION,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    VehicleLimits,
)
from service.vtx_service.runner import PlanRunner


LIMITS = VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0)


def _build_request(**overrides: object) -> PlanRequest:
    """Khởi tạo PlanRequest chuẩn cho test runner."""
    base: dict[str, object] = {
        "request_id": b"\x06" * 16,
        "idl_version": IDL_VERSION,
        "start": (50000.0, 50000.0),
        "start_heading_deg": 45.0,
        "goal": (300000.0, 250000.0),
        "goal_heading_deg": 45.0,
        "is_goal_heading_free": True,
        "islands": (),
        "dynamic_obstacles": (),
        "safezones": (),
        "use_preloaded_map": False,
        "limits": LIMITS,
        "budget": SearchBudget(15.0),
    }
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


@pytest.fixture()
def runner() -> Generator[PlanRunner, None, None]:
    """Fixture cung cấp instance PlanRunner đã khởi động và tự dọn dẹp sau test."""
    instance = PlanRunner(preloaded=None)
    instance.start()
    try:
        yield instance
    finally:
        instance.stop()


def test_mission_plans_successfully_in_child_process(runner: PlanRunner) -> None:
    """Kiểm tra lập kế hoạch đường bay thành công trong tiến trình con cách ly."""
    # Arrange & Act
    reply = runner.submit(_build_request())

    # Assert
    assert reply.status is PlanStatus.OK
    assert len(reply.waypoints) >= 2


def test_reply_data_integrity_across_process_boundary(runner: PlanRunner) -> None:
    """Kiểm tra tính toàn vẹn dữ liệu phản hồi khi truyền qua ranh giới tiến trình (IPC)."""
    # Arrange
    req = _build_request()

    # Act
    reply1 = runner.submit(req)
    reply2 = runner.submit(req)

    # Assert
    assert reply1.request_id == b"\x06" * 16
    assert isinstance(reply1.status, PlanStatus)
    assert reply1.path_length_m == reply2.path_length_m


def test_child_start_cost_within_performance_envelope(runner: PlanRunner) -> None:
    """Kiểm tra chi phí khởi động tiến trình con và tìm kiếm nằm trong ngưỡng hiệu năng cho phép."""
    # Arrange: Warm-up để forkserver sẵn sàng
    runner.submit(_build_request())

    # Act
    started = time.perf_counter()
    runner.submit(_build_request())
    elapsed = time.perf_counter() - started

    # Assert: Thời gian thực thi lần 2 phải dưới 1.0 giây
    assert elapsed < 1.0


def test_hung_child_process_times_out_and_runner_recovers() -> None:
    """Kiểm tra cơ chế thời hạn cứng: Giết tiến trình con bị treo và runner tiếp tục phục vụ bình thường."""
    # Arrange
    instance = PlanRunner(preloaded=None, grace_s=0.5)
    instance.start()
    try:
        instance._force_hang_next = True
        started = time.perf_counter()

        # Act
        hung = instance.submit(_build_request(budget=SearchBudget(0.5)))
        elapsed = time.perf_counter() - started

        # Assert
        assert hung.status is PlanStatus.TIMEOUT
        assert hung.request_id == b"\x06" * 16
        assert hung.waypoints == ()
        assert elapsed < 5.0
        assert hung.plan_wall_time_s > 0.0
        assert hung.stats.is_budget_bound is True

        # Runner vẫn hoạt động tốt cho request tiếp theo
        assert instance.submit(_build_request()).status is PlanStatus.OK
    finally:
        instance.stop()


def test_child_exception_becomes_internal_error_without_killing_runner() -> None:
    """Kiểm tra ngoại lệ trong tiến trình con chuyển thành INTERNAL_ERROR mà không làm chết runner."""
    # Arrange
    instance = PlanRunner(preloaded=None)
    instance.start()
    try:
        instance._force_raise_next = True

        # Act
        broken = instance.submit(_build_request())

        # Assert
        assert broken.status is PlanStatus.INTERNAL_ERROR
        assert broken.detail != ""
        assert broken.plan_wall_time_s > 0.0
        assert broken.stats.is_budget_bound is False
        assert instance.submit(_build_request()).status is PlanStatus.OK
    finally:
        instance.stop()


def test_config_mutation_in_child_does_not_leak_to_parent(
    runner: PlanRunner,
) -> None:
    """Kiểm tra tính cách ly: Thay đổi cấu hình trong tiến trình con không rò rỉ sang tiến trình cha."""
    # Arrange
    before = config.NUM_START_CORNERS

    # Act
    runner.submit(_build_request())

    # Assert
    assert before == config.NUM_START_CORNERS


def test_start_ensures_pythonpath_environment_variable(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Kiểm tra phương thức start() bổ sung đầy đủ PYTHONPATH cho forkserver."""
    # Arrange
    sentinel = "/custom/user/path"
    monkeypatch.setenv("PYTHONPATH", sentinel)

    # Act
    instance = PlanRunner(preloaded=None)
    instance.start()
    try:
        repo_root = str(Path(__file__).resolve().parents[3])
        service_root = str(Path(__file__).resolve().parents[3] / "src" / "service")
        entries = os.environ["PYTHONPATH"].split(os.pathsep)

        # Assert
        assert repo_root in entries
        assert service_root in entries
        assert sentinel in entries
    finally:
        instance.stop()
