"""Siêu dữ liệu phiên bản, và giá trị ngân sách service THỰC SỰ dùng.

Client gửi `time_budget_s` nhưng service chưa tôn trọng nó. Nhận một trường rồi
lặng lẽ bỏ qua là cách chắc chắn để client tin vào một điều không đúng, nên
service báo cáo ngược giá trị thật.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import config
import pytest

from vtx_service.runtime import (
    config_hash,
    effective_max_iterations,
    effective_time_budget_s,
    planner_config_snapshot,
    planner_version,
)
import vtx_service.runtime as runtime

# Derived independently of vtx_service.runtime._REPO_ROOT: this must locate
# the real repo root on its own, so the test can tell a correct root from a
# broken one instead of trusting the same value it is meant to check.
_TRUE_REPO_ROOT = Path(__file__).resolve().parents[2]


def test_snapshot_is_discovered_not_hardcoded() -> None:
    snapshot = planner_config_snapshot()
    # Vài knob chắc chắn v0 đọc. KHÔNG khẳng định tổng số: con số đó phải được
    # phép đổi khi thuật toán đổi - đó chính là mục đích của cơ chế này.
    for name in ("TIME_BUDGET_S", "MAX_ITERATIONS", "NUM_START_CORNERS", "GOAL_THRESHOLD"):
        assert name in snapshot
    assert len(snapshot) > 20


def test_snapshot_excludes_constants_the_shipped_planner_never_reads() -> None:
    # CIRCLE_GRAZE_TOL_M bị khai tử và không planner nào đọc nó.
    assert "CIRCLE_GRAZE_TOL_M" not in planner_config_snapshot()


def test_hash_is_stable_and_sensitive() -> None:
    first = config_hash()
    assert first == config_hash()
    original = config.NUM_START_CORNERS
    try:
        config.NUM_START_CORNERS = original + 1
        assert config_hash() != first
    finally:
        config.NUM_START_CORNERS = original
    assert config_hash() == first


def test_version_is_a_non_empty_string() -> None:
    assert isinstance(planner_version(), str) and planner_version()


def test_version_matches_git_describe_inside_a_checkout() -> None:
    """Pins planner_version() to the real `git describe`, not merely non-empty.

    `test_version_is_a_non_empty_string` cannot tell a correct root from a
    broken one: `planner_version()`'s own fallback ("unknown") is itself a
    non-empty string. This test computes the expected value from an
    independently-derived repo root (not from `runtime._REPO_ROOT`), so it
    goes red if `_REPO_ROOT` is ever pointed somewhere without a `.git`.
    """
    if not (_TRUE_REPO_ROOT / ".git").exists():
        pytest.skip("not running inside a git checkout")
    expected = subprocess.run(
        ["git", "describe", "--always", "--dirty"],
        cwd=_TRUE_REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()
    assert planner_version() == expected


def test_planner_version_never_calls_subprocess(monkeypatch: pytest.MonkeyPatch) -> None:
    """`planner_version()` không được gọi subprocess - giá trị đã có sẵn từ lúc import.

    Thiết kế cũ (cache "tính lúc gọi đầu tiên", function-level) đo được KHÔNG
    rẻ đi: `PlanRunner` tạo một tiến trình con MỚI cho mỗi request, và
    `forkserver` fork+exec một interpreter hoàn toàn mới (`ForkServer.
    ensure_running` dùng `spawnv_passfds`) - tiến trình con không kế thừa
    trạng thái Python đã import của cha, nên mỗi tiến trình con dùng-một-lần
    tự trả phí `git describe` (3,79-5,12 s đo được) rồi chết theo nó, cache
    kiểu đó không giúp được gì. `_PLANNER_VERSION` giờ được tính Ở MODULE
    LEVEL, một lần, lúc module này được `_PRELOAD` import trong tiến trình
    forkserver (trước khi có tiến trình con nào) - nên `planner_version()`
    chỉ đọc một hằng số, không bao giờ chạm `subprocess.run`. Khẳng định trên
    CƠ CHẾ (subprocess có bị gọi không), không phải trên đồng hồ treo tường:
    một khẳng định thời gian sẽ chập chờn đúng trên filesystem đã gây ra vấn
    đề này.
    """

    def _boom(*args: object, **kwargs: object) -> object:
        raise AssertionError(
            "planner_version() gọi subprocess.run - phải là hằng số import-time"
        )

    monkeypatch.setattr(runtime.subprocess, "run", _boom)

    assert runtime.planner_version() == runtime._PLANNER_VERSION
    assert runtime.planner_version() == runtime.planner_version()


def test_effective_budget_comes_from_config_not_from_the_request() -> None:
    assert effective_time_budget_s() == float(config.TIME_BUDGET_S or 0.0)
    assert effective_max_iterations() == config.MAX_ITERATIONS


def test_effective_budget_is_a_float_even_when_config_says_none() -> None:
    original = config.TIME_BUDGET_S
    try:
        config.TIME_BUDGET_S = None
        assert effective_time_budget_s() == 0.0
    finally:
        config.TIME_BUDGET_S = original
