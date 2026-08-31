"""Kiểm thử đơn vị cho các tiện ích runtime siêu dữ liệu và ngân sách service.vtx_service.runtime."""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from path_planning import config
from service.vtx_service import runtime
from service.vtx_service.runtime import (
    MAX_REQUEST_TIME_BUDGET_S,
    config_hash,
    effective_time_budget_s,
    planner_config_snapshot,
    planner_version,
)

_TRUE_REPO_ROOT = Path(__file__).resolve().parents[3]


def test_snapshot_is_discovered_dynamically_from_config() -> None:
    """Kiểm tra snapshot cấu hình tự động trích xuất các hằng số từ config."""
    # Arrange & Act
    snapshot = planner_config_snapshot()

    # Assert
    for name in ("TIME_BUDGET_S", "NUM_START_CORNERS", "GOAL_THRESHOLD"):
        assert name in snapshot
    assert len(snapshot) > 20


def test_config_hash_is_stable_and_sensitive_to_modifications() -> None:
    """Kiểm tra mã hash cấu hình ổn định và thay đổi ngay khi cấu hình thay đổi."""
    # Arrange
    first = config_hash()

    # Act & Assert
    assert first == config_hash()
    original = config.NUM_START_CORNERS
    try:
        config.NUM_START_CORNERS = original + 1
        assert config_hash() != first
    finally:
        config.NUM_START_CORNERS = original
    assert config_hash() == first


def test_planner_version_is_non_empty_string() -> None:
    """Kiểm tra hàm planner_version trả về chuỗi phiên bản không rỗng."""
    # Arrange & Act
    version = planner_version()

    # Assert
    assert isinstance(version, str) and len(version) > 0


def test_version_matches_git_describe_in_repo_checkout() -> None:
    """Kiểm tra chuỗi phiên bản khớp với git describe khi chạy trong git repo."""
    # Arrange
    if not (_TRUE_REPO_ROOT / ".git").exists():
        pytest.skip("Không chạy trong git checkout")

    # Act
    expected = subprocess.run(
        ["git", "describe", "--always", "--dirty"],
        cwd=_TRUE_REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()

    # Assert
    assert planner_version() == expected


def test_planner_version_does_not_call_subprocess_at_call_time(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Kiểm tra planner_version không gọi lại subprocess lúc thực thi (được cache import-time)."""
    # Arrange
    def _boom(*args: object, **kwargs: object) -> object:
        raise AssertionError("planner_version() không được gọi lại subprocess lúc runtime")

    monkeypatch.setattr(runtime.subprocess, "run", _boom)

    # Act & Assert
    assert runtime.planner_version() == runtime._PLANNER_VERSION


def test_no_request_budget_falls_back_to_config_default() -> None:
    """Kiểm tra khi client không truyền budget hoặc truyền 0 sẽ dùng config.TIME_BUDGET_S."""
    # Arrange & Act & Assert
    assert effective_time_budget_s() == float(config.TIME_BUDGET_S)
    assert effective_time_budget_s(0.0) == float(config.TIME_BUDGET_S)


def test_valid_request_budget_is_respected() -> None:
    """Kiểm tra ngân sách hợp lệ từ client được giữ nguyên."""
    # Arrange & Act & Assert
    assert effective_time_budget_s(2.5) == 2.5


@pytest.mark.parametrize("junk", [-1.0, float("inf"), float("nan")])
def test_invalid_request_budget_safely_falls_back_to_default(junk: float) -> None:
    """Kiểm tra giá trị ngân sách rác (âm, inf, nan) tự động rơi về giá trị mặc định mà không crash."""
    # Arrange & Act & Assert
    assert effective_time_budget_s(junk) == float(config.TIME_BUDGET_S)


def test_oversized_request_budget_is_clamped_to_max_ceiling() -> None:
    """Kiểm tra ngân sách vượt quá ngưỡng tối đa bị kẹp xuống MAX_REQUEST_TIME_BUDGET_S."""
    # Arrange & Act & Assert
    assert effective_time_budget_s(10_000.0) == MAX_REQUEST_TIME_BUDGET_S
