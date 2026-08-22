"""Siêu dữ liệu phiên bản, và giá trị ngân sách service THỰC SỰ dùng.

Client gửi `time_budget_s` nhưng service chưa tôn trọng nó. Nhận một trường rồi
lặng lẽ bỏ qua là cách chắc chắn để client tin vào một điều không đúng, nên
service báo cáo ngược giá trị thật.
"""

from __future__ import annotations

import config

from vtx_service.runtime import (
    config_hash,
    effective_max_iterations,
    effective_time_budget_s,
    planner_config_snapshot,
    planner_version,
)


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
