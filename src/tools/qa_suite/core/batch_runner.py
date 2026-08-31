"""Module thực thi hồi quy hàng loạt (Batch Regression Engine).

Hỗ trợ chạy hàng loạt các kịch bản chuẩn (presets) hoặc các kịch bản sinh ngẫu
nhiên (random batch), tổng hợp số liệu thống kê thời gian và độ dài quỹ đạo,
cùng việc kiểm định độc lập qua Validation Oracle.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import TYPE_CHECKING

import numpy as np

from path_planning.scenario.generator import generate_random_scenario
from path_planning.scenario.presets import get_all_scenarios
from tools.qa_suite.core.runner import ExecutionDriver


if TYPE_CHECKING:
    from path_planning.types import Topology
    from tools.qa_suite.core.runner import QAResult


@dataclass
class BatchSummary:
    """Tổng hợp số liệu thống kê sau một phiên chạy kiểm thử hàng loạt.

    Attributes:
        total_tests: Tổng số lượng kịch bản đã thực thi.
        success_count: Số kịch bản thuật toán tìm được đường bay thành công.
        fail_count: Số kịch bản tìm đường bay thất bại (NO_PATH, TIMEOUT, vv).
        success_rate: Tỷ lệ phần trăm thành công (0.0 đến 100.0).
        oracle_violation_count: Số kịch bản bị Oracle từ chối/phát hiện lỗi.
        wall_time_stats: Thống kê thời gian chạy (min, max, mean, p50, p90, p95).
        path_length_stats: Thống kê chiều dài đường bay (min, max, mean) (m).
        results: Danh sách chi tiết kết quả từng kịch bản (QAResult).
        timestamp: Chuỗi thời gian ISO 8601 khi kết thúc đợt kiểm thử.
    """

    total_tests: int
    success_count: int
    fail_count: int
    success_rate: float
    oracle_violation_count: int
    wall_time_stats: dict[str, float]
    path_length_stats: dict[str, float]
    results: list[QAResult]
    timestamp: str


def _calculate_wall_time_stats(wall_times: list[float]) -> dict[str, float]:
    """Tính toán các chỉ số thống kê phân vị và trung bình cho thời gian chạy."""
    if not wall_times:
        return {
            "min": 0.0,
            "max": 0.0,
            "mean": 0.0,
            "p50": 0.0,
            "p90": 0.0,
            "p95": 0.0,
        }
    arr = np.array(wall_times, dtype=float)
    return {
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
        "mean": float(np.mean(arr)),
        "p50": float(np.percentile(arr, 50)),
        "p90": float(np.percentile(arr, 90)),
        "p95": float(np.percentile(arr, 95)),
    }


def _calculate_path_length_stats(
    path_lengths: list[float],
) -> dict[str, float]:
    """Tính toán các chỉ số thống kê min, max, mean cho chiều dài đường bay."""
    if not path_lengths:
        return {
            "min": 0.0,
            "max": 0.0,
            "mean": 0.0,
        }
    arr = np.array(path_lengths, dtype=float)
    return {
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
        "mean": float(np.mean(arr)),
    }


class BatchRegressionEngine:
    """Động cơ điều phối và tổng hợp kết quả kiểm thử hồi quy hàng loạt."""

    def __init__(
        self,
        driver: ExecutionDriver | None = None,
        time_budget_s: float = 15.0,
    ) -> None:
        """Khởi tạo BatchRegressionEngine.

        Args:
            driver: Bộ điều phối thực thi kịch bản (ExecutionDriver).
            time_budget_s: Ngân sách thời gian tối đa cho mỗi kịch bản (giây).
        """
        self.driver = driver if driver is not None else ExecutionDriver()
        self.time_budget_s = time_budget_s

    def _build_summary(self, results: list[QAResult]) -> BatchSummary:
        """Tổng hợp danh sách QAResult thành đối tượng BatchSummary."""
        total_tests = len(results)
        success_count = sum(1 for r in results if r.is_success)
        fail_count = total_tests - success_count
        success_rate = (success_count / total_tests * 100.0) if total_tests > 0 else 0.0

        oracle_violation_count = sum(
            1
            for r in results
            if (r.is_success and not r.oracle_verdict.is_ok)
            or (len(r.waypoints) >= 2 and not r.oracle_verdict.is_ok)
        )

        wall_times = [r.wall_time_s for r in results]
        wall_time_stats = _calculate_wall_time_stats(wall_times)

        successful_lengths = [
            r.path_length_m for r in results if r.is_success and r.path_length_m > 0
        ]
        path_length_stats = _calculate_path_length_stats(successful_lengths)

        timestamp = datetime.now(timezone.utc).isoformat()

        return BatchSummary(
            total_tests=total_tests,
            success_count=success_count,
            fail_count=fail_count,
            success_rate=success_rate,
            oracle_violation_count=oracle_violation_count,
            wall_time_stats=wall_time_stats,
            path_length_stats=path_length_stats,
            results=results,
            timestamp=timestamp,
        )

    def run_presets(
        self,
        target_names: list[str] | None = None,
        progress_callback: Callable[[int, int, str], None] | None = None,
    ) -> BatchSummary:
        """Chạy kiểm thử trên tập kịch bản chuẩn (Preset scenarios).

        Args:
            target_names: Danh sách tên kịch bản muốn chạy.
            progress_callback: Callback nhận (current_idx, total, name).

        Returns:
            BatchSummary: Báo cáo tổng kết toàn bộ kịch bản đã chạy.

        Raises:
            KeyError: Nếu kịch bản trong target_names không tồn tại.
        """
        all_presets = get_all_scenarios()

        if target_names is not None:
            for name in target_names:
                if name not in all_presets:
                    avail = list(all_presets.keys())
                    raise KeyError(
                        f"Scenario '{name}' not found in presets. Available: {avail}"
                    )
            selected_items = [(name, all_presets[name]) for name in target_names]
        else:
            selected_items = list(all_presets.items())

        total = len(selected_items)
        results: list[QAResult] = []

        for idx, (name, builder) in enumerate(selected_items, start=1):
            if progress_callback is not None:
                progress_callback(idx, total, name)
            scenario = builder()
            result = self.driver.run_scenario(
                scenario, name=name, time_budget_s=self.time_budget_s
            )
            results.append(result)

        return self._build_summary(results)

    def run_random_batch(
        self,
        count: int = 10,
        seed: int = 42,
        topology: Topology = "random",
        progress_callback: Callable[[int, int, str], None] | None = None,
    ) -> BatchSummary:
        """Chạy kiểm thử trên tập kịch bản sinh ngẫu nhiên.

        Args:
            count: Số lượng kịch bản ngẫu nhiên cần sinh và chạy.
            seed: Seed khởi tạo ban đầu để đảm bảo tính tái lập.
            topology: Kiểu phân bố ("random", "center_cluster", "wall_block").
            progress_callback: Callback nhận (current_idx, total, name).

        Returns:
            BatchSummary: Báo cáo tổng kết toàn bộ kịch bản đã chạy.
        """
        results: list[QAResult] = []

        for i in range(count):
            current_seed = seed + i
            name = f"random_{topology}_{current_seed}"
            if progress_callback is not None:
                progress_callback(i + 1, count, name)

            scenario = generate_random_scenario(seed=current_seed, topology=topology)
            result = self.driver.run_scenario(
                scenario, name=name, time_budget_s=self.time_budget_s
            )
            results.append(result)

        return self._build_summary(results)
