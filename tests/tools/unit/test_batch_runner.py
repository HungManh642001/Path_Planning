"""Kiểm thử đơn vị cho module BatchRegressionEngine và ReportGenerator."""

from __future__ import annotations

import json
from pathlib import Path
from unittest.mock import MagicMock

from path_planning.validation.oracle import ValidationResult
from tools.qa_suite.core.batch_runner import BatchRegressionEngine, BatchSummary
from tools.qa_suite.core.report_generator import ReportGenerator
from tools.qa_suite.core.runner import ExecutionDriver, ExecutionMode, QAResult


def test_batch_regression_engine_runs_presets(tmp_path: Path) -> None:
    """Kiểm thử BatchRegressionEngine chạy tập con preset thành công."""
    driver = ExecutionDriver(mode=ExecutionMode.LOCAL)
    engine = BatchRegressionEngine(driver=driver, time_budget_s=5.0)

    progress_calls: list[tuple[int, int, str]] = []

    def on_progress(current: int, total: int, name: str) -> None:
        progress_calls.append((current, total, name))

    target_scenarios = [
        "scenario_01_open_ocean",
        "scenario_02_single_obstacle",
    ]
    summary = engine.run_presets(
        target_names=target_scenarios,
        progress_callback=on_progress,
    )

    assert isinstance(summary, BatchSummary)
    assert summary.total_tests == 2
    assert summary.success_count == 2
    assert summary.fail_count == 0
    assert summary.success_rate == 100.0
    assert summary.oracle_violation_count == 0
    assert len(summary.results) == 2
    assert summary.timestamp is not None

    # Kiểm tra thống kê thời gian
    assert "min" in summary.wall_time_stats
    assert "max" in summary.wall_time_stats
    assert "mean" in summary.wall_time_stats
    assert "p50" in summary.wall_time_stats
    assert "p90" in summary.wall_time_stats
    assert "p95" in summary.wall_time_stats
    assert summary.wall_time_stats["min"] > 0.0

    # Kiểm tra thống kê chiều dài
    assert "min" in summary.path_length_stats
    assert "max" in summary.path_length_stats
    assert "mean" in summary.path_length_stats
    assert summary.path_length_stats["min"] > 0.0

    # Kiểm tra progress callback
    assert len(progress_calls) == 2
    assert progress_calls[0] == (1, 2, "scenario_01_open_ocean")
    assert progress_calls[1] == (2, 2, "scenario_02_single_obstacle")


def test_batch_regression_engine_runs_random_batch() -> None:
    """Kiểm thử BatchRegressionEngine chạy loạt kịch bản ngẫu nhiên."""
    driver = ExecutionDriver(mode=ExecutionMode.LOCAL)
    engine = BatchRegressionEngine(driver=driver, time_budget_s=5.0)

    summary = engine.run_random_batch(
        count=2,
        seed=100,
        topology="random",
    )

    assert isinstance(summary, BatchSummary)
    assert summary.total_tests == 2
    assert len(summary.results) == 2
    for res in summary.results:
        assert isinstance(res, QAResult)


def test_batch_regression_engine_handles_failures_and_violations() -> None:
    """Kiểm thử thống kê khi có kịch bản thất bại và vi phạm Oracle."""
    mock_driver = MagicMock(spec=ExecutionDriver)

    mock_res_ok = QAResult(
        scenario_name="test_ok",
        status="OK",
        is_success=True,
        waypoints=[((0.0, 0.0), 0.0), ((100.0, 100.0), 0.0)],
        path_length_m=141.42,
        wall_time_s=0.1,
        applied_time_budget_s=5.0,
        iterations=10,
        oracle_verdict=ValidationResult(True, "ok"),
    )
    mock_res_fail = QAResult(
        scenario_name="test_fail",
        status="NO_PATH",
        is_success=False,
        waypoints=[],
        path_length_m=0.0,
        wall_time_s=0.5,
        applied_time_budget_s=5.0,
        iterations=50,
        oracle_verdict=ValidationResult(False, "no path found"),
        error_detail="Search space exhausted",
    )
    mock_res_violation = QAResult(
        scenario_name="test_violation",
        status="OK",
        is_success=True,
        waypoints=[((0.0, 0.0), 0.0), ((100.0, 100.0), 0.0)],
        path_length_m=200.0,
        wall_time_s=0.2,
        applied_time_budget_s=5.0,
        iterations=20,
        oracle_verdict=ValidationResult(False, "Collision with island at (50, 50)"),
    )

    mock_driver.run_scenario.side_effect = [
        mock_res_ok,
        mock_res_fail,
        mock_res_violation,
    ]

    engine = BatchRegressionEngine(driver=mock_driver)
    summary = engine.run_presets(
        target_names=[
            "scenario_01_open_ocean",
            "scenario_02_single_obstacle",
            "scenario_03_narrow_gap",
        ]
    )

    assert summary.total_tests == 3
    assert summary.success_count == 2
    assert summary.fail_count == 1
    assert round(summary.success_rate, 2) == 66.67
    assert summary.oracle_violation_count == 1
    assert summary.wall_time_stats["min"] == 0.1
    assert summary.wall_time_stats["max"] == 0.5
    assert summary.path_length_stats["min"] == 141.42
    assert summary.path_length_stats["max"] == 200.0


def test_report_generator_json_export(tmp_path: Path) -> None:
    """Kiểm thử xuất báo cáo định dạng JSON."""
    mock_result = QAResult(
        scenario_name="test_json_scen",
        status="OK",
        is_success=True,
        waypoints=[((10.0, 20.0), 0.5), ((30.0, 40.0), 0.5)],
        path_length_m=100.0,
        wall_time_s=0.05,
        applied_time_budget_s=5.0,
        iterations=15,
        oracle_verdict=ValidationResult(True, "ok"),
    )
    summary = BatchSummary(
        total_tests=1,
        success_count=1,
        fail_count=0,
        success_rate=100.0,
        oracle_violation_count=0,
        wall_time_stats={
            "min": 0.05,
            "max": 0.05,
            "mean": 0.05,
            "p50": 0.05,
            "p90": 0.05,
            "p95": 0.05,
        },
        path_length_stats={"min": 100.0, "max": 100.0, "mean": 100.0},
        results=[mock_result],
        timestamp="2026-08-31T12:00:00Z",
    )

    json_path = tmp_path / "summary.json"
    ReportGenerator.export_json(summary, json_path)

    assert json_path.exists()
    data = json.loads(json_path.read_text(encoding="utf-8"))
    assert data["total_tests"] == 1
    assert data["success_rate"] == 100.0
    assert len(data["results"]) == 1
    assert data["results"][0]["scenario_name"] == "test_json_scen"
    assert data["results"][0]["oracle_verdict"]["is_ok"] is True


def test_report_generator_csv_export(tmp_path: Path) -> None:
    """Kiểm thử xuất báo cáo định dạng CSV."""
    mock_result = QAResult(
        scenario_name="test_csv_scen",
        status="OK",
        is_success=True,
        waypoints=[((0.0, 0.0), 0.0), ((10.0, 10.0), 0.0)],
        path_length_m=50.0,
        wall_time_s=0.02,
        applied_time_budget_s=5.0,
        iterations=8,
        oracle_verdict=ValidationResult(True, "ok"),
    )
    summary = BatchSummary(
        total_tests=1,
        success_count=1,
        fail_count=0,
        success_rate=100.0,
        oracle_violation_count=0,
        wall_time_stats={
            "min": 0.02,
            "max": 0.02,
            "mean": 0.02,
            "p50": 0.02,
            "p90": 0.02,
            "p95": 0.02,
        },
        path_length_stats={"min": 50.0, "max": 50.0, "mean": 50.0},
        results=[mock_result],
        timestamp="2026-08-31T12:00:00Z",
    )

    csv_path = tmp_path / "summary.csv"
    ReportGenerator.export_csv(summary, csv_path)

    assert csv_path.exists()
    content = csv_path.read_text(encoding="utf-8")
    expected_header = (
        "Scenario,Status,Success,WallTime_s,PathLength_m,Iterations,"
        "OracleValid,FailureReason"
    )
    assert expected_header in content
    assert "test_csv_scen,OK,True" in content


def test_report_generator_html_export(tmp_path: Path) -> None:
    """Kiểm thử xuất báo cáo định dạng HTML độc lập."""
    mock_result1 = QAResult(
        scenario_name="scen_pass",
        status="OK",
        is_success=True,
        waypoints=[((0.0, 0.0), 0.0), ((10.0, 10.0), 0.0)],
        path_length_m=50.0,
        wall_time_s=0.02,
        applied_time_budget_s=5.0,
        iterations=8,
        oracle_verdict=ValidationResult(True, "ok"),
    )
    mock_result2 = QAResult(
        scenario_name="scen_fail",
        status="NO_PATH",
        is_success=False,
        waypoints=[],
        path_length_m=0.0,
        wall_time_s=0.5,
        applied_time_budget_s=5.0,
        iterations=50,
        oracle_verdict=ValidationResult(False, "no path found"),
        error_detail="Obstacle blocked goal",
    )
    summary = BatchSummary(
        total_tests=2,
        success_count=1,
        fail_count=1,
        success_rate=50.0,
        oracle_violation_count=0,
        wall_time_stats={
            "min": 0.02,
            "max": 0.5,
            "mean": 0.26,
            "p50": 0.26,
            "p90": 0.45,
            "p95": 0.48,
        },
        path_length_stats={"min": 50.0, "max": 50.0, "mean": 50.0},
        results=[mock_result1, mock_result2],
        timestamp="2026-08-31T12:00:00Z",
    )

    html_path = tmp_path / "report.html"
    ReportGenerator.export_html(summary, html_path)

    assert html_path.exists()
    content = html_path.read_text(encoding="utf-8")
    assert "<!DOCTYPE html>" in content
    assert "VTX Path Planning" in content
    assert "scen_pass" in content
    assert "scen_fail" in content
    assert "50.0%" in content
    assert "Obstacle blocked goal" in content
    assert "<table" in content
    assert "<style>" in content
    assert "<script>" in content
