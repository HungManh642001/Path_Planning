"""Kiểm thử đơn vị cho module CLI của VTX QA Suite (tools.qa_suite.cli)."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from path_planning.validation.oracle import ValidationResult
from tools.qa_suite.cli import build_parser, main
from tools.qa_suite.core.batch_runner import BatchSummary
from tools.qa_suite.core.runner import QAResult
from tools.qa_suite.core.stress_tester import StressTestSummary


def _create_mock_batch_summary() -> BatchSummary:
    """Tạo đối tượng BatchSummary mẫu cho kiểm thử."""
    result = QAResult(
        scenario_name="scenario_01_open_space",
        status="OK",
        is_success=True,
        waypoints=[((100.0, 100.0), 0.0), ((500.0, 500.0), 0.0)],
        path_length_m=400.0,
        wall_time_s=0.05,
        applied_time_budget_s=15.0,
        iterations=10,
        oracle_verdict=ValidationResult(True, ""),
    )
    return BatchSummary(
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
        path_length_stats={"min": 400.0, "max": 400.0, "mean": 400.0},
        results=[result],
        timestamp="2026-08-31T00:00:00Z",
    )


def test_cli_parser_recognizes_subcommands() -> None:
    """Kiểm thử parser nhận diện đúng các subcommands và flags."""
    parser = build_parser()

    # run-presets
    args = parser.parse_args(
        ["run-presets", "--target", "local", "--presets", "scenario_01_open_space"]
    )
    assert args.command == "run-presets"
    assert args.target == "local"
    assert args.presets == ["scenario_01_open_space"]

    # batch-random
    args = parser.parse_args(
        [
            "batch-random",
            "--num-tests",
            "20",
            "--seed",
            "123",
            "--topology",
            "wall_block",
        ]
    )
    assert args.command == "batch-random"
    assert args.num_tests == 20
    assert args.seed == 123
    assert args.topology == "wall_block"

    # stress-test
    args = parser.parse_args(
        [
            "stress-test",
            "--concurrency",
            "8",
            "--requests",
            "50",
            "--timeout",
            "4.0",
        ]
    )
    assert args.command == "stress-test"
    assert args.concurrency == 8
    assert args.requests == 50
    assert args.timeout == 4.0

    # serve
    args = parser.parse_args(["serve", "--port", "8505"])
    assert args.command == "serve"
    assert args.port == 8505


def test_cli_run_presets_local(monkeypatch: pytest.MonkeyPatch) -> None:
    """Kiểm thử lệnh run-presets thực thi thành công."""
    mock_summary = _create_mock_batch_summary()
    mock_run_presets = MagicMock(return_value=mock_summary)

    with patch(
        "tools.qa_suite.core.batch_runner.BatchRegressionEngine.run_presets",
        mock_run_presets,
    ):
        ret = main(
            ["run-presets", "--target", "local", "--presets", "scenario_01_open_space"]
        )
        assert ret == 0
        mock_run_presets.assert_called_once()
        assert mock_run_presets.call_args.kwargs["target_names"] == [
            "scenario_01_open_space"
        ]


def test_cli_run_presets_export_html(tmp_path: Path) -> None:
    """Kiểm thử lệnh run-presets xuất file HTML báo cáo."""
    mock_summary = _create_mock_batch_summary()
    mock_run_presets = MagicMock(return_value=mock_summary)
    html_out = tmp_path / "report.html"

    with patch(
        "tools.qa_suite.core.batch_runner.BatchRegressionEngine.run_presets",
        mock_run_presets,
    ):
        ret = main(["run-presets", "--target", "local", "--export-html", str(html_out)])
        assert ret == 0
        assert html_out.exists()


def test_cli_batch_random(tmp_path: Path) -> None:
    """Kiểm thử lệnh batch-random thực thi và xuất đa định dạng."""
    mock_summary = _create_mock_batch_summary()
    mock_run_random = MagicMock(return_value=mock_summary)
    csv_out = tmp_path / "report.csv"
    json_out = tmp_path / "report.json"

    with patch(
        "tools.qa_suite.core.batch_runner.BatchRegressionEngine.run_random_batch",
        mock_run_random,
    ):
        ret = main(
            [
                "batch-random",
                "--num-tests",
                "5",
                "--seed",
                "100",
                "--topology",
                "random",
                "--export-csv",
                str(csv_out),
                "--export-json",
                str(json_out),
            ]
        )
        assert ret == 0
        assert csv_out.exists()
        assert json_out.exists()


def test_cli_stress_test() -> None:
    """Kiểm thử lệnh stress-test thực thi và in kết quả."""
    mock_stress_summary = StressTestSummary(
        total_requests=10,
        concurrency=2,
        success_count=10,
        error_count=0,
        timeout_count=0,
        throughput_rps=50.0,
        wall_time_s=0.2,
        latency_p50_s=0.01,
        latency_p90_s=0.02,
        latency_p95_s=0.02,
        latency_p99_s=0.02,
        latencies=[0.01] * 10,
    )
    mock_run_sync = MagicMock(return_value=mock_stress_summary)

    with patch(
        "tools.qa_suite.core.stress_tester.NatsStressTester.run_stress_test_sync",
        mock_run_sync,
    ):
        ret = main(["stress-test", "--concurrency", "2", "--requests", "10"])
        assert ret == 0
        mock_run_sync.assert_called_once()


def test_cli_serve() -> None:
    """Kiểm thử lệnh serve gọi streamlit run."""
    with patch("subprocess.run") as mock_subproc:
        mock_subproc.return_value.returncode = 0
        ret = main(["serve", "--port", "8502"])
        assert ret == 0
        mock_subproc.assert_called_once()
        args, _ = mock_subproc.call_args
        cmd = args[0]
        assert "streamlit" in cmd
        assert "--server.port" in cmd
        assert "8502" in cmd


def test_cli_no_command() -> None:
    """Kiểm thử chạy CLI không có subcommand trả về mã lỗi 1."""
    ret = main([])
    assert ret == 1
