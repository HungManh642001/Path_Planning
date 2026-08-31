"""Giao diện dòng lệnh (CLI) cho bộ công cụ kiểm thử VTX QA Suite.

Cung cấp các lệnh:
1. run-presets: Chạy kiểm thử trên tập kịch bản chuẩn (Local hoặc NATS).
2. batch-random: Chạy kiểm thử hồi quy trên tập kịch bản ngẫu nhiên.
3. stress-test: Đo tải đồng thời và độ trễ trên NATS Microservice.
4. serve: Khởi chạy giao diện Web tương tác Streamlit.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

from path_planning.scenario.presets import get_all_scenarios
from service.vtx_service.transport import DEFAULT_NATS_SERVER, DEFAULT_SUBJECT
from tools.qa_suite.core.batch_runner import BatchRegressionEngine
from tools.qa_suite.core.report_generator import ReportGenerator
from tools.qa_suite.core.runner import ExecutionDriver, ExecutionMode
from tools.qa_suite.core.stress_tester import NatsStressTester


def build_parser() -> argparse.ArgumentParser:
    """Xây dựng ArgumentParser với các subcommand của QA Suite."""
    parser = argparse.ArgumentParser(
        prog="python -m tools.qa_suite.cli",
        description=(
            "VTX Path Planning QA Suite - Unified Verification & Benchmark Tool"
        ),
    )
    subparsers = parser.add_subparsers(
        dest="command", help="Lệnh kiểm thử cần thực thi"
    )

    # Subcommand: run-presets
    presets_parser = subparsers.add_parser(
        "run-presets", help="Chạy kiểm thử trên các kịch bản chuẩn (Presets)"
    )
    presets_parser.add_argument(
        "--target",
        choices=["local", "nats"],
        default="local",
        help="Chế độ thực thi: 'local' (nội bộ) hoặc 'nats' (microservice)",
    )
    presets_parser.add_argument(
        "--presets",
        nargs="+",
        default=None,
        help="Danh sách tên các preset cần chạy (mặc định: chạy toàn bộ 18 presets)",
    )
    presets_parser.add_argument(
        "--nats-url",
        default=DEFAULT_NATS_SERVER,
        help="Địa chỉ NATS server kết nối tới (khi dùng target=nats)",
    )
    presets_parser.add_argument(
        "--subject",
        default=DEFAULT_SUBJECT,
        help="Subject NATS gửi request lập lịch đường bay",
    )
    presets_parser.add_argument(
        "--time-budget",
        type=float,
        default=15.0,
        help="Ngân sách thời gian tối đa cho mỗi kịch bản (giây)",
    )
    presets_parser.add_argument(
        "--export-html",
        type=str,
        default=None,
        help="Đường dẫn file HTML xuất báo cáo tổng hợp",
    )
    presets_parser.add_argument(
        "--export-csv",
        type=str,
        default=None,
        help="Đường dẫn file CSV xuất báo cáo tổng hợp",
    )
    presets_parser.add_argument(
        "--export-json",
        type=str,
        default=None,
        help="Đường dẫn file JSON xuất báo cáo tổng hợp",
    )

    # Subcommand: batch-random
    random_parser = subparsers.add_parser(
        "batch-random",
        help="Chạy kiểm thử hồi quy trên tập kịch bản sinh ngẫu nhiên",
    )
    random_parser.add_argument(
        "--num-tests",
        type=int,
        default=10,
        help="Số lượng kịch bản ngẫu nhiên cần sinh và kiểm thử",
    )
    random_parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Seed khởi tạo sinh số ngẫu nhiên",
    )
    random_parser.add_argument(
        "--topology",
        choices=["random", "center_cluster", "wall_block"],
        default="random",
        help="Kiểu hình học phân bố chướng ngại vật",
    )
    random_parser.add_argument(
        "--target",
        choices=["local", "nats"],
        default="local",
        help="Chế độ thực thi: 'local' hoặc 'nats'",
    )
    random_parser.add_argument(
        "--nats-url",
        default=DEFAULT_NATS_SERVER,
        help="Địa chỉ NATS server kết nối tới",
    )
    random_parser.add_argument(
        "--subject",
        default=DEFAULT_SUBJECT,
        help="Subject NATS gửi request",
    )
    random_parser.add_argument(
        "--time-budget",
        type=float,
        default=15.0,
        help="Ngân sách thời gian tối đa cho mỗi kịch bản (giây)",
    )
    random_parser.add_argument(
        "--export-html",
        type=str,
        default=None,
        help="Đường dẫn file HTML xuất báo cáo tổng hợp",
    )
    random_parser.add_argument(
        "--export-csv",
        type=str,
        default=None,
        help="Đường dẫn file CSV xuất báo cáo tổng hợp",
    )
    random_parser.add_argument(
        "--export-json",
        type=str,
        default=None,
        help="Đường dẫn file JSON xuất báo cáo tổng hợp",
    )

    # Subcommand: stress-test
    stress_parser = subparsers.add_parser(
        "stress-test",
        help="Đo tải đồng thời (stress test) NATS Microservice",
    )
    stress_parser.add_argument(
        "--nats-url",
        default=DEFAULT_NATS_SERVER,
        help="Địa chỉ NATS server kết nối tới",
    )
    stress_parser.add_argument(
        "--subject",
        default=DEFAULT_SUBJECT,
        help="Subject NATS gửi request",
    )
    stress_parser.add_argument(
        "--concurrency",
        type=int,
        default=5,
        help="Số lượng request đồng thời tối đa",
    )
    stress_parser.add_argument(
        "--requests",
        type=int,
        default=50,
        help="Tổng số lượng requests cần gửi",
    )
    stress_parser.add_argument(
        "--timeout",
        type=float,
        default=6.0,
        help="Thời gian timeout tối đa cho mỗi request (giây)",
    )
    stress_parser.add_argument(
        "--scenario",
        type=str,
        default=None,
        help="Tên kịch bản preset dùng để đo tải (mặc định: scenario_01_open_ocean)",
    )

    # Subcommand: serve
    serve_parser = subparsers.add_parser(
        "serve",
        help="Khởi chạy ứng dụng Web tương tác Streamlit",
    )
    serve_parser.add_argument(
        "--port",
        type=int,
        default=8501,
        help="Cổng (port) lắng nghe cho máy chủ Streamlit (mặc định: 8501)",
    )

    return parser


def _print_batch_summary(summary_title: str, summary: object) -> None:
    """In bảng tóm tắt kết quả kiểm thử ra stdout."""
    # Lấy các thuộc tính từ summary (BatchSummary)
    total = getattr(summary, "total_tests", 0)
    success = getattr(summary, "success_count", 0)
    failed = getattr(summary, "fail_count", 0)
    rate = getattr(summary, "success_rate", 0.0)
    violations = getattr(summary, "oracle_violation_count", 0)
    wall_stats = getattr(summary, "wall_time_stats", {})
    len_stats = getattr(summary, "path_length_stats", {})

    print(f"\n{'=' * 70}")
    print(f"  {summary_title}")
    print(f"{'=' * 70}")
    print(f"  Total Scenarios Executed : {total}")
    print(f"  Passed (Valid Paths)    : {success}")
    print(f"  Failed (No Path/Timeout): {failed}")
    print(f"  Success Rate            : {rate:.1f}%")
    print(f"  Oracle Violations       : {violations}")
    print(f"{'-' * 70}")
    print(
        f"  Wall Time (s)           : Mean={wall_stats.get('mean', 0.0):.4f}s | "
        f"P50={wall_stats.get('p50', 0.0):.4f}s | "
        f"P95={wall_stats.get('p95', 0.0):.4f}s"
    )
    print(
        f"  Path Length (m)         : Mean={len_stats.get('mean', 0.0):,.1f}m | "
        f"Min={len_stats.get('min', 0.0):,.1f}m | "
        f"Max={len_stats.get('max', 0.0):,.1f}m"
    )
    print(f"{'=' * 70}\n")


def _export_reports(
    summary: object,
    html_path: str | None,
    csv_path: str | None,
    json_path: str | None,
) -> None:
    """Xuất file báo cáo theo cấu hình."""
    from tools.qa_suite.core.batch_runner import BatchSummary

    if not isinstance(summary, BatchSummary):
        return

    if html_path:
        ReportGenerator.export_html(summary, html_path)
        print(f"  [+] HTML Report exported to: {html_path}")
    if csv_path:
        ReportGenerator.export_csv(summary, csv_path)
        print(f"  [+] CSV Report exported to: {csv_path}")
    if json_path:
        ReportGenerator.export_json(summary, json_path)
        print(f"  [+] JSON Report exported to: {json_path}")


def main(argv: list[str] | None = None) -> int:
    """Hàm entrypoint chính xử lý tham số dòng lệnh và điều phối thực thi.

    Args:
        argv: Danh sách tham số dòng lệnh (mặc định lấy từ sys.argv[1:]).

    Returns:
        int: Mã thoát (0: thành công, 1: lỗi tham số hoặc thất bại).
    """
    parser = build_parser()
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    if not args.command:
        parser.print_help()
        return 1

    if args.command == "run-presets":
        mode = ExecutionMode.LOCAL if args.target == "local" else ExecutionMode.NATS
        driver = ExecutionDriver(
            mode=mode, nats_url=args.nats_url, subject=args.subject
        )
        engine = BatchRegressionEngine(driver=driver, time_budget_s=args.time_budget)

        def progress_cb(cur: int, total: int, name: str) -> None:
            print(f"  [{cur:02d}/{total:02d}] Executing preset: {name} ...")

        summary = engine.run_presets(
            target_names=args.presets, progress_callback=progress_cb
        )
        _print_batch_summary("VTX QA Suite - Presets Benchmark Summary", summary)
        _export_reports(summary, args.export_html, args.export_csv, args.export_json)
        return 0

    elif args.command == "batch-random":
        mode = ExecutionMode.LOCAL if args.target == "local" else ExecutionMode.NATS
        driver = ExecutionDriver(
            mode=mode, nats_url=args.nats_url, subject=args.subject
        )
        engine = BatchRegressionEngine(driver=driver, time_budget_s=args.time_budget)

        def progress_random_cb(cur: int, total: int, name: str) -> None:
            print(f"  [{cur:03d}/{total:03d}] Executing random test: {name} ...")

        summary = engine.run_random_batch(
            count=args.num_tests,
            seed=args.seed,
            topology=args.topology,
            progress_callback=progress_random_cb,
        )
        _print_batch_summary("VTX QA Suite - Random Batch Summary", summary)
        _export_reports(summary, args.export_html, args.export_csv, args.export_json)
        return 0

    elif args.command == "stress-test":
        tester = NatsStressTester()
        scenario = None
        if args.scenario:
            presets = get_all_scenarios()
            if args.scenario in presets:
                scenario = presets[args.scenario]()
            else:
                print(f"Error: Preset scenario '{args.scenario}' not found in presets.")
                return 1

        print(
            f"[*] Starting NATS Stress Test: Concurrency={args.concurrency}, "
            f"Requests={args.requests} ..."
        )

        def progress_stress(cur: int, total: int) -> None:
            if cur % max(1, total // 10) == 0 or cur == total:
                print(f"  -> Progress: {cur}/{total} requests completed.")

        summary = tester.run_stress_test_sync(
            server_url=args.nats_url,
            subject=args.subject,
            scenario=scenario,
            total_requests=args.requests,
            concurrency=args.concurrency,
            timeout_s=args.timeout,
            progress_callback=progress_stress,
        )

        print(f"\n{'=' * 70}")
        print("  VTX QA Suite - NATS Concurrency & Stress Test Summary")
        print(f"{'=' * 70}")
        print(f"  Total Requests Sent     : {summary.total_requests}")
        print(f"  Concurrency Level       : {summary.concurrency}")
        print(f"  Successful Requests     : {summary.success_count}")
        print(f"  Error / Rejected Count  : {summary.error_count}")
        print(f"  Timeout Count (> {args.timeout:.1f}s) : {summary.timeout_count}")
        print(f"  Throughput (RPS)        : {summary.throughput_rps:.2f} req/s")
        print(f"  Total Wall Time         : {summary.wall_time_s:.3f}s")
        print(f"{'-' * 70}")
        print("  Latency Distribution (s):")
        print(f"    p50 (Median) : {summary.latency_p50_s * 1000:.2f} ms")
        print(f"    p90          : {summary.latency_p90_s * 1000:.2f} ms")
        print(f"    p95          : {summary.latency_p95_s * 1000:.2f} ms")
        print(f"    p99          : {summary.latency_p99_s * 1000:.2f} ms")
        print(f"{'=' * 70}\n")
        return 0

    elif args.command == "serve":
        app_file = Path(__file__).parent / "app.py"
        cmd = [
            sys.executable,
            "-m",
            "streamlit",
            "run",
            str(app_file),
            "--server.port",
            str(args.port),
            "--browser.gatherUsageStats",
            "false",
        ]
        print(f"[*] Launching VTX QA Suite Web UI on port {args.port}...")
        res = subprocess.run(cmd, env=os.environ.copy())  # noqa: S603
        return res.returncode

    return 0


if __name__ == "__main__":
    sys.exit(main())
