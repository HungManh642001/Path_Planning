"""Giao diện Tab 2: Batch Regression & Benchmark Runner (Streamlit).

Cho phép thực thi kiểm thử hàng loạt trên toàn bộ 18 kịch bản chuẩn (Presets)
hoặc sinh ngẫu nhiên N kịch bản (Random Batch), tổng hợp thống kê hiệu năng,
cung cấp chức năng xem lại 1-click (One-Click Reproduce) và xuất báo cáo đa định dạng.
"""

from __future__ import annotations

import tempfile
from pathlib import Path
from typing import cast

import streamlit as st

from path_planning.types import Topology
from service.vtx_service.transport import DEFAULT_NATS_SERVER, DEFAULT_SUBJECT
from tools.qa_suite.core.batch_runner import BatchRegressionEngine, BatchSummary
from tools.qa_suite.core.report_generator import ReportGenerator
from tools.qa_suite.core.runner import ExecutionDriver, ExecutionMode


def render_tab_batch() -> None:
    """Hiển thị toàn bộ giao diện Batch Regression & Benchmark Runner."""
    st.subheader("📊 Batch Regression & Benchmark Runner")

    col_setup, col_exec = st.columns([1, 1], gap="medium")

    with col_setup:
        st.markdown("#### ⚙️ Batch Suite Configuration")
        batch_type = st.radio(
            "Batch Type",
            options=["18 Preset Scenarios", "Random Generated Batch"],
            index=0,
            horizontal=True,
        )

        if batch_type == "Random Generated Batch":
            num_tests_input = st.slider(
                "Number of Scenarios (N)",
                min_value=5,
                max_value=100,
                value=15,
                step=5,
            )
            seed_input = st.number_input(
                "Random Seed", min_value=1, max_value=999999, value=42, step=1
            )
            topology_str = st.selectbox(
                "Obstacle Topology",
                options=["random", "center_cluster", "wall_block"],
                index=0,
            )
            num_tests = int(num_tests_input)
            seed = int(seed_input)
            topology = cast(Topology, topology_str)
        else:
            num_tests = 18
            seed = 42
            topology = "random"

    with col_exec:
        st.markdown("#### 🎯 Execution Target")
        exec_mode_str = st.radio(
            "Execution Target Mode",
            options=["Local Python Core", "NATS Microservice"],
            index=0,
            horizontal=True,
        )
        is_nats = exec_mode_str == "NATS Microservice"

        nats_url = DEFAULT_NATS_SERVER
        nats_subject = DEFAULT_SUBJECT
        if is_nats:
            nats_url = st.text_input(
                "NATS Server URL", value=DEFAULT_NATS_SERVER, key="batch_nats_url"
            )
            nats_subject = st.text_input(
                "NATS Subject", value=DEFAULT_SUBJECT, key="batch_nats_subj"
            )

        time_budget = float(
            st.number_input(
                "Time Budget per Scenario (s)",
                min_value=1.0,
                max_value=60.0,
                value=15.0,
                step=1.0,
                key="batch_time_budget",
            )
        )

    run_batch_clicked = st.button(
        "▶️ Run Batch Regression", type="primary", use_container_width=True
    )

    if run_batch_clicked:
        mode = ExecutionMode.NATS if is_nats else ExecutionMode.LOCAL
        driver = ExecutionDriver(mode=mode, nats_url=nats_url, subject=nats_subject)
        engine = BatchRegressionEngine(driver=driver, time_budget_s=time_budget)

        prog_bar = st.progress(0.0)
        status_text = st.empty()

        def progress_cb(current: int, total: int, name: str) -> None:
            pct = current / max(1, total)
            prog_bar.progress(pct)
            status_text.text(f"Running [{current}/{total}]: {name} ...")

        with st.spinner("Executing batch regression suite..."):
            if batch_type == "18 Preset Scenarios":
                summary = engine.run_presets(progress_callback=progress_cb)
            else:
                summary = engine.run_random_batch(
                    count=num_tests,
                    seed=seed,
                    topology=topology,
                    progress_callback=progress_cb,
                )

        prog_bar.progress(1.0)
        status_text.text("Batch execution completed!")
        st.session_state["batch_summary"] = summary

    # Hiển thị kết quả tổng hợp nếu có
    summary = cast(BatchSummary | None, st.session_state.get("batch_summary"))

    if summary is not None:
        st.divider()
        st.markdown("### 📈 Benchmark Results Summary")

        # 5 Thẻ chỉ số KPI
        kpi1, kpi2, kpi3, kpi4, kpi5 = st.columns(5)
        kpi1.metric("Total Tests", summary.total_tests)

        rate_delta = f"{summary.success_count}/{summary.total_tests} passed"
        kpi2.metric("Success Rate", f"{summary.success_rate:.1f}%", delta=rate_delta)

        violation_color = "normal" if summary.oracle_violation_count == 0 else "inverse"
        viol_delta = (
            None if summary.oracle_violation_count == 0 else "Violations detected"
        )
        kpi3.metric(
            "Oracle Violations",
            summary.oracle_violation_count,
            delta=viol_delta,
            delta_color=violation_color,
        )

        mean_wall = summary.wall_time_stats.get("mean", 0.0)
        p95_wall = summary.wall_time_stats.get("p95", 0.0)
        kpi4.metric(
            "Mean Wall Time", f"{mean_wall:.4f} s", delta=f"P95: {p95_wall:.4f} s"
        )

        mean_len = summary.path_length_stats.get("mean", 0.0)
        kpi5.metric("Mean Path Length", f"{mean_len:,.1f} m")

        # Bảng chi tiết từng ca kiểm thử
        st.markdown("#### 📋 Detailed Test Scenarios")
        table_rows: list[dict[str, object]] = []
        for idx, r in enumerate(summary.results, start=1):
            table_rows.append(
                {
                    "#": idx,
                    "Scenario": r.scenario_name,
                    "Status": r.status,
                    "Success": "PASSED" if r.is_success else "FAILED",
                    "Wall Time (s)": f"{r.wall_time_s:.4f}",
                    "Path Length (m)": f"{r.path_length_m:,.1f}",
                    "Iterations": r.iterations,
                    "Oracle Valid": "VALID" if r.oracle_verdict.is_ok else "VIOLATION",
                    "Details": r.error_detail or "-",
                }
            )
        st.dataframe(table_rows, use_container_width=True)

        # One-Click Reproduce to Tab 1
        st.markdown("#### 🔍 One-Click Reproduce & Inspect")
        col_select, col_btn = st.columns([3, 1])
        with col_select:
            scenario_names = [r.scenario_name for r in summary.results]
            selected_scenario = st.selectbox(
                "Select scenario to inspect in Tab 1 (Visual Inspector):",
                options=scenario_names,
            )
        with col_btn:
            st.write("")
            st.write("")
            if st.button("🔎 Inspect Scenario in Tab 1", use_container_width=True):
                st.session_state["selected_scenario_name"] = selected_scenario
                msg = (
                    f"Selected '{selected_scenario}'! Please switch to "
                    "**'🔍 Visual Scenario Inspector'** tab."
                )
                st.info(msg)

        # Export Báo Cáo
        st.markdown("#### 📥 Export Reports")
        col_html, col_csv, col_json = st.columns(3)

        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp_path = Path(tmp_dir)

            # Tạo file tạm thời để đọc nội dung cho nút download
            html_file = tmp_path / "report.html"
            csv_file = tmp_path / "report.csv"
            json_file = tmp_path / "report.json"

            ReportGenerator.export_html(summary, html_file)
            ReportGenerator.export_csv(summary, csv_file)
            ReportGenerator.export_json(summary, json_file)

            html_data = html_file.read_text(encoding="utf-8")
            csv_data = csv_file.read_text(encoding="utf-8")
            json_data = json_file.read_text(encoding="utf-8")

        with col_html:
            st.download_button(
                label="📄 Download HTML Report",
                data=html_data,
                file_name="vtx_qa_batch_report.html",
                mime="text/html",
                use_container_width=True,
            )
        with col_csv:
            st.download_button(
                label="📊 Download CSV Table",
                data=csv_data,
                file_name="vtx_qa_batch_report.csv",
                mime="text/csv",
                use_container_width=True,
            )
        with col_json:
            st.download_button(
                label="📦 Download JSON Data",
                data=json_data,
                file_name="vtx_qa_batch_report.json",
                mime="application/json",
                use_container_width=True,
            )
