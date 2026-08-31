"""Giao diện Tab 3: NATS Concurrency & Stress Tester (Streamlit).

Cho phép đo tải đồng thời (stress & load profiling) trên NATS Path Planning
Microservice, theo dõi thông lượng (RPS), phân bố độ trễ (latency percentiles)
và biểu đồ phân phối Histogram.
"""

from __future__ import annotations

from typing import cast

import plotly.graph_objects as go
import streamlit as st

from path_planning.scenario.presets import get_all_scenarios
from service.vtx_service.transport import DEFAULT_NATS_SERVER, DEFAULT_SUBJECT
from tools.qa_suite.core.stress_tester import (
    NatsStressTester,
    StressTestSummary,
)


def _create_latency_histogram(summary: StressTestSummary) -> go.Figure:
    """Tạo biểu đồ Plotly Histogram phân bố độ trễ kèm các mốc phân vị."""
    fig = go.Figure()

    latencies_ms = [lat * 1000.0 for lat in summary.latencies]

    fig.add_trace(
        go.Histogram(
            x=latencies_ms,
            nbinsx=30,
            marker={
                "color": "rgba(59, 130, 246, 0.75)",
                "line": {"color": "rgb(37, 99, 235)", "width": 1.5},
            },
            name="Request Latencies",
            hovertemplate="Latency: %{x:.1f} ms<br>Count: %{y}<extra></extra>",
        )
    )

    # Thêm các đường chỉ thị phân vị
    p50_ms = summary.latency_p50_s * 1000.0
    p90_ms = summary.latency_p90_s * 1000.0
    p95_ms = summary.latency_p95_s * 1000.0
    p99_ms = summary.latency_p99_s * 1000.0

    percentiles = [
        ("P50", p50_ms, "green"),
        ("P90", p90_ms, "orange"),
        ("P95", p95_ms, "red"),
        ("P99", p99_ms, "purple"),
    ]

    for label, val_ms, color in percentiles:
        fig.add_vline(
            x=val_ms,
            line_width=2,
            line_dash="dash",
            line_color=color,
            annotation_text=f"{label}: {val_ms:.1f}ms",
            annotation_position="top right",
            annotation_font={"size": 11, "color": color},
        )

    fig.update_layout(
        title={
            "text": "NATS Microservice Latency Distribution (ms)",
            "font": {"size": 15},
        },
        xaxis={"title": "Response Time (ms)", "showgrid": True, "zeroline": False},
        yaxis={"title": "Request Count", "showgrid": True, "zeroline": False},
        template="plotly_white",
        margin={"l": 40, "r": 40, "t": 60, "b": 40},
        bargap=0.05,
    )

    return fig


def render_tab_stress() -> None:
    """Hiển thị toàn bộ giao diện NATS Concurrency & Stress Tester."""
    st.subheader("⚡ NATS Concurrency & Stress Tester")

    col_cfg, col_load = st.columns([1, 1], gap="medium")

    presets = get_all_scenarios()
    preset_names = list(presets.keys())
    default_scenario_idx = (
        preset_names.index("scenario_01_open_ocean")
        if "scenario_01_open_ocean" in preset_names
        else 0
    )

    with col_cfg:
        st.markdown("#### 🌐 NATS Server & Subject")
        nats_url = st.text_input(
            "NATS Server URL", value=DEFAULT_NATS_SERVER, key="stress_nats_url"
        )
        nats_subject = st.text_input(
            "NATS Subject", value=DEFAULT_SUBJECT, key="stress_nats_subj"
        )

        selected_scenario_name = str(
            st.selectbox(
                "Payload Scenario",
                options=preset_names,
                index=default_scenario_idx,
                help="Kịch bản được mã hóa làm payload gửi đo tải",
            )
        )

    with col_load:
        st.markdown("#### 🏎️ Concurrency & Load Parameters")
        concurrency = int(
            st.slider(
                "Concurrency Level (In-flight clients)",
                min_value=1,
                max_value=50,
                value=5,
                step=1,
            )
        )
        total_requests = int(
            st.slider(
                "Total Requests to Send",
                min_value=10,
                max_value=500,
                value=50,
                step=10,
            )
        )
        timeout_s = float(
            st.number_input(
                "Request Timeout (s)",
                min_value=1.0,
                max_value=30.0,
                value=6.0,
                step=0.5,
            )
        )

    run_stress_clicked = st.button(
        "⚡ Start Stress Test", type="primary", use_container_width=True
    )

    if run_stress_clicked:
        scenario = presets[selected_scenario_name]()
        tester = NatsStressTester()

        prog_bar = st.progress(0.0)
        status_text = st.empty()

        def progress_cb(completed: int, total: int) -> None:
            pct = completed / max(1, total)
            prog_bar.progress(pct)
            status_text.text(f"Sent & Received [{completed}/{total}] requests...")

        with st.spinner("Running NATS concurrency stress test..."):
            try:
                summary = tester.run_stress_test_sync(
                    server_url=nats_url,
                    subject=nats_subject,
                    scenario=scenario,
                    total_requests=total_requests,
                    concurrency=concurrency,
                    timeout_s=timeout_s,
                    progress_callback=progress_cb,
                )
                prog_bar.progress(1.0)
                status_text.text("Stress test completed successfully!")
                st.session_state["stress_summary"] = summary
            except Exception as exc:
                st.error(f"Stress test encountered an error: {exc}")

    # Hiển thị kết quả tổng hợp đo tải
    summary = cast(StressTestSummary | None, st.session_state.get("stress_summary"))

    if summary is not None:
        st.divider()
        st.markdown("### 📊 Performance & Latency Breakdown")

        # Row 1: Metrics KPIs
        c1, c2, c3, c4 = st.columns(4)
        c1.metric("Total Requests", summary.total_requests)

        success_rate = (
            summary.success_count / summary.total_requests * 100.0
            if summary.total_requests > 0
            else 0.0
        )
        delta_txt = (
            f"{summary.success_count} OK / "
            f"{summary.error_count} Err / "
            f"{summary.timeout_count} Timeout"
        )
        c2.metric(
            "Success Rate",
            f"{success_rate:.1f}%",
            delta=delta_txt,
        )

        c3.metric(
            "Throughput (RPS)",
            f"{summary.throughput_rps:.1f} req/s",
            delta=f"Total Wall: {summary.wall_time_s:.2f} s",
        )

        c4.metric(
            "P50 Latency (Median)",
            f"{summary.latency_p50_s * 1000:.1f} ms",
            delta=f"P95: {summary.latency_p95_s * 1000:.1f} ms",
        )

        # Row 2: Percentile details
        p1, p2, p3, p4 = st.columns(4)
        p1.info(f"**P50 (Median):** `{summary.latency_p50_s * 1000:.2f} ms`")
        p2.info(f"**P90:** `{summary.latency_p90_s * 1000:.2f} ms`")
        p3.info(f"**P95:** `{summary.latency_p95_s * 1000:.2f} ms`")
        p4.info(f"**P99:** `{summary.latency_p99_s * 1000:.2f} ms`")

        # Plotly Histogram Latency Distribution
        if summary.latencies:
            fig = _create_latency_histogram(summary)
            st.plotly_chart(fig, use_container_width=True)
