"""Kiểm thử đơn vị cho các view và entrypoint Streamlit App của QA Suite."""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock, patch

from tools.qa_suite.app import main as app_main
from tools.qa_suite.core.stress_tester import StressTestSummary
from tools.qa_suite.views.tab_batch import render_tab_batch
from tools.qa_suite.views.tab_inspector import (
    _compute_waypoint_table_data,
    render_tab_inspector,
)
from tools.qa_suite.views.tab_stress import (
    _create_latency_histogram,
    render_tab_stress,
)


def _mock_columns(spec: Any, **kwargs: Any) -> list[MagicMock]:
    """Tạo danh sách Mock columns tương ứng với spec truyền vào."""
    count = spec if isinstance(spec, int) else len(spec)
    return [MagicMock() for _ in range(count)]


def test_compute_waypoint_table_data() -> None:
    """Kiểm thử hàm tính toán bảng hiển thị waypoint."""
    waypoints = [
        ((0.0, 0.0), 0.0),
        ((1000.0, 0.0), 0.0),
        ((1000.0, 1000.0), 1.5707963267948966),
    ]
    data = _compute_waypoint_table_data(waypoints)
    assert len(data) == 3
    assert data[0]["WP"] == "W_0"
    assert data[0]["Leg Length (m)"] == "1,000.0"
    assert data[1]["WP"] == "W_1"
    assert "90.0°" in str(data[1]["Turn Angle (deg)"])
    assert data[2]["WP"] == "W_2"
    assert data[2]["Leg Length (m)"] == "-"


def test_create_latency_histogram() -> None:
    """Kiểm thử tạo histogram độ trễ Plotly từ StressTestSummary."""
    summary = StressTestSummary(
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
        latencies=[0.01, 0.012, 0.015, 0.02],
    )
    fig = _create_latency_histogram(summary)
    assert len(fig.data) == 1
    assert (
        fig.layout.title.text  # type: ignore[union-attr]
        == "NATS Microservice Latency Distribution (ms)"
    )


def test_render_views_smoke() -> None:
    """Smoke test gọi render các view Streamlit với mock streamlit."""
    with (
        patch("streamlit.selectbox", return_value="scenario_01_open_ocean"),
        patch("streamlit.radio", return_value="Local Python Core"),
        patch("streamlit.number_input", return_value=100.0),
        patch("streamlit.slider", return_value=10),
        patch("streamlit.checkbox", return_value=True),
        patch("streamlit.button", return_value=False),
        patch("streamlit.columns", side_effect=_mock_columns),
        patch("streamlit.expander", return_value=MagicMock()),
    ):
        render_tab_inspector()
        render_tab_batch()
        render_tab_stress()


def test_app_main_smoke() -> None:
    """Smoke test gọi app main với mock streamlit tabs."""
    mock_tab = MagicMock()
    with (
        patch("streamlit.set_page_config"),
        patch("streamlit.title"),
        patch("streamlit.caption"),
        patch("streamlit.tabs", return_value=(mock_tab, mock_tab, mock_tab)),
        patch("tools.qa_suite.app.render_tab_inspector"),
        patch("tools.qa_suite.app.render_tab_batch"),
        patch("tools.qa_suite.app.render_tab_stress"),
    ):
        app_main()
