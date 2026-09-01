"""Kiểm thử đơn vị cho các view và entrypoint Streamlit App của QA Suite."""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock, patch

from path_planning.scenario.presets import get_all_scenarios
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


class FakeSessionState(dict[str, Any]):
    """Giả lập an toàn cho st.session_state trong unit test."""

    pass


def _mock_columns(spec: Any, **kwargs: Any) -> list[MagicMock]:
    """Tạo danh sách Mock columns tương ứng với spec truyền vào."""
    import streamlit as st

    count = spec if isinstance(spec, int) else len(spec)
    cols: list[MagicMock] = []
    for _ in range(count):
        col = MagicMock()
        col.button.side_effect = lambda *a, **kw: st.button(*a, **kw)
        cols.append(col)
    return cols


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
    fake_state = FakeSessionState()
    with (
        patch("streamlit.session_state", fake_state),
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


def test_render_tab_inspector_interactive_studio_circle_mode() -> None:
    """Kiểm thử render Tab 1 ở chế độ Interactive Studio: thêm Circle Obstacle."""
    fake_state = FakeSessionState(
        {
            "active_scenario": get_all_scenarios()["scenario_01_open_ocean"](),
            "studio_mode": "⭕ Add Circle Obstacle",
            "circle_radius": 20000.0,
            "draft_polygon_vertices": [],
            "last_clicked_point": None,
        }
    )

    def mock_radio(label: str, *args: Any, **kwargs: Any) -> str:
        if "Scenario Source" in label:
            return "🎨 Interactive Studio (GUI Drawing)"
        if "Studio Mode" in label:
            return "⭕ Add Circle Obstacle"
        return "Local Python Core"

    mock_selection = {"points": [{"x": 150000.0, "y": 250000.0}]}

    with (
        patch("streamlit.session_state", fake_state),
        patch("streamlit.radio", side_effect=mock_radio),
        patch("streamlit.selectbox", return_value="scenario_01_open_ocean"),
        patch("streamlit.number_input", return_value=20000.0),
        patch("streamlit.checkbox", return_value=True),
        patch("streamlit.button", return_value=False),
        patch("streamlit.columns", side_effect=_mock_columns),
        patch("streamlit.expander", return_value=MagicMock()),
        patch("streamlit.plotly_chart", return_value=mock_selection),
        patch("streamlit.rerun") as mock_rerun,
    ):
        render_tab_inspector()
        assert mock_rerun.called
        active = fake_state["active_scenario"]
        assert len(active["dynamic_obstacles"]) >= 1
        assert active["dynamic_obstacles"][-1][0] == (150000.0, 250000.0)


def test_render_tab_inspector_studio_polygon_mode() -> None:
    """Kiểm thử chế độ vẽ đa giác (Add Polygon) và click thêm đỉnh."""
    fake_state = FakeSessionState(
        {
            "active_scenario": get_all_scenarios()["scenario_01_open_ocean"](),
            "studio_mode": "📐 Add Polygon (Click Vertices)",
            "draft_polygon_vertices": [(100000.0, 100000.0)],
            "last_clicked_point": None,
        }
    )

    def mock_radio(label: str, *args: Any, **kwargs: Any) -> str:
        if "Scenario Source" in label:
            return "🎨 Interactive Studio (GUI Drawing)"
        if "Studio Mode" in label:
            return "📐 Add Polygon (Click Vertices)"
        return "Local Python Core"

    mock_selection = {"points": [{"x": 200000.0, "y": 200000.0}]}

    with (
        patch("streamlit.session_state", fake_state),
        patch("streamlit.radio", side_effect=mock_radio),
        patch("streamlit.selectbox", return_value="scenario_01_open_ocean"),
        patch("streamlit.number_input", return_value=20000.0),
        patch("streamlit.checkbox", return_value=True),
        patch("streamlit.button", return_value=False),
        patch("streamlit.columns", side_effect=_mock_columns),
        patch("streamlit.expander", return_value=MagicMock()),
        patch("streamlit.plotly_chart", return_value=mock_selection),
        patch("streamlit.rerun") as mock_rerun,
    ):
        render_tab_inspector()
        assert mock_rerun.called
        assert (200000.0, 200000.0) in fake_state["draft_polygon_vertices"]
        assert len(fake_state["draft_polygon_vertices"]) == 2


def test_render_tab_inspector_studio_move_start_and_goal() -> None:
    """Kiểm thử thao tác di chuyển Start và Goal qua click bản đồ."""
    fake_state = FakeSessionState(
        {
            "active_scenario": get_all_scenarios()["scenario_01_open_ocean"](),
            "studio_mode": "🚀 Move Start (O)",
            "last_clicked_point": None,
        }
    )

    def mock_radio_start(label: str, *args: Any, **kwargs: Any) -> str:
        if "Scenario Source" in label:
            return "🎨 Interactive Studio (GUI Drawing)"
        if "Studio Mode" in label:
            return "🚀 Move Start (O)"
        return "Local Python Core"

    mock_selection_start = {"points": [{"x": 80000.0, "y": 90000.0}]}

    with (
        patch("streamlit.session_state", fake_state),
        patch("streamlit.radio", side_effect=mock_radio_start),
        patch("streamlit.selectbox", return_value="scenario_01_open_ocean"),
        patch("streamlit.number_input", return_value=45.0),
        patch("streamlit.checkbox", return_value=True),
        patch("streamlit.button", return_value=False),
        patch("streamlit.columns", side_effect=_mock_columns),
        patch("streamlit.expander", return_value=MagicMock()),
        patch("streamlit.plotly_chart", return_value=mock_selection_start),
        patch("streamlit.rerun") as mock_rerun,
    ):
        render_tab_inspector()
        assert mock_rerun.called
        assert fake_state["active_scenario"]["start"] == (80000.0, 90000.0)

    # Test Move Goal
    fake_state["studio_mode"] = "🎯 Move Goal (T)"
    fake_state["last_clicked_point"] = None

    def mock_radio_goal(label: str, *args: Any, **kwargs: Any) -> str:
        if "Scenario Source" in label:
            return "🎨 Interactive Studio (GUI Drawing)"
        if "Studio Mode" in label:
            return "🎯 Move Goal (T)"
        return "Local Python Core"

    mock_selection_goal = {"points": [{"x": 400000.0, "y": 420000.0}]}

    with (
        patch("streamlit.session_state", fake_state),
        patch("streamlit.radio", side_effect=mock_radio_goal),
        patch("streamlit.selectbox", return_value="scenario_01_open_ocean"),
        patch("streamlit.number_input", return_value=45.0),
        patch("streamlit.checkbox", return_value=True),
        patch("streamlit.button", return_value=False),
        patch("streamlit.columns", side_effect=_mock_columns),
        patch("streamlit.expander", return_value=MagicMock()),
        patch("streamlit.plotly_chart", return_value=mock_selection_goal),
        patch("streamlit.rerun") as mock_rerun2,
    ):
        render_tab_inspector()
        assert mock_rerun2.called
        assert fake_state["active_scenario"]["goal"] == (400000.0, 420000.0)


def test_render_tab_inspector_studio_quick_actions() -> None:
    """Kiểm thử các nút thao tác nhanh (Undo, Clear, Reset, Complete Polygon)."""
    fake_state = FakeSessionState(
        {
            "active_scenario": get_all_scenarios()["scenario_01_open_ocean"](),
            "draft_polygon_vertices": [
                (100000.0, 100000.0),
                (200000.0, 100000.0),
                (200000.0, 200000.0),
            ],
            "studio_mode": "📐 Add Polygon (Click Vertices)",
        }
    )

    def mock_radio(label: str, *args: Any, **kwargs: Any) -> str:
        if "Scenario Source" in label:
            return "🎨 Interactive Studio (GUI Drawing)"
        if "Studio Mode" in label:
            return "📐 Add Polygon (Click Vertices)"
        return "Local Python Core"

    def mock_button_complete(label: str, *args: Any, **kwargs: Any) -> bool:
        return "Complete Polygon" in label

    with (
        patch("streamlit.session_state", fake_state),
        patch("streamlit.radio", side_effect=mock_radio),
        patch("streamlit.selectbox", return_value="scenario_01_open_ocean"),
        patch("streamlit.number_input", return_value=20000.0),
        patch("streamlit.checkbox", return_value=True),
        patch("streamlit.button", side_effect=mock_button_complete),
        patch("streamlit.columns", side_effect=_mock_columns),
        patch("streamlit.expander", return_value=MagicMock()),
        patch("streamlit.plotly_chart", return_value=None),
        patch("streamlit.rerun") as mock_rerun,
    ):
        render_tab_inspector()
        assert mock_rerun.called
        assert len(fake_state["active_scenario"]["islands"]) >= 1
        assert fake_state["draft_polygon_vertices"] == []

    # Test Clear
    def mock_button_clear(label: str, *args: Any, **kwargs: Any) -> bool:
        return "Clear All Obstacles" in label

    with (
        patch("streamlit.session_state", fake_state),
        patch("streamlit.radio", side_effect=mock_radio),
        patch("streamlit.selectbox", return_value="scenario_01_open_ocean"),
        patch("streamlit.number_input", return_value=20000.0),
        patch("streamlit.checkbox", return_value=True),
        patch("streamlit.button", side_effect=mock_button_clear),
        patch("streamlit.columns", side_effect=_mock_columns),
        patch("streamlit.expander", return_value=MagicMock()),
        patch("streamlit.plotly_chart", return_value=None),
        patch("streamlit.rerun") as mock_rerun3,
    ):
        render_tab_inspector()
        assert mock_rerun3.called
        assert len(fake_state["active_scenario"]["obstacles"]) == 0


def test_parse_svg_path() -> None:
    """Kiểm thử hàm parse SVG path thành tọa độ đỉnh đa giác."""
    from tools.qa_suite.views.tab_inspector import parse_svg_path

    path_svg = "M 100000,100000 L 200000,100000 L 200000,200000 Z"
    pts = parse_svg_path(path_svg)
    assert len(pts) == 3
    assert pts[0] == (100000.0, 100000.0)
    assert pts[1] == (200000.0, 100000.0)
    assert pts[2] == (200000.0, 200000.0)


def test_render_tab_inspector_studio_shapes_handling() -> None:
    """Kiểm thử bắt sự kiện kéo thả vùng chọn trên bản đồ (box selection)."""
    fake_state = FakeSessionState(
        {
            "active_scenario": get_all_scenarios()["scenario_01_open_ocean"](),
            "studio_mode": "⭕ Add Circle Obstacle",
            "last_studio_interaction": None,
        }
    )

    def mock_radio(label: str, *args: Any, **kwargs: Any) -> str:
        if "Scenario Source" in label:
            return "🎨 Interactive Studio (GUI Drawing)"
        if "Studio Mode" in label:
            return "⭕ Add Circle Obstacle"
        return "Local Python Core"

    mock_selection_box = {
        "box": [
            {
                "x": [100000.0, 150000.0],
                "y": [100000.0, 150000.0],
            }
        ]
    }

    with (
        patch("streamlit.session_state", fake_state),
        patch("streamlit.radio", side_effect=mock_radio),
        patch("streamlit.selectbox", return_value="scenario_01_open_ocean"),
        patch("streamlit.number_input", return_value=20000.0),
        patch("streamlit.checkbox", return_value=True),
        patch("streamlit.button", return_value=False),
        patch("streamlit.columns", side_effect=_mock_columns),
        patch("streamlit.expander", return_value=MagicMock()),
        patch("streamlit.plotly_chart", return_value=mock_selection_box),
        patch("streamlit.rerun") as mock_rerun,
    ):
        render_tab_inspector()
        assert mock_rerun.called
        active = fake_state["active_scenario"]
        assert len(active["dynamic_obstacles"]) >= 1
        assert active["dynamic_obstacles"][-1][0] == (125000.0, 125000.0)
        assert active["dynamic_obstacles"][-1][1] == 25000.0


def test_render_tab_inspector_dual_input_circle() -> None:
    """Kiểm thử thêm circle bằng nút Add Circle at (X, Y)."""
    fake_state = FakeSessionState(
        {
            "active_scenario": get_all_scenarios()["scenario_01_open_ocean"](),
            "studio_mode": "⭕ Add Circle Obstacle",
            "circle_input_x": 50000.0,
            "circle_input_y": 60000.0,
            "circle_radius": 15000.0,
        }
    )

    def mock_radio(label: str, *args: Any, **kwargs: Any) -> str:
        if "Scenario Source" in label:
            return "🎨 Interactive Studio (GUI Drawing)"
        if "Studio Mode" in label:
            return "⭕ Add Circle Obstacle"
        return "Local Python Core"

    def mock_button(label: str, *args: Any, **kwargs: Any) -> bool:
        return "Add Circle at (X, Y)" in label

    with (
        patch("streamlit.session_state", fake_state),
        patch("streamlit.radio", side_effect=mock_radio),
        patch("streamlit.selectbox", return_value="scenario_01_open_ocean"),
        patch("streamlit.number_input", return_value=50000.0),
        patch("streamlit.checkbox", return_value=True),
        patch("streamlit.button", side_effect=mock_button),
        patch("streamlit.columns", side_effect=_mock_columns),
        patch("streamlit.expander", return_value=MagicMock()),
        patch("streamlit.plotly_chart", return_value=None),
        patch("streamlit.rerun") as mock_rerun,
    ):
        render_tab_inspector()
        assert mock_rerun.called
        active = fake_state["active_scenario"]
        assert len(active["dynamic_obstacles"]) >= 1


def test_render_tab_inspector_delete_obstacle_by_index() -> None:
    """Kiểm thử xóa vật cản cụ thể từ bảng Active Obstacles Manager."""
    scen = get_all_scenarios()["scenario_02_single_obstacle"]()
    fake_state = FakeSessionState(
        {
            "active_scenario": scen,
            "studio_mode": "🔍 Pan / Inspect",
        }
    )

    def mock_radio(label: str, *args: Any, **kwargs: Any) -> str:
        if "Scenario Source" in label:
            return "🎨 Interactive Studio (GUI Drawing)"
        return "Local Python Core"

    # Click delete on the first obstacle (del_obs_0)
    def mock_button(label: str, *args: Any, **kwargs: Any) -> bool:
        key = kwargs.get("key", "")
        return str(key) == "del_obs_0" or "❌" in label

    with (
        patch("streamlit.session_state", fake_state),
        patch("streamlit.radio", side_effect=mock_radio),
        patch("streamlit.selectbox", return_value="scenario_02_single_obstacle"),
        patch("streamlit.number_input", return_value=20000.0),
        patch("streamlit.checkbox", return_value=True),
        patch("streamlit.button", side_effect=mock_button),
        patch("streamlit.columns", side_effect=_mock_columns),
        patch("streamlit.expander", return_value=MagicMock()),
        patch("streamlit.plotly_chart", return_value=None),
        patch("streamlit.rerun") as mock_rerun,
    ):
        render_tab_inspector()
        assert mock_rerun.called
        active = fake_state["active_scenario"]
        assert len(active["obstacles"]) < len(scen["obstacles"])


def test_render_tab_inspector_dual_input_polygon() -> None:
    """Kiểm thử thêm đỉnh và hoàn thành polygon bằng nút bấm thủ công."""
    fake_state = FakeSessionState(
        {
            "active_scenario": get_all_scenarios()["scenario_01_open_ocean"](),
            "studio_mode": "📐 Add Polygon (Click Vertices)",
            "draft_polygon_vertices": [
                (100000.0, 100000.0),
                (200000.0, 100000.0),
            ],
            "last_selected_base_preset": "scenario_01_open_ocean",
        }
    )

    def mock_radio(label: str, *args: Any, **kwargs: Any) -> str:
        if "Scenario Source" in label:
            return "🎨 Interactive Studio (GUI Drawing)"
        if "Studio Mode" in label:
            return "📐 Add Polygon (Click Vertices)"
        return "Local Python Core"

    def mock_button_add_vertex(label: str, *args: Any, **kwargs: Any) -> bool:
        return "Add Vertex" in label

    with (
        patch("streamlit.session_state", fake_state),
        patch("streamlit.radio", side_effect=mock_radio),
        patch("streamlit.selectbox", return_value="scenario_01_open_ocean"),
        patch("streamlit.number_input", return_value=50000.0),
        patch("streamlit.checkbox", return_value=True),
        patch("streamlit.button", side_effect=mock_button_add_vertex),
        patch("streamlit.columns", side_effect=_mock_columns),
        patch("streamlit.expander", return_value=MagicMock()),
        patch("streamlit.plotly_chart", return_value=None),
        patch("streamlit.rerun") as mock_rerun,
    ):
        render_tab_inspector()
        assert mock_rerun.called
        assert len(fake_state["draft_polygon_vertices"]) == 3


def test_render_tab_inspector_modebar_config() -> None:
    """Kiểm thử cấu hình ModeBar Plotly xóa các công cụ vẽ client-side thừa."""
    fake_state = FakeSessionState(
        {
            "active_scenario": get_all_scenarios()["scenario_01_open_ocean"](),
            "studio_mode": "🔍 Pan / Inspect",
        }
    )

    captured_config: dict[str, Any] = {}

    def mock_plotly_chart(fig: Any, *args: Any, **kwargs: Any) -> Any:
        nonlocal captured_config
        captured_config = kwargs.get("config", {})
        return None

    with (
        patch("streamlit.session_state", fake_state),
        patch("streamlit.radio", return_value="Preset Scenarios (18 Cases)"),
        patch("streamlit.selectbox", return_value="scenario_01_open_ocean"),
        patch("streamlit.number_input", return_value=20000.0),
        patch("streamlit.checkbox", return_value=True),
        patch("streamlit.button", return_value=False),
        patch("streamlit.columns", side_effect=_mock_columns),
        patch("streamlit.expander", return_value=MagicMock()),
        patch("streamlit.plotly_chart", side_effect=mock_plotly_chart),
        patch("streamlit.rerun"),
    ):
        render_tab_inspector()
        add_buttons = captured_config.get("modeBarButtonsToAdd", [])
        assert "select2d" in add_buttons
        assert "lasso2d" in add_buttons
        assert "eraseshape" in add_buttons
        remove_buttons = captured_config.get("modeBarButtonsToRemove", [])
        assert "drawline" in remove_buttons
        assert "drawcircle" in remove_buttons
        assert "drawclosedpath" in remove_buttons
