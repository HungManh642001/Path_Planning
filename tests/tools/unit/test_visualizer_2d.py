"""Kiểm thử đơn vị cho module PlotlyVisualizer2D của VTX QA Suite."""

from __future__ import annotations

import math

import plotly.graph_objects as go

from path_planning.scenario.generator import create_scenario
from path_planning.scenario.presets import get_all_scenarios
from path_planning.validation.oracle import ValidationResult
from tools.qa_suite.core import PlotlyVisualizer2D
from tools.qa_suite.core.runner import ExecutionDriver, ExecutionMode, QAResult


def test_create_scenario_figure_without_result() -> None:
    """Kiểm thử tạo biểu đồ cho kịch bản khi không có kết quả chạy đường bay."""
    scenario = get_all_scenarios()["scenario_14_combined_obstacles"]()
    fig = PlotlyVisualizer2D.create_scenario_figure(scenario)

    assert isinstance(fig, go.Figure)
    assert len(fig.data) > 0

    # Kiểm tra cấu hình layout tỉ lệ 1:1 và hovermode
    assert fig.layout.yaxis.scaleanchor == "x"
    assert fig.layout.yaxis.scaleratio == 1
    assert fig.layout.hovermode == "closest"
    assert fig.layout.showlegend is True

    # Kiểm tra các trace cơ bản
    trace_names = [trace.name for trace in fig.data if trace.name is not None]
    assert any("Start" in name or "Takeoff" in name for name in trace_names)
    assert any("Goal" in name for name in trace_names)
    assert any("Island" in name for name in trace_names)
    assert any("Circle" in name for name in trace_names)
    assert any("Buffer" in name for name in trace_names)


def test_create_scenario_figure_with_valid_result() -> None:
    """Kiểm thử tạo biểu đồ khi có kết quả chạy đường bay thành công có góc rẽ."""
    scenario = get_all_scenarios()["scenario_06_coastal_path"]()
    driver = ExecutionDriver(mode=ExecutionMode.LOCAL)
    result = driver.run_scenario(scenario, name="scenario_06_coastal_path")

    assert result.is_success is True
    assert len(result.waypoints) >= 3

    fig = PlotlyVisualizer2D.create_scenario_figure(
        scenario, result, show_fillet_arcs=True
    )
    assert isinstance(fig, go.Figure)

    trace_names = [trace.name for trace in fig.data if trace.name is not None]
    assert any("Waypoint" in name or "Trajectory" in name for name in trace_names)
    assert any("Fillet Arc" in name for name in trace_names)

    # Kiểm tra tiêu đề hiển thị tên kịch bản và trạng thái
    assert fig.layout.title.text is not None
    assert "scenario_06_coastal_path" in fig.layout.title.text
    assert "OK" in fig.layout.title.text


def test_create_scenario_figure_with_show_fillet_arcs_false() -> None:
    """Kiểm thử tắt hiển thị fillet arcs."""
    scenario = get_all_scenarios()["scenario_06_coastal_path"]()
    driver = ExecutionDriver(mode=ExecutionMode.LOCAL)
    result = driver.run_scenario(scenario, name="scenario_06_coastal_path")

    fig = PlotlyVisualizer2D.create_scenario_figure(
        scenario, result, show_fillet_arcs=False
    )
    trace_names = [trace.name for trace in fig.data if trace.name is not None]

    assert not any("Fillet Arc" in name for name in trace_names)
    assert any("Waypoint" in name or "Trajectory" in name for name in trace_names)


def test_create_scenario_figure_with_blocked_or_empty_result() -> None:
    """Kiểm thử tạo biểu đồ khi kết quả thất bại hoặc rỗng."""
    scenario = get_all_scenarios()["scenario_01_open_ocean"]()
    empty_result = QAResult(
        scenario_name="scenario_blocked",
        status="NO_PATH",
        is_success=False,
        waypoints=[],
        path_length_m=0.0,
        wall_time_s=0.05,
        applied_time_budget_s=15.0,
        iterations=10,
        oracle_verdict=ValidationResult(False, "no path found"),
        error_detail="no path found",
        raw_response=None,
    )

    fig = PlotlyVisualizer2D.create_scenario_figure(scenario, empty_result)
    assert isinstance(fig, go.Figure)
    trace_names = [trace.name for trace in fig.data if trace.name is not None]

    assert not any("Waypoint" in name or "Trajectory" in name for name in trace_names)
    assert not any("Fillet Arc" in name for name in trace_names)
    assert fig.layout.title.text is not None
    assert "NO_PATH" in fig.layout.title.text


def test_create_scenario_figure_with_safezones() -> None:
    """Kiểm thử tạo biểu đồ cho kịch bản có định nghĩa vùng an toàn (Safezones)."""
    scenario = create_scenario(
        {
            "start": (10000, 10000),
            "start_heading": math.pi / 4,
            "goal": (90000, 90000),
            "goal_heading": math.pi / 4,
            "num_islands": 0,
            "num_dynamic_obstacles": 0,
            "safezones": [
                [
                    (5000.0, 5000.0),
                    (95000.0, 5000.0),
                    (95000.0, 95000.0),
                    (5000.0, 95000.0),
                ]
            ],
            "map_bounds": (100000.0, 100000.0),
            "seed": 42,
        }
    )

    fig = PlotlyVisualizer2D.create_scenario_figure(scenario)
    assert isinstance(fig, go.Figure)

    trace_names = [trace.name for trace in fig.data if trace.name is not None]
    assert any("Safezone" in name for name in trace_names)


def test_create_scenario_figure_with_buffer_toggle() -> None:
    """Kiểm thử cờ show_buffer bật tắt hiển thị đệm an toàn."""
    scenario = get_all_scenarios()["scenario_02_single_obstacle"]()

    fig_with_buffer = PlotlyVisualizer2D.create_scenario_figure(
        scenario, show_buffer=True, safe_margin=500.0
    )
    trace_names_with = [
        trace.name for trace in fig_with_buffer.data if trace.name is not None
    ]
    assert any("Buffer" in name for name in trace_names_with)

    fig_without_buffer = PlotlyVisualizer2D.create_scenario_figure(
        scenario, show_buffer=False
    )
    trace_names_without = [
        trace.name for trace in fig_without_buffer.data if trace.name is not None
    ]
    assert not any("Buffer" in name for name in trace_names_without)
