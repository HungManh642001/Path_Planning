"""Giao diện Tab 1: Visual Scenario Inspector & Single-Test Runner (Streamlit).

Cho phép trực quan hóa bản đồ 2D nhiệm vụ, tùy chỉnh các tham số giới hạn động học,
thực thi thuật toán lập lịch (Local hoặc NATS) và phân tích thẩm định Validation Oracle.
"""

from __future__ import annotations

import math
from typing import cast

import streamlit as st

from path_planning import config
from path_planning.geometry import spatial
from path_planning.scenario.presets import get_all_scenarios
from path_planning.types import Scenario
from service.vtx_service.transport import DEFAULT_NATS_SERVER, DEFAULT_SUBJECT
from tools.qa_suite.core.runner import ExecutionDriver, ExecutionMode, QAResult
from tools.qa_suite.core.visualizer_2d import PlotlyVisualizer2D


def _compute_waypoint_table_data(
    waypoints: list[tuple[tuple[float, float], float]],
) -> list[dict[str, object]]:
    """Tính toán bảng chi tiết tọa độ, hướng và góc rẽ cho từng waypoint."""
    table_data: list[dict[str, object]] = []
    n = len(waypoints)

    for i in range(n):
        pos = waypoints[i][0]
        heading_rad = waypoints[i][1]
        heading_deg = math.degrees(heading_rad)

        # Đoạn thẳng tới điểm tiếp theo
        if i < n - 1:
            next_pos = waypoints[i + 1][0]
            leg_len = math.hypot(next_pos[0] - pos[0], next_pos[1] - pos[1])
            leg_str = f"{leg_len:,.1f}"
        else:
            leg_str = "-"

        # Góc rẽ tại waypoint nội bộ
        if 0 < i < n - 1:
            prev_pos = waypoints[i - 1][0]
            next_pos = waypoints[i + 1][0]
            h_in = spatial.angle_to_heading(prev_pos, pos)
            h_out = spatial.angle_to_heading(pos, next_pos)
            turn_deg = math.degrees(abs(spatial.angle_diff(h_out, h_in)))
            turn_str = f"{turn_deg:.1f}°"
        else:
            turn_str = "-"

        table_data.append(
            {
                "WP": f"W_{i}",
                "X (m)": f"{pos[0]:,.1f}",
                "Y (m)": f"{pos[1]:,.1f}",
                "Heading (deg)": f"{heading_deg:.1f}°",
                "Leg Length (m)": leg_str,
                "Turn Angle (deg)": turn_str,
            }
        )

    return table_data


def render_tab_inspector() -> None:
    """Hiển thị toàn bộ giao diện Visual Scenario Inspector."""
    st.subheader("🔍 Visual Scenario Inspector & Single-Test Runner")

    presets = get_all_scenarios()
    preset_names = list(presets.keys())

    # Kiểm tra kịch bản chuyển từ Tab 2 sang
    default_scenario = st.session_state.get("selected_scenario_name", preset_names[0])
    if default_scenario not in preset_names:
        default_index = 0
    else:
        default_index = preset_names.index(str(default_scenario))

    col_ctrl, col_map = st.columns([1, 2.5], gap="medium")

    with col_ctrl:
        st.markdown("#### ⚙️ Configuration & Controls")

        selected_name = str(
            st.selectbox(
                "Select Scenario Preset",
                options=preset_names,
                index=default_index,
                help="Chọn một trong 18 kịch bản chuẩn có sẵn",
            )
        )

        exec_mode_str = st.radio(
            "Execution Mode",
            options=["Local Python Core", "NATS Microservice"],
            index=0,
            horizontal=True,
        )
        is_nats = exec_mode_str == "NATS Microservice"

        nats_url = DEFAULT_NATS_SERVER
        nats_subject = DEFAULT_SUBJECT
        if is_nats:
            nats_url = st.text_input("NATS Server URL", value=DEFAULT_NATS_SERVER)
            nats_subject = st.text_input("NATS Subject", value=DEFAULT_SUBJECT)

        with st.expander("🛠️ Vehicle Constraints Override", expanded=False):
            turn_radius = float(
                st.number_input(
                    "Turn Radius R (m)",
                    min_value=100.0,
                    max_value=50000.0,
                    value=float(config.R),
                    step=100.0,
                )
            )
            st.number_input(
                "Max Turn Angle α_max (deg)",
                min_value=10.0,
                max_value=180.0,
                value=float(config.ALPHA_MAX),
                step=5.0,
            )
            st.number_input(
                "Takeoff Straight L0 (m)",
                min_value=0.0,
                max_value=100000.0,
                value=float(config.L0),
                step=100.0,
            )
            st.number_input(
                "Sensor Lock DSS (m)",
                min_value=0.0,
                max_value=100000.0,
                value=float(config.DSS),
                step=100.0,
            )
            safe_margin = float(
                st.number_input(
                    "Safe Margin (m)",
                    min_value=0.0,
                    max_value=50000.0,
                    value=float(config.SAFE_MARGIN),
                    step=50.0,
                )
            )
            time_budget = float(
                st.number_input(
                    "Time Budget (s)",
                    min_value=1.0,
                    max_value=120.0,
                    value=15.0,
                    step=1.0,
                )
            )

        show_fillets = bool(st.checkbox("Show Fillet Arcs", value=True))
        show_buffer = bool(st.checkbox("Show Obstacle Buffer", value=True))

        run_clicked = st.button(
            "🚀 Run Planning", type="primary", use_container_width=True
        )

    # Khởi tạo kịch bản từ preset
    scenario: Scenario = presets[selected_name]()

    cached_result = cast(QAResult | None, st.session_state.get("inspector_result"))
    current_scenario_in_state = cast(
        str | None, st.session_state.get("inspector_scenario_name")
    )

    if (
        run_clicked
        or (cached_result is None)
        or (current_scenario_in_state != selected_name)
    ):
        mode = ExecutionMode.NATS if is_nats else ExecutionMode.LOCAL
        driver = ExecutionDriver(mode=mode, nats_url=nats_url, subject=nats_subject)
        with st.spinner("Executing path planning algorithm..."):
            result = driver.run_scenario(
                scenario, name=selected_name, time_budget_s=time_budget
            )
            st.session_state["inspector_result"] = result
            st.session_state["inspector_scenario_name"] = selected_name
    else:
        result = cached_result

    with col_map:
        # Metric KPIs
        kpi1, kpi2, kpi3, kpi4 = st.columns(4)
        kpi1.metric("Status", result.status)
        kpi2.metric("Wall Time", f"{result.wall_time_s:.4f} s")
        kpi3.metric("Path Length", f"{result.path_length_m:,.1f} m")
        kpi4.metric("Iterations", f"{result.iterations:,}")

        # Validation Oracle Banner
        if result.oracle_verdict.is_ok:
            st.success(
                "✅ **Validation Oracle: PASS** — All kinodynamic & obstacle non-collision constraints satisfied."  # noqa: E501
            )
        else:
            msg = f"❌ **Validation Oracle: REJECTED** — {result.oracle_verdict.detail}"
            st.error(msg)

        # Plotly 2D Interactive Figure
        fig = PlotlyVisualizer2D.create_scenario_figure(
            scenario=scenario,
            result=result,
            turn_radius=turn_radius,
            show_fillet_arcs=show_fillets,
            safe_margin=safe_margin,
            show_buffer=show_buffer,
        )
        st.plotly_chart(fig, use_container_width=True)

        # Waypoint Details Table
        if result.waypoints:
            with st.expander(
                f"📋 Trajectory Waypoints ({len(result.waypoints)} points)",
                expanded=False,
            ):
                table_rows = _compute_waypoint_table_data(result.waypoints)
                st.dataframe(table_rows, use_container_width=True)
