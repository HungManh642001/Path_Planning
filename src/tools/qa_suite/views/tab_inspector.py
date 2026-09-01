# pyright: reportMissingTypeArgument=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownParameterType=false
"""Giao diện Tab 1: Visual Scenario Inspector & Single-Test Runner (Streamlit).

Cho phép trực quan hóa bản đồ 2D nhiệm vụ, cấu hình kịch bản tự do (Custom Scenario),
tải lên file JSON kịch bản, tùy chỉnh các tham số giới hạn động học & safe margin,
thực thi thuật toán lập lịch (Local hoặc NATS) và phân tích thẩm định Validation Oracle.
"""

from __future__ import annotations

import contextlib
import json
import math
from typing import cast

import streamlit as st

from path_planning import config
from path_planning.geometry import spatial
from path_planning.scenario.presets import get_all_scenarios
from path_planning.types import Scenario
from service.vtx_service.transport import DEFAULT_NATS_SERVER, DEFAULT_SUBJECT
from tools.qa_suite.core.runner import ExecutionDriver, ExecutionMode, QAResult
from tools.qa_suite.core.scenario_custom import (
    build_custom_scenario,
    scenario_from_dict,
    scenario_to_json,
)
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

    col_ctrl, col_map = st.columns([1.1, 2.4], gap="medium")

    with col_ctrl:
        st.markdown("#### ⚙️ Configuration & Controls")

        scenario_source = st.radio(
            "Scenario Source",
            options=[
                "Preset Scenarios (18 Cases)",
                "Custom Scenario (Manual Form)",
                "Import Scenario (JSON)",
            ],
            index=0,
            horizontal=False,
        )

        scenario: Scenario | None = None
        scenario_display_name = "custom"

        if scenario_source == "Preset Scenarios (18 Cases)":
            selected_name = str(
                st.selectbox(
                    "Select Scenario Preset",
                    options=preset_names,
                    index=default_index,
                    help="Chọn một trong 18 kịch bản chuẩn có sẵn",
                )
            )
            scenario = presets[selected_name]()
            scenario_display_name = selected_name

        elif scenario_source == "Custom Scenario (Manual Form)":
            scenario_display_name = "custom_form_scenario"
            with st.expander("📍 Endpoints & Map Configuration", expanded=True):
                c_s1, c_s2 = st.columns(2)
                start_x = c_s1.number_input("Start X (m)", value=50000.0, step=5000.0)
                start_y = c_s2.number_input("Start Y (m)", value=50000.0, step=5000.0)
                start_heading_deg = st.number_input(
                    "Start Heading (deg)",
                    value=45.0,
                    min_value=-180.0,
                    max_value=360.0,
                    step=5.0,
                )

                c_g1, c_g2 = st.columns(2)
                goal_x = c_g1.number_input("Goal X (m)", value=450000.0, step=5000.0)
                goal_y = c_g2.number_input("Goal Y (m)", value=450000.0, step=5000.0)
                is_free_goal = st.checkbox(
                    "Free Goal Heading (Tiếp cận tự do)", value=False
                )
                goal_heading_deg: float | None = None
                if not is_free_goal:
                    goal_heading_deg = st.number_input(
                        "Goal Approach Heading (deg)",
                        value=45.0,
                        min_value=-180.0,
                        max_value=360.0,
                        step=5.0,
                    )

                c_m1, c_m2 = st.columns(2)
                map_w = c_m1.number_input(
                    "Map Width (m)", value=float(config.MAP_WIDTH), step=10000.0
                )
                map_h = c_m2.number_input(
                    "Map Height (m)", value=float(config.MAP_HEIGHT), step=10000.0
                )

            with st.expander("⭕ Obstacles Configuration", expanded=False):
                st.markdown("**Circle Obstacles** (định dạng `x, y, radius` mỗi dòng):")
                circles_text = st.text_area(
                    "Circles (x, y, r)",
                    value="250000.0, 250000.0, 30000.0\n150000.0, 300000.0, 20000.0",
                    height=80,
                    help="Nhập tọa độ tâm và bán kính mỗi vòng tròn một dòng",
                )
                custom_circles: list[tuple[tuple[float, float], float]] = []
                for line in circles_text.strip().splitlines():
                    parts = [p.strip() for p in line.split(",") if p.strip()]
                    if len(parts) == 3:
                        with contextlib.suppress(ValueError):
                            custom_circles.append(
                                ((float(parts[0]), float(parts[1])), float(parts[2]))
                            )

                st.markdown("**Polygon Islands** (JSON list các đỉnh):")
                islands_text = st.text_area(
                    "Islands",
                    value="[]",
                    height=70,
                    help="Ví dụ: [[[200000, 200000], [220000, 200000]]]",
                )
                custom_islands: list[list[tuple[float, float]]] = []
                with contextlib.suppress(Exception):
                    loaded_islands = json.loads(islands_text)
                    if isinstance(loaded_islands, list):
                        for poly in loaded_islands:
                            if isinstance(poly, list) and len(poly) >= 3:
                                custom_islands.append(
                                    [(float(p[0]), float(p[1])) for p in poly]
                                )

            goal_h_rad = (
                math.radians(goal_heading_deg) if goal_heading_deg is not None else None
            )
            scenario = build_custom_scenario(
                start=(start_x, start_y),
                start_heading=math.radians(start_heading_deg),
                goal=(goal_x, goal_y),
                goal_heading=goal_h_rad,
                map_bounds=(map_w, map_h),
                dynamic_obstacles=custom_circles,
                islands=custom_islands,
            )

        else:  # Import Scenario (JSON)
            scenario_display_name = "imported_json_scenario"
            uploaded_file = st.file_uploader("Upload Scenario JSON File", type=["json"])
            json_text_input = st.text_area(
                "Or Paste Scenario JSON Content", value="", height=120
            )

            if uploaded_file is not None:
                try:
                    data = json.load(uploaded_file)
                    scenario = scenario_from_dict(data)
                    st.success("✅ Scenario JSON file loaded successfully!")
                except Exception as exc:
                    st.error(f"❌ Failed to parse uploaded JSON file: {exc}")
            elif json_text_input.strip():
                try:
                    data = json.loads(json_text_input)
                    scenario = scenario_from_dict(data)
                    st.success("✅ Scenario JSON content parsed successfully!")
                except Exception as exc:
                    st.error(f"❌ Failed to parse JSON text: {exc}")

            if scenario is None:
                # Fallback to default scenario
                scenario = presets["scenario_01_open_space"]()

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

        with st.expander("🛠️ Vehicle Constraints Override", expanded=True):
            turn_radius = float(
                st.number_input(
                    "Turn Radius R (m)",
                    min_value=100.0,
                    max_value=50000.0,
                    value=float(config.R),
                    step=100.0,
                )
            )
            alpha_max_deg = float(
                st.number_input(
                    "Max Turn Angle α_max (deg)",
                    min_value=10.0,
                    max_value=180.0,
                    value=float(config.ALPHA_MAX),
                    step=5.0,
                )
            )
            l0 = float(
                st.number_input(
                    "Takeoff Straight L0 (m)",
                    min_value=0.0,
                    max_value=100000.0,
                    value=float(config.L0),
                    step=100.0,
                )
            )
            dss = float(
                st.number_input(
                    "Sensor Lock DSS (m)",
                    min_value=0.0,
                    max_value=100000.0,
                    value=float(config.DSS),
                    step=100.0,
                )
            )
            safe_margin = float(
                st.number_input(
                    "Safe Margin (m)",
                    min_value=0.0,
                    max_value=50000.0,
                    value=float(config.SAFE_MARGIN),
                    step=50.0,
                    help="Khoảng cách đệm an toàn giãn nở vật cản",
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

        c_btn1, c_btn2 = st.columns(2)
        run_clicked = c_btn1.button(
            "🚀 Run Planning", type="primary", use_container_width=True
        )
        c_btn2.download_button(
            label="💾 Export JSON",
            data=scenario_to_json(scenario),
            file_name=f"{scenario_display_name}.json",
            mime="application/json",
            use_container_width=True,
        )

    alpha_max_rad = math.radians(alpha_max_deg)

    cached_result = cast(QAResult | None, st.session_state.get("inspector_result"))
    current_scenario_in_state = cast(
        str | None, st.session_state.get("inspector_scenario_name")
    )
    current_margin_in_state = cast(
        float | None, st.session_state.get("inspector_safe_margin")
    )

    state_changed = (current_scenario_in_state != scenario_display_name) or (
        current_margin_in_state != safe_margin
    )

    if run_clicked or (cached_result is None) or state_changed:
        mode = ExecutionMode.NATS if is_nats else ExecutionMode.LOCAL
        driver = ExecutionDriver(mode=mode, nats_url=nats_url, subject=nats_subject)
        with st.spinner("Executing path planning algorithm..."):
            result = driver.run_scenario(
                scenario,
                name=scenario_display_name,
                time_budget_s=time_budget,
                turn_radius=turn_radius,
                l0=l0,
                dss=dss,
                safe_margin=safe_margin,
                alpha_max_rad=alpha_max_rad,
            )
            st.session_state["inspector_result"] = result
            st.session_state["inspector_scenario_name"] = scenario_display_name
            st.session_state["inspector_safe_margin"] = safe_margin
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
