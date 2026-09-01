# pyright: reportMissingTypeArgument=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownParameterType=false
"""Giao diện Tab 1: Visual Scenario Inspector & Single-Test Runner (Streamlit).

Cho phép trực quan hóa bản đồ 2D nhiệm vụ, thao tác kịch bản trực tiếp trên bản đồ
(Interactive Scenario Studio), cấu hình kịch bản tự do (Custom Scenario), tải lên
file JSON kịch bản, tùy chỉnh các tham số giới hạn động học & safe margin, thực thi
thuật toán lập lịch (Local hoặc NATS) và phân tích thẩm định Validation Oracle.
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
    add_circle_obstacle,
    add_polygon_obstacle,
    build_custom_scenario,
    clear_all_obstacles,
    remove_last_obstacle,
    remove_obstacle_by_index,
    scenario_from_dict,
    scenario_to_dict,
    scenario_to_json,
    simplify_polygon_rdp,
    update_goal_position,
    update_start_position,
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


def _parse_svg_path(path_str: str) -> list[tuple[float, float]]:
    """Phân tích chuỗi SVG path (vd: 'M 10 20 L 30 40 Z') thành danh sách (x, y)."""
    clean_str = (
        path_str.replace("M", " ")
        .replace("L", " ")
        .replace("Z", " ")
        .replace("z", " ")
        .replace(",", " ")
    )
    tokens = clean_str.split()
    coords: list[tuple[float, float]] = []
    i = 0
    while i < len(tokens) - 1:
        with contextlib.suppress(ValueError):
            x = float(tokens[i])
            y = float(tokens[i + 1])
            coords.append((x, y))
        i += 2
    return coords


def render_tab_inspector() -> None:
    """Hiển thị toàn bộ giao diện Visual Scenario Inspector & Interactive Studio."""
    st.subheader("🔍 Visual Scenario Inspector & Interactive Studio")

    presets = get_all_scenarios()
    preset_names = list(presets.keys())

    # Kiểm tra kịch bản chuyển từ Tab 2 sang
    default_scenario = st.session_state.get("selected_scenario_name", preset_names[0])
    if default_scenario not in preset_names:
        default_index = 0
    else:
        default_index = preset_names.index(str(default_scenario))

    # Khởi tạo trạng thái Session State an toàn cho Interactive Studio
    if (
        "active_scenario" not in st.session_state
        or st.session_state.get("active_scenario") is None
    ):
        st.session_state["active_scenario"] = presets[preset_names[default_index]]()
    if "draft_polygon_vertices" not in st.session_state:
        st.session_state["draft_polygon_vertices"] = []
    if "studio_mode" not in st.session_state:
        st.session_state["studio_mode"] = "🔍 Pan / Inspect"
    if "circle_radius" not in st.session_state:
        st.session_state["circle_radius"] = 25000.0
    if "last_clicked_point" not in st.session_state:
        st.session_state["last_clicked_point"] = None

    col_ctrl, col_map = st.columns([1.1, 2.4], gap="medium")

    with col_ctrl:
        st.markdown("#### ⚙️ Configuration & Controls")

        scenario_source = st.radio(
            "Scenario Source",
            options=[
                "Preset Scenarios (18 Cases)",
                "🎨 Interactive Studio (GUI Drawing)",
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
            st.session_state["active_scenario"] = scenario
            scenario_display_name = selected_name

        elif scenario_source == "🎨 Interactive Studio (GUI Drawing)":
            scenario_display_name = "interactive_studio_scenario"
            scenario = cast(
                Scenario,
                st.session_state.get("active_scenario")
                or presets[preset_names[default_index]](),
            )

            with st.expander("🎨 Studio Toolbar & Controls", expanded=True):
                base_preset_name = str(
                    st.selectbox(
                        "Base Preset Template",
                        options=preset_names,
                        index=default_index,
                        help="Chọn kịch bản mẫu làm nền tảng",
                    )
                )
                last_preset = st.session_state.get("last_selected_base_preset")
                if last_preset is not None and last_preset != base_preset_name:
                    st.session_state["last_selected_base_preset"] = base_preset_name
                    st.session_state["active_scenario"] = presets[base_preset_name]()
                    st.session_state["draft_polygon_vertices"] = []
                    st.session_state["last_clicked_point"] = None
                    scenario = cast(Scenario, st.session_state["active_scenario"])
                elif last_preset is None:
                    st.session_state["last_selected_base_preset"] = base_preset_name

                studio_mode_options = [
                    "🔍 Pan / Inspect",
                    "⭕ Add Circle Obstacle",
                    "📐 Add Polygon (Click Vertices)",
                    "🚀 Move Start (O)",
                    "🎯 Move Goal (T)",
                ]
                current_studio_mode = st.session_state.get(
                    "studio_mode", studio_mode_options[0]
                )
                mode_index = (
                    studio_mode_options.index(current_studio_mode)
                    if current_studio_mode in studio_mode_options
                    else 0
                )
                studio_mode = st.radio(
                    "Studio Mode",
                    options=studio_mode_options,
                    index=mode_index,
                    help="Chọn chế độ thao tác trên bản đồ",
                )
                st.session_state["studio_mode"] = studio_mode

                if studio_mode == "⭕ Add Circle Obstacle":
                    c_c1, c_c2 = st.columns(2)
                    center_x = float(
                        c_c1.number_input(
                            "Center X (m)",
                            value=float(
                                st.session_state.get("circle_input_x", 50000.0)
                            ),
                            step=5000.0,
                        )
                    )
                    center_y = float(
                        c_c2.number_input(
                            "Center Y (m)",
                            value=float(
                                st.session_state.get("circle_input_y", 50000.0)
                            ),
                            step=5000.0,
                        )
                    )
                    circle_radius = float(
                        st.number_input(
                            "Circle Radius (m)",
                            value=float(st.session_state.get("circle_radius", 25000.0)),
                            min_value=500.0,
                            max_value=200000.0,
                            step=5000.0,
                            help="Bán kính vòng tròn chướng ngại vật",
                        )
                    )
                    st.session_state["circle_radius"] = circle_radius
                    st.session_state["circle_input_x"] = center_x
                    st.session_state["circle_input_y"] = center_y

                    if st.button("➕ Add Circle at (X, Y)", use_container_width=True):
                        st.session_state["active_scenario"] = add_circle_obstacle(
                            st.session_state["active_scenario"],
                            (center_x, center_y),
                            circle_radius,
                        )
                        st.session_state["last_clicked_point"] = None
                        st.rerun()

                    st.info(
                        "👉 Click vào bản đồ hoặc bấm nút để thêm vòng tròn chướng ngại vật."  # noqa: E501
                    )

                elif studio_mode == "📐 Add Polygon (Click Vertices)":
                    draft_vertices = cast(
                        list[tuple[float, float]],
                        st.session_state.get("draft_polygon_vertices", []),
                    )
                    c_v1, c_v2 = st.columns(2)
                    vert_x = float(
                        c_v1.number_input("Vertex X (m)", value=50000.0, step=5000.0)
                    )
                    vert_y = float(
                        c_v2.number_input("Vertex Y (m)", value=50000.0, step=5000.0)
                    )
                    if st.button("➕ Add Vertex (X, Y)", use_container_width=True):
                        curr_draft = list(
                            st.session_state.get("draft_polygon_vertices", [])
                        )
                        curr_draft.append((vert_x, vert_y))
                        st.session_state["draft_polygon_vertices"] = curr_draft
                        st.rerun()

                    st.info(
                        f"👉 Click vào bản đồ hoặc nhập tọa độ để thêm đỉnh đa giác. "
                        f"(Hiện có: {len(draft_vertices)} đỉnh)"
                    )
                    if draft_vertices:
                        st.caption(
                            "Đỉnh nháp: "
                            + " ➔ ".join(
                                f"V{i + 1}({x:,.0f}, {y:,.0f})"
                                for i, (x, y) in enumerate(draft_vertices)
                            )
                        )
                    c_poly1, c_poly2 = st.columns(2)
                    if c_poly1.button(
                        "✅ Complete Polygon",
                        disabled=len(draft_vertices) < 3,
                        use_container_width=True,
                    ):
                        if len(draft_vertices) >= 3:
                            simplified = simplify_polygon_rdp(
                                draft_vertices, epsilon=3000.0
                            )
                            st.session_state["active_scenario"] = add_polygon_obstacle(
                                st.session_state["active_scenario"], simplified
                            )
                            st.session_state["draft_polygon_vertices"] = []
                            st.session_state["last_clicked_point"] = None
                            st.rerun()
                        else:
                            st.warning("⚠️ Đa giác cần tối thiểu 3 đỉnh.")
                    if c_poly2.button(
                        "🗑️ Remove Last Vertex",
                        disabled=len(draft_vertices) == 0,
                        use_container_width=True,
                    ):
                        curr_draft = list(
                            st.session_state.get("draft_polygon_vertices", [])
                        )
                        if curr_draft:
                            curr_draft.pop()
                            st.session_state["draft_polygon_vertices"] = curr_draft
                            st.rerun()
                    if st.button(
                        "❌ Cancel Draft",
                        disabled=len(draft_vertices) == 0,
                        use_container_width=True,
                    ):
                        st.session_state["draft_polygon_vertices"] = []
                        st.session_state["last_clicked_point"] = None
                        st.rerun()

                elif studio_mode == "🚀 Move Start (O)":
                    curr_start = scenario["start"]
                    curr_heading_deg = math.degrees(scenario["start_heading"])
                    c_s1, c_s2 = st.columns(2)
                    start_x = float(
                        c_s1.number_input(
                            "Start X (m)", value=float(curr_start[0]), step=5000.0
                        )
                    )
                    start_y = float(
                        c_s2.number_input(
                            "Start Y (m)", value=float(curr_start[1]), step=5000.0
                        )
                    )
                    new_start_h_deg = float(
                        st.number_input(
                            "Start Heading (deg)",
                            value=float(curr_heading_deg),
                            min_value=-180.0,
                            max_value=360.0,
                            step=5.0,
                        )
                    )
                    if (
                        abs(start_x - curr_start[0]) > 1e-4
                        or abs(start_y - curr_start[1]) > 1e-4
                        or abs(new_start_h_deg - curr_heading_deg) > 1e-4
                    ):
                        st.session_state["active_scenario"] = update_start_position(
                            st.session_state["active_scenario"],
                            (start_x, start_y),
                            heading_rad=math.radians(new_start_h_deg),
                        )
                        st.rerun()
                    st.info(
                        "👉 Click vào bản đồ hoặc chỉnh sửa tọa độ bên trên để cập nhật điểm xuất phát."  # noqa: E501
                    )

                elif studio_mode == "🎯 Move Goal (T)":
                    curr_goal = scenario["goal"]
                    is_free_goal = scenario.get("goal_heading") is None
                    c_g1, c_g2 = st.columns(2)
                    goal_x = float(
                        c_g1.number_input(
                            "Goal X (m)", value=float(curr_goal[0]), step=5000.0
                        )
                    )
                    goal_y = float(
                        c_g2.number_input(
                            "Goal Y (m)", value=float(curr_goal[1]), step=5000.0
                        )
                    )
                    free_goal_cb = st.checkbox(
                        "Free Goal Heading (Tiếp cận tự do)", value=is_free_goal
                    )
                    if free_goal_cb:
                        if (
                            not is_free_goal
                            or abs(goal_x - curr_goal[0]) > 1e-4
                            or abs(goal_y - curr_goal[1]) > 1e-4
                        ):
                            st.session_state["active_scenario"] = update_goal_position(
                                st.session_state["active_scenario"],
                                (goal_x, goal_y),
                                clear_heading=True,
                            )
                            st.rerun()
                    else:
                        gh = scenario.get("goal_heading")
                        curr_g_deg = math.degrees(gh) if gh is not None else 0.0
                        new_goal_h_deg = float(
                            st.number_input(
                                "Goal Heading (deg)",
                                value=float(curr_g_deg),
                                min_value=-180.0,
                                max_value=360.0,
                                step=5.0,
                            )
                        )
                        if (
                            scenario.get("goal_heading") is None
                            or abs(goal_x - curr_goal[0]) > 1e-4
                            or abs(goal_y - curr_goal[1]) > 1e-4
                            or abs(new_goal_h_deg - curr_g_deg) > 1e-4
                        ):
                            st.session_state["active_scenario"] = update_goal_position(
                                st.session_state["active_scenario"],
                                (goal_x, goal_y),
                                heading_rad=math.radians(new_goal_h_deg),
                            )
                            st.rerun()
                    st.info(
                        "👉 Click vào bản đồ hoặc chỉnh sửa tọa độ bên trên để cập nhật điểm mục tiêu."  # noqa: E501
                    )

                else:
                    st.info("💡 Chế độ xem & dịch chuyển bản đồ bình thường.")

                st.markdown("##### 📋 Active Obstacles Manager")
                active_obstacles = scenario.get("obstacles", [])
                if not active_obstacles:
                    st.caption("Chưa có vật cản nào trên bản đồ.")
                else:
                    for idx, obs in enumerate(active_obstacles):
                        c_info, c_del = st.columns([4, 1])
                        if obs["type"] == "circle":
                            cx, cy = obs["center"]
                            r = obs["radius"]
                            c_info.markdown(
                                f"**#{idx + 1} ⭕ Circle**: `({cx / 1000:,.1f}, {cy / 1000:,.1f}) km`, $R={r / 1000:,.1f}$ km"  # noqa: E501
                            )
                        else:
                            poly = obs.get("polygon", [])
                            c_info.markdown(
                                f"**#{idx + 1} 📐 Polygon**: Đảo `{len(poly)}` đỉnh"
                            )
                        if c_del.button(
                            "❌", key=f"del_obs_{idx}", help=f"Xóa vật cản #{idx + 1}"
                        ):
                            st.session_state["active_scenario"] = (
                                remove_obstacle_by_index(
                                    st.session_state["active_scenario"], idx
                                )
                            )
                            st.session_state["last_clicked_point"] = None
                            st.rerun()

                st.markdown("##### ⚡ Quick Actions")
                qa_c1, qa_c2 = st.columns(2)
                if qa_c1.button("↩️ Undo Last Obstacle", use_container_width=True):
                    st.session_state["active_scenario"] = remove_last_obstacle(
                        st.session_state["active_scenario"]
                    )
                    st.session_state["last_clicked_point"] = None
                    st.rerun()

                if qa_c2.button("🗑️ Clear All Obstacles", use_container_width=True):
                    st.session_state["active_scenario"] = clear_all_obstacles(
                        st.session_state["active_scenario"]
                    )
                    st.session_state["draft_polygon_vertices"] = []
                    st.session_state["last_clicked_point"] = None
                    st.rerun()

                if st.button("🔄 Reset to Preset", use_container_width=True):
                    st.session_state["active_scenario"] = presets[base_preset_name]()
                    st.session_state["draft_polygon_vertices"] = []
                    st.session_state["last_clicked_point"] = None
                    st.rerun()

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
            st.session_state["active_scenario"] = scenario

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
                    st.session_state["active_scenario"] = scenario
                    st.success("✅ Scenario JSON file loaded successfully!")
                except Exception as exc:
                    st.error(f"❌ Failed to parse uploaded JSON file: {exc}")
            elif json_text_input.strip():
                try:
                    data = json.loads(json_text_input)
                    scenario = scenario_from_dict(data)
                    st.session_state["active_scenario"] = scenario
                    st.success("✅ Scenario JSON content parsed successfully!")
                except Exception as exc:
                    st.error(f"❌ Failed to parse JSON text: {exc}")

            if scenario is None:
                scenario = presets["scenario_01_open_ocean"]()
                st.session_state["active_scenario"] = scenario

        # Hiển thị tóm tắt số lượng vật cản hiện tại
        circles = scenario.get("dynamic_obstacles") or []
        islands = scenario.get("islands") or []
        obstacles = scenario.get("obstacles") or []
        total_obs = len(obstacles) if obstacles else (len(circles) + len(islands))
        st.caption(
            f"Obstacles: {len(circles)} circles, {len(islands)} islands | "
            f"Total: {total_obs}"
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

    scenario_dict_repr = json.dumps(scenario_to_dict(scenario), sort_keys=True)
    cached_result = cast(QAResult | None, st.session_state.get("inspector_result"))
    current_scenario_in_state = cast(
        str | None, st.session_state.get("inspector_scenario_name")
    )
    current_dict_in_state = cast(
        str | None, st.session_state.get("inspector_scenario_dict")
    )
    current_margin_in_state = cast(
        float | None, st.session_state.get("inspector_safe_margin")
    )
    current_r_in_state = cast(
        float | None, st.session_state.get("inspector_turn_radius")
    )
    current_budget_in_state = cast(
        float | None, st.session_state.get("inspector_time_budget")
    )

    state_changed = (
        (current_scenario_in_state != scenario_display_name)
        or (current_dict_in_state != scenario_dict_repr)
        or (current_margin_in_state != safe_margin)
        or (current_r_in_state != turn_radius)
        or (current_budget_in_state != time_budget)
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
            st.session_state["inspector_scenario_dict"] = scenario_dict_repr
            st.session_state["inspector_safe_margin"] = safe_margin
            st.session_state["inspector_turn_radius"] = turn_radius
            st.session_state["inspector_time_budget"] = time_budget
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
        draft_vertices = cast(
            list[tuple[float, float]],
            st.session_state.get("draft_polygon_vertices", []),
        )
        fig = PlotlyVisualizer2D.create_scenario_figure(
            scenario=scenario,
            result=result,
            turn_radius=turn_radius,
            show_fillet_arcs=show_fillets,
            safe_margin=safe_margin,
            show_buffer=show_buffer,
            draft_polygon_vertices=draft_vertices if draft_vertices else None,
            dragmode="pan",
            enable_click_grid=True,
        )

        plotly_config = {
            "modeBarButtonsToAdd": [
                "drawcircle",
                "drawclosedpath",
                "eraseshape",
            ],
            "displaylogo": False,
            "responsive": True,
        }

        selection = st.plotly_chart(
            fig,
            use_container_width=True,
            on_select="rerun",
            selection_mode=["points", "box"],
            key="inspector_map",
            config=plotly_config,
        )

        # Xử lý sự kiện click / selection từ Plotly
        if (
            scenario_source == "🎨 Interactive Studio (GUI Drawing)"
            and selection
            and isinstance(selection, dict)
        ):
            # 1. Xử lý các hình vẽ tự do từ Plotly ModeBar (shapes)
            shapes = selection.get("shapes", [])
            if shapes and isinstance(shapes, list):
                last_shape = shapes[-1]
                if isinstance(last_shape, dict):
                    shape_type = str(last_shape.get("type", ""))
                    shape_sig = (
                        f"{shape_type}_{last_shape.get('x0')}_{last_shape.get('y0')}_"
                        f"{last_shape.get('x1')}_{last_shape.get('y1')}_"
                        f"{last_shape.get('path')}"
                    )
                    if st.session_state.get("last_processed_shape") != shape_sig:
                        st.session_state["last_processed_shape"] = shape_sig
                        if shape_type == "circle":
                            x0 = float(last_shape.get("x0", 0.0))
                            x1 = float(last_shape.get("x1", 0.0))
                            y0 = float(last_shape.get("y0", 0.0))
                            y1 = float(last_shape.get("y1", 0.0))
                            cx = (x0 + x1) / 2.0
                            cy = (y0 + y1) / 2.0
                            r = max(abs(x1 - x0), abs(y1 - y0)) / 2.0
                            if r > 10.0:
                                st.session_state["active_scenario"] = (
                                    add_circle_obstacle(
                                        st.session_state["active_scenario"],
                                        (cx, cy),
                                        r,
                                    )
                                )
                                st.rerun()
                        elif shape_type == "path" and "path" in last_shape:
                            poly_pts = _parse_svg_path(str(last_shape["path"]))
                            if len(poly_pts) >= 3:
                                simplified = simplify_polygon_rdp(
                                    poly_pts, epsilon=3000.0
                                )
                                st.session_state["active_scenario"] = (
                                    add_polygon_obstacle(
                                        st.session_state["active_scenario"],
                                        simplified,
                                    )
                                )
                                st.rerun()

            # 2. Xử lý sự kiện click điểm (points)
            pts = selection.get("points", [])
            if pts and isinstance(pts, list):
                last_pt = pts[-1]
                if (
                    isinstance(last_pt, dict)
                    and "x" in last_pt
                    and "y" in last_pt
                    and last_pt["x"] is not None
                    and last_pt["y"] is not None
                ):
                    click_coord = (float(last_pt["x"]), float(last_pt["y"]))
                    if st.session_state.get("last_clicked_point") != click_coord:
                        st.session_state["last_clicked_point"] = click_coord
                        studio_mode = st.session_state.get(
                            "studio_mode", "🔍 Pan / Inspect"
                        )

                        if studio_mode == "⭕ Add Circle Obstacle":
                            circle_radius = float(
                                st.session_state.get("circle_radius", 25000.0)
                            )
                            st.session_state["active_scenario"] = add_circle_obstacle(
                                st.session_state["active_scenario"],
                                click_coord,
                                circle_radius,
                            )
                            st.rerun()
                        elif studio_mode == "📐 Add Polygon (Click Vertices)":
                            curr_draft = list(
                                st.session_state.get("draft_polygon_vertices", [])
                            )
                            curr_draft.append(click_coord)
                            st.session_state["draft_polygon_vertices"] = curr_draft
                            st.rerun()
                        elif studio_mode == "🚀 Move Start (O)":
                            st.session_state["active_scenario"] = update_start_position(
                                st.session_state["active_scenario"], click_coord
                            )
                            st.rerun()
                        elif studio_mode == "🎯 Move Goal (T)":
                            st.session_state["active_scenario"] = update_goal_position(
                                st.session_state["active_scenario"], click_coord
                            )
                            st.rerun()

        # Waypoint Details Table
        if result.waypoints:
            with st.expander(
                f"📋 Trajectory Waypoints ({len(result.waypoints)} points)",
                expanded=False,
            ):
                table_rows = _compute_waypoint_table_data(result.waypoints)
                st.dataframe(table_rows, use_container_width=True)
