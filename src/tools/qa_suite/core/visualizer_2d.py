"""Module trực quan hóa 2D tương tác bằng Plotly (PlotlyVisualizer2D).

Hỗ trợ vẽ bản đồ nhiệm vụ, vùng an toàn, các đảo đa giác kèm đệm an toàn,
chướng ngại vật tròn, điểm cất cánh / mục tiêu kèm vector hướng bay, cùng
quỹ đạo bay gồm các đoạn thẳng nối waypoint và cung lượn fillet arcs.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import plotly.graph_objects as go

from path_planning import config
from path_planning.geometry import spatial
from path_planning.types import CircleGeometry, Point, PolygonCoords, Scenario
from path_planning.validation import oracle


if TYPE_CHECKING:
    from tools.qa_suite.core.runner import QAResult


class PlotlyVisualizer2D:
    """Bộ trực quan hóa bản đồ 2D và quỹ đạo bay bằng Plotly."""

    @staticmethod
    def create_scenario_figure(
        scenario: Scenario,
        result: QAResult | None = None,
        turn_radius: float = config.R,
        show_fillet_arcs: bool = True,
        title: str | None = None,
        safe_margin: float = 500.0,
        show_buffer: bool = True,
    ) -> go.Figure:
        """Tạo biểu đồ Plotly 2D tương tác trực quan hóa kịch bản và kết quả đường bay.

        Args:
            scenario: Dữ liệu kịch bản nhiệm vụ (Scenario dict).
            result: Kết quả thực thi kịch bản (QAResult), hoặc None nếu chỉ xem.
            turn_radius: Bán kính quay vòng R (m) dùng để vẽ cung lượn fillet arc.
            show_fillet_arcs: Cờ bật/tắt vẽ cung lượn bo góc rẽ tại các waypoint.
            title: Tiêu đề tùy chỉnh cho biểu đồ (nếu None sẽ tự sinh theo kết quả).
            safe_margin: Khoảng cách đệm an toàn mở rộng (m) để vẽ đường bao buffer.
            show_buffer: Cờ bật/tắt hiển thị đường bao đệm an toàn (buffer).

        Returns:
            go.Figure: Đối tượng biểu đồ Plotly sẵn sàng để hiển thị.
        """
        fig = go.Figure()

        # 1. Vẽ vùng an toàn (Safezones) nếu có
        safezones = scenario.get("safezones")
        if safezones:
            for idx, zone in enumerate(safezones):
                if not zone:
                    continue
                xs = [p[0] for p in zone] + [zone[0][0]]
                ys = [p[1] for p in zone] + [zone[0][1]]
                fig.add_trace(
                    go.Scatter(
                        x=xs,
                        y=ys,
                        mode="lines",
                        fill="toself",
                        fillcolor="rgba(173, 216, 230, 0.25)",
                        line={"color": "royalblue", "width": 2, "dash": "dash"},
                        name="Safezone",
                        legendgroup="Safezone",
                        showlegend=(idx == 0),
                        hovertext=f"Safezone {idx + 1}",
                        hoverinfo="text",
                    )
                )

        # 2. Thu thập và vẽ chướng ngại vật Đa giác (Islands)
        islands: list[PolygonCoords] = list(scenario.get("islands") or [])
        for obs in scenario.get("obstacles") or []:
            if obs.get("type") == "polygon" and "polygon" in obs:
                poly = obs["polygon"]
                if poly not in islands:
                    islands.append(poly)

        buffer_shown = False
        for idx, island in enumerate(islands):
            if not island:
                continue
            xs = [p[0] for p in island] + [island[0][0]]
            ys = [p[1] for p in island] + [island[0][1]]
            fig.add_trace(
                go.Scatter(
                    x=xs,
                    y=ys,
                    mode="lines",
                    fill="toself",
                    fillcolor="rgba(139, 69, 19, 0.65)",
                    line={"color": "rgb(90, 20, 10)", "width": 2},
                    name="Island Obstacle",
                    legendgroup="Island",
                    showlegend=(idx == 0),
                    hovertext=f"Island Obstacle ({len(island)} vertices)",
                    hoverinfo="text",
                )
            )

            # Vẽ đệm an toàn Safe Margin cho đảo đa giác
            if show_buffer and safe_margin > 0:
                buf_poly = spatial.inflate_polygon(island, safe_margin)
                if buf_poly:
                    bxs = [p[0] for p in buf_poly] + [buf_poly[0][0]]
                    bys = [p[1] for p in buf_poly] + [buf_poly[0][1]]
                    fig.add_trace(
                        go.Scatter(
                            x=bxs,
                            y=bys,
                            mode="lines",
                            line={
                                "color": "rgba(139, 0, 0, 0.45)",
                                "width": 1.5,
                                "dash": "dash",
                            },
                            name="Obstacle Buffer (Safe Margin)",
                            legendgroup="Buffer",
                            showlegend=not buffer_shown,
                            hovertext=f"Safe Margin (+{safe_margin:.0f}m)",
                            hoverinfo="text",
                        )
                    )
                    buffer_shown = True

        # 3. Thu thập và vẽ chướng ngại vật Tròn (Circle Obstacles)
        dynamic_obs: list[CircleGeometry] = list(
            scenario.get("dynamic_obstacles") or []
        )
        for obs in scenario.get("obstacles") or []:
            if obs.get("type") == "circle" and "center" in obs and "radius" in obs:
                cg = (obs["center"], float(obs["radius"]))
                if cg not in dynamic_obs:
                    dynamic_obs.append(cg)

        circle_segments = 64
        angles = [2 * math.pi * k / circle_segments for k in range(circle_segments + 1)]

        for idx, (center, radius) in enumerate(dynamic_obs):
            cx, cy = center
            cxs = [cx + radius * math.cos(a) for a in angles]
            cys = [cy + radius * math.sin(a) for a in angles]
            fig.add_trace(
                go.Scatter(
                    x=cxs,
                    y=cys,
                    mode="lines",
                    fill="toself",
                    fillcolor="rgba(255, 140, 0, 0.6)",
                    line={"color": "darkorange", "width": 2},
                    name="Circle Obstacle",
                    legendgroup="Circle Obstacle",
                    showlegend=(idx == 0),
                    hovertext=f"Circle Obstacle (r={radius:.0f}m)",
                    hoverinfo="text",
                )
            )

            # Vẽ đệm an toàn Safe Margin cho chướng ngại vật tròn
            if show_buffer and safe_margin > 0:
                buf_r = radius + safe_margin
                bcxs = [cx + buf_r * math.cos(a) for a in angles]
                bcys = [cy + buf_r * math.sin(a) for a in angles]
                fig.add_trace(
                    go.Scatter(
                        x=bcxs,
                        y=bcys,
                        mode="lines",
                        line={
                            "color": "rgba(255, 69, 0, 0.45)",
                            "width": 1.5,
                            "dash": "dash",
                        },
                        name="Obstacle Buffer (Safe Margin)",
                        legendgroup="Buffer",
                        showlegend=not buffer_shown,
                        hovertext=f"Safe Margin (r={buf_r:.0f}m)",
                        hoverinfo="text",
                    )
                )
                buffer_shown = True

        # 4. Vẽ điểm xuất phát (Start O) và đích (Goal T) kèm vector chỉ hướng
        start: Point = scenario["start"]
        start_heading: float = scenario["start_heading"]
        goal: Point = scenario["goal"]
        goal_heading: float | None = scenario.get("goal_heading")

        # Start Marker
        fig.add_trace(
            go.Scatter(
                x=[start[0]],
                y=[start[1]],
                mode="markers+text",
                marker={
                    "size": 14,
                    "color": "forestgreen",
                    "symbol": "circle",
                    "line": {"width": 2, "color": "darkgreen"},
                },
                name="Start (O)",
                text=["O"],
                textposition="top right",
                hovertext=(
                    f"Start Point O: ({start[0]:.1f}, {start[1]:.1f})<br>"
                    f"Heading: {math.degrees(start_heading):.1f}°"
                ),
                hoverinfo="text",
            )
        )

        # Tính toán chiều dài mũi tên định hướng phù hợp kích thước bản đồ
        map_bounds = scenario.get("map_bounds") or (
            config.MAP_WIDTH,
            config.MAP_HEIGHT,
        )
        arrow_len = max(2000.0, 0.04 * max(map_bounds[0], map_bounds[1]))

        # Start heading arrow
        hx = start[0] + arrow_len * math.cos(start_heading)
        hy = start[1] + arrow_len * math.sin(start_heading)
        fig.add_annotation(
            ax=start[0],
            ay=start[1],
            x=hx,
            y=hy,
            xref="x",
            yref="y",
            axref="x",
            ayref="y",
            showarrow=True,
            arrowhead=3,
            arrowsize=1.4,
            arrowwidth=2.5,
            arrowcolor="forestgreen",
            text="",
        )

        # Goal Marker
        goal_hover = f"Goal Point T: ({goal[0]:.1f}, {goal[1]:.1f})<br>" + (
            f"Heading: {math.degrees(goal_heading):.1f}°"
            if goal_heading is not None
            else "Heading: Free"
        )
        fig.add_trace(
            go.Scatter(
                x=[goal[0]],
                y=[goal[1]],
                mode="markers+text",
                marker={
                    "size": 16,
                    "color": "crimson",
                    "symbol": "star",
                    "line": {"width": 2, "color": "darkred"},
                },
                name="Goal (T)",
                text=["T"],
                textposition="top right",
                hovertext=goal_hover,
                hoverinfo="text",
            )
        )

        # Goal heading arrow (nếu có hướng tiếp cận cố định)
        if goal_heading is not None:
            gax = goal[0] - arrow_len * math.cos(goal_heading)
            gay = goal[1] - arrow_len * math.sin(goal_heading)
            fig.add_annotation(
                ax=gax,
                ay=gay,
                x=goal[0],
                y=goal[1],
                xref="x",
                yref="y",
                axref="x",
                ayref="y",
                showarrow=True,
                arrowhead=3,
                arrowsize=1.4,
                arrowwidth=2.5,
                arrowcolor="crimson",
                text="",
            )

        # 5. Vẽ quỹ đạo bay nếu có kết quả (Trajectory Waypoints & Fillet Arcs)
        if result is not None and result.waypoints:
            wps = result.waypoints
            wx = [wp[0][0] for wp in wps]
            wy = [wp[0][1] for wp in wps]
            wp_hovers = [
                (
                    f"WP {i}: ({wp[0][0]:.1f}, {wp[0][1]:.1f})<br>"
                    f"Heading: {math.degrees(wp[1]):.1f}°"
                )
                for i, wp in enumerate(wps)
            ]

            fig.add_trace(
                go.Scatter(
                    x=wx,
                    y=wy,
                    mode="lines+markers",
                    line={"color": "royalblue", "width": 3},
                    marker={
                        "size": 8,
                        "color": "royalblue",
                        "symbol": "circle",
                        "line": {"width": 1, "color": "navy"},
                    },
                    name="Trajectory Waypoints",
                    text=wp_hovers,
                    hoverinfo="text",
                )
            )

            # Vẽ cung lượn Fillet Arcs tại các góc rẽ nội bộ
            if show_fillet_arcs and len(wps) >= 3:
                arc_legend_shown = False
                for i in range(1, len(wps) - 1):
                    w_prev = wps[i - 1][0]
                    w_curr = wps[i][0]
                    w_next = wps[i + 1][0]

                    arc = oracle.arc_points(
                        w_prev, w_curr, w_next, turn_radius=turn_radius
                    )
                    if arc:
                        ax = [p[0] for p in arc]
                        ay = [p[1] for p in arc]
                        fig.add_trace(
                            go.Scatter(
                                x=ax,
                                y=ay,
                                mode="lines",
                                line={"color": "magenta", "width": 4},
                                name="Fillet Arcs (R)",
                                legendgroup="Fillet Arcs",
                                showlegend=not arc_legend_shown,
                                hovertext=f"Fillet Arc W{i} (R={turn_radius:.0f}m)",
                                hoverinfo="text",
                            )
                        )
                        arc_legend_shown = True

                        # Tính góc rẽ alpha_i và gắn chú thích góc
                        h_in = spatial.angle_to_heading(w_prev, w_curr)
                        h_out = spatial.angle_to_heading(w_curr, w_next)
                        alpha = abs(spatial.angle_diff(h_out, h_in))
                        alpha_deg = math.degrees(alpha)
                        fig.add_trace(
                            go.Scatter(
                                x=[w_curr[0]],
                                y=[w_curr[1]],
                                mode="text",
                                text=[f"α_{i}={alpha_deg:.1f}°"],
                                textposition="bottom right",
                                textfont={"color": "darkmagenta", "size": 10},
                                showlegend=False,
                                hoverinfo="skip",
                            )
                        )

        # 6. Cấu hình tiêu đề và Layout tổng thể
        if title is not None:
            fig_title = title
        elif result is not None:
            status_icon = "✅" if result.is_success else "❌"
            fig_title = (
                f"{status_icon} {result.scenario_name} | Status: {result.status} | "
                f"Length: {result.path_length_m:.1f}m | Time: {result.wall_time_s:.3f}s"
            )
        else:
            fig_title = "Mission Scenario Map"

        fig.update_layout(
            title={"text": fig_title, "font": {"size": 15, "color": "#1e293b"}},
            xaxis={
                "title": "East (m)",
                "showgrid": True,
                "zeroline": False,
                "gridcolor": "rgba(200, 200, 200, 0.4)",
            },
            yaxis={
                "title": "North (m)",
                "showgrid": True,
                "zeroline": False,
                "gridcolor": "rgba(200, 200, 200, 0.4)",
                "scaleanchor": "x",
                "scaleratio": 1,
            },
            hovermode="closest",
            showlegend=True,
            legend={
                "x": 0.01,
                "y": 0.99,
                "bgcolor": "rgba(255, 255, 255, 0.85)",
                "bordercolor": "rgba(0, 0, 0, 0.2)",
                "borderwidth": 1,
            },
            template="plotly_white",
            margin={"l": 50, "r": 50, "t": 60, "b": 50},
        )

        return fig
