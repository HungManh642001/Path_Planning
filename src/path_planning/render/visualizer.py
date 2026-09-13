"""Trực quan hóa kịch bản nhiệm vụ, chướng ngại vật và quỹ đạo bay.

Tiếp nhận dữ liệu đầu ra của bộ lập kế hoạch; không có gì ở đây phản hồi ngược
vào tìm kiếm, do đó quan hệ phụ thuộc là render -> core và không bao giờ đảo ngược.

Ghi chú về kiểu dữ liệu: matplotlib chỉ cung cấp một phần stubs kiểu, do đó
gói này được kiểm tra ở chế độ `standard` của pyright thay vì `strict` (xem khối
executionEnvironments trong pyproject.toml). Mọi hàm ở đây vẫn được chú thích
kiểu đầy đủ; sự nới lỏng duy nhất là không đòi hỏi các chữ ký hàm của chính
matplotlib phải được định kiểu hoàn chỉnh.
"""

from __future__ import annotations

import logging
import math
from collections.abc import Sequence
from itertools import pairwise
from typing import TYPE_CHECKING, Literal

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.patches import Circle as MplCircle, Polygon as MplPolygon, Rectangle

from path_planning import config
from path_planning.render import sampling


if TYPE_CHECKING:
    from matplotlib.figure import Figure

    from path_planning.render.sampling import RenderMode
    from path_planning.types import (
        Obstacle,
        PlanResultView,
        Point,
        PreprocessedScenario,
        Scenario,
    )

logger = logging.getLogger(__name__)

Extents = tuple[tuple[float, float], tuple[float, float]]
"""Giới hạn trục tọa độ dạng ``((xmin, xmax), (ymin, ymax))``."""

Fit = Literal["map", "content"]
"""Cách :func:`plot_scenario` căn chỉnh khung hình hiển thị."""


def _plot_extents(scenario: Scenario | None, pad: float = 2000.0) -> Extents:
    """Tính toán giới hạn trục tọa độ cho kịch bản theo góc nhìn toàn bản đồ.

    Args:
        scenario: Kịch bản đang được vẽ, hoặc ``None``.
        pad: Khoảng đệm bổ sung xung quanh hộp bao (m).

    Returns:
        Giới hạn trục tọa độ: hộp bao của tất cả đa giác safezone cộng phần đệm
        khi có mặt, nếu không sẽ dùng hình chữ nhật ``config.MAP_WIDTH/HEIGHT``.
    """
    safezones = scenario.get("safezones") if scenario else None
    if safezones:
        xs = [p[0] for poly in safezones for p in poly]
        ys = [p[1] for poly in safezones for p in poly]
        return (min(xs) - pad, max(xs) + pad), (min(ys) - pad, max(ys) + pad)
    return (-pad, config.MAP_WIDTH + pad), (-pad, config.MAP_HEIGHT + pad)


def _obstacle_bbox(obstacle: Obstacle) -> tuple[float, float, float, float]:
    """Trả về ``(xmin, xmax, ymin, ymax)`` của một vật cản, đã giãn nở hoặc thô."""
    if obstacle["type"] == "circle":
        (cx, cy), r = obstacle["center"], obstacle["radius"]
        return (cx - r, cx + r, cy - r, cy + r)
    xs = [p[0] for p in obstacle["polygon"]]
    ys = [p[1] for p in obstacle["polygon"]]
    return (min(xs), max(xs), min(ys), max(ys))


def _content_extents(
    scenario: Scenario | None,
    preprocessed: PreprocessedScenario | None = None,
    result: PlanResultView | None = None,
    pad_frac: float = 0.08,
    min_pad: float = 1000.0,
    obstacle_gate_frac: float = 1.0,
) -> Extents:
    """Tính toán giới hạn trục tọa độ tự động khớp theo nội dung đường bay.

    Hai lượt duyệt: trước hết là TRỌNG TÂM nhiệm vụ (CHỈ gồm start/goal, các
    waypoint trung gian và quỹ đạo bay thực tế), sau đó là tất cả những gì GẦN
    vùng trọng tâm -- các vật cản có bbox giao cắt, và các đỉnh biên safezone
    nằm trong vùng trọng tâm mở rộng thêm ``obstacle_gate_frac * core_span``.
    Điều này giữ cho đường bay luôn nổi bật ngay cả khi kịch bản mang một
    safezone bao bọc khổng lồ (hình tứ giác trải rộng toàn bản đồ) hoặc một
    cụm vật cản xa hàng trăm km ngoài đường bay: những đối tượng đó vẫn được vẽ,
    chỉ là bị cắt theo khung nhìn.

    Args:
        scenario: Kịch bản đang được vẽ, hoặc ``None``.
        preprocessed: Kịch bản tiền xử lý cung cấp các điểm đầu cuối và vật cản.
        result: Kết quả kế hoạch cung cấp đường bay thực tế.
        pad_frac: Tỷ lệ đệm theo chiều rộng khung hình.
        min_pad: Khoảng đệm tối thiểu (m).
        obstacle_gate_frac: Khoảng cách vượt ngoài vùng trọng tâm (tính theo độ
            rộng trọng tâm) mà một vật cản vẫn có thể làm mở rộng khung hình.

    Returns:
        Giới hạn trục tọa độ, mặc định dùng hình chữ nhật ``config.MAP_WIDTH/HEIGHT``
        khi không có vùng trọng tâm nhiệm vụ để căn khung.
    """
    xs: list[float] = []
    ys: list[float] = []

    def add(p: Point) -> None:
        """Đưa một điểm vào khung hình."""
        xs.append(p[0])
        ys.append(p[1])

    # --- Lượt 1: Vùng trọng tâm nhiệm vụ (điểm đầu/cuối, waypoint, quỹ đạo) ---
    if preprocessed:
        start_pos = preprocessed.get("start_pos")
        if start_pos is not None:
            add(start_pos)
        goal_pos = preprocessed.get("goal_pos")
        if goal_pos is not None:
            add(goal_pos)
        add(preprocessed["start_state"]["waypoint"])
        add(preprocessed["goal_state"]["waypoint"])
    if result and result.get("path"):
        for wp, _heading in result["path"] or []:
            add(wp)

    if not xs:
        return (
            (-min_pad, config.MAP_WIDTH + min_pad),
            (-min_pad, config.MAP_HEIGHT + min_pad),
        )

    # --- Cổng lọc (Gate): bbox trọng tâm mở rộng thêm gate * core_span ---
    cxmin, cxmax, cymin, cymax = min(xs), max(xs), min(ys), max(ys)
    gate = obstacle_gate_frac * max(cxmax - cxmin, cymax - cymin, 1.0)
    gxmin, gxmax = cxmin - gate, cxmax + gate
    gymin, gymax = cymin - gate, cymax + gate

    # --- Lượt 2a: Các chướng ngại vật có bbox giao cắt với cổng lọc ---
    obstacles: list[Obstacle] = (
        list(preprocessed.get("obstacles", [])) if preprocessed else []
    )
    if scenario:
        obstacles.extend(
            {"type": "polygon", "polygon": island}
            for island in scenario.get("islands", [])
        )
        obstacles.extend(
            {"type": "circle", "center": center, "radius": radius}
            for center, radius in scenario.get("dynamic_obstacles", [])
        )
    for obstacle in obstacles:
        oxmin, oxmax, oymin, oymax = _obstacle_bbox(obstacle)
        if oxmax >= gxmin and oxmin <= gxmax and oymax >= gymin and oymin <= gymax:
            add((oxmin, oymin))
            add((oxmax, oymax))

    # --- Lượt 2b: Các đỉnh biên safezone nằm trong phạm vi cổng lọc ---
    # Hiển thị hành lang bay thực tế gần đường bay; vùng an toàn safezone bao quanh
    # khổng lồ không đóng góp đỉnh nào trong cổng lọc, tránh làm phình khung hình.
    for safezone in (scenario.get("safezones") or []) if scenario else []:
        for vx, vy in safezone:
            if gxmin <= vx <= gxmax and gymin <= vy <= gymax:
                add((vx, vy))

    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    span = max(maxx - minx, maxy - miny, 1.0)
    pad = max(min_pad, pad_frac * span)
    return (minx - pad, maxx + pad), (miny - pad, maxy + pad)


def _point_at_arclength(pts: Sequence[Point], s: float) -> Point:
    """Tìm điểm tại chiều dài cung ``s`` dọc theo đường gấp khúc polyline.

    Args:
        pts: Danh sách các điểm của đường gấp khúc.
        s: Chiều dài cung tính từ điểm bắt đầu (m); được kẹp trong giới hạn 2 đầu mút.

    Returns:
        Tọa độ điểm nội suy.
    """
    if s <= 0:
        return pts[0]
    acc = 0.0
    for a, b in pairwise(pts):
        d = math.dist(a, b)
        if acc + d >= s:
            t = (s - acc) / d if d > 0 else 0.0
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        acc += d
    return pts[-1]


def _draw_operating_area(ax: Axes, scenario: Scenario) -> None:
    """Vẽ từng đa giác safezone, hoặc hình chữ nhật toàn bản đồ nếu không có."""
    safezones = scenario.get("safezones")
    if safezones:
        for safezone in safezones:
            ax.add_patch(
                MplPolygon(
                    safezone,
                    closed=True,
                    fill=True,
                    facecolor="lightblue",
                    edgecolor="blue",
                    linewidth=2,
                    alpha=0.3,
                )
            )
    else:
        ax.add_patch(
            Rectangle(
                (0, 0),
                config.MAP_WIDTH,
                config.MAP_HEIGHT,
                fill=True,
                facecolor="lightblue",
                edgecolor="blue",
                linewidth=2,
                alpha=0.3,
            )
        )


def _draw_obstacles(
    ax: Axes, scenario: Scenario, preprocessed: PreprocessedScenario
) -> None:
    """Vẽ các vật cản thô, và vùng đệm an toàn giãn nở dưới dạng nét đứt."""
    for island in scenario.get("islands", []):
        ax.add_patch(
            MplPolygon(
                island,
                fill=True,
                facecolor="saddlebrown",
                edgecolor="darkred",
                linewidth=1.5,
                alpha=0.7,
            )
        )

    if not config.PLOT_BUFFER_ZONES:
        return
    for obstacle in preprocessed.get("obstacles", []):
        if obstacle["type"] == "circle":
            ax.add_patch(
                MplCircle(
                    obstacle["center"],
                    obstacle["radius"],
                    fill=False,
                    edgecolor="darkred",
                    linewidth=1,
                    linestyle="--",
                    alpha=0.5,
                )
            )
        else:
            ax.add_patch(
                MplPolygon(
                    obstacle["polygon"],
                    fill=False,
                    edgecolor="darkred",
                    linewidth=1,
                    linestyle="--",
                    alpha=0.5,
                )
            )


def _draw_endpoints(ax: Axes, preprocessed: PreprocessedScenario) -> None:
    """Đánh dấu điểm cất cánh O, đích T và hướng bay của hai đoạn thẳng bắt buộc.

    Kịch bản thiếu điểm đầu cuối sẽ không có gì để vẽ, nên được bỏ qua
    thay vì gán mặc định -- điểm giữ chỗ có thể vẽ một điểm tại gốc tọa độ (0, 0)
    và bị hiểu nhầm là dữ liệu thực.
    """
    takeoff = preprocessed.get("start_pos")
    target = preprocessed.get("goal_pos")
    if takeoff is None or target is None:
        return

    ax.plot(
        takeoff[0], takeoff[1], "go", markersize=12, label="Takeoff Point O", zorder=5
    )

    first_wp = preprocessed["start_state"]["waypoint"]
    ax.arrow(
        takeoff[0],
        takeoff[1],
        first_wp[0] - takeoff[0],
        first_wp[1] - takeoff[1],
        head_width=500,
        head_length=500,
        fc="green",
        ec="green",
        alpha=0.3,
    )

    ax.plot(target[0], target[1], "r*", markersize=20, label="Goal T", zorder=5)

    last_wp = preprocessed["goal_state"]["waypoint"]
    ax.arrow(
        last_wp[0],
        last_wp[1],
        target[0] - last_wp[0],
        target[1] - last_wp[1],
        head_width=500,
        head_length=500,
        fc="red",
        ec="red",
        alpha=0.3,
    )


def _draw_waypoints_only(ax: Axes, waypoints: Sequence[Point]) -> None:
    """Vẽ đường bay chỉ gồm các đoạn thẳng thô; cơ chế dự phòng khi lấy mẫu thất bại."""
    for i in range(len(waypoints) - 1):
        ax.plot(
            [waypoints[i][0], waypoints[i + 1][0]],
            [waypoints[i][1], waypoints[i + 1][1]],
            "b-",
            linewidth=2.5,
            label="Trajectory" if i == 0 else "",
        )


def _draw_trajectory(
    ax: Axes,
    preprocessed: PreprocessedScenario,
    result: PlanResultView,
    trajectory_mode: RenderMode,
) -> None:
    """Vẽ quỹ đạo bay thực tế: các đoạn thẳng nối với cung lượn fillet bán kính R.

    Đây là mô hình kinodynamic thực tế của bộ lập kế hoạch. Nó thay thế bộ kết
    xuất Dubins cũ vốn từng làm rơi các phân đoạn (LRL/RRL không sinh mẫu),
    khiến đường bay bị nhảy cóc giữa các waypoint.

    Args:
        ax: Trục tọa độ matplotlib để vẽ.
        preprocessed: Kịch bản tiền xử lý cung cấp bán kính R và các điểm đầu cuối.
        result: Kết quả kế hoạch cung cấp đường bay.
        trajectory_mode: Đoạn thẳng 'straight' hoặc cung lượn 'dubins'.
    """
    path = result["path"] or []
    waypoints = [wp for wp, _heading in path]

    turn_radius = preprocessed.get("turn_radius", config.R)
    # Bao phủ toàn bộ nhiệm vụ O..T (đường bay của planner chỉ gồm W_1..W_{n-1}).
    full = sampling.build_full_path(path, preprocessed)
    samples = sampling.sample_trajectory(full, turn_radius, mode=trajectory_mode)
    if len(samples) < 2:
        _draw_waypoints_only(ax, waypoints)
        return

    label = "Dubins Trajectory" if trajectory_mode == "dubins" else "Straight Segments"
    ax.plot(
        [s[0] for s in samples],
        [s[1] for s in samples],
        "b-",
        linewidth=3.0,
        label=label,
        alpha=0.9,
        zorder=3,
    )

    label_every = max(1, len(waypoints) // 5)
    for i, wp in enumerate(waypoints):
        ax.plot(wp[0], wp[1], "bo", markersize=8, alpha=0.7, zorder=4)
        if i % label_every == 0:
            ax.text(wp[0] + 300, wp[1] + 300, f"W{i}", fontsize=9, alpha=0.6)

    # Đánh dấu vị trí bắt đầu và kết thúc của từng cung lượn (chấm nhỏ).
    if trajectory_mode == "dubins":
        for j, turn in enumerate(sampling.turn_markers(full, turn_radius)):
            ax.plot(
                *turn["start"],
                "o",
                color="lime",
                markersize=4,
                zorder=5,
                label="Turn start" if j == 0 else None,
            )
            ax.plot(
                *turn["end"],
                "o",
                color="magenta",
                markersize=4,
                zorder=5,
                label="Turn end" if j == 0 else None,
            )

    # Đánh dấu các mốc đoạn thẳng bắt buộc TRÊN quỹ đạo bay: L0 sau O (kết thúc
    # chặng thẳng cất cánh) và d_ss trước T (bắt đầu chặng tiếp cận khóa mục tiêu),
    # cả hai đều được xác định theo chiều dài cung thay vì theo chỉ số waypoint.
    l0 = preprocessed["start_state"].get("straight_length", config.L0)
    dss = preprocessed["goal_state"].get("engagement_distance", config.DSS)
    flown_len = sum(math.dist(a, b) for a, b in pairwise(samples))
    ax.plot(
        *_point_at_arclength(samples, l0),
        "g^",
        markersize=10,
        zorder=5,
        label="L₀ point",
    )
    ax.plot(
        *_point_at_arclength(samples, flown_len - dss),
        "rs",
        markersize=10,
        zorder=5,
        label="d_ss point",
    )


def _info_footer(
    scenario: Scenario,
    preprocessed: PreprocessedScenario,
    result: PlanResultView | None,
) -> str:
    """Xây dựng chuỗi văn bản tóm tắt thông số và kết quả hiển thị dưới chân bản đồ.

    Các tham số lấy từ kịch bản tiền xử lý -- các giá trị mà bộ lập kế hoạch
    thực sự sử dụng -- không lấy từ ``config``.

    Args:
        scenario: Kịch bản đang được vẽ.
        preprocessed: Kịch bản tiền xử lý.
        result: Kết quả kế hoạch, nếu đã chạy lập kế hoạch.

    Returns:
        Chuỗi văn bản footer, một hoặc hai dòng.
    """
    turn_radius = preprocessed.get("turn_radius", config.R)
    alpha_deg = math.degrees(preprocessed.get("alpha_max_rad", config.ALPHA_MAX_RAD))
    l0 = preprocessed["start_state"].get("straight_length", config.L0)
    dss = preprocessed["goal_state"].get("engagement_distance", config.DSS)
    text = (
        f"R = {turn_radius:.0f}m | α_max = {alpha_deg:.1f}° | L₀ = {l0:.0f}m"
        f" | d_ss = {dss:.0f}m"
        f" | Islands: {len(scenario.get('islands', []))}"
        f" | Dynamic Obstacles: {len(scenario.get('dynamic_obstacles', []))}"
    )
    if result is None:
        return text

    stats = result.get("stats")
    iterations = stats.get("iterations", 0) if stats else 0
    status = "✓ SUCCESS" if result.get("is_success", False) else "✗ FAILED"
    budget_s = (
        stats.get("time_budget_s", config.TIME_BUDGET_S)
        if stats
        else config.TIME_BUDGET_S
    )
    cut = " (budget)" if stats and stats.get("is_budget_bound") else ""
    text += f"\n{status} | Iter: {iterations} in <= {budget_s:g}s{cut}"

    path = result.get("path")
    if path:
        # Tổng khoảng cách bay trên TOÀN BỘ nhiệm vụ O -> W1 ... W_{n-1} -> T
        # (các đoạn thẳng, cùng phép đo mà performance_eval sử dụng).
        full_mission = sampling.build_full_path(path, preprocessed)
        total_km = (
            sum(
                math.dist(full_mission[i][0], full_mission[i + 1][0])
                for i in range(len(full_mission) - 1)
            )
            / 1000.0
        )
        text += f" | Waypoints: {len(path)} | Total distance: {total_km:.1f} km"
    return text


def plot_scenario(
    scenario: Scenario,
    preprocessed: PreprocessedScenario,
    result: PlanResultView | None = None,
    title: str = "Mission Scenario",
    save_path: str | None = None,
    figsize: tuple[float, float] = (14, 12),
    trajectory_mode: RenderMode = "dubins",
    fit: Fit = "map",
) -> Figure:
    """Vẽ đồ thị kịch bản nhiệm vụ và quỹ đạo đường bay đã lập kế hoạch.

    Args:
        scenario: Kịch bản gốc từ :mod:`path_planning.scenario.generator`.
        preprocessed: Kịch bản tiền xử lý từ
            :func:`path_planning.scenario.preprocessing.prepare_scenario`.
        result: Kết quả lập kế hoạch; nếu bỏ trống chỉ vẽ kịch bản.
        title: Tiêu đề biểu đồ.
        save_path: Đường dẫn lưu ảnh; đóng figure sau khi lưu nếu có.
        figsize: Kích thước biểu đồ (inch).
        trajectory_mode: Đoạn thẳng 'straight' hoặc cung lượn 'dubins'.
        fit: Chế độ căn khung hình. ``'map'`` giữ khung toàn bộ bản đồ/safezone;
            ``'content'`` tự căn chỉnh theo quỹ đạo bay và vật cản lân cận.

    Returns:
        Đối tượng matplotlib figure.
    """
    fig, ax = plt.subplots(figsize=figsize, dpi=config.FIGURE_DPI)

    xlim, ylim = (
        _content_extents(scenario, preprocessed, result)
        if fit == "content"
        else _plot_extents(scenario)
    )
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    _draw_operating_area(ax, scenario)
    _draw_obstacles(ax, scenario, preprocessed)
    if config.PLOT_START_END_MARKERS:
        _draw_endpoints(ax, preprocessed)

    if result and result.get("path"):
        try:
            _draw_trajectory(ax, preprocessed, result, trajectory_mode)
        except Exception as exc:
            logger.debug(
                "Arc interpolation failed, degrading to straight line.",
                exc_info=exc,
            )
            # Hạ cấp xuống vẽ các đoạn thẳng thô thay vì làm mất toàn bộ biểu đồ:
            # đoạn này chạy trong batch harness, nơi một kịch bản không vẽ được
            # không được phép làm dừng mười lăm kịch bản còn lại.
            waypoints = [wp for wp, _heading in result["path"] or []]
            _draw_waypoints_only(ax, waypoints)
            for wp in waypoints:
                ax.plot(wp[0], wp[1], "bo", markersize=6, alpha=0.7)

    ax.set_xlabel("East (m)", fontsize=11)
    ax.set_ylabel("North (m)", fontsize=11)
    ax.set_title(title, fontsize=13, fontweight="bold")

    # Chú giải đặt bên trong trục tọa độ, góc trên bên trái; hộp thông tin nằm
    # dưới trục dạng footer để không bao giờ bị đè lên nhau. Khử trùng lặp theo
    # nhãn vì mỗi điểm đánh dấu góc rẽ sẽ sinh ra một mục riêng.
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles, strict=True))
    ax.legend(
        by_label.values(), by_label.keys(), loc="upper left", fontsize=9, framealpha=0.9
    )

    fig.text(
        0.5,
        0.01,
        _info_footer(scenario, preprocessed, result),
        ha="center",
        va="bottom",
        fontsize=9,
        family="monospace",
        bbox={"boxstyle": "round", "facecolor": "wheat", "alpha": 0.8},
    )

    plt.tight_layout(rect=(0, 0.06, 1, 1))

    if save_path:
        plt.savefig(save_path, dpi=config.FIGURE_DPI, bbox_inches="tight")
        logger.info(f"Figure saved to {save_path}")
        plt.close()

    return fig
