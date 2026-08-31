"""Trực quan hóa kịch bản nhiệm vụ, chướng ngại vật và quỹ đạo bay.

Consumes the planner's output; nothing here feeds back into the search, so the
dependency runs render -> core and never the reverse.

Note on typing: matplotlib ships only partial type information, so this package
is checked in pyright's `standard` mode rather than `strict` (see the
executionEnvironments block in pyproject.toml). Every function here is still
fully annotated; what is relaxed is only the demand that matplotlib's own
signatures be fully known.
"""

from __future__ import annotations

import logging
import math
from itertools import pairwise
from typing import TYPE_CHECKING, Literal

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.patches import Circle as MplCircle, Polygon as MplPolygon, Rectangle

from path_planning import config
from path_planning.render import trajectory as tr


logger = logging.getLogger(__name__)


if TYPE_CHECKING:
    from collections.abc import Sequence

    from matplotlib.figure import Figure

    from path_planning.render.sampling import RenderMode
    from path_planning.types import (
        Obstacle,
        PlanResultView,
        Point,
        PreprocessedScenario,
        Scenario,
    )

Extents = tuple[tuple[float, float], tuple[float, float]]
"""Axis limits as ``((xmin, xmax), (ymin, ymax))``."""

Fit = Literal["map", "content"]
"""How :func:`plot_scenario` frames the view."""


def _plot_extents(scenario: Scenario | None, pad: float = 2000.0) -> Extents:
    """Tính toán giới hạn trục tọa độ cho kịch bản theo góc nhìn toàn bản đồ.

    Args:
        scenario: The scenario being drawn, or ``None``.
        pad: Margin added around the bounding box (m).

    Returns:
        The axis limits: the bounding box of all safezone polygons plus padding
        when present, else the global ``config.MAP_WIDTH/HEIGHT`` rectangle.
    """
    safezones = scenario.get("safezones") if scenario else None
    if safezones:
        xs = [p[0] for poly in safezones for p in poly]
        ys = [p[1] for poly in safezones for p in poly]
        return (min(xs) - pad, max(xs) + pad), (min(ys) - pad, max(ys) + pad)
    return (-pad, config.MAP_WIDTH + pad), (-pad, config.MAP_HEIGHT + pad)


def _obstacle_bbox(obstacle: Obstacle) -> tuple[float, float, float, float]:
    """Return ``(xmin, xmax, ymin, ymax)`` of one obstacle, inflated or raw."""
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

    Two passes: first the mission CORE (start/goal, interior waypoints and the
    flown path ONLY), then everything else NEAR the core -- obstacles whose bbox
    intersects, and safezone-boundary vertices that fall within, the core
    expanded by ``obstacle_gate_frac * core_span``. This keeps the flight
    prominent even when the scenario carries a giant enclosing safezone (a quad
    spanning the whole map) or a far-off obstacle cluster hundreds of km
    off-route: those still get drawn, just clipped.

    Args:
        scenario: The scenario being drawn, or ``None``.
        preprocessed: The prepared scenario supplying endpoints and obstacles.
        result: The plan result supplying the flown path.
        pad_frac: Margin as a fraction of the framed span.
        min_pad: Minimum margin (m).
        obstacle_gate_frac: How far beyond the core, in core spans, an obstacle
            may sit and still widen the frame.

    Returns:
        The axis limits, falling back to the ``config.MAP_WIDTH/HEIGHT``
        rectangle when there is no mission core to frame.
    """
    xs: list[float] = []
    ys: list[float] = []

    def add(p: Point) -> None:
        """Include one point in the frame."""
        xs.append(p[0])
        ys.append(p[1])

    # --- Pass 1: mission core (endpoints, interior waypoints, flown path) ---
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

    # --- Gate: the core bbox expanded by obstacle_gate_frac * core_span ---
    cxmin, cxmax, cymin, cymax = min(xs), max(xs), min(ys), max(ys)
    gate = obstacle_gate_frac * max(cxmax - cxmin, cymax - cymin, 1.0)
    gxmin, gxmax = cxmin - gate, cxmax + gate
    gymin, gymax = cymin - gate, cymax + gate

    # --- Pass 2a: obstacles whose bbox intersects the gate ---
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

    # --- Pass 2b: safezone-boundary vertices that fall within the gate ---
    # A real operating corridor near the flight is shown; a giant enclosing
    # safezone contributes no in-gate vertices, so it does not blow up the frame.
    for safezone in (scenario.get("safezones") or []) if scenario else []:
        for vx, vy in safezone:
            if gxmin <= vx <= gxmax and gymin <= vy <= gymax:
                add((vx, vy))

    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    span = max(maxx - minx, maxy - miny, 1.0)
    pad = max(min_pad, pad_frac * span)
    return (minx - pad, maxx + pad), (miny - pad, maxy + pad)


def _point_at_arclength(pts: Sequence[Point], s: float) -> Point:
    """Find the point at arc length ``s`` along a polyline.

    Args:
        pts: The polyline points.
        s: Arc length from the start (m); clamped to the polyline's ends.

    Returns:
        The interpolated point.
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
    """Draw each safezone polygon, or the full map rectangle when none is given."""
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
    """Draw the raw obstacles, and the inflated buffer zones as dashed outlines."""
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
    """Mark the takeoff point, the goal, and the two mandatory leg directions.

    A scenario without both endpoints has nothing to mark, so it is skipped
    rather than defaulted -- a placeholder would draw a marker at the map origin
    and read as real data.
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
    """Draw the path as bare straight segments; the fallback when sampling fails."""
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
    """Draw the flown trajectory: straight legs plus radius-R turn arcs.

    This is the planner's actual kinodynamic model. It replaced a legacy Dubins
    renderer whose placeholder sampler dropped whole segments (LRL/RRL produced
    no samples), so the line appeared to jump between waypoints.

    Args:
        ax: The axes to draw on.
        preprocessed: The prepared scenario supplying R and the endpoints.
        result: The plan result supplying the path.
        trajectory_mode: Straight legs or filleted arcs.
    """
    path = result["path"] or []
    waypoints = [wp for wp, _heading in path]

    turn_radius = preprocessed.get("turn_radius", config.R)
    # Span the full mission O..T (the planner path covers only W_1..W_{n-1}).
    full = tr.build_full_path(path, preprocessed)
    samples = tr.sample_trajectory(full, turn_radius, mode=trajectory_mode)
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

    # Mark where each turn arc begins and ends (small dots).
    if trajectory_mode == "dubins":
        for j, turn in enumerate(tr.turn_markers(full, turn_radius)):
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

    # Mark the mandatory-straight endpoints ON the flown path: L0 after O (end
    # of the takeoff straight) and d_ss before T (start of the engagement
    # run-in), both located by arc length rather than by waypoint index.
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

    Parameters come from the preprocessed scenario -- the values the planner
    actually used -- not from ``config``.

    Args:
        scenario: The scenario being drawn.
        preprocessed: The prepared scenario.
        result: The plan result, if planning ran.

    Returns:
        The footer text, one or two lines.
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
        # Total flown distance over the FULL mission O -> W1 ... W_{n-1} -> T
        # (straight chords, the same measure performance_eval uses).
        full_mission = tr.build_full_path(path, preprocessed)
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
        scenario: The original scenario from :mod:`core.map_generator`.
        preprocessed: The prepared scenario from
            :func:`core.preprocessing.prepare_scenario`.
        result: The plan result; omitted, only the scenario is drawn.
        title: Figure title.
        save_path: Where to save the figure; the figure is closed after saving.
        figsize: Figure size in inches.
        trajectory_mode: Straight legs or filleted arcs.
        fit: View framing. ``'map'`` keeps the legacy full-map / safezone-bbox
            view; ``'content'`` auto-fits the axes to the flown path, endpoints
            and nearby obstacles so a small mission is easy to follow inside a
            large map.

    Returns:
        The matplotlib figure.
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
            # Degrade to bare segments rather than losing the whole plot: this
            # runs inside the batch harness, where one unplottable scenario must
            # not abort the other fifteen.
            waypoints = [wp for wp, _heading in result["path"] or []]
            _draw_waypoints_only(ax, waypoints)
            for wp in waypoints:
                ax.plot(wp[0], wp[1], "bo", markersize=6, alpha=0.7)

    ax.set_xlabel("East (m)", fontsize=11)
    ax.set_ylabel("North (m)", fontsize=11)
    ax.set_title(title, fontsize=13, fontweight="bold")

    # Legend inside the axes, upper-left; the info box lives below the axes as a
    # figure footer so the two never overlap. Deduplicated by label because the
    # per-turn markers would otherwise contribute one entry each.
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
