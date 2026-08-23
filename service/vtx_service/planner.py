"""Điểm vào của service: một mission vào, một đường bay ra.

Đây là chỗ DUY NHẤT trong service gọi tới planner. Mọi thứ nó dùng từ ``core/``
đều là hàm công khai đang tồn tại, không có bản sao nào: ``prepare_scenario``
chuẩn bị bài toán, ``plan_trajectory`` giải nó, ``full_mission_path`` ghép hai
đầu mút vào. Thuật toán đổi thì đường đi này đổi theo mà không cần sửa gì.
"""

from __future__ import annotations

import math
import time
from typing import Any

import core.kinodynamic_astar_v0 as astar
import core.mission as mission
import core.preprocessing as prep

from vtx_service.angles import math_rad_to_bearing_deg
from vtx_service.map_file import PreloadedMap
from vtx_service.messages import (
    IDL_VERSION,
    PlanReply,
    PlanRequest,
    PlanStatus,
    SearchStats,
    Waypoint,
)
from vtx_service.runtime import (
    config_hash,
    effective_max_iterations,
    effective_time_budget_s,
    planner_version,
)
from vtx_service.scenario_builder import build_scenario

_REASON_TO_STATUS = {
    "no_path": PlanStatus.NO_PATH,
    "start_leg_blocked": PlanStatus.START_LEG_BLOCKED,
    "goal_leg_blocked": PlanStatus.GOAL_LEG_BLOCKED,
}
"""Tập lý do có mã riêng. Mọi chuỗi khác đến từ oracle và mang tham số, nên nó
đi nguyên văn vào ``detail`` thay vì bị ép vào enum làm mất thông tin."""


def plan(request: PlanRequest, preloaded: PreloadedMap | None = None) -> PlanReply:
    """Lập kế hoạch cho một mission.

    Args:
        request: Mission cần giải, toạ độ mét trong hệ Oxy.
        preloaded: Bản đồ nền tĩnh, hoặc ``None`` khi service không nạp bản đồ
            nào. Chỉ dùng khi ``request.use_preloaded_map`` bật.

    Returns:
        Đường bay đầy đủ ``O..T``, kèm trạng thái, bộ đếm search và nhận dạng
        phiên bản/cấu hình.
    """
    started = time.perf_counter()

    if request.idl_version != IDL_VERSION:
        return _refusal(request, f"idl_version {request.idl_version} != {IDL_VERSION}")

    if request.use_preloaded_map:
        if preloaded is None:
            return _refusal(
                request, "yêu cầu preloaded map nhưng service không nạp bản đồ nào"
            )
        request = preloaded.merged_into(request)

    try:
        preprocessed = prep.prepare_scenario(
            build_scenario(request),
            turn_radius=request.limits.turn_radius_m,
            l0=request.limits.l0_m,
            dss=request.limits.dss_m,
            safe_margin=request.limits.safe_margin_m,
            alpha_max_rad=math.radians(request.limits.alpha_max_deg),
        )
    except (ValueError, KeyError, TypeError) as exc:
        return _refusal(request, f"hình học không dựng được: {exc}")

    search_started = time.perf_counter()
    result = astar.plan_trajectory(preprocessed)
    search_elapsed = time.perf_counter() - search_started

    status, detail = _classify(result)
    return PlanReply(
        request_id=request.request_id,
        idl_version=IDL_VERSION,
        status=status,
        detail=detail,
        waypoints=_waypoints_out(result, preprocessed),
        path_length_m=_planar_length(result, preprocessed),
        plan_wall_time_s=time.perf_counter() - started,
        applied_time_budget_s=effective_time_budget_s(),
        stats=_stats_out(result, search_elapsed),
        planner_version=planner_version(),
        config_hash=config_hash(),
    )


def _classify(result: dict[str, Any]) -> tuple[PlanStatus, str]:
    """Ánh xạ kết quả planner sang trạng thái đối ngoại và phần diễn giải."""
    if result["success"]:
        return PlanStatus.OK, ""
    reason = result["failure_reason"] or ""
    return _REASON_TO_STATUS.get(reason, PlanStatus.ORACLE_REJECTED), reason


def _full_path(result: dict[str, Any], preprocessed: dict[str, Any]) -> list[Any]:
    """Đường bay đầy đủ ``O..T``, hoặc rỗng khi không có đường nào."""
    if not result["path"]:
        return []
    return mission.full_mission_path(result["path"], preprocessed)


def _waypoints_out(
    result: dict[str, Any], preprocessed: dict[str, Any]
) -> tuple[Waypoint, ...]:
    """Đưa đường bay về quy ước góc của client. Toạ độ không đổi."""
    return tuple(
        Waypoint(position=position, heading_deg=math_rad_to_bearing_deg(heading))
        for position, heading in _full_path(result, preprocessed)
    )


def _planar_length(result: dict[str, Any], preprocessed: dict[str, Any]) -> float:
    """Tổng chiều dài các dây cung.

    Cùng công thức ``scripts/ab_planners.py`` dùng, nên số liệu so sánh được với
    các benchmark đã ghi.
    """
    full = _full_path(result, preprocessed)
    return sum(math.dist(full[i][0], full[i + 1][0]) for i in range(len(full) - 1))


def _stats_out(result: dict[str, Any], search_elapsed: float) -> SearchStats:
    """Đóng gói bộ đếm search, kèm cờ cho biết ngân sách có chạm trần không."""
    stats = result["stats"]
    budget_s = effective_time_budget_s()
    budget_bound = stats["iterations"] >= stats["max_iterations"] or (
        budget_s > 0.0 and search_elapsed >= budget_s
    )
    return SearchStats(
        iterations=stats["iterations"],
        max_iterations=stats["max_iterations"],
        open_set_size=stats["open_set_size"],
        search_failed=stats["search_failed"],
        budget_bound=budget_bound,
    )


def _refusal(request: PlanRequest, detail: str) -> PlanReply:
    """Từ chối một request không hợp lệ, không chạy search."""
    return PlanReply(
        request_id=request.request_id,
        idl_version=IDL_VERSION,
        status=PlanStatus.INVALID_REQUEST,
        detail=detail,
        waypoints=(),
        path_length_m=0.0,
        plan_wall_time_s=0.0,
        applied_time_budget_s=effective_time_budget_s(),
        stats=SearchStats(0, effective_max_iterations(), 0, True, False),
        planner_version=planner_version(),
        config_hash=config_hash(),
    )
