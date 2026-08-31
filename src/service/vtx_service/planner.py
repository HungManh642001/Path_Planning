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

from path_planning import planner
from path_planning.geometry import spatial
from path_planning.scenario import preprocessing
from path_planning.trajectory import mission_path
from service.vtx_service.angles import math_rad_to_bearing_deg
from service.vtx_service.map_file import PreloadedMap
from service.vtx_service.messages import (
    IDL_VERSION,
    PlanReply,
    PlanRequest,
    PlanStatus,
    SearchStats,
    Waypoint,
)
from service.vtx_service.runtime import (
    config_hash,
    effective_time_budget_s,
    planner_version,
)
from service.vtx_service.scenario_builder import build_scenario


_REASON_TO_STATUS = {
    "no_path": PlanStatus.NO_PATH,
    "start_leg_blocked": PlanStatus.START_LEG_BLOCKED,
    "goal_leg_blocked": PlanStatus.GOAL_LEG_BLOCKED,
}
"""Tập lý do có mã riêng. Mọi chuỗi khác đến từ oracle và mang tham số, nên nó
đi nguyên văn vào ``detail`` thay vì bị ép vào enum làm mất thông tin."""


def plan(request: PlanRequest, preloaded: PreloadedMap | None = None) -> PlanReply:
    """Điểm vào duy nhất cho một yêu cầu lập kế hoạch đường bay."""
    started = time.perf_counter()
    if request.idl_version != IDL_VERSION:
        return _refusal(
            request,
            f"sai phiên bản idl_version (nhận {request.idl_version}, cần {IDL_VERSION})",
            started,
        )

    if request.use_preloaded_map:
        if preloaded is None:
            return _refusal(
                request,
                "request đòi use_preloaded_map nhưng service không có bản đồ nền",
                started,
            )
        request = preloaded.merged_into(request)

    try:
        preprocessed = preprocessing.prepare_scenario(
            build_scenario(request),
            turn_radius=request.limits.turn_radius_m,
            l0=request.limits.l0_m,
            dss=request.limits.dss_m,
            safe_margin=request.limits.safe_margin_m,
            alpha_max_rad=math.radians(request.limits.alpha_max_deg),
        )
    except (ValueError, KeyError, TypeError) as exc:
        return _refusal(request, f"hình học không dựng được: {exc}", started)

    # Ngân sách của client đi THẲNG vào thuật toán làm điều kiện dừng duy
    # nhất; effective_time_budget_s quyết định nó được nhận nguyên vẹn, rơi về
    # mặc định, hay bị kẹp. Reply báo cáo lại đúng con số này.
    budget_s = effective_time_budget_s(request.budget.time_budget_s)
    result = planner.plan_trajectory(preprocessed, time_budget_s=budget_s)

    status, detail = _classify(result)
    return PlanReply(
        request_id=request.request_id,
        idl_version=IDL_VERSION,
        status=status,
        detail=detail,
        waypoints=_waypoints_out(result, preprocessed),
        path_length_m=_planar_length(result, preprocessed),
        plan_wall_time_s=time.perf_counter() - started,
        applied_time_budget_s=budget_s,
        stats=_stats_out(result),
        planner_version=planner_version(),
        config_hash=config_hash(),
    )


def _classify(result: dict[str, Any]) -> tuple[PlanStatus, str]:
    """Ánh xạ kết quả planner sang trạng thái đối ngoại và phần diễn giải."""
    if result["is_success"]:
        return PlanStatus.OK, ""
    reason = result["failure_reason"] or ""
    return _REASON_TO_STATUS.get(reason, PlanStatus.ORACLE_REJECTED), reason


def _full_path(result: dict[str, Any], preprocessed: dict[str, Any]) -> list[Any]:
    """Đường bay đầy đủ ``O..T``, hoặc rỗng khi không có đường nào."""
    if not result["path"]:
        return []
    return mission_path.full_mission_path(result["path"], preprocessed)


def _waypoints_out(
    result: dict[str, Any], preprocessed: dict[str, Any]
) -> tuple[Waypoint, ...]:
    """Đưa đường bay về quy ước góc của client. Toạ độ không đổi."""
    return tuple(
        Waypoint(position=position, heading_deg=math_rad_to_bearing_deg(heading))
        for position, heading in _full_path(result, preprocessed)
    )


def _planar_length(result: dict[str, Any], preprocessed: dict[str, Any]) -> float:
    """Tổng chiều dài quỹ đạo bay thực tế gồm các đoạn thẳng và cung lượn Dubins."""
    full = _full_path(result, preprocessed)
    turn_radius = float(preprocessed.get("turn_radius", 0.0))
    return spatial.calculate_dubins_path_length(full, turn_radius)


def _stats_out(result: dict[str, Any]) -> SearchStats:
    """Đóng gói bộ đếm search, kèm cờ cho biết ngân sách có chạm trần không.

    ``is_budget_bound`` đến THẲNG từ planner. Trước đây service tự suy ra nó bằng
    cách so thời gian đo được ở ngoài với ngân sách - phép so đó tính cả phần
    làm mượt và phần oracle, nên một mission vừa kịp giờ vẫn có thể bị báo là
    chạm trần. Chỉ vòng lặp search biết nó dừng vì đồng hồ hay vì hết biên.
    """
    stats = result["stats"]
    return SearchStats(
        iterations=stats["iterations"],
        open_set_size=stats["open_set_size"],
        is_search_failed=stats["is_search_failed"],
        is_budget_bound=stats["is_budget_bound"],
    )


def _refusal(request: PlanRequest, detail: str, started: float) -> PlanReply:
    """Từ chối một request không hợp lệ, không chạy search.

    ``applied_time_budget_s`` ở đây là ngân sách MẶC ĐỊNH của service chứ không
    phải ngân sách giải được từ request: không có lần search nào chạy, nên nói
    "đã áp dụng X giây" cho một con số đến từ một request bị từ chối là bịa.
    """
    return PlanReply(
        request_id=request.request_id,
        idl_version=IDL_VERSION,
        status=PlanStatus.INVALID_REQUEST,
        detail=detail,
        waypoints=(),
        path_length_m=0.0,
        plan_wall_time_s=time.perf_counter() - started,
        applied_time_budget_s=effective_time_budget_s(),
        stats=SearchStats(0, 0, True, False),
        planner_version=planner_version(),
        config_hash=config_hash(),
    )
