# pyright: reportMissingTypeArgument=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownParameterType=false
"""Động cơ điều phối thực thi kịch bản kiểm thử (Local / NATS).

Cung cấp lớp :class:`ExecutionDriver` và cấu trúc dữ liệu kết quả :class:`QAResult`
hỗ trợ chạy kịch bản trực tiếp bằng thư viện nội bộ hoặc gửi qua microservice NATS,
đồng thời tự động thẩm định kết quả qua Validation Oracle độc lập.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

from path_planning import config
from path_planning.geometry import spatial
from path_planning.planner import plan_trajectory
from path_planning.scenario.preprocessing import prepare_scenario
from path_planning.validation import oracle
from path_planning.validation.oracle import ValidationResult
from service.vtx_service.angles import (
    bearing_deg_to_math_rad,
    math_rad_to_bearing_deg,
)
from service.vtx_service.messages import (
    IDL_VERSION,
    Circle,
    PlanRequest,
    SearchBudget,
    VehicleLimits,
)
from service.vtx_service.transport import (
    DEFAULT_NATS_SERVER,
    DEFAULT_SUBJECT,
    NatsClient,
)


if TYPE_CHECKING:
    from path_planning.types import Scenario

_REASON_TO_STATUS: dict[str, str] = {
    "start_leg_blocked": "NO_PATH",
    "goal_leg_blocked": "NO_PATH",
    "no_path": "NO_PATH",
    "budget_bound_no_path": "NO_PATH",
    "timeout": "TIMEOUT",
    "invalid_request": "INVALID_REQUEST",
    "internal_error": "INTERNAL_ERROR",
}


def _classify_failure_status(reason: str | None) -> str:
    """Chuyển đổi failure_reason nội bộ sang chuỗi status chuẩn."""
    if not reason:
        return "FAILED"
    return _REASON_TO_STATUS.get(reason, "ORACLE_REJECTED")


def scenario_to_plan_request(
    scenario: Scenario,
    name: str,
    time_budget_s: float,
    limits: VehicleLimits | None = None,
) -> PlanRequest:
    """Chuyển đổi Scenario dict sang PlanRequest của microservice."""
    start_heading_rad = float(scenario["start_heading"])
    start_heading_deg = math_rad_to_bearing_deg(start_heading_rad)

    goal_heading = scenario.get("goal_heading")
    if goal_heading is None:
        goal_heading_deg = 0.0
        is_goal_heading_free = True
    else:
        goal_heading_deg = math_rad_to_bearing_deg(float(goal_heading))
        is_goal_heading_free = False

    islands_raw = scenario.get("islands") or []
    islands = tuple(
        tuple((float(p[0]), float(p[1])) for p in poly) for poly in islands_raw
    )

    dynamic_obstacles_raw = scenario.get("dynamic_obstacles") or []
    dynamic_obstacles = tuple(
        Circle(center=(float(center[0]), float(center[1])), radius_m=float(radius))
        for center, radius in dynamic_obstacles_raw
    )

    safezones_raw = scenario.get("safezones") or []
    safezones = tuple(
        tuple((float(p[0]), float(p[1])) for p in zone) for zone in safezones_raw
    )

    req_id = name.encode("utf-8")[:16].ljust(16, b"\x00")

    if limits is None:
        limits = VehicleLimits(
            turn_radius_m=config.R,
            l0_m=config.L0,
            dss_m=config.DSS,
            safe_margin_m=config.SAFE_MARGIN,
            alpha_max_deg=config.ALPHA_MAX,
        )
    budget = SearchBudget(time_budget_s=time_budget_s)

    return PlanRequest(
        request_id=req_id,
        idl_version=IDL_VERSION,
        start=(float(scenario["start"][0]), float(scenario["start"][1])),
        start_heading_deg=start_heading_deg,
        goal=(float(scenario["goal"][0]), float(scenario["goal"][1])),
        goal_heading_deg=goal_heading_deg,
        is_goal_heading_free=is_goal_heading_free,
        islands=islands,
        dynamic_obstacles=dynamic_obstacles,
        safezones=safezones,
        use_preloaded_map=False,
        limits=limits,
        budget=budget,
    )


_scenario_to_plan_request = scenario_to_plan_request


class ExecutionMode(str, Enum):
    """Chế độ thực thi thuật toán lập lịch."""

    LOCAL = "local"
    NATS = "nats"


@dataclass
class QAResult:
    """Kết quả kiểm thử toàn diện của một kịch bản lập lịch.

    Attributes:
        scenario_name: Tên hoặc mã định danh của kịch bản kiểm thử.
        status: Trạng thái trả về (OK, NO_PATH, TIMEOUT, INVALID_REQUEST, ...).
        is_success: Cờ thành công (True nếu tìm được đường bay hợp lệ).
        waypoints: Danh sách các waypoint ((x, y), heading_rad) của đường bay.
        path_length_m: Tổng chiều dài đường bay tính bằng mét (kèm cung lượn Dubins).
        wall_time_s: Thời gian thực thi thực tế (wall-clock time) tính bằng giây.
        applied_time_budget_s: Ngân sách thời gian được áp dụng cho thuật toán.
        iterations: Số bước lặp tìm kiếm A*.
        oracle_verdict: Kết quả kiểm tra độc lập từ Validation Oracle.
        error_detail: Chi tiết nguyên nhân thất bại nếu có.
        raw_response: Dữ liệu phản hồi thô (PlanResult dict hoặc PlanReply dataclass).
    """

    scenario_name: str
    status: str
    is_success: bool
    waypoints: list[tuple[tuple[float, float], float]]
    path_length_m: float
    wall_time_s: float
    applied_time_budget_s: float
    iterations: int
    oracle_verdict: ValidationResult
    error_detail: str | None = None
    raw_response: object = None


class ExecutionDriver:
    """Bộ điều phối thực thi kịch bản (Local domain core hoặc NATS microservice)."""

    def __init__(
        self,
        mode: ExecutionMode = ExecutionMode.LOCAL,
        nats_url: str = DEFAULT_NATS_SERVER,
        subject: str = DEFAULT_SUBJECT,
    ) -> None:
        """Khởi tạo ExecutionDriver.

        Args:
            mode: Chế độ chạy (LOCAL hoặc NATS).
            nats_url: Địa chỉ NATS server (dùng trong chế độ NATS).
            subject: Subject NATS gửi request (dùng trong chế độ NATS).
        """
        self.mode = mode
        self.nats_url = nats_url
        self.subject = subject

    def run_scenario(
        self,
        scenario: Scenario,
        name: str = "custom",
        time_budget_s: float = 15.0,
        limits: VehicleLimits | None = None,
        turn_radius: float | None = None,
        l0: float | None = None,
        dss: float | None = None,
        safe_margin: float | None = None,
        alpha_max_rad: float | None = None,
    ) -> QAResult:
        """Thực thi một kịch bản theo chế độ đã cấu hình.

        Args:
            scenario: Dữ liệu kịch bản nhiệm vụ (Scenario dict).
            name: Tên kịch bản hoặc mã định danh.
            time_budget_s: Ngân sách thời gian tối đa cho thuật toán tìm kiếm (giây).
            limits: Đối tượng VehicleLimits chứa ràng buộc động học.
            turn_radius: Bán kính quay vòng tối thiểu R (m).
            l0: Khoảng cách thẳng ổn định sau cất cánh L0 (m).
            dss: Khoảng cách thẳng tiếp cận khóa mục tiêu DSS (m).
            safe_margin: Khoảng cách đệm an toàn giãn nở vật cản (m).
            alpha_max_rad: Góc chuyển hướng tối đa (rad).

        Returns:
            QAResult chứa đường bay, thời gian, trạng thái và kết quả thẩm định.

        Raises:
            ValueError: Nếu chế độ thực thi không hợp lệ.
        """
        res_r = (
            turn_radius
            if turn_radius is not None
            else (limits.turn_radius_m if limits else config.R)
        )
        res_l0 = l0 if l0 is not None else (limits.l0_m if limits else config.L0)
        res_dss = dss if dss is not None else (limits.dss_m if limits else config.DSS)
        res_margin = (
            safe_margin
            if safe_margin is not None
            else (limits.safe_margin_m if limits else config.SAFE_MARGIN)
        )
        if alpha_max_rad is not None:
            res_alpha_rad = alpha_max_rad
            res_alpha_deg = math.degrees(alpha_max_rad)
        elif limits is not None:
            res_alpha_deg = limits.alpha_max_deg
            res_alpha_rad = math.radians(limits.alpha_max_deg)
        else:
            res_alpha_rad = config.ALPHA_MAX_RAD
            res_alpha_deg = config.ALPHA_MAX

        resolved_limits = VehicleLimits(
            turn_radius_m=res_r,
            l0_m=res_l0,
            dss_m=res_dss,
            safe_margin_m=res_margin,
            alpha_max_deg=res_alpha_deg,
        )

        if self.mode == ExecutionMode.LOCAL:
            return self._run_local(
                scenario,
                name=name,
                time_budget_s=time_budget_s,
                turn_radius=res_r,
                l0=res_l0,
                dss=res_dss,
                safe_margin=res_margin,
                alpha_max_rad=res_alpha_rad,
            )
        elif self.mode == ExecutionMode.NATS:
            return self._run_nats(
                scenario,
                name=name,
                time_budget_s=time_budget_s,
                limits=resolved_limits,
                turn_radius=res_r,
                l0=res_l0,
                dss=res_dss,
                safe_margin=res_margin,
                alpha_max_rad=res_alpha_rad,
            )
        else:
            raise ValueError(f"Unknown execution mode: {self.mode}")

    def _run_local(
        self,
        scenario: Scenario,
        name: str = "custom",
        time_budget_s: float = 15.0,
        turn_radius: float = config.R,
        l0: float = config.L0,
        dss: float = config.DSS,
        safe_margin: float = config.SAFE_MARGIN,
        alpha_max_rad: float = config.ALPHA_MAX_RAD,
    ) -> QAResult:
        """Thực thi kịch bản bằng hàm nội bộ trong thư viện Python (Local Mode)."""
        start_time = time.perf_counter()
        try:
            prep = prepare_scenario(
                scenario,
                turn_radius=turn_radius,
                l0=l0,
                dss=dss,
                safe_margin=safe_margin,
                alpha_max_rad=alpha_max_rad,
            )
            plan_res = plan_trajectory(prep, time_budget_s=time_budget_s)
            wall_time_s = time.perf_counter() - start_time
            is_success = bool(plan_res.get("is_success", False))
            raw_path = plan_res.get("path")
            waypoints: list[tuple[tuple[float, float], float]] = (
                list(raw_path) if raw_path else []
            )
            stats = plan_res.get("stats", {})
            iterations = int(stats.get("iterations", 0))
            applied_budget = float(stats.get("time_budget_s", time_budget_s))
            path_len = (
                spatial.calculate_dubins_path_length(waypoints, turn_radius)
                if waypoints
                else 0.0
            )

            failure_reason = plan_res.get("failure_reason")
            if is_success:
                status = "OK"
                error_detail = None
            else:
                status = _classify_failure_status(failure_reason)
                error_detail = failure_reason or "Planning failed"

            if waypoints and len(waypoints) >= 2:
                oracle_verdict = oracle.path_is_valid(
                    waypoints,
                    prep["circle_obstacles"],
                    prep["polygon_obstacles"],
                    turn_radius=prep["turn_radius"],
                    alpha_max_rad=prep["alpha_max_rad"],
                    l0=prep["start_state"].get("straight_length", l0),
                    dss=prep["goal_state"].get("engagement_distance", dss),
                )
            else:
                oracle_verdict = ValidationResult(False, error_detail or "no path")

            return QAResult(
                scenario_name=name,
                status=status,
                is_success=is_success,
                waypoints=waypoints,
                path_length_m=path_len,
                wall_time_s=wall_time_s,
                applied_time_budget_s=applied_budget,
                iterations=iterations,
                oracle_verdict=oracle_verdict,
                error_detail=error_detail,
                raw_response=plan_res,
            )
        except Exception as exc:
            wall_time_s = time.perf_counter() - start_time
            return QAResult(
                scenario_name=name,
                status="INTERNAL_ERROR",
                is_success=False,
                waypoints=[],
                path_length_m=0.0,
                wall_time_s=wall_time_s,
                applied_time_budget_s=time_budget_s,
                iterations=0,
                oracle_verdict=ValidationResult(False, f"Exception: {exc}"),
                error_detail=str(exc),
                raw_response=None,
            )

    def _run_nats(
        self,
        scenario: Scenario,
        name: str = "custom",
        time_budget_s: float = 15.0,
        limits: VehicleLimits | None = None,
        turn_radius: float = config.R,
        l0: float = config.L0,
        dss: float = config.DSS,
        safe_margin: float = config.SAFE_MARGIN,
        alpha_max_rad: float = config.ALPHA_MAX_RAD,
    ) -> QAResult:
        """Thực thi kịch bản qua NATS Microservice (NATS Mode)."""
        start_time = time.perf_counter()
        try:
            request = _scenario_to_plan_request(
                scenario, name, time_budget_s, limits=limits
            )
            client = NatsClient(server_url=self.nats_url, subject=self.subject)
            reply = client.request_plan_sync(request, timeout_s=time_budget_s + 2.0)
            wall_time_s = (
                reply.plan_wall_time_s
                if reply.plan_wall_time_s > 0
                else (time.perf_counter() - start_time)
            )

            is_success = reply.is_ok
            status = reply.status.name
            error_detail = reply.detail if reply.detail else None

            waypoints: list[tuple[tuple[float, float], float]] = [
                (
                    (float(wp.position[0]), float(wp.position[1])),
                    bearing_deg_to_math_rad(wp.heading_deg),
                )
                for wp in reply.waypoints
            ]

            if waypoints and len(waypoints) >= 2:
                prep = prepare_scenario(
                    scenario,
                    turn_radius=turn_radius,
                    l0=l0,
                    dss=dss,
                    safe_margin=safe_margin,
                    alpha_max_rad=alpha_max_rad,
                )
                oracle_verdict = oracle.path_is_valid(
                    waypoints,
                    prep["circle_obstacles"],
                    prep["polygon_obstacles"],
                    turn_radius=prep["turn_radius"],
                    alpha_max_rad=prep["alpha_max_rad"],
                    l0=prep["start_state"].get("straight_length", l0),
                    dss=prep["goal_state"].get("engagement_distance", dss),
                )
            else:
                oracle_verdict = ValidationResult(False, error_detail or "no path")

            return QAResult(
                scenario_name=name,
                status=status,
                is_success=is_success,
                waypoints=waypoints,
                path_length_m=reply.path_length_m,
                wall_time_s=wall_time_s,
                applied_time_budget_s=reply.applied_time_budget_s,
                iterations=reply.stats.iterations,
                oracle_verdict=oracle_verdict,
                error_detail=error_detail,
                raw_response=reply,
            )
        except TimeoutError as exc:
            wall_time_s = time.perf_counter() - start_time
            return QAResult(
                scenario_name=name,
                status="TIMEOUT",
                is_success=False,
                waypoints=[],
                path_length_m=0.0,
                wall_time_s=wall_time_s,
                applied_time_budget_s=time_budget_s,
                iterations=0,
                oracle_verdict=ValidationResult(False, "NATS request timed out"),
                error_detail=f"NATS Request Timeout: {exc}",
                raw_response=None,
            )
        except Exception as exc:
            wall_time_s = time.perf_counter() - start_time
            return QAResult(
                scenario_name=name,
                status="INTERNAL_ERROR",
                is_success=False,
                waypoints=[],
                path_length_m=0.0,
                wall_time_s=wall_time_s,
                applied_time_budget_s=time_budget_s,
                iterations=0,
                oracle_verdict=ValidationResult(False, f"Exception: {exc}"),
                error_detail=str(exc),
                raw_response=None,
            )
