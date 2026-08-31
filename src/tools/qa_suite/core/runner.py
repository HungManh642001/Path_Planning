"""Module điều phối thực thi kịch bản kiểm thử (ExecutionDriver).

Hỗ trợ chạy đồng bộ trên thư viện Python nội bộ (LOCAL) hoặc gửi request qua
NATS Microservice (NATS), sau đó thẩm định kết quả độc lập bằng Validation Oracle.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum

from path_planning import config
from path_planning.geometry import spatial
from path_planning.planner import plan_trajectory
from path_planning.scenario.preprocessing import prepare_scenario
from path_planning.types import Scenario
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


class ExecutionMode(str, Enum):
    """Chế độ thực thi cho bộ kiểm thử."""

    LOCAL = "local"
    NATS = "nats"


@dataclass
class QAResult:
    """Cấu trúc dữ liệu chứa kết quả thực thi một kịch bản QA.

    Attributes:
        scenario_name: Tên hoặc mã định danh của kịch bản.
        status: Chuỗi trạng thái (OK, NO_PATH, TIMEOUT, ORACLE_REJECTED, etc.).
        is_success: Cờ cho biết thuật toán tìm thấy đường bay hợp lệ hay không.
        waypoints: Danh sách điểm waypoint [( (x, y), heading_rad ), ...].
        path_length_m: Tổng chiều dài đường bay tính bằng mét.
        wall_time_s: Thời gian thực thi thực tế (giây).
        applied_time_budget_s: Ngân sách thời gian thực tế đã áp dụng (giây).
        iterations: Số bước lặp tìm kiếm A*.
        oracle_verdict: Kết quả thẩm định độc lập từ Validation Oracle.
        error_detail: Chi tiết lỗi hoặc lý do thất bại (nếu có).
        raw_response: Đối tượng phản hồi nguyên bản (PlanResult dict hoặc PlanReply).
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


_REASON_TO_STATUS = {
    "no_path": "NO_PATH",
    "start_leg_blocked": "START_LEG_BLOCKED",
    "goal_leg_blocked": "GOAL_LEG_BLOCKED",
}


def _classify_failure_status(reason: str | None) -> str:
    """Chuyển đổi failure_reason nội bộ sang chuỗi status chuẩn."""
    if not reason:
        return "FAILED"
    return _REASON_TO_STATUS.get(reason, "ORACLE_REJECTED")


def scenario_to_plan_request(
    scenario: Scenario, name: str, time_budget_s: float
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
    ) -> QAResult:
        """Thực thi một kịch bản theo chế độ đã cấu hình.

        Args:
            scenario: Dữ liệu kịch bản nhiệm vụ (Scenario dict).
            name: Tên kịch bản hoặc mã định danh.
            time_budget_s: Ngân sách thời gian tối đa cho thuật toán tìm kiếm (giây).

        Returns:
            QAResult chứa đường bay, thời gian, trạng thái và kết quả thẩm định.

        Raises:
            ValueError: Nếu chế độ thực thi không hợp lệ.
        """
        if self.mode == ExecutionMode.LOCAL:
            return self._run_local(scenario, name=name, time_budget_s=time_budget_s)
        elif self.mode == ExecutionMode.NATS:
            return self._run_nats(scenario, name=name, time_budget_s=time_budget_s)
        else:
            raise ValueError(f"Unknown execution mode: {self.mode}")

    def _run_local(
        self,
        scenario: Scenario,
        name: str = "custom",
        time_budget_s: float = 15.0,
    ) -> QAResult:
        """Thực thi kịch bản bằng hàm nội bộ trong thư viện Python (Local Mode)."""
        start_time = time.perf_counter()
        try:
            prep = prepare_scenario(scenario)
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
            turn_radius = float(prep.get("turn_radius", config.R))
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
                    l0=prep["start_state"].get("straight_length", config.L0),
                    dss=prep["goal_state"].get("engagement_distance", config.DSS),
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
    ) -> QAResult:
        """Thực thi kịch bản qua NATS Microservice (NATS Mode)."""
        start_time = time.perf_counter()
        try:
            request = _scenario_to_plan_request(scenario, name, time_budget_s)
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
                prep = prepare_scenario(scenario)
                oracle_verdict = oracle.path_is_valid(
                    waypoints,
                    prep["circle_obstacles"],
                    prep["polygon_obstacles"],
                    turn_radius=prep["turn_radius"],
                    alpha_max_rad=prep["alpha_max_rad"],
                    l0=prep["start_state"].get("straight_length", config.L0),
                    dss=prep["goal_state"].get("engagement_distance", config.DSS),
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
                oracle_verdict=ValidationResult(False, f"Timeout: {exc}"),
                error_detail=f"Timeout communicating with NATS: {exc}",
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
                oracle_verdict=ValidationResult(False, f"NATS error: {exc}"),
                error_detail=str(exc),
                raw_response=None,
            )
