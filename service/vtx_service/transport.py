"""Lớp DDS. Module DUY NHẤT trong service được phép import một binding DDS.

Mọi thứ khác - hợp đồng dữ liệu, adapter, runner, bản đồ - độc lập với stack đã
chọn. Đổi stack là viết lại file này, không đụng chỗ nào khác.

QoS: cả hai topic RELIABLE + VOLATILE; request KEEP_ALL, reply KEEP_LAST(8).
VOLATILE là bắt buộc, không phải mặc định tuỳ tiện: TRANSIENT_LOCAL trên topic
request nghĩa là service khởi động lại sẽ nhận và lập kế hoạch lại một mission
cũ đã hết hiệu lực. Một lệnh bay không được phép phát lại.

KHÔNG dùng `from __future__ import annotations` trong file này. cyclonedds phân
giải chú thích kiểu LÚC CHẠY, còn PEP 563 biến chúng thành chuỗi, nên
`Topic(...)` ném `TypeError: Type array[uint8, 16] ... cannot be resolved`. Đã
đo: có dòng đó thì hỏng, bỏ ra thì chạy. Mọi module khác trong service vẫn dùng
bình thường - chỉ module khai báo IdlStruct mới bị.
"""

import sys
import time
import traceback
from collections.abc import Callable
from dataclasses import dataclass

from cyclonedds.core import (
    InstanceState,
    Policy,
    Qos,
    ReadCondition,
    SampleState,
    ViewState,
    WaitSet,
)
from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.annotations import key
from cyclonedds.idl.types import array, sequence, uint8, uint32
from cyclonedds.pub import DataWriter, Publisher
from cyclonedds.sub import DataReader, Subscriber
from cyclonedds.topic import Topic
from cyclonedds.util import duration

from vtx_service import messages as msg

REQUEST_TOPIC = "VtxPathPlanRequest"
REPLY_TOPIC = "VtxPathPlanReply"

_RELIABLE = Policy.Reliability.Reliable(duration(seconds=10))
# IgnoreLocal.Participant là BẮT BUỘC, không phải tuỳ chọn. Không có nó, một
# DataWriter khớp với DataReader của CHÍNH participant mình, nên
# `wait_for_service` trả True ngay cả khi không có service nào - đã đo:
# current_count = 1 với một participant duy nhất, = 0 khi bật IgnoreLocal.
_IGNORE_SELF = Policy.IgnoreLocal.Participant
REQUEST_QOS = Qos(_RELIABLE, Policy.History.KeepAll, Policy.Durability.Volatile, _IGNORE_SELF)
REPLY_QOS = Qos(_RELIABLE, Policy.History.KeepLast(8), Policy.Durability.Volatile, _IGNORE_SELF)


# --- kiểu trên dây, khớp service/idl/vtx_path_planning.idl --------------------


@dataclass
class Point2D(IdlStruct, typename="vtx.planning.Point2D"):
    x: float
    y: float


@dataclass
class Polygon(IdlStruct, typename="vtx.planning.Polygon"):
    vertices: sequence[Point2D]


@dataclass
class Circle(IdlStruct, typename="vtx.planning.Circle"):
    center: Point2D
    radius_m: float


@dataclass
class VehicleLimits(IdlStruct, typename="vtx.planning.VehicleLimits"):
    turn_radius_m: float
    l0_m: float
    dss_m: float
    safe_margin_m: float
    alpha_max_deg: float


@dataclass
class SearchBudget(IdlStruct, typename="vtx.planning.SearchBudget"):
    time_budget_s: float
    max_iterations: uint32


@dataclass
class WireRequest(IdlStruct, typename="vtx.planning.VtxPathPlanRequest"):
    request_id: array[uint8, 16]
    key("request_id")
    idl_version: uint32
    start: Point2D
    start_heading_deg: float
    goal: Point2D
    goal_heading_deg: float
    goal_heading_free: bool
    islands: sequence[Polygon]
    dynamic_obstacles: sequence[Circle]
    safezones: sequence[Polygon]
    use_preloaded_map: bool
    limits: VehicleLimits
    budget: SearchBudget


@dataclass
class Waypoint(IdlStruct, typename="vtx.planning.Waypoint"):
    position: Point2D
    heading_deg: float


@dataclass
class SearchStats(IdlStruct, typename="vtx.planning.SearchStats"):
    iterations: uint32
    max_iterations: uint32
    open_set_size: uint32
    search_failed: bool
    budget_bound: bool


@dataclass
class WireReply(IdlStruct, typename="vtx.planning.VtxPathPlanReply"):
    request_id: array[uint8, 16]
    key("request_id")
    idl_version: uint32
    status: uint32
    detail: str
    waypoints: sequence[Waypoint]
    path_length_m: float
    plan_wall_time_s: float
    applied_time_budget_s: float
    stats: SearchStats
    planner_version: str
    config_hash: str


# --- dịch giữa kiểu trên dây và kiểu nội bộ ----------------------------------


def _ring(polygon: Polygon) -> tuple[msg.Point, ...]:
    return tuple((v.x, v.y) for v in polygon.vertices)


def _to_domain(wire: WireRequest) -> msg.PlanRequest:
    return msg.PlanRequest(
        request_id=bytes(wire.request_id),
        idl_version=int(wire.idl_version),
        start=(wire.start.x, wire.start.y),
        start_heading_deg=wire.start_heading_deg,
        goal=(wire.goal.x, wire.goal.y),
        goal_heading_deg=wire.goal_heading_deg,
        goal_heading_free=wire.goal_heading_free,
        islands=tuple(_ring(p) for p in wire.islands),
        dynamic_obstacles=tuple(
            msg.Circle(center=(c.center.x, c.center.y), radius_m=c.radius_m)
            for c in wire.dynamic_obstacles
        ),
        safezones=tuple(_ring(p) for p in wire.safezones),
        use_preloaded_map=wire.use_preloaded_map,
        limits=msg.VehicleLimits(
            wire.limits.turn_radius_m,
            wire.limits.l0_m,
            wire.limits.dss_m,
            wire.limits.safe_margin_m,
            wire.limits.alpha_max_deg,
        ),
        budget=msg.SearchBudget(wire.budget.time_budget_s, int(wire.budget.max_iterations)),
    )


def _to_wire_request(request: msg.PlanRequest) -> WireRequest:
    def rings(source: tuple[tuple[msg.Point, ...], ...]) -> list[Polygon]:
        return [Polygon(vertices=[Point2D(x, y) for x, y in ring]) for ring in source]

    return WireRequest(
        request_id=list(request.request_id),
        idl_version=request.idl_version,
        start=Point2D(*request.start),
        start_heading_deg=request.start_heading_deg,
        goal=Point2D(*request.goal),
        goal_heading_deg=request.goal_heading_deg,
        goal_heading_free=request.goal_heading_free,
        islands=rings(request.islands),
        dynamic_obstacles=[
            Circle(center=Point2D(*c.center), radius_m=c.radius_m)
            for c in request.dynamic_obstacles
        ],
        safezones=rings(request.safezones),
        use_preloaded_map=request.use_preloaded_map,
        limits=VehicleLimits(
            request.limits.turn_radius_m,
            request.limits.l0_m,
            request.limits.dss_m,
            request.limits.safe_margin_m,
            request.limits.alpha_max_deg,
        ),
        budget=SearchBudget(request.budget.time_budget_s, request.budget.max_iterations),
    )


def _to_wire_reply(reply: msg.PlanReply) -> WireReply:
    return WireReply(
        request_id=list(reply.request_id),
        idl_version=reply.idl_version,
        status=int(reply.status),
        detail=reply.detail,
        waypoints=[
            Waypoint(position=Point2D(*w.position), heading_deg=w.heading_deg)
            for w in reply.waypoints
        ],
        path_length_m=reply.path_length_m,
        plan_wall_time_s=reply.plan_wall_time_s,
        applied_time_budget_s=reply.applied_time_budget_s,
        stats=SearchStats(
            reply.stats.iterations,
            reply.stats.max_iterations,
            reply.stats.open_set_size,
            reply.stats.search_failed,
            reply.stats.budget_bound,
        ),
        planner_version=reply.planner_version,
        config_hash=reply.config_hash,
    )


def _from_wire_reply(wire: WireReply) -> msg.PlanReply:
    return msg.PlanReply(
        request_id=bytes(wire.request_id),
        idl_version=int(wire.idl_version),
        status=msg.PlanStatus(int(wire.status)),
        detail=wire.detail,
        waypoints=tuple(
            msg.Waypoint(position=(w.position.x, w.position.y), heading_deg=w.heading_deg)
            for w in wire.waypoints
        ),
        path_length_m=wire.path_length_m,
        plan_wall_time_s=wire.plan_wall_time_s,
        applied_time_budget_s=wire.applied_time_budget_s,
        stats=msg.SearchStats(
            int(wire.stats.iterations),
            int(wire.stats.max_iterations),
            int(wire.stats.open_set_size),
            wire.stats.search_failed,
            wire.stats.budget_bound,
        ),
        planner_version=wire.planner_version,
        config_hash=wire.config_hash,
    )


def _internal_error_reply(request_id: bytes, detail: str) -> msg.PlanReply:
    """Reply PLAN_INTERNAL_ERROR mang đúng ``request_id``, để client không treo.

    Dùng khi handler ném lỗi, hoặc khi dịch reply của nó ra kiểu trên dây thất
    bại - cả hai đều không được phép làm chết vòng ``serve``.
    """
    return msg.PlanReply(
        request_id=request_id,
        idl_version=msg.IDL_VERSION,
        status=msg.PlanStatus.INTERNAL_ERROR,
        detail=detail,
        waypoints=(),
        path_length_m=0.0,
        plan_wall_time_s=0.0,
        applied_time_budget_s=0.0,
        stats=msg.SearchStats(0, 0, 0, True, False),
        planner_version="",
        config_hash="",
    )


# --- transport ----------------------------------------------------------------


class DdsTransport:
    """Hai topic, tương quan bằng ``request_id``.

    Cùng một lớp đóng cả hai vai: :meth:`serve` cho phía service, :meth:`request`
    cho phía client trong test và công cụ chẩn đoán.
    """

    def __init__(self, domain_id: int = 0) -> None:
        self._participant = DomainParticipant(domain_id)
        self._request_topic = Topic(
            self._participant, REQUEST_TOPIC, WireRequest, qos=REQUEST_QOS
        )
        self._reply_topic = Topic(self._participant, REPLY_TOPIC, WireReply, qos=REPLY_QOS)
        publisher = Publisher(self._participant)
        subscriber = Subscriber(self._participant)
        self._request_writer = DataWriter(publisher, self._request_topic, qos=REQUEST_QOS)
        self._request_reader = DataReader(subscriber, self._request_topic, qos=REQUEST_QOS)
        self._reply_writer = DataWriter(publisher, self._reply_topic, qos=REPLY_QOS)
        self._reply_reader = DataReader(subscriber, self._reply_topic, qos=REPLY_QOS)
        self._running = False

    def serve(self, handler: Callable[[msg.PlanRequest], msg.PlanReply]) -> None:
        """Nhận request và trả lời, tuần tự, tới khi :meth:`close`.

        Args:
            handler: Hàm nhận một request và trả về một reply. Được gọi lần
                lượt, không bao giờ đồng thời.
        """
        condition = ReadCondition(
            self._request_reader, ViewState.Any | InstanceState.Alive | SampleState.NotRead
        )
        waitset = WaitSet(self._participant)
        waitset.attach(condition)
        self._running = True

        while self._running:
            if waitset.wait(duration(milliseconds=200)) == 0:
                continue
            for wire in self._request_reader.take(N=16, condition=condition):
                if not self._running:
                    return
                self._handle_one(wire, handler)

    def _handle_one(
        self, wire: object, handler: Callable[[msg.PlanRequest], msg.PlanReply]
    ) -> None:
        """Xử lý một request, không bao giờ để lỗi thoát ra ngoài.

        Một request hỏng - handler ném lỗi, hoặc dịch reply của nó ra kiểu
        trên dây thất bại - không được phép hạ cả service: trả về
        PLAN_INTERNAL_ERROR mang đúng ``request_id`` rồi tiếp tục vòng lặp.
        Nếu dựng CẢ reply lỗi cũng thất bại, log rồi bỏ qua mẫu tin này -
        vòng lặp vẫn phải sống tới mẫu tiếp theo.
        """
        request_id = bytes(wire.request_id)  # type: ignore[attr-defined]
        try:
            wire_reply = _to_wire_reply(handler(_to_domain(wire)))  # type: ignore[arg-type]
        except Exception:  # noqa: BLE001 - một request hỏng không được hạ cả service
            print(
                f"[transport] request {request_id.hex()} lỗi khi xử lý:\n"
                f"{traceback.format_exc(limit=5)}",
                file=sys.stderr,
            )
            try:
                wire_reply = _to_wire_reply(
                    _internal_error_reply(request_id, "internal error khi xử lý request")
                )
            except Exception:  # noqa: BLE001 - dựng reply lỗi cũng không được hạ service
                print(
                    f"[transport] request {request_id.hex()} lỗi cả khi dựng reply lỗi:\n"
                    f"{traceback.format_exc(limit=5)}",
                    file=sys.stderr,
                )
                return

        try:
            self._reply_writer.write(wire_reply)
        except Exception:  # noqa: BLE001 - ghi lỗi không được hạ service
            print(
                f"[transport] request {request_id.hex()} lỗi khi ghi reply:\n"
                f"{traceback.format_exc(limit=5)}",
                file=sys.stderr,
            )

    def wait_for_service(self, timeout_s: float) -> bool:
        """Chờ tới khi có một service khớp trên topic request.

        Ghi trước khi khớp là mất mẫu tin trong im lặng với QoS VOLATILE.

        Args:
            timeout_s: Thời gian chờ tối đa.

        Returns:
            ``True`` nếu đã khớp.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._request_writer.get_publication_matched_status().current_count > 0:
                # Cho reader phía kia kịp khớp nốt chiều còn lại.
                time.sleep(0.5)
                return True
            time.sleep(0.1)
        return False

    def request(self, request: msg.PlanRequest, timeout_s: float = 30.0) -> msg.PlanReply:
        """Gửi một request và chờ reply khớp ``request_id``.

        Args:
            request: Mission cần gửi.
            timeout_s: Thời gian chờ tối đa.

        Returns:
            Reply tương ứng.

        Raises:
            TimeoutError: Khi không có reply khớp trong thời gian chờ.
        """
        condition = ReadCondition(
            self._reply_reader, ViewState.Any | InstanceState.Alive | SampleState.NotRead
        )
        waitset = WaitSet(self._participant)
        waitset.attach(condition)

        self._request_writer.write(_to_wire_request(request))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if waitset.wait(duration(milliseconds=200)) == 0:
                continue
            for wire in self._reply_reader.take(N=16, condition=condition):
                if bytes(wire.request_id) == request.request_id:
                    return _from_wire_reply(wire)
        raise TimeoutError(f"không có reply cho {request.request_id.hex()[:8]}")

    def close(self) -> None:
        """Dừng vòng phục vụ và giải phóng thực thể DDS."""
        self._running = False
