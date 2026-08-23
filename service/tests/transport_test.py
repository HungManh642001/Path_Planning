"""Round-trip qua DDS thật, so với việc gọi handler trong tiến trình.

Test tự bỏ qua CÓ LÝ DO khi binding chưa có. Một test bị bỏ qua trong im lặng
còn tệ hơn không có test.
"""

from __future__ import annotations

import re
import threading
import time
import uuid
from pathlib import Path

import pytest

from vtx_service.messages import (
    IDL_VERSION,
    Circle,
    PlanReply,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    SearchStats,
    VehicleLimits,
    Waypoint,
)

pytest.importorskip(
    "cyclonedds", reason="chưa cài binding DDS; xem quyết định ở Task 1"
)

from vtx_service.transport import DdsTransport  # noqa: E402

IDL_PATH = Path(__file__).resolve().parents[1] / "idl" / "vtx_path_planning.idl"
DOMAIN = 92


def _request(request_id: bytes) -> PlanRequest:
    return PlanRequest(
        request_id=request_id,
        idl_version=IDL_VERSION,
        start=(50000.0, 50000.0),
        start_heading_deg=45.0,
        goal=(300000.0, 250000.0),
        goal_heading_deg=137.5,
        goal_heading_free=False,
        islands=(((1e5, 1e5), (1.2e5, 1e5), (1.1e5, 1.3e5)),),
        dynamic_obstacles=(Circle(center=(2e5, 1.5e5), radius_m=12000.0),),
        safezones=(((0.0, 0.0), (5e5, 0.0), (5e5, 5e5)),),
        use_preloaded_map=False,
        limits=VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0),
        budget=SearchBudget(15.0, 50000),
    )


def _reply(request: PlanRequest) -> PlanReply:
    return PlanReply(
        request_id=request.request_id,
        idl_version=IDL_VERSION,
        status=PlanStatus.ORACLE_REJECTED,
        detail="first W1..W2 l=7421.3 < L0=8000",
        waypoints=(Waypoint((1.5, -2.5), 137.5), Waypoint((3e5, 2.5e5), 42.0)),
        path_length_m=123456.78901234567,
        plan_wall_time_s=0.0421,
        applied_time_budget_s=15.0,
        stats=SearchStats(1234, 50000, 56, True, False),
        planner_version="v1.0-3-gabc1234-dirty",
        config_hash="0123456789abcdef",
    )


def test_idl_status_enum_matches_python_exactly() -> None:
    text = IDL_PATH.read_text(encoding="utf-8")
    match = re.search(r"enum\s+PlanStatus\s*\{(.*?)\}", text, re.DOTALL)
    assert match
    members = [item.strip() for item in match.group(1).split(",") if item.strip()]
    assert members == [f"PLAN_{member.name}" for member in PlanStatus]


def test_idl_has_no_frame_field() -> None:
    """Cấm KHAI BÁO trường ``frame`` - không cấm nhắc tới từ đó trong comment.

    Chuỗi con "frame" hợp lệ trong một comment giải thích lý do không có
    trường đó (đúng comment mà một người đọc cần nhất). Chỉ một khai báo
    trường thật sự - tên trường ``frame`` theo ngay sau bởi ``[...]`` tuỳ
    chọn rồi dấu ``;`` - mới là vi phạm. Khớp trên TÊN trường, bất kể kiểu:
    một type dạng ``sequence<Point2D>`` hay ``octet ...[16]`` không phải một
    token đơn giản, nên đừng đòi kiểu đứng trước - comment đã bị bóc trước,
    nên bỏ yêu cầu đó không làm chính comment giải thích khớp lại.
    """
    text = IDL_PATH.read_text(encoding="utf-8")
    without_comments = re.sub(r"//.*", "", text)
    field_declaration = re.compile(r"\bframe\s*(\[[^\]]*\])?\s*;")
    assert not field_declaration.search(without_comments)


def test_a_request_survives_the_wire_unchanged() -> None:
    request_id = uuid.uuid4().bytes
    seen: list[PlanRequest] = []
    done = threading.Event()

    def handler(incoming: PlanRequest) -> PlanReply:
        seen.append(incoming)
        done.set()
        return _reply(incoming)

    service = DdsTransport(domain_id=DOMAIN)
    client = DdsTransport(domain_id=DOMAIN)
    thread = threading.Thread(target=service.serve, args=(handler,), daemon=True)
    thread.start()
    try:
        assert client.wait_for_service(timeout_s=20.0)
        reply = client.request(_request(request_id), timeout_s=30.0)
        assert done.wait(timeout=5.0)

        got = seen[0]
        assert got.request_id == request_id
        assert got.start == (50000.0, 50000.0)
        assert got.goal_heading_deg == 137.5
        assert got.goal_heading_free is False
        assert len(got.islands[0]) == 3
        assert got.dynamic_obstacles[0].radius_m == 12000.0
        assert len(got.safezones[0]) == 3
        assert got.limits.alpha_max_deg == 90.0

        assert reply.request_id == request_id
        assert reply.status is PlanStatus.ORACLE_REJECTED
        assert reply.detail == "first W1..W2 l=7421.3 < L0=8000"
        # double phải qua dây nguyên vẹn từng bit.
        assert reply.path_length_m == 123456.78901234567
        assert reply.waypoints[0].position == (1.5, -2.5)
        assert reply.stats.iterations == 1234
        assert reply.config_hash == "0123456789abcdef"
    finally:
        service.close()
        client.close()


def test_a_reply_for_another_request_is_ignored() -> None:
    """Tương quan bằng request_id, không phải bằng thứ tự đến."""
    def handler(incoming: PlanRequest) -> PlanReply:
        return _reply(incoming)

    service = DdsTransport(domain_id=DOMAIN + 3)
    client = DdsTransport(domain_id=DOMAIN + 3)
    threading.Thread(target=service.serve, args=(handler,), daemon=True).start()
    try:
        assert client.wait_for_service(timeout_s=20.0)
        first = uuid.uuid4().bytes
        second = uuid.uuid4().bytes
        assert client.request(_request(first), timeout_s=30.0).request_id == first
        assert client.request(_request(second), timeout_s=30.0).request_id == second
    finally:
        service.close()
        client.close()


def test_a_handler_exception_does_not_kill_serve() -> None:
    """handler ném lỗi -> reply PLAN_INTERNAL_ERROR đúng request_id, service vẫn sống.

    Task 11 chạy ``serve`` làm vòng lặp chính của service: một request hỏng
    (handler ném lỗi, hoặc dịch reply của nó ra kiểu trên dây thất bại) không
    được phép hạ cả service. Test này ném lỗi cho MỘT request rồi khẳng định
    một request TỐT tiếp theo, trên cùng transport, vẫn được trả lời bình
    thường.
    """
    bad_id = uuid.uuid4().bytes
    good_id = uuid.uuid4().bytes

    def handler(incoming: PlanRequest) -> PlanReply:
        if incoming.request_id == bad_id:
            raise RuntimeError("boom - lỗi giả lập trong handler")
        return _reply(incoming)

    service = DdsTransport(domain_id=DOMAIN + 6)
    client = DdsTransport(domain_id=DOMAIN + 6)
    threading.Thread(target=service.serve, args=(handler,), daemon=True).start()
    try:
        assert client.wait_for_service(timeout_s=20.0)

        bad_reply = client.request(_request(bad_id), timeout_s=30.0)
        assert bad_reply.request_id == bad_id
        assert bad_reply.status is PlanStatus.INTERNAL_ERROR

        # Service phải vẫn sống: một request TỐT sau đó vẫn được trả lời đúng.
        good_reply = client.request(_request(good_id), timeout_s=30.0)
        assert good_reply.request_id == good_id
        assert good_reply.status is PlanStatus.ORACLE_REJECTED
    finally:
        service.close()
        client.close()
