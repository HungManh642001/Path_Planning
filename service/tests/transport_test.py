"""Round-trip qua DDS thật, so với việc gọi handler trong tiến trình.

Test tự bỏ qua CÓ LÝ DO khi binding chưa có. Một test bị bỏ qua trong im lặng
còn tệ hơn không có test.
"""

from __future__ import annotations

import dataclasses
import re
import threading
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

from cyclonedds.idl.types import uint32  # noqa: E402

import vtx_service.transport as transport_module  # noqa: E402
from vtx_service.transport import (  # noqa: E402
    DdsTransport,
    VehicleLimits as WireVehicleLimits,
    WireReply,
    _to_wire_request,
)

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


def test_idl_status_field_type_is_documented_as_diverging() -> None:
    """The one genuine IDL/Python divergence the ordinal check above cannot see.

    The IDL declares ``PlanStatus status;`` (the enum) as the wire type for
    ``VtxPathPlanReply.status``; ``transport.py``'s ``WireReply`` declares
    ``status: uint32`` instead. That is deliberate - cyclonedds only accepts
    a cyclonedds-native enum (``cyclonedds.idl.IdlEnum``) as an IdlStruct
    field, and giving ``msg.PlanStatus`` that base class would pull the DDS
    binding into ``messages.py``, which its module docstring forbids. Both
    files carry a comment explaining this; this test pins the two field
    TYPES (IDL enum vs. Python uint32) as a documented, deliberate split so
    a future edit cannot silently widen it (e.g. a new field that quietly
    also becomes plain ``uint32`` in Python while the IDL keeps a real type).
    """
    text = IDL_PATH.read_text(encoding="utf-8")
    assert re.search(r"\bPlanStatus\s+status\s*;", text)
    status_field = next(f for f in dataclasses.fields(WireReply) if f.name == "status")
    assert status_field.type is uint32


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


def test_invalid_geometry_becomes_invalid_request_not_internal_error() -> None:
    """F2: a ``ValueError`` escaping the dataclass validation layer (in
    ``_to_domain``, when it constructs ``msg.VehicleLimits`` / ``msg.Circle``
    / ``msg.PlanRequest``) must classify as ``PLAN_INVALID_REQUEST`` with the
    original message preserved in ``detail`` - not fall into the generic
    ``except Exception`` catch around the handler call, which used to turn it
    into a generic ``PLAN_INTERNAL_ERROR`` and discard the real message
    (e.g. "turn_radius_m phải dương").

    The bad geometry - all-zero ``VehicleLimits``, the single most likely
    client mistake per spec §6 - has to be injected at the WIRE level: a
    domain ``msg.PlanRequest`` cannot even be constructed with it, since
    ``msg.VehicleLimits.__post_init__`` validates immediately. A real DDS
    client sends raw, unvalidated floats over the wire, so this is exactly
    what reaches ``_to_domain`` in production. The handler asserts it is
    never called - proof this is classified BEFORE reaching it.
    """

    class _FakeWriter:
        def __init__(self) -> None:
            self.written: list[object] = []

        def write(self, sample: object) -> None:
            self.written.append(sample)

    def handler(incoming: PlanRequest) -> PlanReply:
        raise AssertionError("handler must not be reached for a malformed request")

    service = DdsTransport(domain_id=DOMAIN + 9)
    fake_writer = _FakeWriter()
    try:
        service._reply_writer = fake_writer  # type: ignore[assignment]
        good_wire = _to_wire_request(_request(uuid.uuid4().bytes))
        bad_wire = dataclasses.replace(
            good_wire, limits=WireVehicleLimits(0.0, 0.0, 0.0, 0.0, 0.0)
        )
        service._handle_one(bad_wire, handler)
    finally:
        service.close()

    assert len(fake_writer.written) == 1
    reply = fake_writer.written[0]
    assert reply.status == int(PlanStatus.INVALID_REQUEST)  # type: ignore[attr-defined]
    assert "turn_radius_m" in reply.detail  # type: ignore[attr-defined]


def test_a_non_value_error_during_translation_becomes_internal_error_not_a_dead_serve_loop(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """R25: F2 narrowed ``_handle_one``'s catch around ``_to_domain`` from
    ``except Exception`` to ``except ValueError``, which reopened the exact
    hole R18(a) closed - a non-``ValueError`` exception during translation
    now has nowhere to land, since ``serve()`` does not wrap
    ``_handle_one``. Every current ``__post_init__`` happens to raise only
    ``ValueError``, so this was latent, not visible - but a future validator
    raising ``TypeError``/``KeyError``/... would kill ``serve()``
    permanently. Simulate that by monkeypatching ``_to_domain`` to raise a
    non-``ValueError``, and assert both that the bad request gets
    ``INTERNAL_ERROR`` and that the loop survives to answer a good request
    right after.
    """
    real_to_domain = transport_module._to_domain

    def _boom(wire: object) -> PlanRequest:
        raise TypeError("hình học không phải ValueError - lỗi giả lập cho R25")

    def handler(incoming: PlanRequest) -> PlanReply:
        return _reply(incoming)

    service = DdsTransport(domain_id=DOMAIN + 12)
    client = DdsTransport(domain_id=DOMAIN + 12)
    thread = threading.Thread(target=service.serve, args=(handler,), daemon=True)
    thread.start()
    try:
        assert client.wait_for_service(timeout_s=20.0)

        monkeypatch.setattr(transport_module, "_to_domain", _boom)
        bad_id = uuid.uuid4().bytes
        bad_reply = client.request(_request(bad_id), timeout_s=30.0)
        assert bad_reply.request_id == bad_id
        assert bad_reply.status is PlanStatus.INTERNAL_ERROR

        # Vòng phục vụ phải vẫn sống: một request TỐT ngay sau đó vẫn được
        # trả lời đúng - nếu R25 chưa sửa, serve() đã chết ở request trên.
        monkeypatch.setattr(transport_module, "_to_domain", real_to_domain)
        good_id = uuid.uuid4().bytes
        good_reply = client.request(_request(good_id), timeout_s=30.0)
        assert good_reply.request_id == good_id
        assert good_reply.status is PlanStatus.ORACLE_REJECTED
    finally:
        service.close()
        client.close()
