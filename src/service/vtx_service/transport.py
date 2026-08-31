"""Lớp giao vận NATS cho VTX Algorithm Service.

Module phụ trách kết nối NATS Server, đăng ký Queue Group và điều phối
các yêu cầu lập lịch đường bay (Request-Reply) bất đồng bộ với Protocol Buffers.
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor

import nats
from nats.aio.client import Client as NatsClientConnection
from nats.aio.msg import Msg

from service.vtx_service import codec, messages as msg


logger = logging.getLogger("vtx-planner.transport")

DEFAULT_NATS_SERVER = "nats://localhost:4222"
DEFAULT_SUBJECT = "vtx.algorithms.path_planning.plan"
DEFAULT_QUEUE_GROUP = "vtx.algorithms.path_planning"


class NatsTransport:
    """Bộ giao vận NATS cho service lập lịch đường bay.

    Attributes:
        server_url: Địa chỉ NATS server (vd: 'nats://localhost:4222').
        subject: NATS subject lắng nghe yêu cầu.
        queue: Tên Queue Group để cân bằng tải giữa các instance.
        nc: Kết nối NATS client.
    """

    def __init__(
        self,
        server_url: str = DEFAULT_NATS_SERVER,
        subject: str = DEFAULT_SUBJECT,
        queue: str = DEFAULT_QUEUE_GROUP,
    ) -> None:
        """Khởi tạo NatsTransport.

        Args:
            server_url: Địa chỉ NATS server kết nối tới.
            subject: Subject nhận request lập lịch.
            queue: Tên queue group cân bằng tải.
        """
        self.server_url = server_url
        self.subject = subject
        self.queue = queue
        self.nc: NatsClientConnection | None = None
        self._sub = None
        self._shutdown_event: asyncio.Event | None = None
        self._executor = ThreadPoolExecutor(thread_name_prefix="nats-worker")

    async def start(
        self, handler: Callable[[msg.PlanRequest], msg.PlanReply]
    ) -> None:
        """Kết nối NATS và đăng ký lắng nghe request với Queue Group.

        Args:
            handler: Hàm xử lý nghiệp vụ PlanRequest -> PlanReply.
        """
        self.nc = await nats.connect(self.server_url)
        logger.info("đã kết nối NATS server tại %s", self.server_url)

        loop = asyncio.get_running_loop()

        async def on_message(nats_msg: Msg) -> None:
            """Xử lý tin nhắn đến từ NATS."""
            if not nats_msg.reply:
                logger.warning("nhận message không có reply inbox, bỏ qua")
                return

            try:
                request = codec.decode_request(nats_msg.data)
            except Exception as exc:
                logger.error("lỗi giải mã Protobuf request: %s", exc)
                error_reply = msg.PlanReply(
                    request_id=b"\x00" * 16,
                    idl_version=msg.IDL_VERSION,
                    status=msg.PlanStatus.INVALID_REQUEST,
                    detail=f"Lỗi giải mã Protobuf request: {exc}",
                    waypoints=(),
                    path_length_m=0.0,
                    plan_wall_time_s=0.0,
                    applied_time_budget_s=0.0,
                    stats=msg.SearchStats(
                        iterations=0,
                        open_set_size=0,
                        is_search_failed=True,
                        is_budget_bound=False,
                    ),
                    planner_version="unknown",
                    config_hash="unknown",
                )
                await self.nc.publish(nats_msg.reply, codec.encode_reply(error_reply))
                return

            # Chạy handler CPU-bound trong executor để không block asyncio loop
            try:
                reply = await loop.run_in_executor(
                    self._executor, handler, request
                )
            except Exception as exc:
                logger.exception("lỗi ngoại lệ trong quá trình xử lý: %s", exc)
                reply = msg.PlanReply(
                    request_id=request.request_id,
                    idl_version=request.idl_version,
                    status=msg.PlanStatus.INTERNAL_ERROR,
                    detail=f"Ngoại lệ xử lý nội bộ: {exc}",
                    waypoints=(),
                    path_length_m=0.0,
                    plan_wall_time_s=0.0,
                    applied_time_budget_s=0.0,
                    stats=msg.SearchStats(
                        iterations=0,
                        open_set_size=0,
                        is_search_failed=True,
                        is_budget_bound=False,
                    ),
                    planner_version="unknown",
                    config_hash="unknown",
                )

            encoded_reply = codec.encode_reply(reply)
            if self.nc is not None:
                await self.nc.publish(nats_msg.reply, encoded_reply)

        self._sub = await self.nc.subscribe(
            self.subject, queue=self.queue, cb=on_message
        )
        await self.nc.flush()
        logger.info(
            "đang lắng nghe trên subject '%s' (queue group '%s')",
            self.subject,
            self.queue,
        )

    async def close(self) -> None:
        """Đóng kết nối NATS và dọn dẹp tài nguyên."""
        if self._shutdown_event is not None:
            self._shutdown_event.set()

        if self.nc is not None and not self.nc.is_closed:
            logger.info("đang đóng kết nối NATS...")
            await self.nc.drain()
            await self.nc.close()
            self.nc = None
        self._executor.shutdown(wait=False)

    async def run_forever(
        self, handler: Callable[[msg.PlanRequest], msg.PlanReply]
    ) -> None:
        """Khởi động và duy trì vòng lặp phục vụ cho tới khi bị dừng."""
        self._shutdown_event = asyncio.Event()
        await self.start(handler)
        await self._shutdown_event.wait()

    def stop(self) -> None:
        """Kích hoạt tín hiệu dừng service từ bên ngoài (đồng bộ)."""
        if self._shutdown_event is not None:
            self._shutdown_event.set()

    def serve(
        self, handler: Callable[[msg.PlanRequest], msg.PlanReply]
    ) -> None:
        """Hàm đồng bộ chạy service vòng lặp vô tận."""
        asyncio.run(self.run_forever(handler))


class NatsClient:
    """Client NATS để gửi yêu cầu lập lịch và nhận kết quả.

    Attributes:
        server_url: Địa chỉ NATS server.
        subject: Subject gửi request.
        nc: Kết nối NATS client.
    """

    def __init__(
        self,
        server_url: str = DEFAULT_NATS_SERVER,
        subject: str = DEFAULT_SUBJECT,
    ) -> None:
        """Khởi tạo NatsClient."""
        self.server_url = server_url
        self.subject = subject
        self.nc: NatsClientConnection | None = None

    async def connect(self) -> None:
        """Kết nối tới NATS server."""
        if self.nc is None or self.nc.is_closed:
            self.nc = await nats.connect(self.server_url)

    async def close(self) -> None:
        """Đóng kết nối NATS client."""
        if self.nc is not None and not self.nc.is_closed:
            await self.nc.close()
            self.nc = None

    async def request_plan(
        self, request: msg.PlanRequest, timeout_s: float = 6.0
    ) -> msg.PlanReply:
        """Gửi PlanRequest qua NATS và đợi PlanReply phản hồi.

        Args:
            request: Yêu cầu lập lịch gửi đi.
            timeout_s: Thời gian chờ tối đa (giây).

        Returns:
            PlanReply: Kết quả nhận được từ service.

        Raises:
            RuntimeError: Nếu client chưa được kết nối.
            TimeoutError: Nếu quá thời gian timeout mà không có phản hồi.
        """
        if self.nc is None or self.nc.is_closed:
            await self.connect()
        assert self.nc is not None

        encoded_req = codec.encode_request(request)
        msg_reply = await self.nc.request(
            self.subject, encoded_req, timeout=timeout_s
        )
        return codec.decode_reply(msg_reply.data)

    def request_plan_sync(
        self, request: msg.PlanRequest, timeout_s: float = 6.0
    ) -> msg.PlanReply:
        """Wrapper đồng bộ của request_plan."""
        async def _run() -> msg.PlanReply:
            await self.connect()
            try:
                return await self.request_plan(request, timeout_s=timeout_s)
            finally:
                await self.close()

        return asyncio.run(_run())
