"""Vòng đời service: nạp bản đồ, khởi động runner, rồi mới kết nối NATS.

THỨ TỰ Ở ĐÂY LÀ MỘT RÀNG BUỘC, KHÔNG PHẢI SỞ THÍCH. `PlanRunner.start()` phải
chạy trước khi khởi tạo network: nó ép tiến trình forkserver ra đời trong lúc tiến
trình này còn sạch thread. Nếu để networking lên trước, forkserver sẽ ra đời từ
một tiến trình đang chạy thread nền, và mọi tiến trình con sau đó thừa hưởng rủi
ro deadlock.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
from pathlib import Path
from types import FrameType

from service.vtx_service.map_file import PreloadedMap
from service.vtx_service.messages import PlanReply, PlanRequest, PlanStatus
from service.vtx_service.runner import PlanRunner
from service.vtx_service.runtime import (
    MAX_REQUEST_TIME_BUDGET_S,
    config_hash,
    effective_time_budget_s,
    planner_version,
)
from service.vtx_service.transport import (
    DEFAULT_NATS_SERVER,
    DEFAULT_QUEUE_GROUP,
    DEFAULT_SUBJECT,
    NatsTransport,
)


def main(argv: list[str] | None = None) -> int:
    """Chạy service lập lịch đường bay NATS.

    Args:
        argv: Danh sách tham số dòng lệnh. Nếu None, dùng sys.argv.

    Returns:
        int: Mã exit của tiến trình.
    """
    parser = argparse.ArgumentParser(description="VTX path planning NATS microservice")
    parser.add_argument(
        "--nats-server",
        type=str,
        default=DEFAULT_NATS_SERVER,
        help=f"URL NATS server (mặc định: {DEFAULT_NATS_SERVER})",
    )
    parser.add_argument(
        "--subject",
        type=str,
        default=DEFAULT_SUBJECT,
        help=f"NATS subject lắng nghe yêu cầu (mặc định: {DEFAULT_SUBJECT})",
    )
    parser.add_argument(
        "--queue",
        type=str,
        default=DEFAULT_QUEUE_GROUP,
        help=f"Tên Queue Group cân bằng tải (mặc định: {DEFAULT_QUEUE_GROUP})",
    )
    parser.add_argument(
        "--preloaded-map",
        type=Path,
        default=None,
        help="file XML bản đồ nền; bỏ trống thì mọi request phải tự chứa",
    )
    parser.add_argument("--grace-seconds", type=float, default=2.0)
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=args.log_level,
        format="%(asctime)s %(levelname)s vtx-planner %(message)s",
        stream=sys.stderr,
    )
    log = logging.getLogger("vtx-planner")

    preloaded = PreloadedMap.load(args.preloaded_map) if args.preloaded_map else None
    if preloaded is not None:
        log.info(
            "bản đồ nền: %d safezone, %d đảo, %d vòng tròn",
            len(preloaded.safezones),
            len(preloaded.islands),
            len(preloaded.dynamic_obstacles),
        )
    else:
        log.info("không có bản đồ nền; mọi request phải tự chứa")

    runner = PlanRunner(preloaded=preloaded, grace_s=args.grace_seconds)
    runner.start()  # PHẢI trước transport - xem docstring của module
    log.info(
        "planner %s, config %s, ngân sách mặc định %.1f s, trần theo request %.1f s",
        planner_version(),
        config_hash(),
        effective_time_budget_s(),
        MAX_REQUEST_TIME_BUDGET_S,
    )

    transport = NatsTransport(
        server_url=args.nats_server,
        subject=args.subject,
        queue=args.queue,
    )

    def handle(request: PlanRequest) -> PlanReply:
        """Xử lý một request lập lịch từ NATS.

        Args:
            request: Yêu cầu lập lịch nhận được.

        Returns:
            PlanReply: Kết quả lập lịch trả về cho client.
        """
        reply = runner.submit(request)
        log.info(
            "request %s -> %s, %d waypoint, %.3f s",
            request.request_id.hex()[:8],
            reply.status.name,
            len(reply.waypoints),
            reply.plan_wall_time_s,
        )
        if reply.status is not PlanStatus.OK:
            log.warning(
                "request %s detail: %s", request.request_id.hex()[:8], reply.detail
            )
        return reply

    def stop(signum: int, frame: FrameType | None) -> None:
        """Xử lý tín hiệu dừng service.

        Args:
            signum: Mã tín hiệu hệ thống.
            frame: Khung thực thi hiện tại.
        """
        log.info("nhận tín hiệu %s, đang dừng", signal.Signals(signum).name)
        transport.stop()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        transport.serve(handle)
    finally:
        runner.stop()
        log.info("đã dừng")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
