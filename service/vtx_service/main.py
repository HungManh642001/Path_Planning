"""Vòng đời service: nạp bản đồ, khởi động runner, rồi mới lên DDS.

THỨ TỰ Ở ĐÂY LÀ MỘT RÀNG BUỘC, KHÔNG PHẢI SỞ THÍCH. `PlanRunner.start()` phải
chạy trước khi khởi tạo DDS: nó ép tiến trình forkserver ra đời trong lúc tiến
trình này còn sạch thread. Nếu để DDS lên trước, forkserver sẽ ra đời từ một
tiến trình đang chạy thread nền của DDS, và mọi tiến trình con sau đó thừa
hưởng rủi ro deadlock đúng như mục 3 của spec mô tả.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
from pathlib import Path
from types import FrameType


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="VTX path planning DDS service")
    parser.add_argument("--domain-id", type=int, default=0)
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

    from vtx_service.map_file import PreloadedMap
    from vtx_service.messages import PlanRequest, PlanReply, PlanStatus
    from vtx_service.runner import PlanRunner
    from vtx_service.runtime import config_hash, effective_time_budget_s, planner_version

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
    runner.start()  # PHẢI trước DDS - xem docstring của module
    log.info(
        "planner %s, config %s, ngân sách thực tế %.1f s",
        planner_version(),
        config_hash(),
        effective_time_budget_s(),
    )

    from vtx_service.transport import DdsTransport

    transport = DdsTransport(domain_id=args.domain_id)
    log.info("sẵn sàng trên domain %d", args.domain_id)

    def handle(request: PlanRequest) -> PlanReply:
        reply = runner.submit(request)
        log.info(
            "request %s -> %s, %d waypoint, %.3f s",
            request.request_id.hex()[:8],
            reply.status.name,
            len(reply.waypoints),
            reply.plan_wall_time_s,
        )
        return reply

    def stop(signum: int, frame: FrameType | None) -> None:
        log.info("nhận tín hiệu %s, đang dừng", signal.Signals(signum).name)
        transport.close()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        transport.serve(handle)
    finally:
        transport.close()
        runner.stop()
        log.info("đã dừng")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
