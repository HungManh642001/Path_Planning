"""Chạy mỗi lần lập kế hoạch trong một tiến trình con, với thời hạn cứng.

Hai lý do, cả hai đều thật:

Thứ nhất, planner là Python thuần, CPU-bound, và chỉ kiểm tra ngân sách tại các
điểm trong vòng lặp search - nó KHÔNG hủy được từ bên ngoài một cách lịch sự.
Giết một tiến trình con là cách trung thực duy nhất để có thời hạn cứng.

Thứ hai, planner đọc 35 hằng số global từ ``config``. Trong một tiến trình con
dùng-một-lần thì mọi thay đổi đều chết theo nó.

VÌ SAO ``forkserver`` CHỨ KHÔNG PHẢI ``fork``: DDS chạy thread nền ở tầng C, và
``fork()`` từ một tiến trình có thread là công thức kinh điển của deadlock trong
bản sao - fork chỉ mang theo thread đang gọi, nên một mutex do thread khác đang
giữ sẽ bị giữ vĩnh viễn. Đo trên máy phát triển: ``fork`` với ``core`` nạp sẵn
là 37,7 ms và sống sót 15/15 lần dưới lưu lượng DDS, nhưng 15 lần thành công
không phải bằng chứng an toàn cho một deadlock xác suất. ``forkserver`` +
preload, khởi động TRƯỚC khi DDS tồn tại, là 56,4 ms và an toàn về CẤU TRÚC.
40 ms là vô nghĩa so với 16 ms - 4 s thời gian lập kế hoạch.

Hệ quả với chỗ gọi: :meth:`PlanRunner.start` phải chạy TRƯỚC khi khởi tạo bất kỳ
thứ gì thuộc DDS.
"""

from __future__ import annotations

import multiprocessing as mp
import os
import time
import traceback
from multiprocessing.connection import Connection
from pathlib import Path

from service.vtx_service.map_file import PreloadedMap
from service.vtx_service.messages import (
    IDL_VERSION,
    PlanReply,
    PlanRequest,
    PlanStatus,
    SearchStats,
)

_PRELOAD = [
    "config",
    "core.types",
    "core.spatial_utils",
    "core.preprocessing",
    "core.path_validation",
    "core.mission",
    "core.arc_geometry",
    "core.goal_shot",
    "core.kinodynamic_astar_v0",
    "vtx_service.planner",
]
"""Module nạp sẵn trong tiến trình forkserver.

Trả giá import một lần thay vì mỗi request: đo được 1012 ms xuống 56 ms.
"""


_REPO_ROOT = Path(__file__).resolve().parents[3]
_SERVICE_ROOT = Path(__file__).resolve().parents[1]


def _ensure_pythonpath_for_forkserver() -> None:
    """Đảm bảo tiến trình forkserver import được ``_PRELOAD`` - ĐO ĐƯỢC cái giá
    của việc không làm điều này: ~50x mỗi request, ÂM THẦM.

    ``forkserver`` là fork+exec một interpreter HOÀN TOÀN MỚI (xem docstring
    module), nên nó đọc biến môi trường ``PYTHONPATH`` của tiến trình cha -
    KHÔNG thấy được các mục ``sys.path`` mà cha thêm LÚC CHẠY (ví dụ
    ``conftest.py`` chèn gốc repo và ``service/`` vào ``sys.path`` mỗi lần
    pytest chạy, nhưng không đặt biến môi trường). Khi forkserver không import
    được một module trong ``_PRELOAD``, ``multiprocessing`` NUỐT ImportError
    ÂM THẦM - không lỗi, không log - và mỗi tiến trình con dùng-một-lần quay về
    tự import mọi thứ từ đầu, kể cả ``git describe`` lúc import module
    (``runtime._PLANNER_VERSION`` - chính thứ R15 định loại khỏi đường
    request). ĐO ĐƯỢC trên máy này, 3 lần submit mỗi cách:
    ``sys.path`` sửa lúc chạy (như conftest.py làm)  -> 3,52 / 3,69 / 4,05 s
    ``PYTHONPATH`` đặt qua biến môi trường            -> 0,07 / 0,07 / 0,08 s
    ~50x, và KHÔNG hiện ra ở đâu cả trừ đồng hồ treo tường.

    Production tình cờ ổn vì unit systemd đặt ``PYTHONPATH``. Nhưng phụ thuộc
    đó VÔ HÌNH: chạy service theo cách khác (vd. gọi tay ``python -m
    vtx_service.main`` từ một checkout không đặt biến) là chậm hơn 50x mà
    không một cảnh báo nào. Nên KHÔNG dựa vào chỗ gọi đã đặt đúng biến -
    ``start()`` tự đảm bảo lấy, PHẢI gọi TRƯỚC ``mp.get_context("forkserver")``.

    NỐI THÊM vào ``PYTHONPATH`` đã có, không THAY THẾ: một chỗ gọi có thể có
    mục riêng hợp lệ trong đó. Bỏ qua mục nào đã có sẵn.
    """
    existing = os.environ.get("PYTHONPATH", "")
    entries = [p for p in existing.split(os.pathsep) if p]
    for required in (str(_REPO_ROOT), str(_SERVICE_ROOT)):
        if required not in entries:
            entries.append(required)
    os.environ["PYTHONPATH"] = os.pathsep.join(entries)


def _child(pipe: Connection, request: PlanRequest, preloaded: PreloadedMap | None,
           hang: bool, raise_: bool) -> None:
    """Thân tiến trình con: lập kế hoạch, gửi reply, thoát."""
    try:
        if hang:
            while True:
                time.sleep(3600)
        if raise_:
            raise RuntimeError("lỗi giả lập trong tiến trình con")
        from service.vtx_service.planner import plan

        pipe.send(("ok", plan(request, preloaded=preloaded)))
    except BaseException:  # noqa: BLE001 - báo lỗi về cha thay vì chết câm
        pipe.send(("loi", traceback.format_exc(limit=5)))
    finally:
        pipe.close()


class PlanRunner:
    """Chạy các request lần lượt, mỗi request một tiến trình con."""

    def __init__(self, preloaded: PreloadedMap | None, grace_s: float = 2.0) -> None:
        """Khởi tạo.

        Args:
            preloaded: Bản đồ nền tĩnh, hoặc ``None``.
            grace_s: Cộng thêm vào ngân sách ĐÃ ÁP DỤNG của request để ra thời
                hạn cứng. Ngân sách đó luôn hữu hạn và bị kẹp dưới
                ``runtime.MAX_REQUEST_TIME_BUDGET_S``, nên thời hạn này cũng
                vậy - không còn cần một trần tuyệt đối riêng như trước, hồi
                ngân sách còn có thể là "không giới hạn" (F1).
        """
        self._preloaded = preloaded
        self._grace_s = grace_s
        self._ctx: mp.context.BaseContext | None = None
        # Cửa hậu chỉ dùng cho test; production không bao giờ đặt chúng.
        self._force_hang_next = False
        self._force_raise_next = False

    def start(self) -> None:
        """Khởi động forkserver. PHẢI gọi trước khi khởi tạo DDS."""
        _ensure_pythonpath_for_forkserver()
        mp.set_forkserver_preload(_PRELOAD)
        self._ctx = mp.get_context("forkserver")
        # Ép forkserver ra đời NGAY BÂY GIỜ, trong khi tiến trình này còn sạch
        # thread. Nếu để nó ra đời ở request đầu tiên thì DDS đã lên rồi.
        # `join()` chứ không bỏ mặc: một Process không join để lại zombie tới
        # khi bị thu gom, và ở đây không có lý do gì để không chờ nó.
        primer = self._ctx.Process(target=_noop)
        primer.start()
        primer.join()

    def submit(self, request: PlanRequest) -> PlanReply:
        """Lập kế hoạch cho một request, với thời hạn cứng.

        Args:
            request: Mission cần giải.

        Returns:
            Reply của planner, hoặc một reply ``TIMEOUT`` / ``INTERNAL_ERROR``.
        """
        assert self._ctx is not None, "phải gọi start() trước"
        from service.vtx_service.runtime import effective_time_budget_s

        hang, self._force_hang_next = self._force_hang_next, False
        raise_, self._force_raise_next = self._force_raise_next, False

        started = time.perf_counter()
        parent, child = self._ctx.Pipe(duplex=False)
        process = self._ctx.Process(
            target=_child, args=(child, request, self._preloaded, hang, raise_)
        )
        process.start()
        child.close()

        # Cùng phép giải như tiến trình con sẽ dùng, nên thời hạn cứng ở đây
        # bám đúng ngân sách mà planner thực sự chạy dưới - kể cả khi client
        # xin ít hơn hoặc nhiều hơn mặc định của service.
        budget_s = effective_time_budget_s(request.budget.time_budget_s)
        deadline_s = budget_s + self._grace_s
        if not parent.poll(timeout=deadline_s):
            process.kill()
            process.join(timeout=10)
            parent.close()
            elapsed_s = time.perf_counter() - started
            return self._failed(request, PlanStatus.TIMEOUT,
                                f"vượt thời hạn cứng {deadline_s:.1f} s", elapsed_s)

        try:
            tag, payload = parent.recv()
        except EOFError:
            tag, payload = "loi", "tiến trình con chết không gửi gì"
        finally:
            parent.close()
            process.join(timeout=10)

        if tag != "ok":
            elapsed_s = time.perf_counter() - started
            return self._failed(request, PlanStatus.INTERNAL_ERROR, str(payload), elapsed_s)
        return payload

    def stop(self) -> None:
        """Dừng runner. Không có tiến trình sống lâu nào phải dọn."""
        self._ctx = None

    @staticmethod
    def _failed(
        request: PlanRequest, status: PlanStatus, detail: str, elapsed_s: float
    ) -> PlanReply:
        from service.vtx_service.runtime import (
            config_hash,
            effective_time_budget_s,
            planner_version,
        )

        # budget_bound chỉ đúng cho TIMEOUT: đó là trạng thái DUY NHẤT nơi
        # ngân sách thời gian là lý do request không xong. Một tiến trình con
        # NÉM LỖI (INTERNAL_ERROR) không hề chạm trần thời gian.
        budget_bound = status is PlanStatus.TIMEOUT
        return PlanReply(
            request_id=request.request_id,
            idl_version=IDL_VERSION,
            status=status,
            detail=detail,
            waypoints=(),
            path_length_m=0.0,
            plan_wall_time_s=elapsed_s,
            applied_time_budget_s=effective_time_budget_s(request.budget.time_budget_s),
            stats=SearchStats(0, 0, True, budget_bound),
            planner_version=planner_version(),
            config_hash=config_hash(),
        )


def _noop() -> None:
    """Thân tiến trình rỗng, chỉ để ép forkserver khởi động sớm."""
