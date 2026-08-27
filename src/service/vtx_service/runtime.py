"""Siêu dữ liệu phiên bản và cấu hình đi kèm mỗi reply.

Client phải phân biệt được hai đường bay khác nhau là do input khác hay do cấu
hình planner khác. Trên một codebase nghiên cứu nơi các hằng số được A/B liên
tục, đó không phải tiện nghi mà là điều kiện để reply có nghĩa.
"""

from __future__ import annotations

import hashlib
import inspect
import json
import logging
import re
import subprocess
from pathlib import Path

from path_planning import config, planner as astar


_log = logging.getLogger("vtx-planner")

_CONFIG_REF = re.compile(r"\bconfig\.([A-Z][A-Z0-9_]*)\b")
_REPO_ROOT = Path(__file__).resolve().parents[2]


def planner_config_snapshot() -> dict[str, object]:
    """Liệt kê các hằng số ``config`` mà planner đang ship thực sự đọc.

    Danh sách được PHÁT HIỆN bằng cách quét mã nguồn planner chứ không hardcode,
    nên một knob mới xuất hiện trong reply mà không ai phải nhớ cập nhật chỗ này.

    Returns:
        Ánh xạ tên hằng số sang giá trị hiện tại, sắp theo tên.
    """
    import path_planning.collision.detector as _cd
    import path_planning.search.astar as _sa
    import path_planning.search.heuristic as _sh
    import path_planning.search.state as _ss
    import path_planning.search.successors as _sg
    import path_planning.trajectory.mission as _tm
    import path_planning.trajectory.smoothing as _ts

    modules = [astar, _cd, _sa, _sh, _ss, _sg, _tm, _ts]
    all_source = "\n".join(inspect.getsource(m) for m in modules)
    names = sorted(set(_CONFIG_REF.findall(all_source)))
    return {name: getattr(config, name) for name in names if hasattr(config, name)}


def config_hash() -> str:
    """Băm rút gọn của cấu hình planner hiện hành.

    Returns:
        16 ký tự hex đầu của SHA-256 trên snapshot đã chuẩn hoá.
    """
    blob = json.dumps(planner_config_snapshot(), sort_keys=True, default=repr)
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:16]


def _describe_version() -> str:
    """Chạy ``git describe`` thật. Chỉ gọi một lần, ngay dưới đây, lúc import.

    R26: timeout 30 s, không phải 5 s. ĐO ĐƯỢC ``git describe`` trên mount này
    mất 3,60-4,84 s NGAY CẢ Ở TRẠNG THÁI BÌNH THƯỜNG (không phải sự cố) - dưới
    tải của cả bộ test full suite chạy song song nó vượt qua 5 s, rơi vào
    ``except`` bên dưới, và hàm ÂM THẦM trả về ``"unknown"``. Đây CHÍNH XÁC là
    thất bại mà R9/R10 tồn tại để ngăn (một dấu phiên bản vô nghĩa đi vào MỌI
    reply mà không ai biết), chỉ khác hướng tới: lần này do timeout đua với
    tải hệ thống, không phải do chỉ số thư mục sai. Chi phí trả MỘT LẦN lúc
    import, không nằm trên đường request (xem docstring của
    ``_PLANNER_VERSION`` bên dưới), nên một ngân sách rộng rãi không tốn gì về
    vận hành - cái không chấp nhận được là SỰ IM LẶNG, không phải thời lượng.
    Vì vậy timeout được nới rộng THAY VÌ giữ chặt, và mọi lần thật sự rơi vào
    fallback đều phải LOG, không được trôi qua không dấu vết. ĐỪNG chỉnh số
    này xuống lại dựa trên trực giác "5 s là đủ" - nó đã đo được KHÔNG đủ ngay
    cả không có tải bất thường.
    """
    try:
        proc = subprocess.run(
            ["git", "describe", "--always", "--dirty"],
            cwd=_REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=30,
            check=True,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        _log.warning(
            "planner_version(): 'git describe' thất bại (%s) - dùng 'unknown'. "
            "Mọi reply từ tiến trình này sẽ mang một dấu phiên bản vô nghĩa.",
            exc,
        )
        return "unknown"
    return proc.stdout.strip() or "unknown"


_PLANNER_VERSION = _describe_version()
"""Tính NGAY LÚC IMPORT module này, không phải lúc gọi ``planner_version()``.

Bắt buộc phải import-time, không phải lazy (một cache "tính lúc gọi đầu tiên"
kiểu ``functools.lru_cache`` hay sentinel biến toàn cục) - và lý do là cơ chế
của ``forkserver``, đã xác minh: ``ForkServer.ensure_running`` khởi động bằng
``spawnv_passfds``, tức là fork **rồi exec một interpreter HOÀN TOÀN MỚI**.
Tiến trình forkserver không kế thừa trạng thái đã import của tiến trình cha -
nó tự import lại từ đầu danh sách ``_PRELOAD`` (``runner.py``), trong đó có
``vtx_service.planner`` mà module đó import module này. Nên một cache
kiểu-lazy thì mỗi tiến trình con dùng-một-lần (``PlanRunner`` tạo mới cho MỖI
request) tự trả phí ``git describe`` riêng của nó rồi chết theo nó ngay khi
request xong - đo được vẫn mất 3,79-5,12 s MỖI request dù đã cache kiểu đó,
tức là không rẻ đi chút nào.

Tính ở MODULE LEVEL thì khác: tiến trình forkserver, khi khởi động (một lần,
trước khi DDS tồn tại - xem ``PlanRunner.start()``), tự import module này như
một phần của ``_PRELOAD`` và trả phí ``git describe`` đúng MỘT LẦN ở đó. Mọi
tiến trình con fork() ra sau, kế thừa qua copy-on-write, đã có sẵn giá trị này
trong bộ nhớ - không tốn subprocess nào trên đường request nữa.

Hệ quả vận hành: giá trị này CỐ ĐỊNH cho tới khi forkserver khởi động lại, nên
sau ``git pull`` phải khởi động lại service mới thấy version mới - quy trình
triển khai đã yêu cầu đúng như vậy từ trước.

ĐỪNG "tối ưu" chỗ này thành lazy/``lru_cache`` - trông giống refactor vô hại
nhưng âm thầm khôi phục lại chi phí trên mỗi request, vì phép tính khi đó lại
rơi vào đúng tiến trình con dùng-một-lần mà nó cần tránh.
"""


def planner_version() -> str:
    """Mô tả phiên bản mã nguồn đang chạy.

    Trả về ``_PLANNER_VERSION``, tính một lần lúc MODULE này được import (xem
    docstring của hằng số đó) - hàm ở đây chỉ để giữ API ổn định cho chỗ gọi.

    Returns:
        Kết quả ``git describe --always --dirty`` tại lúc tiến trình khởi
        động, hoặc ``"unknown"`` khi không chạy được (bản triển khai không có
        git, hoặc không phải một repo).
    """
    return _PLANNER_VERSION


MAX_REQUEST_TIME_BUDGET_S = 300.0
"""Trần cứng cho ngân sách một client được phép xin.

Service phục vụ TUẦN TỰ trên một reader KEEP_ALL: mọi request đến trong lúc
một request đang chạy đều XẾP HÀNG phía sau nó. Nên một đề nghị "cho tôi một
giờ" không chỉ là một mission chậm - nó là một giờ ngừng phục vụ, do một client
đơn lẻ quyết định.

Đây cũng là thứ giữ cho thời hạn cứng của ``PlanRunner`` luôn hữu hạn: thời hạn
đó là ngân sách đã áp dụng cộng thời gian ân hạn, và ngân sách đã áp dụng không
bao giờ vượt con số này. Trước đây vai trò ấy do một hằng số riêng
(``unlimited_deadline_s``) đảm nhiệm, cần thiết vì ngân sách có thể là "không
giới hạn"; giờ không còn khái niệm đó nữa nên chỉ còn một con số.
"""


def effective_time_budget_s(requested_s: float = 0.0) -> float:
    """Ngân sách thời gian service THỰC SỰ dùng cho một request.

    Đề nghị của client ĐƯỢC tôn trọng, nhưng service vẫn giữ quyền quyết định
    ở hai đầu: một đề nghị trống rơi về mặc định, một đề nghị quá lớn bị kẹp.
    Reply báo cáo ngược đúng giá trị hàm này trả về, nên client luôn biết đề
    nghị của mình được nhận nguyên vẹn hay đã bị thay.

    Args:
        requested_s: Giá trị client gửi. ``<= 0`` nghĩa là "không đề nghị gì".
            Rác (không phải số hữu hạn) cũng được coi như vậy thay vì ném lỗi:
            nó đến từ dây, và một tiến trình phục vụ tuần tự không được chết vì
            một byte sai của một client.

    Returns:
        Ngân sách tính bằng giây, luôn hữu hạn và > 0.
    """
    try:
        return min(config.resolve_time_budget_s(requested_s), MAX_REQUEST_TIME_BUDGET_S)
    except (ValueError, TypeError):
        return config.resolve_time_budget_s(None)
