"""Siêu dữ liệu phiên bản và cấu hình đi kèm mỗi reply.

Client phải phân biệt được hai đường bay khác nhau là do input khác hay do cấu
hình planner khác. Trên một codebase nghiên cứu nơi các hằng số được A/B liên
tục, đó không phải tiện nghi mà là điều kiện để reply có nghĩa.
"""

from __future__ import annotations

import hashlib
import inspect
import json
import re
import subprocess
from pathlib import Path

import config
import core.kinodynamic_astar_v0 as astar

_CONFIG_REF = re.compile(r"\bconfig\.([A-Z][A-Z0-9_]*)\b")
_REPO_ROOT = Path(__file__).resolve().parents[2]


def planner_config_snapshot() -> dict[str, object]:
    """Liệt kê các hằng số ``config`` mà planner đang ship thực sự đọc.

    Danh sách được PHÁT HIỆN bằng cách quét mã nguồn planner chứ không hardcode,
    nên một knob mới xuất hiện trong reply mà không ai phải nhớ cập nhật chỗ này.

    Returns:
        Ánh xạ tên hằng số sang giá trị hiện tại, sắp theo tên.
    """
    names = sorted(set(_CONFIG_REF.findall(inspect.getsource(astar))))
    return {name: getattr(config, name) for name in names if hasattr(config, name)}


def config_hash() -> str:
    """Băm rút gọn của cấu hình planner hiện hành.

    Returns:
        16 ký tự hex đầu của SHA-256 trên snapshot đã chuẩn hoá.
    """
    blob = json.dumps(planner_config_snapshot(), sort_keys=True, default=repr)
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:16]


_version_cache: str | None = None
"""Kết quả ``planner_version()``, tính một lần cho cả vòng đời tiến trình."""


def planner_version() -> str:
    """Mô tả phiên bản mã nguồn đang chạy.

    Tính MỘT LẦN rồi nhớ lại cho hết vòng đời tiến trình, không phải mỗi lần
    gọi: đo được ``git describe`` một mình mất 3,5-4,9 s trên filesystem 9p
    của máy này, và hàm này chạy trên MỌI reply (``plan()`` gọi nó, và
    ``PlanRunner._failed()`` cũng gọi) - trong khi một mission trung vị lập kế
    hoạch xong trong 16 ms. Cache là đúng về ngữ nghĩa chứ không phải mẹo: mã
    nguồn không đổi trong lúc tiến trình đang sống, và quy trình triển khai đã
    yêu cầu khởi động lại service sau mỗi ``git pull`` - nên giá trị này vốn dĩ
    bất biến suốt một tiến trình. Kết quả lỗi (``"unknown"``) cũng được nhớ lại
    như bất kỳ giá trị nào khác: không có gì để thử lại giữa các lần gọi khi
    git không chạy được.

    Returns:
        Kết quả ``git describe --always --dirty``, hoặc ``"unknown"`` khi không
        chạy được (bản triển khai không có git, hoặc không phải một repo).
    """
    global _version_cache
    if _version_cache is None:
        _version_cache = _describe_version()
    return _version_cache


def _describe_version() -> str:
    """Chạy ``git describe`` thật. Chỉ ``planner_version()`` gọi, đúng một lần."""
    try:
        proc = subprocess.run(
            ["git", "describe", "--always", "--dirty"],
            cwd=_REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=5,
            check=True,
        )
    except (OSError, subprocess.SubprocessError):
        return "unknown"
    return proc.stdout.strip() or "unknown"


def effective_time_budget_s() -> float:
    """Ngân sách thời gian service THỰC SỰ dùng.

    Lấy từ ``config``, không phải từ request: xem mục 4.3 của spec. Reply báo
    cáo ngược giá trị này để client không tưởng rằng đề nghị của mình được nhận.

    Returns:
        Ngân sách tính bằng giây; ``0.0`` nghĩa là không giới hạn.
    """
    return float(config.TIME_BUDGET_S or 0.0)


def effective_max_iterations() -> int:
    """Trần số vòng lặp service THỰC SỰ dùng. Cùng lý do như trên."""
    return int(config.MAX_ITERATIONS)
