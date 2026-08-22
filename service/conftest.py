"""Đưa gốc repo và thư mục service lên sys.path.

Cùng cơ chế import mà service dùng lúc chạy thật, nên test chạy đúng cấu hình
của production.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SERVICE_ROOT = REPO_ROOT / "service"

for path in (REPO_ROOT, SERVICE_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
