"""Ràng buộc số 1: service không được sửa thuật toán.

Cơ chế cưỡng chế, không phải lời nhắc. So với nhánh gốc `main` nên nó đỏ ngay cả
khi thay đổi đã được commit.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PROTECTED = ["core/", "render/", "config.py"]


def test_service_work_does_not_touch_the_algorithm() -> None:
    proc = subprocess.run(
        ["git", "diff", "--stat", "main", "--", *PROTECTED],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    assert proc.stdout.strip() == "", (
        "Nhánh service đã sửa thuật toán, điều bị cấm bởi ràng buộc 1 của spec.\n"
        f"Thay đổi:\n{proc.stdout}"
    )


def test_service_tree_does_not_copy_the_algorithm() -> None:
    service = REPO_ROOT / "service"
    assert service.is_dir()
    assert not (service / "core").exists(), "core/ không được sao chép vào service/"
