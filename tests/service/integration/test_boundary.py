"""Kiểm thử tích hợp ranh giới kiến trúc: Service không sao chép hoặc can thiệp trực tiếp vào nội bộ thuật toán."""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]


def test_service_tree_does_not_duplicate_path_planning_core() -> None:
    """Kiểm tra thư mục src/service không chứa bản sao chép nội bộ của module path_planning."""
    # Arrange
    service_dir = REPO_ROOT / "src" / "service"

    # Act & Assert
    assert service_dir.is_dir(), "Thư mục src/service/ phải tồn tại"
    assert not (service_dir / "path_planning").exists(), (
        "Module path_planning không được sao chép trực tiếp vào src/service/"
    )
    assert not (service_dir / "core").exists(), (
        "Thư mục core legacy không được xuất hiện trong src/service/"
    )


def test_service_imports_path_planning_via_clean_interface() -> None:
    """Kiểm tra mã nguồn service chỉ tương tác với path_planning qua các module chuẩn."""
    # Arrange
    vtx_service_dir = REPO_ROOT / "src" / "service" / "vtx_service"
    service_python_files = list(vtx_service_dir.glob("*.py"))

    # Act & Assert
    assert len(service_python_files) > 0
    for py_file in service_python_files:
        content = py_file.read_text(encoding="utf-8")
        # Kiểm tra không có import tương đối xuyên module
        assert "from ..path_planning" not in content
        assert "from ...path_planning" not in content
