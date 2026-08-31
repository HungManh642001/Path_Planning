"""Kiểm thử đơn vị cho module đọc nạp file bản đồ nền XML service.vtx_service.map_file."""

from __future__ import annotations

from pathlib import Path

import pytest

from service.vtx_service.map_file import PreloadedMap
from service.vtx_service.messages import (
    IDL_VERSION,
    Circle,
    PlanRequest,
    SearchBudget,
    VehicleLimits,
)


MAP_XML = """<vtx-map version="1">
  <safezones>
    <polygon>
      <point x="0" y="0"/>
      <point x="500000" y="0"/>
      <point x="500000" y="500000"/>
      <point x="0" y="500000"/>
    </polygon>
  </safezones>
  <obstacles>
    <polygon>
      <point x="150000" y="120000"/>
      <point x="200000" y="120000"/>
      <point x="175000" y="200000"/>
    </polygon>
    <circle cx="220000" cy="180000" r="15000"/>
  </obstacles>
</vtx-map>
"""


def _write_temp_map(tmp_path: Path, text: str = MAP_XML) -> Path:
    """Ghi nội dung XML ra file tạm để kiểm thử."""
    path = tmp_path / "basemap.xml"
    path.write_text(text, encoding="utf-8")
    return path


def _build_request(**overrides: object) -> PlanRequest:
    """Khởi tạo PlanRequest mặc định cho test nạp bản đồ."""
    base: dict[str, object] = {
        "request_id": b"\x05" * 16,
        "idl_version": IDL_VERSION,
        "start": (50000.0, 50000.0),
        "start_heading_deg": 45.0,
        "goal": (300000.0, 250000.0),
        "goal_heading_deg": 45.0,
        "is_goal_heading_free": True,
        "islands": (),
        "dynamic_obstacles": (),
        "safezones": (),
        "use_preloaded_map": False,
        "limits": VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0),
        "budget": SearchBudget(15.0),
    }
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


def test_load_reads_safezones_islands_and_circles(tmp_path: Path) -> None:
    """Kiểm tra PreloadedMap.load đọc đầy đủ cả 3 loại hình học từ XML."""
    # Arrange
    xml_path = _write_temp_map(tmp_path)

    # Act
    loaded = PreloadedMap.load(xml_path)

    # Assert
    assert len(loaded.safezones) == 1
    assert len(loaded.safezones[0]) == 4
    assert loaded.islands == (
        ((150000.0, 120000.0), (200000.0, 120000.0), (175000.0, 200000.0)),
    )
    assert loaded.dynamic_obstacles == (
        Circle(center=(220000.0, 180000.0), radius_m=15000.0),
    )


def test_repeated_closing_vertex_is_trimmed(tmp_path: Path) -> None:
    """Kiểm tra loại bỏ đỉnh đóng lặp lại để tạo vành mở chuẩn."""
    # Arrange
    closed_xml = MAP_XML.replace(
        '<point x="175000" y="200000"/>',
        '<point x="175000" y="200000"/>\n      <point x="150000" y="120000"/>',
    )
    xml_path = _write_temp_map(tmp_path, closed_xml)

    # Act
    loaded = PreloadedMap.load(xml_path)

    # Assert
    assert len(loaded.islands[0]) == 3


def test_merged_into_appends_map_data_to_request(tmp_path: Path) -> None:
    """Kiểm tra gộp dữ liệu bản đồ nền vào PlanRequest khi cờ use_preloaded_map bật."""
    # Arrange
    loaded = PreloadedMap.load(_write_temp_map(tmp_path))
    request = _build_request(
        use_preloaded_map=True,
        dynamic_obstacles=(Circle(center=(100000.0, 100000.0), radius_m=5000.0),),
        safezones=(((0.0, 0.0), (10.0, 0.0), (10.0, 10.0)),),
    )

    # Act
    merged = loaded.merged_into(request)

    # Assert
    assert len(merged.dynamic_obstacles) == 2
    assert len(merged.safezones) == 2
    assert merged.dynamic_obstacles[0].radius_m == 5000.0


def test_merged_into_returns_identical_object_when_flag_off(tmp_path: Path) -> None:
    """Kiểm tra trả về chính đối tượng request gốc khi use_preloaded_map tắt."""
    # Arrange
    loaded = PreloadedMap.load(_write_temp_map(tmp_path))
    request = _build_request(use_preloaded_map=False)

    # Act
    merged = loaded.merged_into(request)

    # Assert
    assert merged is request


def test_unsupported_version_raises_value_error(tmp_path: Path) -> None:
    """Kiểm tra từ chối phiên bản file XML không được hỗ trợ."""
    # Arrange
    bad_version = MAP_XML.replace('version="1"', 'version="9"')
    xml_path = _write_temp_map(tmp_path, bad_version)

    # Act & Assert
    with pytest.raises(ValueError, match="version"):
        PreloadedMap.load(xml_path)


def test_polygon_with_fewer_than_three_points_is_rejected(tmp_path: Path) -> None:
    """Kiểm tra từ chối đa giác có ít hơn 3 đỉnh."""
    # Arrange
    thin = MAP_XML.replace('      <point x="175000" y="200000"/>\n', "")
    xml_path = _write_temp_map(tmp_path, thin)

    # Act & Assert
    with pytest.raises(ValueError, match="3 đỉnh"):
        PreloadedMap.load(xml_path)


def test_shipped_example_basemap_parses_successfully() -> None:
    """Kiểm tra file cấu hình basemap mẫu đóng gói cùng repo parse thành công."""
    # Arrange
    repo_root = Path(__file__).resolve().parents[3]
    example = repo_root / "src" / "service" / "deploy" / "basemap.example.xml"

    # Act
    loaded = PreloadedMap.load(example)

    # Assert
    assert loaded.safezones or loaded.islands or loaded.dynamic_obstacles
