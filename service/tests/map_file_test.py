"""Bản đồ nền XML: nạp một lần lúc khởi động, gộp vào request khi được yêu cầu.

Mặc định triển khai là KHÔNG nạp bản đồ nào - request tự chứa thì replay được và
chẩn đoán được, còn state ẩn trong service thì không.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from vtx_service.map_file import PreloadedMap
from vtx_service.messages import Circle, PlanRequest, SearchBudget, VehicleLimits

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


def _write(tmp_path: Path, text: str = MAP_XML) -> Path:
    path = tmp_path / "basemap.xml"
    path.write_text(text, encoding="utf-8")
    return path


def _request(**overrides: object) -> PlanRequest:
    base: dict[str, object] = dict(
        request_id=b"\x05" * 16,
        idl_version=1,
        start=(50000.0, 50000.0),
        start_heading_deg=45.0,
        goal=(300000.0, 250000.0),
        goal_heading_deg=45.0,
        goal_heading_free=True,
        islands=(),
        dynamic_obstacles=(),
        safezones=(),
        use_preloaded_map=False,
        limits=VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0),
        budget=SearchBudget(15.0, 50000),
    )
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


def test_load_reads_all_three_kinds(tmp_path: Path) -> None:
    loaded = PreloadedMap.load(_write(tmp_path))
    assert len(loaded.safezones) == 1
    assert len(loaded.safezones[0]) == 4
    assert loaded.islands == (((150000.0, 120000.0), (200000.0, 120000.0), (175000.0, 200000.0)),)
    assert loaded.dynamic_obstacles == (Circle(center=(220000.0, 180000.0), radius_m=15000.0),)


def test_a_repeated_closing_vertex_is_trimmed(tmp_path: Path) -> None:
    """`core/` giả định vành MỞ; một đỉnh lặp tạo ra cạnh dài 0."""
    closed = MAP_XML.replace(
        '<point x="175000" y="200000"/>',
        '<point x="175000" y="200000"/>\n      <point x="150000" y="120000"/>',
    )
    loaded = PreloadedMap.load(_write(tmp_path, closed))
    assert len(loaded.islands[0]) == 3


def test_merge_appends_and_does_not_replace(tmp_path: Path) -> None:
    loaded = PreloadedMap.load(_write(tmp_path))
    request = _request(
        use_preloaded_map=True,
        dynamic_obstacles=(Circle(center=(100000.0, 100000.0), radius_m=5000.0),),
        safezones=(((0.0, 0.0), (10.0, 0.0), (10.0, 10.0)),),
    )
    merged = loaded.merged_into(request)
    assert len(merged.dynamic_obstacles) == 2
    assert len(merged.safezones) == 2
    # Của request đứng trước, để đọc log dễ đối chiếu.
    assert merged.dynamic_obstacles[0].radius_m == 5000.0


def test_flag_off_returns_the_very_same_object(tmp_path: Path) -> None:
    loaded = PreloadedMap.load(_write(tmp_path))
    request = _request(use_preloaded_map=False)
    assert loaded.merged_into(request) is request


def test_a_wrong_version_is_an_error_not_a_warning(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="version"):
        PreloadedMap.load(_write(tmp_path, MAP_XML.replace('version="1"', 'version="9"')))


def test_a_wrong_root_tag_is_rejected(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="vtx-map"):
        PreloadedMap.load(_write(tmp_path, MAP_XML.replace("vtx-map", "other-map")))


def test_a_polygon_with_two_points_is_rejected(tmp_path: Path) -> None:
    thin = MAP_XML.replace('      <point x="175000" y="200000"/>\n', "")
    with pytest.raises(ValueError, match="3 đỉnh"):
        PreloadedMap.load(_write(tmp_path, thin))


def test_a_non_positive_radius_is_rejected(tmp_path: Path) -> None:
    path = _write(tmp_path, MAP_XML.replace('r="15000"', 'r="0"'))
    with pytest.raises(ValueError, match="radius") as exc_info:
        PreloadedMap.load(path)
    # Phải phân biệt được với lỗi bare "radius_m" của Circle.__post_init__ -
    # thông báo phải nêu đích danh file.
    assert str(path) in str(exc_info.value)


def test_a_missing_point_attribute_is_rejected(tmp_path: Path) -> None:
    bad = MAP_XML.replace('<point x="175000" y="200000"/>', '<point y="200000"/>')
    path = _write(tmp_path, bad)
    with pytest.raises(ValueError) as exc_info:
        PreloadedMap.load(path)
    assert str(path) in str(exc_info.value)


def test_a_missing_circle_attribute_is_rejected(tmp_path: Path) -> None:
    bad = MAP_XML.replace('cx="220000" cy="180000" r="15000"', 'cy="180000" r="15000"')
    path = _write(tmp_path, bad)
    with pytest.raises(ValueError) as exc_info:
        PreloadedMap.load(path)
    assert str(path) in str(exc_info.value)


def test_a_non_numeric_coordinate_is_rejected(tmp_path: Path) -> None:
    bad = MAP_XML.replace('x="175000"', 'x="not-a-number"')
    path = _write(tmp_path, bad)
    with pytest.raises(ValueError) as exc_info:
        PreloadedMap.load(path)
    assert str(path) in str(exc_info.value)


def test_empty_sections_are_allowed(tmp_path: Path) -> None:
    loaded = PreloadedMap.load(
        _write(tmp_path, '<vtx-map version="1"><safezones/><obstacles/></vtx-map>')
    )
    assert loaded.safezones == () and loaded.islands == () and loaded.dynamic_obstacles == ()


def test_the_shipped_example_file_parses() -> None:
    example = Path(__file__).resolve().parents[1] / "deploy" / "basemap.example.xml"
    loaded = PreloadedMap.load(example)
    assert loaded.safezones or loaded.islands or loaded.dynamic_obstacles
