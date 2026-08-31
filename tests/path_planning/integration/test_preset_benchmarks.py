"""Kiểm thử tích hợp trên toàn bộ các kịch bản chuẩn benchmark và nghiệm thu qua Oracle."""

from __future__ import annotations

import pytest

from path_planning.planner import plan_trajectory
from path_planning.scenario.preprocessing import prepare_scenario
from path_planning.scenario.presets import get_all_scenarios
from path_planning.types import CircleGeometry, PolygonCoords
from path_planning.validation.oracle import path_is_valid

_SCENARIO_BUILDERS = get_all_scenarios()


@pytest.mark.parametrize("scenario_name", list(_SCENARIO_BUILDERS.keys()))
def test_preset_scenario_produces_oracle_valid_trajectory(
    scenario_name: str,
) -> None:
    """Kiểm tra từng kịch bản benchmark chuẩn sinh ra đường bay hợp lệ 100% theo tiêu chuẩn Oracle.

    Args:
        scenario_name: Tên kịch bản benchmark đang kiểm thử.
    """
    # Arrange (Chuẩn bị kịch bản)
    builder = _SCENARIO_BUILDERS[scenario_name]
    scenario = builder()
    prep = prepare_scenario(scenario)

    # Act (Thực thi thuật toán tìm kiếm đường bay)
    result = plan_trajectory(prep, time_budget_s=25.0)

    # Assert (Kiểm chứng kết quả và nghiệm thu độc lập)
    assert result["is_success"] is True, (
        f"Kịch bản {scenario_name} không tìm thấy đường bay hợp lệ"
    )
    assert result["path"] is not None, f"Kịch bản {scenario_name} trả về path rỗng"

    # Trích xuất hình học vật cản để kiểm định độc lập
    circles: list[CircleGeometry] = [
        (c["center"], c["radius"])
        for c in scenario.get("obstacles", [])
        if c["type"] == "circle"
    ]
    polygons: list[PolygonCoords] = [
        p["polygon"]
        for p in scenario.get("obstacles", [])
        if p["type"] == "polygon"
    ]

    validation = path_is_valid(
        result["path"],
        circle_obstacles=circles,
        polygon_obstacles=polygons,
    )
    assert validation.is_ok is True, (
        f"Kịch bản {scenario_name} vi phạm tiêu chuẩn Oracle: {validation.detail}"
    )
