"""Kiểm thử đơn vị cho các kịch bản chuẩn benchmark trong module scenario.presets."""

from __future__ import annotations

from path_planning.scenario.presets import get_all_scenarios


def test_get_all_scenarios_returns_valid_preset_builders() -> None:
    """Kiểm tra hàm get_all_scenarios trả về danh sách các builder kịch bản chuẩn."""
    # Arrange & Act
    scenario_builders = get_all_scenarios()

    # Assert
    assert len(scenario_builders) >= 16
    for name, builder in scenario_builders.items():
        assert callable(builder), f"{name} không phải là hàm callable"
        scen = builder()
        assert "start" in scen, f"{name} thiếu start"
        assert "goal" in scen, f"{name} thiếu goal"
        assert "start_heading" in scen, f"{name} thiếu start_heading"
        assert "map_bounds" in scen, f"{name} thiếu map_bounds"
        assert "obstacles" in scen, f"{name} thiếu obstacles"


def test_get_all_scenarios_start_and_goal_within_map_bounds() -> None:
    """Kiểm tra tọa độ start và goal của mọi kịch bản đều nằm trong giới hạn bản đồ."""
    # Arrange
    scenario_builders = get_all_scenarios()

    # Act & Assert
    for name, builder in scenario_builders.items():
        scen = builder()
        mw, mh = scen["map_bounds"]
        sx, sy = scen["start"]
        gx, gy = scen["goal"]

        assert 0 <= sx <= mw, f"{name} start x nằm ngoài map"
        assert 0 <= sy <= mh, f"{name} start y nằm ngoài map"
        assert 0 <= gx <= mw, f"{name} goal x nằm ngoài map"
        assert 0 <= gy <= mh, f"{name} goal y nằm ngoài map"
