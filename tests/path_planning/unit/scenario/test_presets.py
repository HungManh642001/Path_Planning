"""Kiểm thử đơn vị cho 16 kịch bản chuẩn benchmark trong module scenario.presets."""

from __future__ import annotations

from path_planning.scenario.presets import get_all_scenarios


def test_get_all_scenarios_returns_sixteen_valid_presets() -> None:
    """Kiểm tra hàm get_all_scenarios trả về đúng 16 kịch bản chuẩn benchmark."""
    # Arrange & Act
    scenarios = get_all_scenarios()

    # Assert
    assert len(scenarios) == 16
    for i, scen in enumerate(scenarios):
        assert "start" in scen, f"Scenario {i} thiếu start"
        assert "goal" in scen, f"Scenario {i} thiếu goal"
        assert "start_heading" in scen, f"Scenario {i} thiếu start_heading"
        assert "map_bounds" in scen, f"Scenario {i} thiếu map_bounds"
        assert "obstacles" in scen, f"Scenario {i} thiếu obstacles"


def test_get_all_scenarios_start_and_goal_within_map_bounds() -> None:
    """Kiểm tra tọa độ start và goal của mọi kịch bản đều nằm trong giới hạn bản đồ."""
    # Arrange
    scenarios = get_all_scenarios()

    # Act & Assert
    for i, scen in enumerate(scenarios):
        mw, mh = scen["map_bounds"]
        sx, sy = scen["start"]
        gx, gy = scen["goal"]

        assert 0 <= sx <= mw, f"Scenario {i} start x nằm ngoài map"
        assert 0 <= sy <= mh, f"Scenario {i} start y nằm ngoài map"
        assert 0 <= gx <= mw, f"Scenario {i} goal x nằm ngoài map"
        assert 0 <= gy <= mh, f"Scenario {i} goal y nằm ngoài map"
