"""Kiểm thử đơn vị cho module ExecutionDriver và QAResult của QA Suite."""

from __future__ import annotations

from unittest.mock import patch

import pytest

from path_planning.scenario.presets import get_all_scenarios
from service.vtx_service.messages import (
    IDL_VERSION,
    PlanReply,
    PlanStatus,
    SearchStats as ProtoSearchStats,
    Waypoint as ProtoWaypoint,
)
from tools.qa_suite.core.runner import ExecutionDriver, ExecutionMode, QAResult


def test_local_execution_driver_solves_preset_scenario() -> None:
    """Kiểm thử ExecutionDriver chạy chế độ LOCAL giải thành công preset scenario_01_open_ocean."""
    scenario = get_all_scenarios()["scenario_01_open_ocean"]()
    driver = ExecutionDriver(mode=ExecutionMode.LOCAL)
    result = driver.run_scenario(scenario, name="scenario_01_open_ocean")

    assert isinstance(result, QAResult)
    assert result.scenario_name == "scenario_01_open_ocean"
    assert result.is_success is True
    assert result.status == "OK"
    assert len(result.waypoints) >= 2
    assert result.path_length_m > 0.0
    assert result.wall_time_s > 0.0
    assert result.applied_time_budget_s == 15.0
    assert result.iterations > 0
    assert result.oracle_verdict.is_ok is True
    assert result.error_detail is None
    assert result.raw_response is not None


def test_local_execution_driver_handles_blocked_start() -> None:
    """Kiểm thử ExecutionDriver chế độ LOCAL xử lý khi điểm xuất phát bị chặn."""
    scenario = get_all_scenarios()["scenario_01_open_ocean"]()
    # Đặt start nằm bên trong chướng ngại vật hoặc tạo chướng ngại vật bao phủ start
    scenario["obstacles"] = [
        {"type": "circle", "center": scenario["start"], "radius": 500.0}
    ]
    scenario["dynamic_obstacles"] = [(scenario["start"], 500.0)]
    driver = ExecutionDriver(mode=ExecutionMode.LOCAL)
    result = driver.run_scenario(scenario, name="scenario_blocked")

    assert isinstance(result, QAResult)
    assert result.is_success is False
    assert result.status != "OK"
    assert result.oracle_verdict.is_ok is False
    assert result.error_detail is not None


def test_nats_execution_driver_success_mocked() -> None:
    """Kiểm thử ExecutionDriver chế độ NATS khi nhận phản hồi PlanReply thành công."""
    scenario = get_all_scenarios()["scenario_01_open_ocean"]()
    driver = ExecutionDriver(mode=ExecutionMode.NATS)

    # Waypoints giả định từ NATS service (bearing deg)
    # Start: (100.0, 100.0), heading 0 deg bearing (North) -> math rad pi/2
    # Mid: (100.0, 500.0), heading 0 deg
    # Goal: (100.0, 1000.0), heading 0 deg
    fake_waypoints = (
        ProtoWaypoint(position=(100.0, 100.0), heading_deg=0.0),
        ProtoWaypoint(position=(100.0, 500.0), heading_deg=0.0),
        ProtoWaypoint(position=(100.0, 1000.0), heading_deg=0.0),
    )
    fake_reply = PlanReply(
        request_id=b"\x01" * 16,
        idl_version=IDL_VERSION,
        status=PlanStatus.OK,
        detail="",
        waypoints=fake_waypoints,
        path_length_m=900.0,
        plan_wall_time_s=0.123,
        applied_time_budget_s=15.0,
        stats=ProtoSearchStats(
            iterations=42,
            open_set_size=5,
            is_search_failed=False,
            is_budget_bound=False,
        ),
        planner_version="vtx-1.0.0",
        config_hash="abc1234",
    )

    with patch(
        "tools.qa_suite.core.runner.NatsClient.request_plan_sync",
        return_value=fake_reply,
    ) as mock_request:
        result = driver.run_scenario(
            scenario, name="scenario_01_open_ocean", time_budget_s=10.0
        )
        assert mock_request.called
        assert isinstance(result, QAResult)
        assert result.is_success is True
        assert result.status == "OK"
        assert len(result.waypoints) == 3
        # Kiểm tra chuyển đổi góc từ bearing deg sang math rad
        # Bearing 0 deg (North) -> Math rad pi/2
        assert pytest.approx(result.waypoints[0][1], 1e-4) == 1.5707963
        assert result.path_length_m == 900.0
        assert result.wall_time_s == 0.123
        assert result.applied_time_budget_s == 15.0
        assert result.iterations == 42
        assert result.raw_response == fake_reply


def test_nats_execution_driver_timeout_mocked() -> None:
    """Kiểm thử ExecutionDriver chế độ NATS khi request bị timeout."""
    scenario = get_all_scenarios()["scenario_01_open_ocean"]()
    driver = ExecutionDriver(mode=ExecutionMode.NATS)

    with patch(
        "tools.qa_suite.core.runner.NatsClient.request_plan_sync",
        side_effect=TimeoutError("Request timed out"),
    ):
        result = driver.run_scenario(scenario, name="scenario_timeout")
        assert isinstance(result, QAResult)
        assert result.is_success is False
        assert result.status == "TIMEOUT"
        assert result.error_detail is not None
        assert "timed out" in result.error_detail.lower()
        assert result.oracle_verdict.is_ok is False


def test_nats_execution_driver_no_path_mocked() -> None:
    """Kiểm thử ExecutionDriver chế độ NATS khi service trả về NO_PATH."""
    scenario = get_all_scenarios()["scenario_01_open_ocean"]()
    driver = ExecutionDriver(mode=ExecutionMode.NATS)

    fake_reply = PlanReply(
        request_id=b"\x01" * 16,
        idl_version=IDL_VERSION,
        status=PlanStatus.NO_PATH,
        detail="no path found",
        waypoints=(),
        path_length_m=0.0,
        plan_wall_time_s=0.05,
        applied_time_budget_s=15.0,
        stats=ProtoSearchStats(
            iterations=100,
            open_set_size=0,
            is_search_failed=True,
            is_budget_bound=False,
        ),
        planner_version="vtx-1.0.0",
        config_hash="abc1234",
    )

    with patch(
        "tools.qa_suite.core.runner.NatsClient.request_plan_sync",
        return_value=fake_reply,
    ):
        result = driver.run_scenario(scenario, name="scenario_no_path")
        assert isinstance(result, QAResult)
        assert result.is_success is False
        assert result.status == "NO_PATH"
        assert result.error_detail == "no path found"
        assert result.oracle_verdict.is_ok is False


def test_execution_driver_invalid_mode() -> None:
    """Kiểm thử ExecutionDriver báo lỗi khi mode không hợp lệ."""
    driver = ExecutionDriver(mode="INVALID_MODE")  # type: ignore[arg-type]
    scenario = get_all_scenarios()["scenario_01_open_ocean"]()
    with pytest.raises(ValueError, match="Unknown execution mode"):
        driver.run_scenario(scenario)


def test_execution_driver_passes_custom_vehicle_limits() -> None:
    """Kiểm thử ExecutionDriver truyền đúng custom safe_margin và limits vào planner."""
    scenario = get_all_scenarios()["scenario_01_open_ocean"]()
    driver = ExecutionDriver(mode=ExecutionMode.LOCAL)
    custom_safe_margin = 1234.0
    custom_turn_radius = 2000.0
    result = driver.run_scenario(
        scenario,
        name="custom_limits_test",
        safe_margin=custom_safe_margin,
        turn_radius=custom_turn_radius,
    )
    assert isinstance(result, QAResult)
    assert result.is_success is True
    assert result.oracle_verdict.is_ok is True
