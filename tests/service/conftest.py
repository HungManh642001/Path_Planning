"""Fixtures dùng chung cho tầng kiểm thử dịch vụ Service (DDS / Gateway)."""

from __future__ import annotations

import json
from typing import Any

import pytest


@pytest.fixture
def sample_service_request_dict() -> dict[str, Any]:
    """Cung cấp payload yêu cầu lập kế hoạch chuẩn gửi tới dịch vụ.

    Returns:
        Dictionary chứa cấu hình kịch bản và tham số request.
    """
    return {
        "request_id": "req-test-001",
        "scenario": {
            "start": [50000.0, 50000.0],
            "start_heading": 0.7853981633974483,
            "goal": [450000.0, 450000.0],
            "goal_heading": 0.7853981633974483,
            "num_islands": 2,
            "num_dynamic_obstacles": 2,
            "map_bounds": [500000.0, 500000.0],
            "safezones": None,
            "topology": "random",
            "seed": 42,
        },
        "time_budget_s": 15.0,
    }


@pytest.fixture
def sample_service_request_json(sample_service_request_dict: dict[str, Any]) -> str:
    """Cung cấp chuỗi JSON được mã hóa từ payload yêu cầu dịch vụ.

    Args:
        sample_service_request_dict: Dictionary dữ liệu yêu cầu.

    Returns:
        Chuỗi JSON hoàn chỉnh.
    """
    return json.dumps(sample_service_request_dict)
