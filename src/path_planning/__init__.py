"""Thư viện lập kế hoạch quỹ đạo Kinodynamic A* cho phương tiện bay không người lái.

Cung cấp thuật toán tìm kiếm đường bay tối ưu thỏa mãn bán kính quay vòng tối thiểu,
ràng buộc đoản trình và tiếp cận mục tiêu trong môi trường chướng ngại vật phức tạp.
"""

from __future__ import annotations

import sys

# Backwards compatibility alias for path_planning.core
import path_planning as _self
from path_planning.planner import KinodynamicAstar, plan_trajectory
from path_planning.scenario import preprocessing
from path_planning.scenario.generator import (
    create_scenario,
    generate_dynamic_obstacles,
    generate_random_islands,
    generate_random_scenario,
)
from path_planning.scenario.preprocessing import prepare_scenario
from path_planning.scenario.presets import get_all_scenarios
from path_planning.validation.oracle import path_is_valid


sys.modules["path_planning.core"] = _self

__all__ = [
    "KinodynamicAstar",
    "create_scenario",
    "generate_dynamic_obstacles",
    "generate_random_islands",
    "generate_random_scenario",
    "get_all_scenarios",
    "path_is_valid",
    "plan_trajectory",
    "prepare_scenario",
    "preprocessing",
]
