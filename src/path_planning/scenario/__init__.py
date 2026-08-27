"""Quản lý kịch bản, tiền xử lý và sinh chướng ngại vật ngẫu nhiên."""

from path_planning.scenario.generator import (
    create_scenario,
    generate_dynamic_obstacles,
    generate_random_islands,
    generate_random_scenario,
)
from path_planning.scenario.preprocessing import (
    compute_inflated_obstacles,
    inflate_obstacles,
    inflation_ring,
    prepare_scenario,
)
from path_planning.scenario.presets import get_all_scenarios


__all__ = [
    "compute_inflated_obstacles",
    "create_scenario",
    "generate_dynamic_obstacles",
    "generate_random_islands",
    "generate_random_scenario",
    "get_all_scenarios",
    "inflate_obstacles",
    "inflation_ring",
    "prepare_scenario",
]
