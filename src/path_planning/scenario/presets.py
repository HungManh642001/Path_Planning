"""Tập hợp 16 kịch bản nhiệm vụ chuẩn (benchmark presets)."""

from __future__ import annotations

import math
from collections.abc import Callable

from path_planning.scenario.generator import create_scenario
from path_planning.types import Obstacle, PolygonCoords, Scenario


def scenario1_open_ocean() -> Scenario:
    """Kịch bản 1: Biển mở - không có chướng ngại vật.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (2000, 2000),
            "start_heading": math.pi / 4,  # 45 độ
            "goal": (450000, 450000),
            "goal_heading": math.pi / 4,
            "num_islands": 0,
            "num_dynamic_obstacles": 0,
            "seed": 42,
        }
    )


def scenario2_single_obstacle() -> Scenario:
    """Kịch bản 2: Một vật cản lớn duy nhất trên đường bay.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (2000, 2000),
            "start_heading": math.pi / 4,
            "goal": (450000, 450000),
            "goal_heading": math.pi / 4,
            "num_islands": 1,
            "num_dynamic_obstacles": 1,
            "seed": 42,
        }
    )


def scenario3_narrow_gap() -> Scenario:
    """Kịch bản 3: Hai vật cản nằm rất gần nhau (khe hẹp).

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    scenario = create_scenario(
        {
            "start": (2000, 2000),
            "start_heading": math.pi / 4,
            "goal": (450000, 450000),
            "goal_heading": math.pi / 4,
            "num_islands": 0,
            "num_dynamic_obstacles": 0,
            "seed": 99,
        }
    )

    # Thêm thủ công 2 đảo gần nhau. Tọa độ được viết dưới dạng số thực float vì
    # PolygonCoords là list[tuple[float, float]] (list mang tính invariant);
    # các giá trị tọa độ giữ nguyên không đổi.
    island1: PolygonCoords = [
        (22000.0, 20000.0),
        (24000.0, 20000.0),
        (24000.0, 22000.0),
        (22000.0, 22000.0),
    ]
    island2: PolygonCoords = [
        (26000.0, 20000.0),
        (28000.0, 20000.0),
        (28000.0, 22000.0),
        (26000.0, 22000.0),
    ]

    hand_placed: list[Obstacle] = [
        {"type": "polygon", "polygon": island1},
        {"type": "polygon", "polygon": island2},
    ]
    scenario["islands"] = [island1, island2]
    scenario["obstacles"] = hand_placed

    return scenario


def scenario4_complex_maze() -> Scenario:
    """Kịch bản 4: Mê cung phức tạp với nhiều vật cản.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (1000, 1000),
            "start_heading": 0,
            "goal": (480000, 480000),
            "goal_heading": 0,
            "num_islands": 12,  # Giảm từ 20 để tăng khả năng thông qua đường bay
            "num_dynamic_obstacles": 6,  # Giảm từ 10
            "seed": 12345,
        }
    )


# ============ KỊCH BẢN DỄ (Ít chướng ngại vật, đường bay đơn giản) ============


def scenario5_sparse_islands() -> Scenario:
    """Kịch bản 5: Dễ - Các đảo thưa thớt, vùng nước thoáng rộng rãi.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (5000, 5000),
            "start_heading": math.pi / 4,
            "goal": (450000, 450000),
            "goal_heading": math.pi / 4,
            "num_islands": 3,
            "num_dynamic_obstacles": 1,
            "seed": 111,
        }
    )


def scenario6_coastal_path() -> Scenario:
    """Kịch bản 6: Dễ - Chướng ngại vật động ven bờ nhẹ, hành lang mở.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (10000, 10000),
            "start_heading": 0,
            "goal": (480000, 480000),
            "goal_heading": 0,
            "num_islands": 2,
            "num_dynamic_obstacles": 2,
            "seed": 222,
        }
    )


def scenario7_diagonal_crossing() -> Scenario:
    """Kịch bản 7: Dễ - Ít vật cản, bay chéo bản đồ.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (20000, 20000),
            "start_heading": math.pi / 4,
            "goal": (470000, 470000),
            "goal_heading": math.pi / 4,
            "num_islands": 4,
            "num_dynamic_obstacles": 0,
            "seed": 333,
        }
    )


def scenario8_open_with_dynamic_obstacles() -> Scenario:
    """Kịch bản 8: Dễ - Địa hình mở với các chướng ngại vật động rải rác.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (10000, 250000),
            "start_heading": 0,
            "goal": (480000, 250000),
            "goal_heading": 0,
            "num_islands": 1,
            "num_dynamic_obstacles": 3,
            "seed": 444,
        }
    )


# ============ KỊCH BẢN TRUNG BÌNH (Độ phức tạp vừa phải) ============


def scenario9_island_archipelago() -> Scenario:
    """Kịch bản 9: Trung bình - Quần đảo với nhiều đảo nhỏ.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (5000, 250000),
            "start_heading": 0,
            "goal": (490000, 250000),
            "goal_heading": 0,
            "num_islands": 8,
            "num_dynamic_obstacles": 2,
            "seed": 555,
        }
    )


def scenario10_dense_dynamic_obstacles() -> Scenario:
    """Kịch bản 10: Trung bình - Mật độ chướng ngại vật động dày kèm một số đảo.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (50000, 50000),
            "start_heading": math.pi / 4,
            "goal": (450000, 450000),
            "goal_heading": math.pi / 4,
            "num_islands": 3,
            "num_dynamic_obstacles": 8,
            "seed": 666,
        }
    )


def scenario11_serpentine_route() -> Scenario:
    """Kịch bản 11: Trung bình - Đường bay zíc zắc uốn lượn qua bãi vật cản.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (50000, 100000),
            "start_heading": 0,
            "goal": (450000, 400000),
            "goal_heading": 0,
            "num_islands": 7,
            "num_dynamic_obstacles": 4,
            "seed": 777,
        }
    )


def scenario12_perimeter_dynamic_obstacles() -> Scenario:
    """Kịch bản 12: Trung bình - Đích được bảo vệ bởi vành đai chướng ngại vật động.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (10000, 250000),
            "start_heading": 0,
            "goal": (480000, 250000),
            "goal_heading": 0,
            "num_islands": 6,
            "num_dynamic_obstacles": 5,
            "seed": 888,
        }
    )


# ============ KỊCH BẢN KHÓ (Độ phức tạp cao, nhiều chướng ngại vật) ============


def scenario13_dense_island_field() -> Scenario:
    """Kịch bản 13: Khó - Mật độ đảo rất dày đặc.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (25000, 25000),
            "start_heading": math.pi / 3,
            "goal": (475000, 475000),
            "goal_heading": math.pi / 3,
            "num_islands": 18,
            "num_dynamic_obstacles": 3,
            "seed": 999,
        }
    )


def scenario14_combined_obstacles() -> Scenario:
    """Kịch bản 14: Khó - Kết hợp nhiều loại chướng ngại vật.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (30000, 30000),
            "start_heading": 0,
            "goal": (470000, 470000),
            "goal_heading": 0,
            "num_islands": 12,
            "num_dynamic_obstacles": 10,
            "seed": 1111,
        }
    )


def scenario15_narrow_channel() -> Scenario:
    """Kịch bản 15: Khó - Eo biển hẹp giữa các đảo.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (50000, 250000),
            "start_heading": 0,
            "goal": (450000, 250000),
            "goal_heading": 0,
            "num_islands": 15,
            "num_dynamic_obstacles": 4,
            "seed": 2222,
        }
    )


def scenario16_extreme_complexity() -> Scenario:
    """Kịch bản 16: Rất khó - Kiểm thử độ phức tạp cực hạn.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (10000, 10000),
            "start_heading": math.pi / 6,
            "goal": (490000, 490000),
            "goal_heading": math.pi / 6,
            "num_islands": 20,
            "num_dynamic_obstacles": 12,
            "seed": 3333,
        }
    )


def scenario17_reversed_approach_open() -> Scenario:
    """Kịch bản 17: Đầu dò phải tiếp cận bằng cách bay NGƯỢC LẠI dọc theo chặng bay đến.

    ``goal_heading`` lệch 180 độ so với phương vị start->goal, do đó không có đoạn
    bay thẳng nào vào đích có thể rẽ vào đó chỉ bằng một góc rẽ
    (yêu cầu góc rẽ > ALPHA_MAX): pha cuối thực sự là một thao tác quay đầu.
    Mọi kịch bản mẫu khác ở đây đều tiếp cận trong phạm vi 45 độ so với phương vị
    bay đến, khiến toàn bộ chế độ này chưa được đo đạc -- và phát bắn
    analytic goal shot tồn tại chính là vì trường hợp này.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (50000, 250000),
            "start_heading": 0,
            "goal": (430000, 250000),
            "goal_heading": math.pi,
            "num_islands": 0,
            "num_dynamic_obstacles": 0,
            "seed": 4242,
        }
    )


def scenario18_reversed_approach_cluttered() -> Scenario:
    """Kịch bản 18: Thao tác quay đầu tương tự, có chướng ngại vật bao quanh bên trong.

    Returns:
        Scenario: Kịch bản nhiệm vụ đã cấu hình.
    """
    return create_scenario(
        {
            "start": (50000, 100000),
            "start_heading": math.pi / 4,
            "goal": (400000, 400000),
            "goal_heading": -3 * math.pi / 4,
            "num_islands": 8,
            "num_dynamic_obstacles": 5,
            "seed": 4343,
        }
    )


def get_all_scenarios() -> dict[str, Callable[[], Scenario]]:
    """Trả về toàn bộ 18 kịch bản định sẵn được phân loại theo độ khó.

    Returns:
        dict[str, Callable]: Ánh xạ từ tên kịch bản tới hàm khởi tạo tương ứng.
    """
    return {
        # Các kịch bản gốc
        "scenario_01_open_ocean": scenario1_open_ocean,
        "scenario_02_single_obstacle": scenario2_single_obstacle,
        "scenario_03_narrow_gap": scenario3_narrow_gap,
        "scenario_04_complex_maze": scenario4_complex_maze,
        # Kịch bản dễ
        "scenario_05_sparse_islands": scenario5_sparse_islands,
        "scenario_06_coastal_path": scenario6_coastal_path,
        "scenario_07_diagonal_crossing": scenario7_diagonal_crossing,
        "scenario_08_open_with_dynamic_obstacles": (
            scenario8_open_with_dynamic_obstacles
        ),
        # Kịch bản trung bình
        "scenario_09_island_archipelago": scenario9_island_archipelago,
        "scenario_10_dense_dynamic_obstacles": (scenario10_dense_dynamic_obstacles),
        "scenario_11_serpentine_route": scenario11_serpentine_route,
        "scenario_12_perimeter_dynamic_obstacles": (
            scenario12_perimeter_dynamic_obstacles
        ),
        # Kịch bản khó
        "scenario_13_dense_island_field": scenario13_dense_island_field,
        "scenario_14_combined_obstacles": scenario14_combined_obstacles,
        "scenario_15_narrow_channel": scenario15_narrow_channel,
        "scenario_16_extreme_complexity": scenario16_extreme_complexity,
        # Hướng tiếp cận đảo ngược: goal_heading quay ngược lại đường bay đến,
        # do đó chặng cuối cần 2 góc rẽ. Không có kịch bản nào ở trên bao phủ
        # trường hợp này.
        "scenario_17_reversed_approach_open": scenario17_reversed_approach_open,
        "scenario_18_reversed_approach_cluttered": (
            scenario18_reversed_approach_cluttered
        ),
    }
