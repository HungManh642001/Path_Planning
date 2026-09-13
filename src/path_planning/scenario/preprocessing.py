"""Tiền xử lý kịch bản nhiệm vụ và tính toán biên an toàn.

Giãn nở chướng ngại vật theo khoảng cách an toàn (safe margin) và xác định
các trạng thái xuất phát/kết thúc (start/goal state) cho thuật toán A*.
Đơn vị tính: khoảng cách bằng mét (m), góc bằng radian (rad).
"""

from __future__ import annotations

import math

from path_planning import config
from path_planning.geometry import spatial
from path_planning.types import (
    CircleGeometry,
    GoalState,
    InflatedObstacleSets,
    Obstacle,
    Point,
    PolygonCoords,
    PreprocessedScenario,
    Scenario,
    StartState,
)


def inflation_ring(*, safe_margin: float = config.SAFE_MARGIN) -> float:
    """Trả về khoảng cách đệm giãn nở chướng ngại vật dùng để hiển thị.

    Chỉ có DUY NHẤT một vành đệm, ``safe_margin`` -- chính là khoảng cách mà
    :func:`inflate_obstacles` áp dụng. Hàm này trước đây trả về MỘT CẶP vì
    vành thứ hai cộng thêm số hạng quay ``R*(1/cos(alpha_max/2)-1)`` để dự trữ
    độ phồng góc xấu nhất; số hạng này đã bị loại bỏ (thuật toán tìm kiếm giờ
    đây kiểm tra chính xác từng cung lượn), nên hai giá trị giống hệt nhau và
    bên gọi duy nhất cũng đã bỏ qua giá trị thứ hai.

    Args:
        safe_margin: Khoảng cách an toàn tối thiểu của người vận hành (m).

    Returns:
        Khoảng cách giãn nở biên tính bằng mét.
    """
    return safe_margin


def inflate_obstacles(
    obstacles: list[Obstacle], *, safe_margin: float = config.SAFE_MARGIN
) -> list[Obstacle]:
    """Giãn nở biên chướng ngại vật ra ngoài một khoảng an toàn safe_margin.

    Các chướng ngại vật được giữ độc lập -- không tính bao lồi sớm, giúp duy trì
    hành lang bay thông suốt giữa hai vật cản.

    Args:
        obstacles: Danh sách bản ghi vật cản thô ban đầu.
        safe_margin: Khoảng cách an toàn tối thiểu của người vận hành (m).

    Returns:
        Danh sách bản ghi vật cản mới với biên đã giãn nở ra ngoài;
        dữ liệu đầu vào không bị thay đổi.
    """
    # Chỉ dùng SAFE_MARGIN. Số hạng góc lượn cũ `R*(1/cos(alpha_max/2)-1)` dự phòng
    # độ phồng xấu nhất của cung fillet lấn vào góc cua; số hạng này đã bị loại bỏ
    # vì thuật toán tìm kiếm hiện kiểm tra độ phồng đó CHÍNH XÁC theo từng góc cua
    # với góc bẻ lái thực tế (`_is_corner_arc_clear`). Nếu tính theo alpha_max và
    # áp dụng cho mọi chướng ngại vật, số hạng này sẽ bít kín 49% hành lang giữa
    # các cặp vật cản (đo trên 1536 cặp) - một đường bay thẳng phải chịu cùng
    # khoản đệm 3.3 km như một góc rẽ 90 độ. Quá trình giãn nở giờ đây thuần túy
    # là khoảng cách an toàn tối thiểu của người vận hành.
    inflated: list[Obstacle] = []
    for obstacle in obstacles:
        if obstacle["type"] == "circle":
            circle = obstacle.copy()
            circle["radius"] = obstacle["radius"] + safe_margin
            inflated.append(circle)
        else:
            polygon = obstacle.copy()
            polygon["polygon"] = spatial.inflate_polygon(
                obstacle["polygon"], safe_margin
            )
            inflated.append(polygon)
    return inflated


def calculate_start_state(
    origin: Point,
    init_heading: float,
    *,
    l0: float = config.L0,
    turn_radius: float = config.R,
    alpha_max_rad: float = config.ALPHA_MAX_RAD,
) -> StartState:
    """Tính toán waypoint đầu tiên W_1 và hướng bay sau khi cất cánh.

    Từ ràng buộc động học ``d_1 = l_1 + R * tan(alpha_1 / 2)`` với điều kiện
    ``l_1 >= L_0``. Điểm ``W_1`` được đặt ở khoảng cách ``d_1`` dọc theo hướng
    ``init_heading``.

    Args:
        origin: Điểm cất cánh ``O``.
        init_heading: Hướng bay ban đầu (rad).
        l0: Chiều dài đoạn bay thẳng tối thiểu để ổn định bay bằng (m).
        turn_radius: Bán kính quay vòng của khí tài (m).
        alpha_max_rad: Góc bẻ lái tối đa cho phép (rad).

    Returns:
        Trạng thái xuất phát: waypoint, hướng bay, chiều dài đoạn thẳng ``l_1`` và
        khoảng cách ``d_1`` tính từ điểm xuất phát O.
    """
    straight_length = l0
    # Bảo thủ: dự trữ chiều dài tiếp tuyến cho trường hợp góc rẽ đầu tiên xấu nhất
    # alpha_1 = alpha_max, do đó d_1 = L0 + R*tan(alpha_max/2) và l_1 = L0
    # chính xác (thỏa mãn l_1 >= L0).
    distance_from_origin = straight_length + turn_radius * math.tan(alpha_max_rad / 2)
    return {
        "waypoint": (
            origin[0] + distance_from_origin * math.cos(init_heading),
            origin[1] + distance_from_origin * math.sin(init_heading),
        ),
        "heading": init_heading,
        "straight_length": straight_length,
        "distance_from_origin": distance_from_origin,
    }


def calculate_end_state(
    target: Point,
    target_heading: float,
    *,
    dss: float = config.DSS,
    turn_radius: float = config.R,
    alpha_max_rad: float = config.ALPHA_MAX_RAD,
) -> GoalState:
    """Tính toán waypoint cuối cùng W_{n-1} trước khi tiếp cận mục tiêu.

    Từ ràng buộc động học ``d_n = l_n + d_ss + R * tan(alpha_{n-1} / 2)`` với
    ``l_n = 0``, suy ra ``d_n = d_ss + R * tan(alpha_{n-1} / 2)``.

    Args:
        target: Vị trí mục tiêu ``T``.
        target_heading: Hướng tiếp cận mục tiêu yêu cầu (rad).
        dss: Chiều dài đoạn thẳng tự dẫn để đầu dò camera khóa mục tiêu (m).
        turn_radius: Bán kính quay vòng của khí tài (m).
        alpha_max_rad: Góc bẻ lái tối đa cho phép (rad).

    Returns:
        Trạng thái đích: waypoint, hướng bay, khoảng cách tiếp cận và khoảng
        cách lùi từ mục tiêu.
    """
    distance_to_target = dss + turn_radius * math.tan(alpha_max_rad / 2)
    # Đi lùi từ mục tiêu dọc theo hướng tiếp cận.
    return {
        "waypoint": (
            target[0] - distance_to_target * math.cos(target_heading),
            target[1] - distance_to_target * math.sin(target_heading),
        ),
        "heading": target_heading,
        "engagement_distance": dss,
        "distance_to_target": distance_to_target,
    }


def compute_inflated_obstacles(
    obstacles: list[Obstacle], *, safe_margin: float = config.SAFE_MARGIN
) -> InflatedObstacleSets:
    """Giãn nở tất cả chướng ngại vật và phân loại theo kiểu tròn/đa giác.

    Args:
        obstacles: Danh sách bản ghi vật cản thô ban đầu.
        safe_margin: Khoảng cách an toàn tối thiểu của người vận hành (m).

    Returns:
        Danh sách chướng ngại vật đã giãn nở kèm danh sách tách riêng cho
        hình tròn và đa giác.
    """
    inflated = inflate_obstacles(obstacles, safe_margin=safe_margin)
    circle_obstacles: list[CircleGeometry] = []
    polygon_obstacles: list[PolygonCoords] = []
    for obstacle in inflated:
        if obstacle["type"] == "circle":
            circle_obstacles.append((obstacle["center"], obstacle["radius"]))
        else:
            polygon_obstacles.append(obstacle["polygon"])
    return {
        "inflated_obstacles": inflated,
        "circle_obstacles": circle_obstacles,
        "polygon_obstacles": polygon_obstacles,
    }


def prepare_scenario(
    scenario: Scenario,
    *,
    l0: float = config.L0,
    dss: float = config.DSS,
    turn_radius: float = config.R,
    alpha_max_rad: float = config.ALPHA_MAX_RAD,
    safe_margin: float = config.SAFE_MARGIN,
) -> PreprocessedScenario:
    """Xử lý thô kịch bản trước khi đưa vào thuật toán tìm kiếm đường đi.

    Tính toán trước trạng thái xuất phát ``W_1``, trạng thái đích ``W_{n-1}``,
    giãn nở chướng ngại vật theo khoảng an toàn, và tách riêng hình học tròn/đa giác
    để tối ưu hóa hiệu năng kiểm tra va chạm.

    Args:
        scenario: Dictionary cấu hình kịch bản đầu vào.
        l0: Chiều dài đoạn bay thẳng cất cánh tối thiểu (m).
        dss: Chiều dài đoạn thẳng tự dẫn khóa mục tiêu (m).
        turn_radius: Bán kính quay vòng tối thiểu của khí tài (m).
        alpha_max_rad: Góc bẻ lái tối đa cho phép (rad).
        safe_margin: Khoảng cách an toàn tối thiểu của người vận hành (m).

    Returns:
        Dictionary cấu hình kịch bản đã qua xử lý sẵn sàng cho thuật toán tìm kiếm.
    """
    start_state = calculate_start_state(
        scenario["start"],
        scenario["start_heading"],
        l0=l0,
        turn_radius=turn_radius,
        alpha_max_rad=alpha_max_rad,
    )
    goal_heading = scenario.get("goal_heading")
    goal_state: GoalState | None = None
    if goal_heading is None:
        # Hướng tiếp cận mục tiêu tự do: không có hướng goal_heading cố định để
        # tính lùi W_{n-1}, nên thuật toán nhắm thẳng tới T và cạnh tìm kiếm cuối cùng
        # trở thành đoạn bay thẳng của đầu dò (>= DSS, theo bất kỳ hướng nào).
        # heading=None đánh dấu cờ chế độ tiếp cận tự do cho bộ lập kế hoạch.
        goal_state = {
            "waypoint": scenario["goal"],
            "heading": None,
            "engagement_distance": dss,
            "distance_to_target": dss,
        }
    else:
        goal_state = calculate_end_state(
            scenario["goal"],
            goal_heading,
            dss=dss,
            turn_radius=turn_radius,
            alpha_max_rad=alpha_max_rad,
        )

    inflated = compute_inflated_obstacles(
        scenario["obstacles"], safe_margin=safe_margin
    )

    # Tập chướng ngại vật thô (chưa giãn nở), chuyển tiếp cho các thành phần
    # cần đo đạc hoặc vẽ vật cản thực tế. Chúng KHÔNG CÒN là mốc tham chiếu
    # khoảng cách an toàn của cung lượn: cả đoạn thẳng và cung lượn đều kiểm tra
    # an toàn với tập vật cản đã giãn nở (thô + SAFE_MARGIN) vì phép giãn nở
    # không còn chứa số hạng góc lượn.
    raw_circles: list[CircleGeometry] = [
        (o["center"], o["radius"])
        for o in scenario["obstacles"]
        if o["type"] == "circle"
    ]
    raw_polygons: list[PolygonCoords] = [
        o["polygon"] for o in scenario["obstacles"] if o["type"] == "polygon"
    ]

    return {
        "start_state": start_state,
        "goal_state": goal_state,
        "start_pos": scenario["start"],
        "goal_pos": scenario["goal"],
        "start_heading": scenario["start_heading"],
        "goal_heading": goal_heading,
        "turn_radius": turn_radius,
        "alpha_max_rad": alpha_max_rad,
        "safe_margin": safe_margin,
        "obstacles": inflated["inflated_obstacles"],
        "circle_obstacles": inflated["circle_obstacles"],
        "polygon_obstacles": inflated["polygon_obstacles"],
        "raw_circle_obstacles": raw_circles,
        "raw_polygon_obstacles": raw_polygons,
        "islands": scenario.get("islands", []),
        "dynamic_obstacles": scenario.get("dynamic_obstacles", []),
        # Vùng hoạt động / giới hạn riêng của kịch bản. `safezones` là danh sách
        # đa giác tùy chọn (khí tài bay phải nằm hoàn toàn trong hợp của chúng);
        # `map_bounds` là hình chữ nhật (chiều rộng, chiều cao). Cả hai được
        # chuyển tiếp để bộ lập kế hoạch ràng buộc tìm kiếm thay vì dùng
        # config.MAP_WIDTH/HEIGHT toàn cục.
        "safezones": scenario.get("safezones"),
        "map_bounds": scenario.get("map_bounds"),
    }


# Bí danh tương thích ngược
preprocess_scenario = prepare_scenario
