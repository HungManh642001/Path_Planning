"""Bộ sinh kịch bản ngẫu nhiên theo thuật toán (Procedural Generator).

Xây dựng kịch bản bay giả lập gồm đảo đa giác và chướng ngại vật tròn.
Đảm bảo tính tái lập qua seed và khoảng cách cách ly giữa các vật cản.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from shapely import Point as ShapelyPoint, Polygon as ShapelyPolygon

from path_planning import config
from path_planning.geometry import spatial
from path_planning.types import (
    CircleGeometry,
    MapBounds,
    Obstacle,
    Point,
    PolygonCoords,
    Scenario,
    ScenarioConfig,
    Topology,
)


_MAX_PLACEMENT_ATTEMPTS = 1000
"""Số lần thử đặt vị trí thất bại liên tiếp tối đa trước khi bộ sinh dừng lại."""


@dataclass(frozen=True)
class _StartGoalGeometry:
    """Hình học đoạn thẳng nối điểm xuất phát và đích.

    Attributes:
        mx: Tọa độ x trung điểm giữa start và goal.
        my: Tọa độ y trung điểm giữa start và goal.
        angle_start_goal: Góc phương vị từ start đến goal (rad).
        angle_perp: Góc vuông góc với phương vị start-goal (rad).
        dist_sg: Khoảng cách giữa start và goal (m).
    """

    mx: float
    my: float
    angle_start_goal: float
    angle_perp: float
    dist_sg: float


def _start_goal_geometry(start: Point, goal: Point) -> _StartGoalGeometry:
    """Tính trước các thông số hình học của đường nối start-goal."""
    angle_start_goal = math.atan2(goal[1] - start[1], goal[0] - start[0])
    return _StartGoalGeometry(
        mx=(start[0] + goal[0]) / 2,
        my=(start[1] + goal[1]) / 2,
        angle_start_goal=angle_start_goal,
        angle_perp=angle_start_goal + math.pi / 2,
        dist_sg=math.hypot(goal[0] - start[0], goal[1] - start[1]),
    )


def _sample_center(
    topology: Topology, map_bounds: MapBounds, geom: _StartGoalGeometry
) -> Point:
    """Lấy mẫu tọa độ tâm chướng ngại vật trong phạm vi 80% diện tích giữa bản đồ.

    Dùng chung cho cả 2 hàm sinh -- đều bố trí tâm theo cùng 20 dòng code và
    3 dạng topo giống nhau, tránh sự phân kỳ ngầm giữa chúng.

    Thứ tự rút ngẫu nhiên là một phần của cam kết: kịch bản được tái lập chính
    xác từ seed, do đó mỗi nhánh phải tiêu thụ đúng các giá trị ngẫu nhiên như cũ
    (hai giá trị đều, hai giá trị gauss, hoặc t kèm nhiễu).

    Args:
        topology: Chiến lược bố trí vật cản.
        map_bounds: Hình chữ nhật giới hạn bản đồ ``(width, height)``.
        geom: Thông số hình học tính trước của đoạn nối start-goal.

    Returns:
        Tọa độ tâm ứng viên nằm trong phạm vi 80% diện tích giữa bản đồ.

    Raises:
        ValueError: Nếu ``topology`` không thuộc 3 chiến lược đã định nghĩa.
    """
    width, height = map_bounds
    if topology == "random":
        center_x = random.uniform(width * 0.1, width * 0.9)
        center_y = random.uniform(height * 0.1, height * 0.9)
    elif topology == "center_cluster":
        # Phân phối Gauss quanh trung điểm nối giữa start và goal.
        center_x = random.gauss(geom.mx, geom.dist_sg / 3)
        center_y = random.gauss(geom.my, geom.dist_sg / 3)
    elif topology == "wall_block":
        # Dọc theo đường vuông góc với đoạn nối start-goal.
        t = random.uniform(-150000, 150000)
        noise = random.uniform(-geom.dist_sg / 3, geom.dist_sg / 3)
        center_x = (
            geom.mx
            + t * math.cos(geom.angle_perp)
            + noise * math.cos(geom.angle_start_goal)
        )
        center_y = (
            geom.my
            + t * math.sin(geom.angle_perp)
            + noise * math.sin(geom.angle_start_goal)
        )
    else:
        # Trước đây trường hợp này bị lọt khiến center_x không được gán: gây lỗi
        # NameError ở lượt đầu, hoặc tái dùng ngầm tâm của vòng lặp trước đó,
        # khiến mọi vật cản bị đặt chồng lên cùng một vị trí.
        raise ValueError(
            f"unknown topology {topology!r}; expected 'random', "
            "'center_cluster' or 'wall_block'"
        )
    return (
        max(width * 0.1, min(center_x, width * 0.9)),
        max(height * 0.1, min(center_y, height * 0.9)),
    )


def _clears_endpoints(shape: ShapelyPolygon, start: Point, goal: Point) -> bool:
    """Kiểm tra vật cản có cách ly an toàn khỏi điểm cất cánh và đích không.

    Vùng đệm trước đây dùng ``config.EPS`` (1e-6 m), khiến vật cản có thể chạm
    sát điểm xuất phát và làm chặng bay cất cánh hoặc tự dẫn bị chặn ngay từ đầu.

    Args:
        shape: Hình học vật cản ứng viên.
        start: Vị trí xuất phát.
        goal: Vị trí đích.

    Returns:
        ``True`` nếu cả hai điểm đầu cuối đều duy trì khoảng cách tối thiểu
        ``config.SPAWN_CLEARANCE_M``.
    """
    return (
        shape.distance(ShapelyPoint(start)) >= config.SPAWN_CLEARANCE_M
        and shape.distance(ShapelyPoint(goal)) >= config.SPAWN_CLEARANCE_M
    )


def generate_random_islands(
    num_islands: int,
    map_bounds: MapBounds,
    start: Point,
    goal: Point,
    *,
    topology: Topology = "random",
    seed: int | None = None,
) -> list[PolygonCoords]:
    """Sinh danh sách các đảo đa giác ngẫu nhiên không giao nhau.

    Args:
        num_islands: Số lượng đảo cần bố trí.
        map_bounds: Hình chữ nhật giới hạn bản đồ ``(width, height)``.
        start: Vị trí xuất phát, được giữ cách ly khỏi vật cản.
        goal: Vị trí đích, được giữ cách ly khỏi vật cản.
        topology: Chiến lược bố trí vật cản.
        seed: Seed ngẫu nhiên để tái lập kết quả.

    Returns:
        Danh sách các đảo đã bố trí, mỗi đảo là một vành hở gồm các đỉnh
        ``(x, y)``. Có thể ít hơn ``num_islands`` nếu bản đồ không đủ chỗ.
    """
    if seed is not None:
        random.seed(seed)

    islands: list[PolygonCoords] = []
    # Dạng ShapelyPolygon của các đảo để kiểm tra khoảng cách cách ly
    placed: list[ShapelyPolygon] = []
    attempts = 0
    geom = _start_goal_geometry(start, goal)

    while len(islands) < num_islands and attempts < _MAX_PLACEMENT_ATTEMPTS:
        center_x, center_y = _sample_center(topology, map_bounds, geom)
        size = random.uniform(config.ISLAND_SIZE_MIN, config.ISLAND_SIZE_MAX)
        num_vertices = random.randint(
            config.ISLAND_VERTICES_MIN, config.ISLAND_VERTICES_MAX
        )

        # Đa giác dạng sao bất quy tắc: bán kính mỗi đỉnh được làm nhiễu độc lập.
        island: PolygonCoords = []
        for i in range(num_vertices):
            angle = 2 * math.pi * i / num_vertices
            radius = size * random.uniform(0.6, 1.0)
            island.append(
                (
                    center_x + radius * math.cos(angle),
                    center_y + radius * math.sin(angle),
                )
            )

        island_polygon = ShapelyPolygon(island)

        # Các đảo không được chồng lấn nhau, tuân theo quy tắc tương tự bộ sinh
        # hình tròn. So sánh theo khoảng cách đa giác thực tế thay vì ước lượng
        # khoảng cách tâm thô, giúp xử lý chính xác các hình dạng bất quy tắc.
        valid = all(
            island_polygon.distance(p) >= config.ISLAND_MIN_SEPARATION_M for p in placed
        ) and _clears_endpoints(island_polygon, start, goal)

        if valid:
            islands.append(island)
            placed.append(island_polygon)
            attempts = 0
        else:
            attempts += 1

    return islands


def generate_dynamic_obstacles(
    num_sites: int,
    map_bounds: MapBounds,
    start: Point,
    goal: Point,
    *,
    topology: Topology = "random",
    seed: int | None = None,
) -> list[CircleGeometry]:
    """Sinh danh sách các chướng ngại vật hình tròn không giao nhau.

    Args:
        num_sites: Số lượng chướng ngại vật cần bố trí.
        map_bounds: Hình chữ nhật giới hạn bản đồ ``(width, height)``.
        start: Vị trí xuất phát, được giữ cách ly khỏi vật cản.
        goal: Vị trí đích, được giữ cách ly khỏi vật cản.
        topology: Chiến lược bố trí vật cản.
        seed: Seed ngẫu nhiên để tái lập kết quả.

    Returns:
        Danh sách các chướng ngại vật đã bố trí dạng ``(tâm, bán_kính)``. Có thể ít hơn
        ``num_sites`` nếu bản đồ quá chật không thể xếp đủ.
    """
    if seed is not None:
        random.seed(seed)

    dynamic_obstacles: list[CircleGeometry] = []
    attempts = 0
    geom = _start_goal_geometry(start, goal)

    while len(dynamic_obstacles) < num_sites and attempts < _MAX_PLACEMENT_ATTEMPTS:
        center = _sample_center(topology, map_bounds, geom)
        radius = random.uniform(config.OBSTACLE_RADIUS_MIN, config.OBSTACLE_RADIUS_MAX)

        # Khoảng cách cách ly được đo giữa BIÊN của hai hình tròn (r_i + r_j + gap),
        # thay vì dùng ngưỡng thô 2*max_radius: việc gán bán kính xấu nhất cho mọi cặp
        # từng khiến khoảng cách hiệu dụng lên tới 100.5 km trên bản đồ 500 km,
        # giới hạn tối đa chỉ ~13 hình tròn bất kể số lượng yêu cầu là bao nhiêu.
        valid = all(
            math.hypot(center[0] - other_center[0], center[1] - other_center[1])
            >= radius + other_radius + config.CIRCLE_MIN_SEPARATION_M
            for other_center, other_radius in dynamic_obstacles
        ) and _clears_endpoints(ShapelyPoint(center).buffer(radius), start, goal)

        if valid:
            dynamic_obstacles.append((center, radius))
            attempts = 0
        else:
            attempts += 1

    return dynamic_obstacles


def create_scenario(scenario_config: ScenarioConfig) -> Scenario:
    """Tạo kịch bản nhiệm vụ hoàn chỉnh gồm điểm đầu cuối và tập chướng ngại vật.

    Args:
        scenario_config: Cấu hình kịch bản. ``start`` và ``goal`` là bắt buộc;
            các tham số bộ sinh (``num_islands``, ``num_dynamic_obstacles``,
            ``topology``, ``seed``, ``map_bounds``, ``safezones``) có giá trị
            mặc định. ``goal_heading`` bằng ``None`` tương ứng với chế độ tiếp
            cận đích tự do (free-goal).

    Returns:
        Kịch bản đã được sinh.

    Raises:
        ValueError: Nếu thiếu ``start`` hoặc ``goal``. Cả hai đều bắt buộc
            vì các bộ lấy mẫu topo bố trí vật cản tương đối so với đoạn nối start-goal.
    """
    start = scenario_config.get("start")
    goal = scenario_config.get("goal")
    if start is None or goal is None:
        raise ValueError("scenario_config requires both 'start' and 'goal'")

    map_bounds = scenario_config.get(
        "map_bounds", (config.MAP_WIDTH, config.MAP_HEIGHT)
    )
    topology = scenario_config.get("topology", "random")
    seed = scenario_config.get("seed")

    islands = generate_random_islands(
        scenario_config.get("num_islands", 0),
        map_bounds,
        start,
        goal,
        topology=topology,
        seed=seed,
    )
    dynamic_obstacles = generate_dynamic_obstacles(
        scenario_config.get("num_dynamic_obstacles", 0),
        map_bounds,
        start,
        goal,
        topology=topology,
        seed=seed,
    )

    obstacles: list[Obstacle] = []
    for island in islands:
        obstacles.append({"type": "polygon", "polygon": island})
    for center, radius in dynamic_obstacles:
        obstacles.append({"type": "circle", "center": center, "radius": radius})

    return {
        "start": start,
        "start_heading": scenario_config.get("start_heading", 0),
        "goal": goal,
        # None => hướng tiếp cận đích tự do (bộ lập kế hoạch tự chọn).
        "goal_heading": scenario_config.get("goal_heading"),
        "map_bounds": map_bounds,
        # Vùng an toàn hoạt động tùy chọn: DANH SÁCH các đa giác (đỉnh (x, y)).
        # Khí tài bay phải nằm hoàn toàn trong hợp (union) của các vùng này.
        # None/rỗng => mặc định dùng hình chữ nhật config.MAP_WIDTH/HEIGHT.
        "safezones": scenario_config.get("safezones"),
        "islands": islands,
        "dynamic_obstacles": dynamic_obstacles,
        "obstacles": obstacles,
    }


# ============ PREDEFINED SCENARIOS ============


def generate_random_scenario(
    seed: int = 42, topology: Topology | None = None
) -> Scenario:
    """Sinh kịch bản ngẫu nhiên hoàn chỉnh với đảo và chướng ngại vật tròn.

    Args:
        seed: Seed ngẫu nhiên để tái lập kết quả.
        topology: Chiến lược topo vật cản tùy chọn ("random", "center_cluster",
            "wall_block"). Nếu là None, một topo sẽ được lấy mẫu ngẫu nhiên.

    Returns:
        Scenario: Dictionary chứa giới hạn bản đồ, điểm start/goal, danh sách
            đảo và chướng ngại vật tròn.
    """
    random.seed(seed)

    # Giới hạn bản đồ
    map_bounds = (config.MAP_WIDTH, config.MAP_HEIGHT)
    width, height = map_bounds

    # Tọa độ ngẫu nhiên của điểm xuất phát và đích trong giới hạn bản đồ
    while True:
        start = (
            random.uniform(width * 0.1, width * 0.9),
            random.uniform(height * 0.1, height * 0.9),
        )
        goal = (
            random.uniform(width * 0.1, width * 0.9),
            random.uniform(height * 0.1, height * 0.9),
        )
        if (
            spatial.distance(start, goal) > 400000
        ):  # Đảm bảo start và goal không quá gần nhau
            break

    heading_start_to_goal = spatial.angle_to_heading(start, goal)

    if topology is None:
        topologies: tuple[Topology, ...] = ("random", "center_cluster", "wall_block")
        selected_topology: Topology = random.choices(
            topologies, weights=[0.1, 0.45, 0.45]
        )[0]
    else:
        selected_topology = topology

    return create_scenario(
        {
            "map_bounds": map_bounds,
            "start": start,
            "start_heading": heading_start_to_goal
            + random.uniform(
                -math.pi / 2, math.pi / 2
            ),  # Thêm độ lệch ngẫu nhiên vào góc hướng xuất phát
            "goal": goal,
            "goal_heading": heading_start_to_goal
            + random.uniform(
                -math.pi / 2, math.pi / 2
            ),  # Thêm độ lệch ngẫu nhiên vào góc hướng tiếp cận đích
            "num_islands": random.randint(0, 20),
            "num_dynamic_obstacles": random.randint(0, 20),
            "topology": selected_topology,
            "seed": seed,
        }
    )
