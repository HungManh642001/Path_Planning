"""Bộ kiểm định tính hợp lệ và an toàn của đường bay (Path Validation Oracle).

Đường bay là danh sách các bộ ``(waypoint, heading)`` với ``waypoint = (x, y)``.
Các hàm kiểm định trong module này hoàn toàn độc lập với thuật toán tìm kiếm A*,
nhằm đảm bảo tính khách quan khi nghiệm thu đường bay.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from shapely.geometry import LineString, Polygon

from path_planning import config


if TYPE_CHECKING:
    from collections.abc import Sequence

    from path_planning.types import (
        CircleGeometry,
        PlannerState,
        Point,
        PolygonCoords,
    )


@dataclass(frozen=True)
class ValidationResult:
    """Kết quả kiểm định một tiêu chí hợp lệ của đường bay.

    Attributes:
        is_ok: True nếu thỏa mãn tiêu chí kiểm định, False nếu vi phạm.
        detail: 'ok' khi thành công, hoặc mô tả chi tiết vị trí/nguyên nhân vi phạm.
    """

    is_ok: bool
    detail: str

    def __bool__(self) -> bool:
        """Trả về True nếu kiểm tra hợp lệ thành công."""
        return self.is_ok

    @classmethod
    def ok(cls) -> ValidationResult:
        """Trả về kết quả kiểm định hợp lệ thành công chuẩn."""
        return cls(True, "ok")


VALIDATION_OK = ValidationResult.ok()

# Độ dài trùng lặp phần ruột ngắn nhất với đa giác mà bộ kiểm định này
# có thể phân biệt được so với tiếp xúc biên hình học (tính bằng mét).
#
# Đây KHÔNG PHẢI là sự nới lỏng cho đường bay — mà là giới hạn độ phân giải
# số học của CHÍNH bộ kiểm định, tương tự như TURN_RESERVE_TOL_M bên dưới.
# Vị từ chính xác là "độ dài trùng lặp phần ruột mang dấu dương", nhưng shapely
# không thể tính "dương" tốt hơn cỡ 1 ULP của tọa độ: một cung lượn TIẾP XÚC
# với cạnh bao lồi (trường hợp bình thường, khi giãn nở chỉ là SAFE_MARGIN và
# các đỉnh bao lồi nằm chính xác trên đa giác thô) báo cáo phần trùng lặp
# vài nanomet. Đo trên 3 đường bay bị ngưỡng 0 từ chối: 8.1e-9 m, 4.4e-9 m và
# 5.8e-11 m (0.06 nanomet). Ở mức 0, oracle loại bỏ nhiệm vụ bay khả thi vì
# làm tròn số học của chính nó (3/300 kịch bản, v0).
#
# Ngưỡng 1e-6 m cao hơn 100 lần so với nhiễu đó và vẫn thấp hơn 6 bậc độ lớn
# so với bất kỳ yêu cầu vận hành nào (SAFE_MARGIN tính bằng mét; giao cắt thực
# tế dài tới hàng kilômet). Trước đây là 1e-3 m (từng là sự nới lỏng thực tế,
# gấp 1000 lần mức cần thiết).
POLYGON_TOUCH_TOL_M = config.ORACLE_POLYGON_TOUCH_TOL_M
TURN_RESERVE_TOL_M = config.ORACLE_TURN_RESERVE_TOL_M
_ARC_SAMPLES = config.ORACLE_ARC_SAMPLES
"""Số đoạn con mà cung lượn fillet tại góc rẽ được lấy mẫu để kiểm tra an toàn."""


# LƯU Ý: Chủ ý tự cài đặt lại hàm khoảng cách thay vì import từ spatial_utils,
# giúp bộ kiểm định độc lập hoàn toàn với mã nguồn bộ lập kế hoạch mà nó đánh giá.
def _point_to_segment_distance(p: Point, a: Point, b: Point) -> float:
    """Tính khoảng cách từ điểm p đến đoạn thẳng a-b (m)."""
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def interior_overlap_length(poly: Polygon, line: LineString) -> float:
    """Đo chiều dài đoạn thẳng line nằm thực sự BÊN TRONG phần ruột của đa giác poly.

    ``poly.intersection(line).length`` là đại lượng không phù hợp: nó đo phần
    trùng lặp với đa giác ĐÓNG (ruột HỢP VỚI đường biên), do đó một hợp âm chạy
    dọc theo cạnh bao lồi một cách hợp lệ sẽ bị tính toàn bộ chiều dài dọc biên
    (hàng kilômet) dù nó không hề đi vào bên trong ruột. Đo trên batch_random_test
    seed 194: báo cáo 11533.475 m "trùng lặp", trong đó 11533.475 m nằm trên biên
    và 0.0 m nằm bên trong. Phép trừ phần biên để lại chính xác độ xâm nhập ruột.

    Hàm này dùng chung với planner: một planner khắt khe hơn oracle sẽ từ chối các
    đường bay khả thi, còn planner lỏng lẻo hơn sẽ sinh đường bay bị oracle đánh
    trượt. Phải có duy nhất một định nghĩa thống nhất về độ xâm lấn.

    Args:
        poly: Đa giác vật cản cần kiểm tra.
        line: Đoạn thẳng hợp âm đang được đo đạc.

    Returns:
        Chiều dài phần xâm nhập vào ruột tính bằng mét; trả về 0.0 nếu đoạn thẳng
        chỉ tiếp xúc hoặc chạy dọc trên biên.
    """
    return poly.intersection(line).length - poly.boundary.intersection(line).length


def _segment_clear(
    a: Point,
    b: Point,
    circle_obstacles: Sequence[CircleGeometry],
    polygon_obstacles: Sequence[PolygonCoords],
) -> bool:
    """Kiểm tra đoạn thẳng đơn lẻ a->b có an toàn trước mọi chướng ngại vật không."""
    for center, radius in circle_obstacles:
        # CHÍNH XÁC: đường tròn bị va chạm khi và chỉ khi đoạn thẳng tới gần hơn
        # bán kính. Không có dung sai — phép kiểm tra không bao giờ tha thứ cho
        # xâm lấn thực tế. Các hợp âm do planner tạo ra giữ được lề an toàn vì
        # hình học được DỰNG trên bán kính + CONSTRUCTION_CLEARANCE_M + GEOM_EPS_M.
        # Đo trên 120 kịch bản: đoạn thẳng được chấp nhận gần nhất vẫn cách biên
        # 0.112 m về PHÍA NGOÀI.
        if _point_to_segment_distance(center, a, b) < radius:
            return False

    # Đoạn thẳng CHỈ bị chặn khi nó đi vào RUỘT đa giác (mô hình DE-9IM giao ruột/ruột,
    # mẫu 'T********'). Việc chạm vào biên được phép: một waypoint có thể nằm trên
    # góc đa giác (các đỉnh là mục tiêu dẫn đường hợp lệ, như các điểm tiếp tuyến
    # đường tròn), và một đoạn thẳng có thể chạy DỌC theo cạnh để men theo biên.
    #
    # Độ dài trùng lặp ruột phải mang ĐỘ ĐO DƯƠNG. Mẫu 'T********' cũng khớp với điểm
    # chạm độ dài 0, và đây không phải là giả định: cung lượn fillet TIẾP XÚC với cạnh
    # đa giác bất cứ khi nào điểm pivot là đỉnh bao lồi và chặng bay ra chạy dọc cạnh
    # — tình huống bình thường khi độ giãn nở chỉ là SAFE_MARGIN, vì các đỉnh bao lồi
    # lúc này nằm chính xác trên đa giác thô. Điểm tiếp xúc của cung lượn rời rạc hóa
    # rơi lệch vào trong một khoảng vi mô float và shapely báo cáo Point có chiều dài 0
    # là giao cắt ruột (batch_random_test seed 166). Yêu cầu độ dài dương giữ lại mọi
    # trường hợp xuyên cắt thực tế (dài từ mét tới kilômet) trong khi bỏ qua điểm chạm.
    #
    # Nó phải là chiều dài BÊN TRONG RUỘT, không phải đa giác đóng — xem hàm
    # interior_overlap_length. Mẫu 'T********' cũng có thể mâu thuẫn với hình học:
    # ở seed 194 nó báo giao cắt ruột chiều 1 cho đoạn thẳng có độ dài ruột đúng 0.0 m,
    # vì đoạn thẳng chạy dọc cạnh và hai vị từ đánh chỉ mục nút khác nhau. Đo trực tiếp
    # chiều dài sẽ giải quyết dứt điểm điều này.
    line = LineString([a, b])
    for coords in polygon_obstacles:
        poly = Polygon(coords)
        if not poly.relate_pattern(line, "T********"):
            continue
        if interior_overlap_length(poly, line) > POLYGON_TOUCH_TOL_M:
            return False
    return True


def segments_clear(
    path: Sequence[PlannerState],
    circle_obstacles: Sequence[CircleGeometry],
    polygon_obstacles: Sequence[PolygonCoords],
) -> ValidationResult:
    """Kiểm tra các đoạn thẳng nối giữa các waypoint liên tiếp có an toàn không.

    Args:
        path: Chuỗi waypoint cần kiểm tra.
        circle_obstacles: Vật cản đường tròn dạng ((cx, cy), bán_kính).
        polygon_obstacles: Danh sách các vòng tọa độ đa giác vật cản.

    Returns:
        Kết quả kiểm định, ghi rõ đoạn thẳng đầu tiên bị chặn nếu thất bại.
    """
    for i in range(len(path) - 1):
        a = path[i][0]
        b = path[i + 1][0]
        if not _segment_clear(a, b, circle_obstacles, polygon_obstacles):
            return ValidationResult(False, f"segment {i} blocked ({a} -> {b})")
    return ValidationResult.ok()


def _seg_heading(a: Point, b: Point) -> float:
    """Tính góc hướng bay từ điểm a đến b (rad)."""
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _norm(delta: float) -> float:
    """Chuẩn hóa góc về khoảng [-pi, pi]."""
    return math.atan2(math.sin(delta), math.cos(delta))


def turn_angles(path: Sequence[PlannerState]) -> list[float]:
    """Tính góc chuyển hướng tại từng waypoint bên trong dựa vào hình học đoạn thẳng.

    Args:
        path: Chuỗi waypoint cần đo đạc.

    Returns:
        Danh sách độ lớn góc rẽ (rad), một giá trị cho mỗi waypoint bên trong.
    """
    angles: list[float] = []
    for i in range(1, len(path) - 1):
        h_in = _seg_heading(path[i - 1][0], path[i][0])
        h_out = _seg_heading(path[i][0], path[i + 1][0])
        angles.append(abs(_norm(h_out - h_in)))
    return angles


def turn_angles_ok(
    path: Sequence[PlannerState], alpha_max_rad: float
) -> ValidationResult:
    """Kiểm tra không có góc rẽ nào vượt quá giới hạn alpha_max của phương tiện.

    Args:
        path: Chuỗi waypoint cần kiểm tra.
        alpha_max_rad: Giới hạn góc rẽ tối đa (rad).

    Returns:
        Kết quả kiểm tra, nêu rõ waypoint đầu tiên vượt giới hạn nếu thất bại.
    """
    for i, angle in enumerate(turn_angles(path)):
        # Chính xác tuyệt đối; planner dựng hình tại alpha_max - GEOM_EPS_RAD.
        if angle > alpha_max_rad:
            return ValidationResult(
                False,
                f"wp[{i + 1}] {path[i + 1][0]} turn angle {math.degrees(angle):.3f}° "
                f"> alpha_max {math.degrees(alpha_max_rad):.3f}°",
            )
    return ValidationResult.ok()


def _seg_len(a: Point, b: Point) -> float:
    """Tính chiều dài đoạn thẳng nối a và b (m)."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def straight_segments_ok(
    path: Sequence[PlannerState], turn_radius: float, l0: float, dss: float
) -> ValidationResult:
    """Kiểm tra các ràng buộc đoản trình đoạn thẳng l1 >= L0, ln >= 0 và l_giua > 0.

    Ràng buộc áp dụng cho đoạn thẳng BAY GIỮA HAI GÓC RẼ, không đồng nhất với
    đoạn thẳng nối giữa hai waypoint: một waypoint có góc rẽ bằng 0 được máy bay
    bay thẳng qua và không chia cắt đoạn thẳng. Bộ lập kế hoạch thường xuyên tạo ra
    các waypoint như vậy — ví dụ xuất phát rời cung lượn theo hướng tiếp tuyến,
    hoặc trượt điểm pivot dọc tia cố tình bay thẳng qua ứng viên ban đầu trước khi
    rẽ ở điểm trượt — do đó các đoạn thẳng liên tiếp thẳng hàng được gộp lại tại đây.
    Nếu tính trọn vẹn phần dự trữ góc rẽ ở cả hai đầu mút cho mỗi đoạn con sẽ bị tính
    trùng hai lần và loại bỏ oan các đường bay khả thi (batch_random_test seed 117:
    đoạn 1701 m giữa hai waypoint thẳng hàng bị tính dự trữ 4000 m của góc rẽ thực
    tiếp theo, trong khi phần dự trữ này thực tế lấn vô hại vào đoạn 4100 m trước đó).

    Args:
        path: Chuỗi waypoint cần kiểm tra.
        turn_radius: Bán kính quay vòng tối thiểu của phương tiện (m).
        l0: Chiều dài đoạn bay thẳng tối thiểu sau cất cánh (m).
        dss: Chiều dài đoản trình bay thẳng tiếp cận trước đích (m).

    Returns:
        Kết quả kiểm định, chỉ rõ đoạn vi phạm đầu tiên nếu thất bại. Đường bay
        không có đoạn thẳng nào được coi là hợp lệ tầm thường (trivial).
    """
    n_seg = len(path) - 1
    if n_seg < 1:
        return ValidationResult(True, "trivial")

    # Góc alpha tại mỗi chỉ số waypoint; hai đầu mút không có góc rẽ trước/sau.
    alphas = [0.0, *turn_angles(path), 0.0]
    reserves = [turn_radius * math.tan(a / 2) for a in alphas]

    # Waypoint thực sự bẻ hướng đường bay sẽ phân định các đoạn bay thẳng;
    # hai điểm đầu mút luôn phân định (mang dự trữ 0 và chặn đoạn đầu/cuối).
    breaks = [0]
    breaks.extend(i for i in range(1, n_seg) if reserves[i] > TURN_RESERVE_TOL_M)
    breaks.append(n_seg)

    for k in range(len(breaks) - 1):
        i, j = breaks[k], breaks[k + 1]
        run = sum(_seg_len(path[m][0], path[m + 1][0]) for m in range(i, j))
        usable = run - reserves[i] - reserves[j]
        span = f"wp[{i}]..wp[{j}]" if j > i + 1 else f"segment {i}"
        # Cả 3 điều kiện đều CHÍNH XÁC: mức dung sai 1 m trước đây từng tha thứ
        # cho các vi phạm thực tế. Tính khả thi đến từ phía khâu dựng hình — các
        # góc xuất phát được dựng tại L0 + GEOM_EPS_M, các góc rẽ tại alpha_max -
        # GEOM_EPS_RAD. Đo đạc biên xấu nhất trên 120 kịch bản đường bay được chấp
        # nhận: L0 +9.96e-9 m, DSS +413 m, đoạn giữa +96 m.
        if i == 0:  # đoản trình đầu tiên: l1 >= L0
            if usable < l0:
                return ValidationResult(False, f"first {span} l={usable:.3f} < L0={l0}")
        elif j == n_seg:  # đoản trình cuối: ln = l - dss >= 0
            if usable - dss < 0.0:
                return ValidationResult(
                    False, f"last {span} usable l={usable - dss:.3f} < 0"
                )
        elif usable <= 0.0:  # đoạn giữa: l > 0
            return ValidationResult(False, f"middle {span} l={usable:.3f} <= 0")
    return ValidationResult.ok()


def _unit(a: Point, b: Point) -> Point:
    """Tính vector đơn vị từ a đến b, hoặc (0, 0) nếu hai điểm trùng nhau."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy)
    return (dx / d, dy / d) if d > 0 else (0.0, 0.0)


def arc_points(
    w_prev: Point,
    w: Point,
    w_next: Point,
    *,
    turn_radius: float,
    n: int = _ARC_SAMPLES,
) -> list[Point]:
    """Lấy mẫu chuỗi điểm rời rạc dọc theo cung lượn bán kính R bo góc rẽ w.

    Hàm để public vì planner kiểm tra CHÍNH XÁC cung lượn mà oracle sẽ kiểm định;
    bản sao cục bộ tại planner sẽ tạo ra định nghĩa thứ hai về hình học bay.

    Args:
        w_prev: Waypoint trước góc rẽ.
        w: Waypoint tại đỉnh góc rẽ.
        w_next: Waypoint sau góc rẽ.
        turn_radius: Bán kính cung lượn fillet (m).
        n: Số đoạn thẳng con dùng để rời rạc hóa cung.

    Returns:
        Danh sách ``n + 1`` điểm dọc theo cung lượn, hoặc danh sách rỗng nếu góc rẽ
        thẳng hàng và không cần lấy mẫu cung lượn.
    """
    u = _unit(w_prev, w)  # hướng bay vào
    v = _unit(w, w_next)  # hướng bay ra
    alpha = abs(_norm(math.atan2(v[1], v[0]) - math.atan2(u[1], u[0])))
    if alpha < 1e-9:
        return []
    tangent = turn_radius * math.tan(alpha / 2)  # chiều dài tiếp tuyến mỗi chặng
    entry = (
        w[0] - u[0] * tangent,
        w[1] - u[1] * tangent,
    )  # tiếp điểm trên chặng bay vào
    s = 1.0 if (u[0] * v[1] - u[1] * v[0]) > 0 else -1.0  # rẽ trái(+)/rẽ phải(-)
    n_in = (-u[1] * s, u[0] * s)  # pháp tuyến hướng vào trong của chặng bay vào
    cx = entry[0] + turn_radius * n_in[0]
    cy = entry[1] + turn_radius * n_in[1]
    start = math.atan2(entry[1] - cy, entry[0] - cx)
    return [
        (
            cx + turn_radius * math.cos(start + s * alpha * (k / n)),
            cy + turn_radius * math.sin(start + s * alpha * (k / n)),
        )
        for k in range(n + 1)
    ]


def arcs_clear(
    path: Sequence[PlannerState],
    turn_radius: float,
    circle_obstacles: Sequence[CircleGeometry],
    polygon_obstacles: Sequence[PolygonCoords],
) -> ValidationResult:
    """Kiểm tra toàn bộ các cung lượn góc rẽ trên đường bay không va chạm vật cản.

    Args:
        path: Chuỗi waypoint cần kiểm tra.
        turn_radius: Bán kính cung lượn fillet (m).
        circle_obstacles: Vật cản đường tròn dạng ((cx, cy), bán_kính).
        polygon_obstacles: Danh sách các vòng tọa độ đa giác vật cản.

    Returns:
        Kết quả kiểm định, ghi rõ cung lượn đầu tiên bị va chạm nếu thất bại.
    """
    for i in range(1, len(path) - 1):
        points = arc_points(
            path[i - 1][0], path[i][0], path[i + 1][0], turn_radius=turn_radius
        )
        for j in range(len(points) - 1):
            if not _segment_clear(
                points[j], points[j + 1], circle_obstacles, polygon_obstacles
            ):
                return ValidationResult(
                    False,
                    f"turn arc at wp[{i}] {path[i][0]} blocked "
                    f"({points[j]} -> {points[j + 1]})",
                )
    return ValidationResult.ok()


def path_is_valid(
    path: Sequence[PlannerState],
    circle_obstacles: Sequence[CircleGeometry],
    polygon_obstacles: Sequence[PolygonCoords],
    *,
    turn_radius: float,
    alpha_max_rad: float,
    l0: float,
    dss: float,
    raw_circle_obstacles: Sequence[CircleGeometry] | None = None,
    raw_polygon_obstacles: Sequence[PolygonCoords] | None = None,
) -> ValidationResult:
    """Kiểm định tổng thể toàn bộ các điều kiện an toàn và động học của đường bay.

    Cả các đoạn thẳng VÀ các cung lượn góc rẽ đều phải an toàn trước các vật cản
    ĐÃ GIÃN NỞ, hiện tại là vật cản thô + SAFE_MARGIN: toàn bộ đường bay đều tuân
    thủ khoảng cách an toàn tối thiểu của người vận hành.

    Args:
        path: Chuỗi waypoint cần kiểm định.
        circle_obstacles: Vật cản đường tròn đã giãn nở ((cx, cy), bán_kính).
        polygon_obstacles: Các vòng tọa độ đa giác vật cản đã giãn nở.
        turn_radius: Bán kính quay vòng của phương tiện (m).
        alpha_max_rad: Giới hạn góc rẽ tối đa (rad).
        l0: Chiều dài đoạn bay thẳng tối thiểu sau cất cánh (m).
        dss: Chiều dài đoản trình bay thẳng tiếp cận trước đích (m).
        raw_circle_obstacles: Cơ chế tương thích ngược; xem phần Note bên dưới.
        raw_polygon_obstacles: Cơ chế tương thích ngược; xem phần Note bên dưới.

    Returns:
        Kết quả kiểm định, ghi rõ bước kiểm tra thất bại đầu tiên.

    Note:
        Các tham số ``raw_*`` là cơ chế tương thích ngược cũ. Chúng từng tồn tại
        vì phép giãn nở trước đây kèm số hạng quay ``R*(1/cos(alpha_max/2)-1)``
        và cung lượn fillet được thiết kế để phồng đúng vào dải đệm đó, do đó
        cung lượn được kiểm định đối chiếu với vật cản thô. Khi số hạng quay bị
        loại bỏ thì dải đệm này không còn, và việc truyền tập vật cản thô vào đây
        sẽ khiến cung rẽ có thể lấn vào phạm vi SAFE_MARGIN. Hãy để mặc định (None)
        trừ khi bạn chủ ý muốn tái lập mô hình cũ.
    """
    if not path or len(path) < 2:
        return ValidationResult(False, "path too short")

    seg_res = segments_clear(path, circle_obstacles, polygon_obstacles)
    if not seg_res.is_ok:
        return ValidationResult(False, f"segments blocked: {seg_res.detail}")

    turn_res = turn_angles_ok(path, alpha_max_rad)
    if not turn_res.is_ok:
        return ValidationResult(False, f"turn angles invalid: {turn_res.detail}")

    arc_circles = (
        circle_obstacles if raw_circle_obstacles is None else raw_circle_obstacles
    )
    arc_polys = (
        polygon_obstacles if raw_polygon_obstacles is None else raw_polygon_obstacles
    )
    arc_res = arcs_clear(path, turn_radius, arc_circles, arc_polys)
    if not arc_res.is_ok:
        return ValidationResult(False, f"turn arcs blocked: {arc_res.detail}")

    straight_res = straight_segments_ok(path, turn_radius, l0, dss)
    if not straight_res.is_ok:
        return ValidationResult(
            False, f"straight segments invalid: {straight_res.detail}"
        )

    return ValidationResult.ok()
