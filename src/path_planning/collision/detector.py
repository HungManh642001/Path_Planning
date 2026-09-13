# pyright: reportMissingTypeArgument=false, reportUnknownMemberType=false, reportUnknownVariableType=false
"""Động cơ kiểm tra va chạm không gian cho đoạn thẳng, cung lượn và biên an toàn.

Cung cấp lớp :class:`CollisionDetector` xử lý toàn bộ truy vấn va chạm 2D
giữa đoạn bay thẳng, cung lượn fillet arc, hình quạt vành khuyên và vùng an toàn.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from shapely.geometry import LineString, Point as ShapelyPoint, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union
from shapely.prepared import PreparedGeometry, prep as shp_prep

from path_planning import config
from path_planning.geometry import arc, spatial
from path_planning.validation import oracle


if TYPE_CHECKING:
    from path_planning.types import Point, PreprocessedScenario


class CollisionDetector:
    """Động cơ kiểm tra va chạm tầm nhìn đoạn thẳng và không gian cung lượn bo góc.

    Attributes:
        scenario: Kịch bản tiền xử lý chứa chướng ngại vật và các đường biên.
        turn_radius: Bán kính quay vòng tối thiểu của phương tiện (m).
        construct_delta: Vùng đệm khoảng cách an toàn cộng thêm vào vật cản (m).
        polygons: Danh sách đa giác Shapely đại diện cho các đảo chướng ngại vật.
        poly_bboxes: Hộp bao (minx, miny, maxx, maxy) cho tất cả các đảo.
        circles: Danh sách chướng ngại vật tròn dạng (center_x, center_y, radius).
        safezone: Đa giác/đa đa giác bao bọc vùng an toàn, nếu được định nghĩa.
        safezone_prep: Hình học Shapely chuẩn bị sẵn để truy vấn điểm nhanh.
        has_explicit_bounds: Cờ xác định giới hạn hộp bao bản đồ có được cấp không.
        bounds_w: Chiều rộng vùng hoạt động tính bằng mét.
        bounds_h: Chiều cao vùng hoạt động tính bằng mét.
    """

    def __init__(
        self,
        preprocessed_scenario: PreprocessedScenario,
        *,
        turn_radius: float = config.R,
    ) -> None:
        """Khởi tạo hình học chướng ngại vật, hộp bao và vùng an toàn.

        Args:
            preprocessed_scenario: Dictionary kịch bản đã tiền xử lý.
            turn_radius: Bán kính quay vòng tối thiểu của phương tiện tính bằng mét.
        """
        self.scenario = preprocessed_scenario
        self.turn_radius = turn_radius
        self.construct_delta = config.CONSTRUCTION_CLEARANCE_M + config.GEOM_EPS_M

        self.polygons: list[Polygon] = [
            Polygon(coords) for coords in preprocessed_scenario["polygon_obstacles"]
        ]
        self.poly_bboxes: list[tuple[float, float, float, float]] = [
            p.bounds for p in self.polygons
        ]
        self.circles: list[tuple[float, float, float]] = [
            (c[0], c[1], r) for c, r in preprocessed_scenario["circle_obstacles"]
        ]

        safezones = preprocessed_scenario.get("safezones")
        self.safezone: BaseGeometry | None = (
            unary_union([Polygon(sz) for sz in safezones]) if safezones else None
        )
        self.safezone_prep: PreparedGeometry | None = (
            shp_prep(self.safezone) if self.safezone is not None else None
        )

        bounds = preprocessed_scenario.get("map_bounds")
        self.has_explicit_bounds: bool = bounds is not None
        self.bounds_w: float
        self.bounds_h: float
        self.bounds_w, self.bounds_h = (
            bounds if bounds else (config.MAP_WIDTH, config.MAP_HEIGHT)
        )

    def is_collision_free(self, p1: Point, p2: Point) -> bool:
        """Kiểm tra đoạn thẳng nối p1 -> p2 có an toàn và không va chạm vật cản không.

        Áp dụng lọc hộp bao trục tọa độ (AABB) trước khi tính khoảng cách
        từ điểm đến đoạn thẳng cho vật cản tròn và kiểm tra giao cắt topo
        Shapely relate_pattern cho các chướng ngại vật đa giác.

        Args:
            p1: Tọa độ điểm đầu đoạn thẳng (x, y) tính bằng mét.
            p2: Tọa độ điểm cuối đoạn thẳng (x, y) tính bằng mét.

        Returns:
            True nếu đoạn thẳng tránh được toàn bộ vật cản và nằm trong vùng an toàn;
            False nếu ngược lại.
        """
        x0, x1 = (p1[0], p2[0]) if p1[0] <= p2[0] else (p2[0], p1[0])
        y0, y1 = (p1[1], p2[1]) if p1[1] <= p2[1] else (p2[1], p1[1])

        for cx, cy, radius in self.circles:
            if (
                cx + radius < x0
                or cx - radius > x1
                or cy + radius < y0
                or cy - radius > y1
            ):
                continue
            if spatial.point_to_line_distance((cx, cy), p1, p2) < radius:
                return False

        line: LineString | None = None
        for idx, (bx0, by0, bx1, by1) in enumerate(self.poly_bboxes):
            if x1 < bx0 or bx1 < x0 or y1 < by0 or by1 < y0:
                continue
            if line is None:
                line = LineString([p1, p2])
            if self.polygons[idx].relate_pattern(line, "T********"):
                return False

        if self.safezone is not None:
            if line is None:
                line = LineString([p1, p2])
            if not self.safezone.covers(line):
                return False
        return True

    def is_corner_arc_clear(self, h_in: float, w: Point, w_next: Point) -> bool:
        """Kiểm tra cung lượn fillet arc bán kính R bo góc rẽ w có an toàn không.

        Args:
            h_in: Góc hướng bay vào góc rẽ w tính bằng radian.
            w: Tọa độ waypoint góc rẽ (x, y) tính bằng mét.
            w_next: Tọa độ waypoint bay ra tiếp theo (x, y) tính bằng mét.

        Returns:
            True nếu đường cong fillet không giao cắt với bất kỳ vật cản nào;
            False nếu ngược lại.
        """
        prev = (w[0] - math.cos(h_in), w[1] - math.sin(h_in))
        pts = oracle.arc_points(
            prev, w, w_next, turn_radius=self.turn_radius, n=config.ARC_CHECK_SAMPLES
        )
        if not pts:
            return True

        ax0 = min(p[0] for p in pts)
        ax1 = max(p[0] for p in pts)
        ay0 = min(p[1] for p in pts)
        ay1 = max(p[1] for p in pts)

        for cx, cy, radius in self.circles:
            if (
                cx + radius < ax0
                or cx - radius > ax1
                or cy + radius < ay0
                or cy - radius > ay1
            ):
                continue
            center = (cx, cy)
            for j in range(len(pts) - 1):
                if spatial.point_to_line_distance(center, pts[j], pts[j + 1]) < radius:
                    return False

        line: LineString | None = None
        for idx, (bx0, by0, bx1, by1) in enumerate(self.poly_bboxes):
            if ax1 < bx0 or bx1 < ax0 or ay1 < by0 or by1 < ay0:
                continue
            if line is None:
                line = LineString(pts)
            if self.polygons[idx].relate_pattern(line, "T********"):
                return False

        if self.safezone is not None:
            if line is None:
                line = LineString(pts)
            if not self.safezone.covers(line):
                return False
        return True

    def on_circle_boundary(self, point: Point, tol: float | None = None) -> bool:
        """Kiểm tra điểm có nằm trên biên chướng ngại vật tròn nào không.

        Args:
            point: Tọa độ điểm cần kiểm tra (x, y).
            tol: Dung sai khoảng cách tính bằng mét. Nếu None, dùng khoảng hở
                dựng hình mặc định.

        Returns:
            True nếu điểm nằm trong phạm vi dung sai của biên hình tròn;
            False nếu ngược lại.
        """
        if tol is None:
            tol = self.construct_delta + config.GEOM_EPS_M
        return arc.is_point_on_any_circle_boundary(
            point, self.scenario["circle_obstacles"], tol
        )

    def is_in_bounds(self, point: Point) -> bool:
        """Kiểm tra điểm có nằm trong phạm vi bản đồ hoặc vùng an toàn safezone không.

        Args:
            point: Tọa độ điểm cần kiểm tra (x, y) tính bằng mét.

        Returns:
            True nếu nằm trong giới hạn bản đồ / vùng an toàn; False nếu ngược lại.
        """
        if self.safezone_prep is not None:
            return self.safezone_prep.covers(ShapelyPoint(*point))
        if not self.has_explicit_bounds:
            return True
        x, y = point
        return 0 < x < self.bounds_w and 0 < y < self.bounds_h

    def check_fixed_legs(self, goal_wp: Point, target: Point) -> bool:
        """Kiểm tra đoạn thẳng tiếp cận mục tiêu W_{n-1} -> T có thông suốt không.

        Args:
            goal_wp: Vị trí waypoint áp chót W_{n-1}.
            target: Vị trí mục tiêu đích T.

        Returns:
            True nếu đoạn thẳng tự dẫn vào đích không va chạm; False nếu ngược lại.
        """
        return self.is_collision_free(goal_wp, target)

    def ray_chord_clear(
        self,
        memo: dict[float, list[float]],
        ray: float,
        dist: float,
        p1: Point,
        p2: Point,
    ) -> bool:
        """Kiểm tra va chạm đoạn thẳng dọc theo tia kèm ghi nhớ đoạn thông suốt/bị chặn.

        Args:
            memo: Bảng ghi nhớ khoảng cách an toàn từ góc tia tới
                [khoảng_an_toàn_nhỏ_nhất, khoảng_chặn_lớn_nhất].
            ray: Góc tia tính bằng radian.
            dist: Khoảng cách từ gốc tia đến điểm cuối tính bằng mét.
            p1: Tọa độ điểm đầu đoạn thẳng (x, y).
            p2: Tọa độ điểm cuối đoạn thẳng (x, y).

        Returns:
            True nếu đoạn thẳng không va chạm; False nếu bị chặn.
        """
        span = memo.get(ray)
        if span is None:
            span = memo[ray] = [0.0, float("inf")]
        if dist <= span[0]:
            return True
        if dist >= span[1]:
            return False
        if self.is_collision_free(p1, p2):
            span[0] = dist
            return True
        span[1] = dist
        return False
