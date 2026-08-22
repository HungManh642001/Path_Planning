"""Bản đồ nền tĩnh dạng XML, nạp một lần lúc worker khởi động.

Mặc định triển khai là không có bản đồ nào: một request tự chứa thì replay được
và chẩn đoán được, còn state ẩn trong service thì không. Bản đồ nền tồn tại cho
trường hợp bản đồ quá lớn để gửi kèm mỗi request.

Gộp là NỐI THÊM, không thay thế. Với ``safezones`` thì planner lấy HỢP của
chúng (``unary_union`` trong ``kinodynamic_astar_v0``), nên thêm một safezone là
NỚI RỘNG vùng bay chứ không thu hẹp. Trực giác thường ngược lại, nên điều này
cũng nằm trong tài liệu vận hành chứ không chỉ ở đây.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree

from vtx_service.messages import Circle, PlanRequest, Point

MAP_VERSION = "1"
_CLOSING_TOL_M = 1e-9


@dataclass(frozen=True)
class PreloadedMap:
    """Vùng bay và chướng ngại vật nền, toạ độ mét trong hệ Oxy."""

    safezones: tuple[tuple[Point, ...], ...]
    islands: tuple[tuple[Point, ...], ...]
    dynamic_obstacles: tuple[Circle, ...]

    @classmethod
    def load(cls, path: Path) -> PreloadedMap:
        """Đọc một bản đồ nền từ file XML.

        Args:
            path: File XML gốc ``<vtx-map version="1">``.

        Returns:
            Bản đồ đã nạp.

        Raises:
            ValueError: Khi tag gốc, ``version``, hoặc hình học không hợp lệ.
        """
        root = ElementTree.parse(path).getroot()
        if root.tag != "vtx-map":
            raise ValueError(f"tag gốc phải là vtx-map, nhận {root.tag!r}")
        version = root.get("version")
        if version != MAP_VERSION:
            raise ValueError(f"version bản đồ {version!r} != {MAP_VERSION!r}")

        return cls(
            safezones=tuple(_polygons(root.find("safezones"))),
            islands=tuple(_polygons(root.find("obstacles"))),
            dynamic_obstacles=tuple(_circles(root.find("obstacles"))),
        )

    def merged_into(self, request: PlanRequest) -> PlanRequest:
        """Nối bản đồ nền vào một request, nếu request yêu cầu.

        Args:
            request: Request gốc.

        Returns:
            Request đã gộp, hoặc CHÍNH ``request`` khi cờ tắt - trả về cùng đối
            tượng để chỗ gọi phân biệt được "không gộp" với "gộp rỗng".
        """
        if not request.use_preloaded_map:
            return request
        return dataclasses.replace(
            request,
            safezones=request.safezones + self.safezones,
            islands=request.islands + self.islands,
            dynamic_obstacles=request.dynamic_obstacles + self.dynamic_obstacles,
        )


def _polygons(section: ElementTree.Element | None) -> list[tuple[Point, ...]]:
    if section is None:
        return []
    return [_ring(node) for node in section.findall("polygon")]


def _ring(node: ElementTree.Element) -> tuple[Point, ...]:
    points: list[Point] = [
        (float(p.get("x", "nan")), float(p.get("y", "nan"))) for p in node.findall("point")
    ]
    # Vành MỞ: `core/` giả định không có đỉnh đóng lặp lại, và một đỉnh trùng
    # lặp tạo ra cạnh dài 0 mà oracle sẽ từ chối.
    if len(points) >= 2 and math.dist(points[0], points[-1]) < _CLOSING_TOL_M:
        points.pop()
    if len(points) < 3:
        raise ValueError(f"đa giác cần ít nhất 3 đỉnh, nhận {len(points)}")
    return tuple(points)


def _circles(section: ElementTree.Element | None) -> list[Circle]:
    if section is None:
        return []
    circles: list[Circle] = []
    for node in section.findall("circle"):
        radius = float(node.get("r", "nan"))
        if not radius > 0.0:
            raise ValueError(f"radius phải dương, nhận {radius}")
        circles.append(
            Circle(
                center=(float(node.get("cx", "nan")), float(node.get("cy", "nan"))),
                radius_m=radius,
            )
        )
    return circles
