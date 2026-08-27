# pyright: reportMissingTypeArgument=false, reportUnknownMemberType=false, reportUnknownVariableType=false
"""Spatial collision detector for straight chords, turn arcs and boundaries.

Provides the :class:`CollisionDetector` class, which handles all 2D collision
queries between straight flight paths, fillet turn arcs, annular sweep sectors,
obstacle bounding boxes, and operational safezone polygons.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from shapely.geometry import LineString, MultiPolygon, Point as ShapelyPoint, Polygon
from shapely.ops import unary_union
from shapely.prepared import PreparedGeometry, prep as shp_prep

from path_planning import config
from path_planning.geometry import arc as ag, spatial as su
from path_planning.validation import oracle as pv


if TYPE_CHECKING:
    from path_planning.types import Point, PreprocessedScenario


class CollisionDetector:
    """Collision engine checking straight line-of-sight and corner clearance.

    Attributes:
        scenario: Preprocessed scenario containing obstacles and boundaries.
        turn_radius: Minimum vehicle turn radius in metres.
        construct_delta: Clearance buffer added to obstacle geometry in metres.
        polygons: List of Shapely polygon representations of island obstacles.
        poly_bboxes: Bounding boxes (minx, miny, maxx, maxy) for all polygon obstacles.
        circles: List of circular obstacles as (center_x, center_y, radius).
        safezone: Enclosing multi-polygon safe operational area, if defined.
        safezone_prep: Prepared Shapely geometry for fast spatial containment queries.
        has_explicit_bounds: Whether bounding box limits were provided.
        bounds_w: Operational area width in metres.
        bounds_h: Operational area height in metres.
    """

    def __init__(
        self,
        preprocessed_scenario: PreprocessedScenario,
        *,
        turn_radius: float = config.R,
    ) -> None:
        """Initialize obstacle geometry, bounding boxes and safezones.

        Args:
            preprocessed_scenario: Preprocessed scenario dictionary.
            turn_radius: Minimum vehicle turning radius in metres.
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
        self.safezone: Polygon | MultiPolygon | None = (
            unary_union([Polygon(sz) for sz in safezones]) if safezones else None  # pyright: ignore[reportAttributeAccessIssue]
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
        """Test whether the straight segment p1 -> p2 is flyable and collision-free.

        Applies axis-aligned bounding box filtering before computing exact
        point-to-segment distances for circular obstacles and Shapely relate_pattern
        topological intersection tests for polygonal obstacles.

        Args:
            p1: Segment start coordinate (x, y) in metres.
            p2: Segment end coordinate (x, y) in metres.

        Returns:
            True if the segment clears all obstacles and remains inside safezone;
            False otherwise.
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
            if su.point_to_line_distance((cx, cy), p1, p2) < radius:
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
        """Test whether the radius-R fillet arc rounding corner w is clear of obstacles.

        Args:
            h_in: Inbound heading angle into corner w in radians.
            w: Corner waypoint position (x, y) in metres.
            w_next: Outbound destination waypoint position (x, y) in metres.

        Returns:
            True if the fillet curve does not intersect any obstacles; False otherwise.
        """
        prev = (w[0] - math.cos(h_in), w[1] - math.sin(h_in))
        pts = pv.arc_points(
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
                if su.point_to_line_distance(center, pts[j], pts[j + 1]) < radius:
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

    def is_sector_clear(
        self, center: Point, r_in: float, r_out: float, phi_a: float, phi_b: float
    ) -> bool:
        """Test whether an annular sector around circle center is free of obstacles.

        Args:
            center: Circle centre coordinate (x, y).
            r_in: Inner boundary radius in metres.
            r_out: Outer boundary radius in metres.
            phi_a: Start azimuth of sector in radians.
            phi_b: End azimuth of sector in radians.

        Returns:
            True if no other obstacles intrude into the annular sector; False otherwise.
        """
        lo, hi = (phi_a, phi_b) if phi_a <= phi_b else (phi_b, phi_a)
        for c2, r2 in self.scenario["circle_obstacles"]:
            dx, dy = c2[0] - center[0], c2[1] - center[1]
            d = math.hypot(dx, dy)
            if d - r2 >= r_out or d + r2 <= r_in:
                continue
            if d <= r2:
                return False
            theta = math.atan2(dy, dx)
            half = math.asin(min(1.0, r2 / d))
            if ag.has_angular_overlap(theta - half, theta + half, lo, hi):
                return False

        if self.poly_bboxes:
            pts = ag.sector_polygon(center, r_in, r_out, lo, hi)
            qx0 = min(p[0] for p in pts)
            qx1 = max(p[0] for p in pts)
            qy0 = min(p[1] for p in pts)
            qy1 = max(p[1] for p in pts)
            quad: Polygon | None = None
            for i, (bx0, by0, bx1, by1) in enumerate(self.poly_bboxes):
                if qx1 < bx0 or bx1 < qx0 or qy1 < by0 or by1 < qy0:
                    continue
                if quad is None:
                    quad = Polygon(pts)
                if self.polygons[i].relate_pattern(quad, "T********"):
                    return False
        return True

    def on_circle_boundary(self, point: Point, tol: float | None = None) -> bool:
        """Test whether a point rides the inflated boundary of any circular obstacle.

        Args:
            point: Query point coordinate (x, y).
            tol: Distance tolerance in metres. If None, uses default construction delta.

        Returns:
            True if point is within tolerance of a circle boundary; False otherwise.
        """
        if tol is None:
            tol = self.construct_delta + config.GEOM_EPS_M
        return any(
            abs(math.hypot(point[0] - center[0], point[1] - center[1]) - radius) < tol
            for center, radius in self.scenario["circle_obstacles"]
        )

    def is_in_bounds(self, point: Point) -> bool:
        """Test whether a point lies inside the valid operational area.

        Args:
            point: Query coordinate (x, y) in metres.

        Returns:
            True if inside map boundary / safezone; False otherwise.
        """
        if self.safezone_prep is not None:
            return self.safezone_prep.covers(ShapelyPoint(*point))
        if not self.has_explicit_bounds:
            return True
        x, y = point
        return 0 < x < self.bounds_w and 0 < y < self.bounds_h

    def check_fixed_legs(self, goal_wp: Point, target: Point) -> bool:
        """Test whether mandatory terminal seeker run-in W_{n-1} -> T is clear.

        Args:
            goal_wp: Penultimate waypoint position W_{n-1}.
            target: Final target destination T.

        Returns:
            True if terminal run-in straight chord is collision-free; False otherwise.
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
        """Collision-test a chord along a ray, memoizing known clear/blocked spans.

        Args:
            memo: Ray clearance map from ray angle to [min_clear, max_blocked].
            ray: Ray angle in radians.
            dist: Distance from ray origin to end point in metres.
            p1: Segment start point (x, y).
            p2: Segment end point (x, y).

        Returns:
            True if chord is collision-free; False if blocked.
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
