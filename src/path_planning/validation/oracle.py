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

# Shortest interior overlap with a polygon that this validator can still tell
# apart from a tangency (metres).
#
# This is NOT a forgiveness of the path — it is the resolution limit of the
# validator's OWN arithmetic, in the same family as TURN_RESERVE_TOL_M below.
# The exact predicate is "positive-length interior overlap", but shapely cannot
# compute "positive" to better than about one ULP of the coordinates: a fillet
# arc TANGENT to a hull edge (the normal case, since inflation is only
# SAFE_MARGIN and hull vertices then sit exactly on the raw polygon) reports an
# overlap of a few nanometres. Measured on the three paths that a zero threshold
# rejected: 8.1e-9 m, 4.4e-9 m and 5.8e-11 m — the last is 0.06 nanometres. At 0
# the oracle rejects flyable missions because of its own rounding (3 of 300
# scenarios, v0).
#
# 1e-6 m is 100x above that noise and still 6 orders below anything operational
# (SAFE_MARGIN is metres; a real crossing runs to kilometres). It was 1e-3 m,
# which WAS a genuine forgiveness — 1000x more than the artefact needs.
POLYGON_TOUCH_TOL_M = config.ORACLE_POLYGON_TOUCH_TOL_M
TURN_RESERVE_TOL_M = config.ORACLE_TURN_RESERVE_TOL_M
_ARC_SAMPLES = config.ORACLE_ARC_SAMPLES
"""Segments the fillet arc at a corner is sampled into for clearance checks."""


# NOTE: intentionally re-implements spatial_utils.point_to_line_distance rather
# than importing it, so this validator stays independent of the planner code it
# validates.
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

    ``poly.intersection(line).length`` is the wrong quantity for this: it
    measures the overlap with the CLOSED polygon (interior UNION boundary), so a
    chord that legitimately runs ALONG a hull edge scores its whole
    edge-following stretch -- kilometres -- even though it never enters the
    interior. Measured on batch_random_test seed 194: 11533.475 m of "overlap",
    of which 11533.475 m is on the boundary and 0.0 m is inside. Subtracting the
    boundary part leaves the penetration itself.

    Shared with the planners on purpose: a planner stricter than its own oracle
    rejects flyable chords, and one more lenient produces paths the oracle then
    fails. There must be exactly one answer to "how far does this chord go
    inside this polygon".

    Args:
        poly: The obstacle polygon.
        line: The chord being measured.

    Returns:
        The interior penetration length in metres; 0.0 for a chord that only
        touches or follows the boundary.
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
        # EXACT: a circle is hit iff the segment comes closer than its radius.
        # No leniency — a check never forgives a real intrusion, otherwise a
        # reported clearance is not the true one. Planner-made chords keep their
        # margin because the geometry is BUILT on radius + CONSTRUCTION_CLEARANCE_M
        # + GEOM_EPS_M. Measured over 120 scenarios: the closest any accepted
        # segment came was 0.112 m OUTSIDE the boundary.
        if _point_to_segment_distance(center, a, b) < radius:
            return False

    # A segment is blocked ONLY when it enters a polygon's INTERIOR (DE-9IM
    # interior/interior overlap, pattern 'T********'). Touching the boundary is
    # allowed: a waypoint may sit on a polygon corner (corners are valid navigation
    # goals, like circle tangent points), and a segment may run ALONG an edge to
    # hug the obstacle boundary. Interior penetration still fails.
    #
    # The interior overlap must have POSITIVE MEASURE. 'T********' also matches a
    # zero-length touch, and that is not a hypothetical: a fillet arc is TANGENT
    # to a polygon edge whenever the pivot is a hull vertex and the outgoing leg
    # runs along a hull edge -- the normal case once inflation is only
    # SAFE_MARGIN, since the hull vertices then lie exactly on the raw polygon.
    # The discretised arc's tangency point lands a float-hair inside and shapely
    # reports a Point of length 0 as an interior intersection (batch_random_test
    # seed 166). Requiring positive length keeps every real crossing -- those are
    # metres to kilometres long -- while ignoring a touch that has no extent.
    #
    # It must be the INTERIOR length, not the closed-polygon one -- see
    # interior_overlap_length. 'T********' can also disagree with the geometry it
    # is derived from: on seed 194 it reports a dimension-1 interior overlap for
    # a chord whose interior overlap measures exactly 0.0 m, because the chord
    # runs along an edge and the two predicates node it differently. Measuring
    # settles it.
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
        path: The waypoints to check.
        circle_obstacles: Circle obstacles as ``(center, radius)``.
        polygon_obstacles: Polygon obstacle rings.

    Returns:
        The verdict, naming the first blocked segment on failure.
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
        path: The waypoints to measure.

    Returns:
        Turn magnitudes in radians, one per interior waypoint, in path order.
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
        path: The waypoints to check.
        alpha_max_rad: Maximum turn angle (rad).

    Returns:
        The verdict, naming the first over-limit waypoint on failure.
    """
    for i, angle in enumerate(turn_angles(path)):
        # Exact; the planner builds to alpha_max - GEOM_EPS_RAD.
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

    The constraint is about the straight FLOWN BETWEEN TWO TURNS, which is not
    the same as a segment between two waypoints: a waypoint whose turn is zero
    is flown straight through and does not split the straight run. The planner
    emits such waypoints routinely -- an arc-hop departure leaves tangentially,
    and an along-ray pivot slide deliberately flies straight through the
    original candidate before turning at the slid pivot -- so consecutive
    segments are merged across them here. Charging each segment the full reserve
    of the turns at both its endpoints instead double-counts the split and
    rejects flyable paths (batch_random_test seed 117: a 1701 m segment between
    two collinear waypoints was charged the 4000 m fillet of the next real turn,
    which in fact reaches back harmlessly into the 4100 m segment before it).

    Args:
        path: The waypoints to check.
        turn_radius: Vehicle turn radius (m).
        l0: Minimum first straight run after takeoff (m).
        dss: Minimum straight run-in before the target (m).

    Returns:
        The verdict, naming the first violating run on failure. A path with no
        segment at all is reported as trivially valid.
    """
    n_seg = len(path) - 1
    if n_seg < 1:
        return ValidationResult(True, "trivial")

    # alpha at each waypoint index; endpoints have no turn before/after them.
    alphas = [0.0, *turn_angles(path), 0.0]
    reserves = [turn_radius * math.tan(a / 2) for a in alphas]

    # Waypoints that actually bend the path delimit the straight runs; the
    # endpoints always do (they carry reserve 0 and bound the first/last run).
    breaks = [0]
    breaks.extend(i for i in range(1, n_seg) if reserves[i] > TURN_RESERVE_TOL_M)
    breaks.append(n_seg)

    for k in range(len(breaks) - 1):
        i, j = breaks[k], breaks[k + 1]
        run = sum(_seg_len(path[m][0], path[m + 1][0]) for m in range(i, j))
        usable = run - reserves[i] - reserves[j]
        span = f"wp[{i}]..wp[{j}]" if j > i + 1 else f"segment {i}"
        # All three are EXACT: the 1 m allowances they used to carry were
        # forgiving real violations. Feasibility comes from the construction
        # side — start corners are built at L0 + GEOM_EPS_M, turns at
        # alpha_max - GEOM_EPS_RAD. Measured worst margins over 120 scenarios of
        # accepted paths: L0 +9.96e-9 m, DSS +413 m, middle +96 m.
        if i == 0:  # first đoản trình: l1 >= L0
            if usable < l0:
                return ValidationResult(False, f"first {span} l={usable:.3f} < L0={l0}")
        elif j == n_seg:  # last đoản trình: ln = l - dss >= 0
            if usable - dss < 0.0:
                return ValidationResult(
                    False, f"last {span} usable l={usable - dss:.3f} < 0"
                )
        elif usable <= 0.0:  # middle: l > 0
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

    Public because the planners check the SAME arc the oracle will check; a
    private copy there would be a second definition of the flown geometry.

    Args:
        w_prev: Waypoint before the corner.
        w: The corner waypoint.
        w_next: Waypoint after the corner.
        turn_radius: Fillet arc radius (m).
        n: Number of arc segments to emit.

    Returns:
        ``n + 1`` points along the arc, or an empty list if the corner is
        collinear and there is no arc to sample.
    """
    u = _unit(w_prev, w)  # incoming direction
    v = _unit(w, w_next)  # outgoing direction
    alpha = abs(_norm(math.atan2(v[1], v[0]) - math.atan2(u[1], u[0])))
    if alpha < 1e-9:
        return []
    tangent = turn_radius * math.tan(alpha / 2)  # tangent length along each leg
    entry = (
        w[0] - u[0] * tangent,
        w[1] - u[1] * tangent,
    )  # tangent point, incoming leg
    s = 1.0 if (u[0] * v[1] - u[1] * v[0]) > 0 else -1.0  # left(+)/right(-) turn
    n_in = (-u[1] * s, u[0] * s)  # inward normal of incoming leg
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
        path: The waypoints to check.
        turn_radius: Fillet arc radius (m).
        circle_obstacles: Circle obstacles as ``(center, radius)``.
        polygon_obstacles: Polygon obstacle rings.

    Returns:
        The verdict, naming the first blocked arc on failure.
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

    Straight segments AND turn arcs must both clear the INFLATED obstacles,
    which are now simply raw + SAFE_MARGIN: the whole flown path honours the
    operator's minimum stand-off.

    Args:
        path: The waypoints to validate.
        circle_obstacles: Inflated circle obstacles as ``(center, radius)``.
        polygon_obstacles: Inflated polygon obstacle rings.
        turn_radius: Vehicle turn radius (m).
        alpha_max_rad: Maximum turn angle (rad).
        l0: Minimum first straight run after takeoff (m).
        dss: Minimum straight run-in before the target (m).
        raw_circle_obstacles: Legacy escape hatch; see below.
        raw_polygon_obstacles: Legacy escape hatch; see below.

    Returns:
        The verdict, naming the first failing check on failure.

    Note:
        The ``raw_*`` parameters are a legacy escape hatch. They existed because
        inflation used to carry a ``R*(1/cos(alpha_max/2)-1)`` turn term and a
        fillet arc was designed to bulge into exactly that band, so arcs were
        validated against the raw obstacle instead. With the turn term gone
        there is no band, and passing raw sets here would let a turn dip inside
        SAFE_MARGIN. Leave them unset unless you are deliberately reproducing
        the old model.
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
