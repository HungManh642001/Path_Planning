"""Bộ chuyển đổi (Codec) giữa đối tượng nghiệp vụ và tin nhắn Protocol Buffers."""

from __future__ import annotations

from service.vtx_service import messages as msg
from service.vtx_service.proto import vtx_path_planning_pb2 as pb


def encode_request(request: msg.PlanRequest) -> bytes:
    """Mã hóa đối tượng PlanRequest thành chuỗi byte Protobuf.

    Args:
        request: Đối tượng yêu cầu lập lịch đường bay.

    Returns:
        bytes: Dữ liệu nhị phân Protobuf đã tuần tự hóa.
    """
    pb_req = pb.PathPlanRequest(
        request_id=request.request_id,
        idl_version=request.idl_version,
        takeoff=pb.Point2D(x=request.start[0], y=request.start[1]),
        takeoff_heading_deg=request.start_heading_deg,
        target=pb.Point2D(x=request.goal[0], y=request.goal[1]),
        target_heading_deg=request.goal_heading_deg,
        is_target_heading_free=request.is_goal_heading_free,
        use_preloaded_map=request.use_preloaded_map,
        limits=pb.VehicleLimits(
            turn_radius_m=request.limits.turn_radius_m,
            l0_m=request.limits.l0_m,
            dss_m=request.limits.dss_m,
            safe_margin_m=request.limits.safe_margin_m,
            alpha_max_deg=request.limits.alpha_max_deg,
        ),
        budget=pb.SearchBudget(time_budget_s=request.budget.time_budget_s),
    )

    for island in request.islands:
        poly = pb.PolygonObstacle(vertices=[pb.Point2D(x=p[0], y=p[1]) for p in island])
        pb_req.islands.append(poly)

    for circle in request.dynamic_obstacles:
        c = pb.CircleObstacle(
            center=pb.Point2D(x=circle.center[0], y=circle.center[1]),
            radius_m=circle.radius_m,
        )
        pb_req.dynamic_obstacles.append(c)

    for safezone in request.safezones:
        sz = pb.PolygonObstacle(vertices=[pb.Point2D(x=p[0], y=p[1]) for p in safezone])
        pb_req.safezones.append(sz)

    return pb_req.SerializeToString()


def decode_request(data: bytes) -> msg.PlanRequest:
    """Giải mã chuỗi byte Protobuf thành đối tượng PlanRequest.

    Args:
        data: Dữ liệu nhị phân Protobuf.

    Returns:
        PlanRequest: Đối tượng yêu cầu lập lịch đã giải mã.
    """
    pb_req = pb.PathPlanRequest()
    pb_req.ParseFromString(data)

    islands = tuple(
        tuple((float(v.x), float(v.y)) for v in poly.vertices)
        for poly in pb_req.islands
    )

    dynamic_obstacles = tuple(
        msg.Circle(
            center=(float(c.center.x), float(c.center.y)),
            radius_m=float(c.radius_m),
        )
        for c in pb_req.dynamic_obstacles
    )

    safezones = tuple(
        tuple((float(v.x), float(v.y)) for v in sz.vertices) for sz in pb_req.safezones
    )

    limits = msg.VehicleLimits(
        turn_radius_m=float(pb_req.limits.turn_radius_m),
        l0_m=float(pb_req.limits.l0_m),
        dss_m=float(pb_req.limits.dss_m),
        safe_margin_m=float(pb_req.limits.safe_margin_m),
        alpha_max_deg=float(pb_req.limits.alpha_max_deg),
    )

    budget = msg.SearchBudget(time_budget_s=float(pb_req.budget.time_budget_s))

    return msg.PlanRequest(
        request_id=bytes(pb_req.request_id),
        idl_version=int(pb_req.idl_version),
        start=(float(pb_req.takeoff.x), float(pb_req.takeoff.y)),
        start_heading_deg=float(pb_req.takeoff_heading_deg),
        goal=(float(pb_req.target.x), float(pb_req.target.y)),
        goal_heading_deg=float(pb_req.target_heading_deg),
        is_goal_heading_free=bool(pb_req.is_target_heading_free),
        islands=islands,
        dynamic_obstacles=dynamic_obstacles,
        safezones=safezones,
        use_preloaded_map=bool(pb_req.use_preloaded_map),
        limits=limits,
        budget=budget,
    )


def encode_reply(reply: msg.PlanReply) -> bytes:
    """Mã hóa đối tượng PlanReply thành chuỗi byte Protobuf.

    Args:
        reply: Đối tượng kết quả lập lịch đường bay.

    Returns:
        bytes: Dữ liệu nhị phân Protobuf đã tuần tự hóa.
    """
    pb_reply = pb.PathPlanReply(
        request_id=reply.request_id,
        idl_version=reply.idl_version,
        status=pb.PlanStatus.Value(reply.status.name)
        if hasattr(pb.PlanStatus, reply.status.name)
        else reply.status.value,
        detail=reply.detail,
        path_length_m=reply.path_length_m,
        plan_wall_time_s=reply.plan_wall_time_s,
        applied_time_budget_s=reply.applied_time_budget_s,
        stats=pb.SearchStats(
            iterations=reply.stats.iterations,
            open_set_size=reply.stats.open_set_size,
            is_search_failed=reply.stats.is_search_failed,
            is_budget_bound=reply.stats.is_budget_bound,
        ),
        planner_version=reply.planner_version,
        config_hash=reply.config_hash,
    )

    for wp in reply.waypoints:
        pb_reply.waypoints.append(
            pb.Waypoint(
                position=pb.Point2D(x=wp.position[0], y=wp.position[1]),
                heading_deg=wp.heading_deg,
            )
        )

    return pb_reply.SerializeToString()


def decode_reply(data: bytes) -> msg.PlanReply:
    """Giải mã chuỗi byte Protobuf thành đối tượng PlanReply.

    Args:
        data: Dữ liệu nhị phân Protobuf.

    Returns:
        PlanReply: Đối tượng kết quả lập lịch đã giải mã.
    """
    pb_reply = pb.PathPlanReply()
    pb_reply.ParseFromString(data)

    waypoints = tuple(
        msg.Waypoint(
            position=(float(wp.position.x), float(wp.position.y)),
            heading_deg=float(wp.heading_deg),
        )
        for wp in pb_reply.waypoints
    )

    stats = msg.SearchStats(
        iterations=int(pb_reply.stats.iterations),
        open_set_size=int(pb_reply.stats.open_set_size),
        is_search_failed=bool(pb_reply.stats.is_search_failed),
        is_budget_bound=bool(pb_reply.stats.is_budget_bound),
    )

    return msg.PlanReply(
        request_id=bytes(pb_reply.request_id),
        idl_version=int(pb_reply.idl_version),
        status=msg.PlanStatus(int(pb_reply.status)),
        detail=str(pb_reply.detail),
        waypoints=waypoints,
        path_length_m=float(pb_reply.path_length_m),
        plan_wall_time_s=float(pb_reply.plan_wall_time_s),
        applied_time_budget_s=float(pb_reply.applied_time_budget_s),
        stats=stats,
        planner_version=str(pb_reply.planner_version),
        config_hash=str(pb_reply.config_hash),
    )
