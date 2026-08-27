"""Tests for the auto-fit plot framing (render.visualizer._content_extents).

The 'content' view frames the axes to the mission CORE (path + start/goal),
then adds only what is NEAR it: obstacles whose bbox intersects, and safezone
vertices that fall within, the core expanded by a gate. Far-outlier obstacles
and giant enclosing safezones are excluded from the frame (still drawn, clipped).
"""

import math

from path_planning import planner as astar
from path_planning.render.visualizer import _content_extents, _plot_extents
from path_planning.scenario import preprocessing as prep


def _twin_safezone_scenario():
    return {
        "start": (449446.4583, 1188023.5911),
        "start_heading": 0.0,
        "goal": (521214.3377, 1164069.7764),
        "goal_heading": None,
        "obstacles": [],
        "islands": [],
        "dynamic_obstacles": [],
        "safezones": [
            [  # small operating corridor containing start & goal
                (444644.39, 1193895.31),
                (458768.76, 1205669.96),
                (479719.30, 1205774.77),
                (534653.91, 1170269.46),
                (534786.43, 1155488.26),
                (459316.41, 1155175.60),
                (444716.15, 1176742.81),
                (444172.21, 1187300.65),
            ],
            [  # giant enclosing safezone
                (26454.20, 1663527.05),
                (1669424.07, 1717332.52),
                (1773367.51, 21327.40),
                (0.0, 0.0),
            ],
        ],
    }


def test_content_extents_excludes_giant_safezone():
    scenario = _twin_safezone_scenario()
    pre = prep.prepare_scenario(
        scenario, turn_radius=10000, l0=4000, alpha_max_rad=math.pi / 2, dss=20000
    )
    result = astar.plan_trajectory(pre)

    (xmin, xmax), (ymin, ymax) = _content_extents(scenario, pre, result)

    # Frame is the small corridor, not the 1.77M-wide enclosing safezone.
    assert xmax < 600_000, f"frame leaked into the giant safezone: xmax={xmax}"
    assert ymin > 1_000_000

    # Start and goal both lie inside the returned box.
    for p in (scenario["start"], scenario["goal"]):
        assert xmin <= p[0] <= xmax and ymin <= p[1] <= ymax

    # Legacy 'map' view, by contrast, spans the giant safezone.
    (lxmin, lxmax), _ = _plot_extents(scenario)
    assert lxmax > 1_500_000


def test_content_extents_excludes_far_obstacles_and_giant_safezone():
    # Real-mission shape (coords at y~1.15e6): a NEAR obstacle just off the
    # route, a FAR SAM obstacle ~250 km away, and a giant enclosing safezone.
    # The auto-fit frame must keep the flight + near obstacle prominent and
    # exclude both the far obstacle and the giant safezone.
    scenario = {
        "start": (465395.95, 1151760.61),
        "start_heading": -1.4049896483812718,
        "goal": (518605.94, 1083146.17),
        "goal_heading": None,
        "obstacles": [
            {
                "type": "circle",
                "center": (474079.39, 1180878.67),
                "radius": 2000.0,
            },  # near
            {
                "type": "circle",
                "center": (598847.37, 1398210.84),
                "radius": 4000.0,
            },  # far (~250 km)
        ],
        "islands": [],
        "dynamic_obstacles": [],
        "safezones": [
            [
                (26454.2, 1663527.1),
                (1669424.1, 1717332.5),
                (1773367.5, 21327.4),
                (0.0, 0.0),
            ]
        ],  # giant enclosing quad
    }
    pre = prep.prepare_scenario(
        scenario, turn_radius=10000, l0=4000, alpha_max_rad=math.pi / 2, dss=20000
    )
    result = astar.plan_trajectory(pre)
    assert result["is_success"]

    (xmin, xmax), (ymin, ymax) = _content_extents(scenario, pre, result)

    # Start and goal inside the frame.
    for p in (scenario["start"], scenario["goal"]):
        assert xmin <= p[0] <= xmax and ymin <= p[1] <= ymax
    # Near obstacle (y~1.18e6) is included; far obstacle (y~1.40e6) is NOT.
    assert ymax > 1_180_000, "near obstacle should be inside the frame"
    assert ymax < 1_300_000, f"far obstacle leaked into the frame: ymax={ymax}"
    # Giant safezone (x up to 1.77e6) is excluded.
    assert xmax < 700_000, f"giant safezone leaked into the frame: xmax={xmax}"


def test_content_extents_falls_back_without_content():
    # No preprocessed / result / obstacles / safezones => config-map fallback.
    (xmin, xmax), (ymin, ymax) = _content_extents({}, None, None)
    assert xmax > xmin and ymax > ymin
