"""Two small rooms with one assumed hot spot on the floor.

Same walls in both. The second spot is in a different place. Nothing
in the histogram is changed between them. The temperature is assumed.
The chassis mass and the motor strength are still the proxy's assumed
numbers, not a measurement of the Maxon motors on the parts list.
"""

from __future__ import annotations

import math

from embr_local_sim.model import WorldSpec

# The flat fan only sees 2 m. Walls closer than that fill the scan and the
# histogram treats the straight path as a scrape, then backs up. These walls
# sit past that range at the start, so the body can drive. The inner face is
# half a wall-thickness inside the number below.
ROOM_HALF_X = 3.20
ROOM_HALF_Y = 3.20
WALL_THICK = 0.03
WALL_HEIGHT = 0.22


def _walls() -> list[dict[str, float | tuple[float, float, float]]]:
    hx = ROOM_HALF_X
    hy = ROOM_HALF_Y
    t = 0.5 * WALL_THICK
    h = 0.5 * WALL_HEIGHT
    return [
        {"x": hx, "y": 0.0, "size": (t, hy, h)},
        {"x": -hx, "y": 0.0, "size": (t, hy, h)},
        {"x": 0.0, "y": hy, "size": (hx, t, h)},
        {"x": 0.0, "y": -hy, "size": (hx, t, h)},
    ]


# Back-left corner, inset so the body is not in the wall. The fan looks
# into the room, so the near walls sit behind it.
CORNER_X = -2.40
CORNER_Y = -2.40


def _room(spot_x: float, spot_y: float, blocks: list | None = None) -> WorldSpec:
    stones = _walls()
    if blocks:
        stones = stones + list(blocks)
    yaw = math.atan2(spot_y - CORNER_Y, spot_x - CORNER_X)
    return WorldSpec(
        kind="hot-room",
        ambient_c=20.0,
        stones=stones,
        heat=[
            {
                "x": spot_x,
                "y": spot_y,
                "z": 0.0,
                "peak_c": 80.0,
                "sigma": 0.18,
                "kind": "smolder",
                "provenance": "assumed",
            }
        ],
        extras={
            "floor_half": 3.0,
            "blocks": list(blocks or []),
            "spawn": {"x": CORNER_X, "y": CORNER_Y, "yaw": yaw},
        },
    )


def room_straight() -> WorldSpec:
    """Hot spot straight ahead. This is the room the histogram is shown first."""
    return _room(1.10, 0.0)


def _path_point(distance_m: float, side_m: float = 0.0) -> tuple[float, float]:
    """A point on the drive from the corner to the moved spot.

    side_m is metres to the left of that drive. These are proxy metres.
    The CAD room stretches them with the rest of the layout.
    """
    dx = 0.80 - CORNER_X
    dy = 0.50 - CORNER_Y
    length = math.hypot(dx, dy)
    ux, uy = dx / length, dy / length
    left_x, left_y = -uy, ux
    return (
        CORNER_X + ux * distance_m + left_x * side_m,
        CORNER_Y + uy * distance_m + left_y * side_m,
    )


def room_blocks() -> list[dict]:
    """Two boxes on the drive from the corner to the moved spot.

    A straight drive hits the first box. The open gap is to the left of
    that drive. The second box closes the other side. Histogram weights
    are unchanged.
    """
    x1, y1 = _path_point(1.70, -0.22)
    x2, y2 = _path_point(1.95, -0.85)
    return [
        {"x": x1, "y": y1, "size": (0.24, 0.24, 0.18), "role": "block"},
        {"x": x2, "y": y2, "size": (0.18, 0.18, 0.16), "role": "block"},
    ]


def room_offset() -> WorldSpec:
    """Same walls. The spot is moved left, and two boxes block the other ways."""
    return _room(0.80, 0.50, room_blocks())


# 1.0 sits just behind the nose. 4.0 sees the ball, and the blue boxes in room 2.
DEFAULT_PULL = 4.0


def chase_camera(
    x: float,
    y: float,
    z: float,
    yaw: float,
    scale: float = 1.0,
    distance_scale: float = 1.0,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Behind the body, looking along the nose.

    `distance_scale` is the scroll-wheel zoom. Bigger sits farther back
    so the two rooms do not look like the same close-up of the rover.
    """
    span = math.sqrt(max(float(scale), 1.0))
    pull = max(0.45, float(distance_scale))
    look_d = 1.8 * span * min(pull, 2.5)
    back_d = 3.0 * span * pull
    height = 1.9 * span * pull
    c = math.cos(yaw)
    s = math.sin(yaw)
    look = (x + look_d * c, y + look_d * s, z + 0.30 * span)
    pos = (x - back_d * c, y - back_d * s, z + height)
    return pos, look


def marker_for(src: dict) -> tuple[float, float, float, float]:
    """Where the viewer draws the spot. The radius is the blob itself.

    A 4 cm ball at floor level disappears. The drawn ball is at least the
    Gaussian width, and its center sits above the floor.
    """
    sigma = float(src.get("sigma", 0.18))
    radius = max(sigma, 0.12)
    return (
        float(src.get("x", 0.0)),
        float(src.get("y", 0.0)),
        radius * 0.45,
        radius,
    )


def begin_hotspot_mission(sim, histogram: bool = False) -> None:
    """What the L4 button starts.

    The default driver is the steer rule. `histogram=True` leaves the slot
    empty so the mission builds the vector-field histogram instead.
    """
    from embr_local_sim.mission import mission_for_robot

    if histogram:
        sim.steer_planner = None
    else:
        from embr_local_sim.planners.learned import LearnedSteerPlanner

        sim.steer_planner = LearnedSteerPlanner()
    cfg = mission_for_robot(sim.params, sim.world, 4)
    cfg.hold_after_sample = True
    # Full stick on a clear line. The chart still slows down while it turns.
    cfg.search_velocity = 1.0
    cfg.approach_velocity = 1.0
    scale = float((sim.world.extras or {}).get("layout_scale", 1.0))
    # The corner is several body-lengths from the spot. The small default
    # clock runs out before a body can cross the room.
    cfg.timeout_s = max(float(cfg.timeout_s), 70.0 * max(scale, 1.0))
    extras = getattr(sim.world, "extras", None) or {}
    cfg.skip_pass = bool(extras.get("skip_pass"))
    if extras.get("body_half_width") is not None:
        cfg.body_half_width = float(extras["body_half_width"])
    # Rooms 1 and 2 do not set this. Room 3 names the corner, once, in odometry.
    if extras.get("aim_at_spot") and extras.get("spot") and extras.get("spawn"):
        from embr_local_sim.inspection import world_point_to_estimated

        spawn = extras["spawn"]
        spot = extras["spot"]
        cfg.goal_x, cfg.goal_y = world_point_to_estimated(
            float(spawn["x"]),
            float(spawn["y"]),
            float(spawn["yaw"]),
            float(spot[0]),
            float(spot[1]),
        )
        points = []
        for point in extras.get("waypoints") or []:
            points.append(
                world_point_to_estimated(
                    float(spawn["x"]),
                    float(spawn["y"]),
                    float(spawn["yaw"]),
                    float(point[0]),
                    float(point[1]),
                )
            )
        if points:
            cfg.goal_points = tuple(points)
            cfg.goal_x, cfg.goal_y = points[0]
        # This body is the assumed CAD plate, slower than a nameplate drive.
        # The hedges are many body-lengths away.
        cfg.timeout_s = max(float(cfg.timeout_s), 220.0)
    sim.start_mission(cfg)
