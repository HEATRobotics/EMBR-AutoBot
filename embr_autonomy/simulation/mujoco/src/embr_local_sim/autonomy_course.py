"""Autonomy test course: two routes, one with a ramp too steep to climb.

Simulation-only slice of the Autonomy Test Deliverable. A walled field with
a centre divider. The left route is blocked by a 15 degree hump. The right
route has a 3 and a 6 degree hump. An assumed smolder sits past the divider.
The CAD body starts on the centre line and the lidar histogram has to pick
the route it can drive. No hardware, no ROS. Temperatures, mass, and torque
are assumed.
"""

from __future__ import annotations

from embr_local_sim.model import WorldSpec

LENGTH = 22.0
WIDTH = 9.0
MID_Y = 0.5 * WIDTH
WALL_T = 0.10
WALL_H = 0.40

SPAWN = (1.5, MID_Y)
SPOT = (20.5, MID_Y)

DIVIDER_X = (5.5, 15.5)

# Measured on the CAD body with its assumed 0.5 N·m rollers: a 7 degree hump
# crosses, 8 degrees stalls on the slope. Re-measure if torque or mass change.
CLIMB_LIMIT_DEG = 7.5

BODY_HALF_WIDTH = 0.60

RAMP_LENGTH = 1.0
# Longer than the 1.19 m wheelbase, so only one crest is under the body.
RAMP_DECK = 1.4

# Toe x, angle, lane ("left" is +y when facing +x).
RAMPS: tuple[tuple[float, float, str], ...] = (
    (5.8, 15.0, "left"),
    (6.2, 3.0, "right"),
    (10.6, 6.0, "right"),
)


def _box(x: float, y: float, half_x: float, half_y: float) -> dict:
    return {"x": x, "y": y, "size": (half_x, half_y, 0.5 * WALL_H), "role": "wall"}


def course_walls() -> list[dict]:
    t = 0.5 * WALL_T
    x0, x1 = DIVIDER_X
    return [
        _box(0.5 * LENGTH, -t, 0.5 * LENGTH, t),
        _box(0.5 * LENGTH, WIDTH + t, 0.5 * LENGTH, t),
        _box(-t, MID_Y, t, MID_Y),
        _box(LENGTH + t, MID_Y, t, MID_Y),
        _box(0.5 * (x0 + x1), MID_Y, 0.5 * (x1 - x0), t),
    ]


def course_ramps() -> list[dict]:
    lane_w = MID_Y - WALL_T
    out = []
    for x, angle, lane in RAMPS:
        y = MID_Y + (0.5 * MID_Y if lane == "left" else -0.5 * MID_Y)
        out.append(
            {
                "x": x,
                "y": y,
                "width": lane_w,
                "length": RAMP_LENGTH,
                "deck": RAMP_DECK,
                "angle_deg": angle,
                # Stand-in for 3D-lidar slope segmentation: a slope past the
                # climb limit is an obstacle, a gentler one is floor.
                "lidar_visible": angle > CLIMB_LIMIT_DEG,
            }
        )
    return out


def autonomy_course_world(seed: int = 1) -> WorldSpec:
    return WorldSpec(
        kind="hot-room",
        ambient_c=20.0,
        stones=course_walls(),
        heat=[
            {
                "x": SPOT[0],
                "y": SPOT[1],
                "z": 0.05,
                "peak_c": 80.0,
                "sigma": 0.6,
                "kind": "smolder",
                "provenance": "assumed",
                "role": "inspection_target",
            }
        ],
        seed=seed,
        extras={
            "floor_half": LENGTH,
            "spawn": {"x": SPAWN[0], "y": SPAWN[1], "yaw": 0.0},
            "spot": SPOT,
            "skip_pass": True,
            # Half the CAD track width plus a margin. The room default (0.22 m)
            # lets the body scrape a wall it is driving beside.
            "body_half_width": BODY_HALF_WIDTH,
            "aim_at_spot": True,
            "ramps": course_ramps(),
            "layout_basis": "Autonomy test course. Two routes, one too steep. Not a surveyed site.",
        },
    )
