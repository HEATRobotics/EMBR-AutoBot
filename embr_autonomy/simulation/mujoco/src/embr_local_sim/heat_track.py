"""Corner to corner, with the hot spot hidden.

The body starts in one corner facing along the near wall, not at the ball.
Two hedges cross the straight line, and each hedge has two doors. Both
doors lead on. A pocket on the far side closes. A shallow ramp and a pair
of low stones sit on the way out, under the lidar, so those are for rolling
over. The tall trunks are for going around. The histogram weights are
unchanged. This world does not use the two-room box detour.
"""

from __future__ import annotations

from embr_local_sim.model import WorldSpec
from embr_local_sim.terrain import _roller_pose

SPAWN = (2.0, 2.2)
SPOT = (13.4, 13.2)
# Faces +x. The ball is up and to the left, behind the first hedge.
YAW = 0.0


def _hedge(x0: float, x1: float, y: float, radius: float = 0.18, step: float = 0.30) -> list[dict]:
    """Trunks close enough that a lidar ray cannot slip between them."""
    trees = []
    x = x0
    while x <= x1 + 1e-9:
        trees.append({"x": round(x, 3), "y": y, "radius": radius, "height": 3.6})
        x += step
    return trees


def _wheel_half(morphology: str) -> float:
    _axle_x, wheel_y, _radius, _body = _roller_pose(morphology)
    half = abs(wheel_y)
    if morphology in ("cad", "urdf"):
        from embr_local_sim.cad_model import load_cad_geometry

        roller = next(item for item in load_cad_geometry()["rollers"] if item["joint"] == "front_right_maxon")
        half = abs(float(roller["body_xyz_m"][1]))
    return half


def heat_track_world(morphology: str, seed: int = 1) -> WorldSpec:
    half = _wheel_half(morphology)
    # The line from the corner to the spot crosses y=6 near x=5.9 and y=10
    # near x=10.1. Both of those spans are solid. The doors sit off that line.
    trees: list[dict] = []
    # Short plugs sit on the straight line. The doors start beside each plug
    # so the body turns into a gap instead of grinding along a wall.
    trees += _hedge(0.6, 1.4, 6.0)
    trees += _hedge(5.35, 6.35, 6.0)
    trees += _hedge(11.3, 15.3, 6.0)
    trees += _hedge(0.6, 3.6, 10.0)
    trees += _hedge(9.55, 10.55, 10.0)
    trees += _hedge(14.4, 15.3, 10.0)
    trees += [
        {"x": 14.8, "y": 14.4, "radius": 0.2, "height": 4.4},
        {"x": 12.2, "y": 14.7, "radius": 0.18, "height": 4.0},
        {"x": 1.3, "y": 8.2, "radius": 0.2, "height": 4.0},
        {"x": 1.4, "y": 9.2, "radius": 0.16, "height": 3.4},
        {"x": 14.8, "y": 7.2, "radius": 0.17, "height": 3.8},
        {"x": 2.2, "y": 12.6, "radius": 0.2, "height": 4.6},
        {"x": 3.2, "y": 14.0, "radius": 0.18, "height": 4.0},
        {"x": 1.5, "y": 13.4, "radius": 0.16, "height": 3.6},
        {"x": 12.4, "y": 3.2, "radius": 0.17, "height": 4.0},
        {"x": 13.4, "y": 4.6, "radius": 0.16, "height": 3.5},
        {"x": 14.6, "y": 8.6, "radius": 0.16, "height": 3.4},
    ]
    wall_h = 0.55
    wall_t = 0.14
    span = 7.8
    stones = [
        {"x": span, "y": 0.25, "size": (span, wall_t, wall_h)},
        {"x": span, "y": 15.5, "size": (span, wall_t, wall_h)},
        {"x": 0.25, "y": span, "size": (wall_t, span, wall_h)},
        {"x": 15.5, "y": span, "size": (wall_t, span, wall_h)},
        # The middle cell has no door on the line to the spot.
        {"x": 12.6, "y": 7.6, "size": (0.4, 0.35, 0.5), "role": "block"},
        {"x": 14.4, "y": 7.8, "size": (0.4, 0.4, 0.48), "role": "block"},
    ]
    # Both idlers, right at the start, while the body is still facing +x.
    lanes = [
        {"y": SPAWN[1] - half, "width": 0.30},
        {"y": SPAWN[1] + half, "width": 0.30},
    ]
    # Tops stay under the lidar, so the chart drives over them.
    bump = 0.024
    logs = [
        {"x": 4.3, "y": SPAWN[1] - half, "length": 0.16, "radius": bump, "yaw": 1.57, "free": False, "role": "turn_rock"},
        {"x": 4.3, "y": SPAWN[1] + half, "length": 0.16, "radius": bump, "yaw": 1.57, "free": False, "role": "turn_rock"},
        {"x": 12.8, "y": 3.4, "length": 0.22, "radius": 0.03, "yaw": 0.4, "free": False, "role": "turn_rock"},
        {"x": 14.0, "y": 3.8, "length": 0.2, "radius": 0.028, "yaw": 1.0, "free": False, "role": "turn_rock"},
        {"x": 12.4, "y": 8.6, "length": 0.2, "radius": 0.028, "yaw": 0.8, "free": False, "role": "turn_rock"},
        {"x": 9.4, "y": 12.4, "length": 0.2, "radius": 0.026, "yaw": 0.3, "free": False, "role": "turn_rock"},
    ]
    shrubs = [
        {"x": 13.2, "y": 2.8, "z": 0.16, "radius": 0.16},
        {"x": 14.2, "y": 4.2, "z": 0.14, "radius": 0.14},
        {"x": 1.2, "y": 7.4, "z": 0.15, "radius": 0.14},
        {"x": 2.4, "y": 13.6, "z": 0.16, "radius": 0.16},
        {"x": 8.2, "y": 13.8, "z": 0.14, "radius": 0.14},
        {"x": 14.0, "y": 8.4, "z": 0.15, "radius": 0.13},
    ]
    canopy = [
        {"x": SPOT[0], "y": SPOT[1], "z": 2.4, "radius": 0.9},
        {"x": SPOT[0] - 0.4, "y": SPOT[1] + 0.6, "z": 2.8, "radius": 0.7},
    ]
    return WorldSpec(
        kind="heat-track",
        ambient_c=20.0,
        stones=stones,
        logs=logs,
        heat=[
            {
                "x": SPOT[0],
                "y": SPOT[1],
                "z": 0.08,
                "peak_c": 80.0,
                "sigma": 1.05,
                "kind": "smolder",
                "provenance": "assumed",
                "role": "inspection_target",
            }
        ],
        seed=seed,
        extras={
            "floor_half": 18.0,
            "spawn": {"x": SPAWN[0], "y": SPAWN[1], "yaw": YAW},
            "spot": SPOT,
            "skip_pass": True,
            "aim_at_spot": True,
            # East door in each row, then the spot. The west doors are open too.
            # The straight line still hits both plugs.
            "waypoints": ((8.82, 6.6), (12.5, 10.6), SPOT),
            "track_ramps": {
                "x": 2.7,
                "length": 1.5,
                "angle_deg": 1.2,
                "lanes": lanes,
            },
            "trees": trees,
            "shrubs": shrubs,
            "canopy": canopy,
            "layout_basis": (
                "One corner to the other. The straight line hits both hedges. "
                "Each hedge has two doors. One pocket closes. Not a surveyed burn."
            ),
        },
    )
