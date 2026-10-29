"""A burn-scar yard sized for this body.

Trunk diameters sit in the 20–50 cm band used for procedural forest stands
(heights here are the lower part of a 4–8 m trunk, so the robot is not a toy
under a canopy). The driving gap is wider than the track width.

Fire is a sensor field plus a visible flame and smoke. MuJoCo has no
combustion, and a CFD fire model is too slow to run beside the robot.
Peaks are assumed inspection targets, not a forecast.
"""

from __future__ import annotations

from embr_local_sim.model import WorldSpec

# CAD idler radius. Other bodies get the same layout in wheel radii.
_CAD_ROLLER_RADIUS_M = 0.089


def _metres(seed: int) -> WorldSpec:
    _ = seed
    # (x, y, radius, height). Radius is a trunk, 0.10–0.25 m.
    trunks = (
        (-1.4, 1.7, 0.16, 4.6),
        (-0.6, -1.8, 0.22, 5.8),
        (2.4, 1.9, 0.14, 4.2),
        (3.6, -2.0, 0.20, 5.4),
        (5.4, 1.7, 0.18, 5.0),
        (6.4, -1.8, 0.24, 6.0),
        (1.2, -2.6, 0.15, 4.4),
        (4.4, 2.7, 0.12, 4.0),
    )
    trees = [
        {"x": x, "y": y, "radius": radius, "height": height, "collision": True}
        for x, y, radius, height in trunks
    ]
    canopy = [
        {"x": x, "y": y, "z": height + 0.4, "radius": max(0.9, radius * 6.0), "collision": False}
        for x, y, radius, height in trunks
    ]
    shrubs = [
        {"x": 2.8, "y": 1.55, "z": 0.35, "radius": 0.28},
        {"x": 4.6, "y": -1.55, "z": 0.40, "radius": 0.30},
    ]
    # A full-width step catches the plate. These lanes sit under the rollers only.
    stones = []
    logs = [
        {"x": 4.4, "y": 1.85, "length": 1.1, "radius": 0.07, "yaw": 0.3, "hinged": False},
    ]
    heat = [
        {
            "x": 2.45,
            "y": 0.0,
            "z": 0.28,
            "peak_c": 180.0,
            "sigma": 0.55,
            "kind": "smolder",
            "provenance": "assumed_residual_heat",
            "role": "inspection_target",
        },
        {
            "x": 4.3,
            "y": 2.15,
            "z": 0.15,
            "peak_c": 420.0,
            "sigma": 0.45,
            "kind": "flame",
            "provenance": "assumed_flame_marker",
            "role": "exclusion",
        },
        {
            "x": 4.6,
            "y": -2.2,
            "z": 0.02,
            "peak_c": 30.0,
            "sigma": 0.25,
            "kind": "ember",
            "buried": True,
            "reveal_m": 0.55,
            "leak": 0.08,
            "provenance": "synthetic_ember_under_rock",
            "role": "buried",
        },
    ]
    return WorldSpec(
        kind="yard",
        stones=stones,
        logs=logs,
        heat=heat,
        seed=seed,
        extras={
            "floor_half": 16.0,
            "track_ramps": {
                "x": 1.15,
                "length": 2.60,
                "angle_deg": 5.0,
                "lanes": [
                    {"y": 0.438, "width": 0.32},
                    {"y": -0.438, "width": 0.32},
                ],
            },
            "trees": trees,
            "canopy": canopy,
            "shrubs": shrubs,
            "flames": [
                {"x": 4.3, "y": 2.15, "height": 1.15, "radius": 0.22, "kind": "flame"},
            ],
            "smoke": [
                {"x": 4.3, "y": 2.15, "z": 1.6, "radius": 0.55, "opacity": 0.28},
                {"x": 4.5, "y": 2.35, "z": 2.1, "radius": 0.40, "opacity": 0.18},
            ],
            "return_xy": (0.0, 0.0),
            "mission_bound": {"xmin": -1.5, "xmax": 8.0, "ymin": -3.2, "ymax": 3.2},
            "route": [
                {"x": 1.0, "y": 0.0, "action": "drive", "name": "Line up with the ramps"},
                {"x": 2.55, "y": 0.0, "action": "sample", "name": "Sample the smolder on the ramp"},
                {"x": 0.15, "y": 0.0, "action": "return", "name": "Back down the ramps to the start"},
            ],
            "goal": "Follow the three instructions: line up, sample on the ramp, turn around and return.",
            "layout_basis": (
                "Yard authored in metres for the CAD roller. Trunk diameters are "
                "in the 20–50 cm stand band. Heat is a sensor field, not combustion."
            ),
        },
    )


def yard_world(morphology: str, seed: int = 1) -> WorldSpec:
    world = _metres(seed)
    if morphology == "cad":
        return world
    from embr_local_sim.model import RobotParams
    from embr_local_sim.worlds import scale_world

    factor = RobotParams().wheel_radius / _CAD_ROLLER_RADIUS_M
    return scale_world(world, factor)
