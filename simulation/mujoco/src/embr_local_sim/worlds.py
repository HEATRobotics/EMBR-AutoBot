"""Three connected environment modes: engineering, forest, wildfire.

Rigid geoms only. Flexible branches, soil, ash, and combustion are NOT modeled.
Default `kind=forest` clutter matches the Milestone A YAML so those tests stay
valid. Viewer/detailed modes add trees, canopy (visual-only), and fire markers.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from embr_local_sim.model import WorldSpec


# Known engineering fixtures (metres). These are scenario-authored, not CAD.
ENGINEERING_WALL_X = 0.50
ENGINEERING_WALL_THICKNESS = 0.04
ENGINEERING_RAMP_DEG = 8.0
ENGINEERING_STEP_HEIGHT = 0.020
ENGINEERING_STEP_X = 0.28
ENGINEERING_STEP_Y = -0.28
ENGINEERING_LOG = {
    "x": 0.42,
    "y": 0.28,
    "length": 0.24,
    "radius": 0.018,
    "yaw": 1.20,
    "hinged": False,
    "secured": True,
}


def default_forest_clutter() -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    """Identical to the original `_default_forest` (Milestone A forest.yaml)."""
    stones = [
        {"x": 0.32, "y": -0.11, "size": (0.035, 0.04, 0.028), "movable": False},
        {"x": 0.24, "y": 0.10, "size": (0.018, 0.018, 0.012), "movable": True, "mass": 0.04},
        {"x": 0.48, "y": 0.16, "size": (0.05, 0.04, 0.04), "movable": False},
    ]
    logs = [
        {"x": 0.58, "y": -0.14, "length": 0.22, "radius": 0.016, "yaw": 0.5, "hinged": True},
    ]
    heat = [
        {
            "x": 0.92,
            "y": 0.04,
            "z": 0.02,
            "peak_c": 160.0,
            "sigma": 0.16,
            "kind": "smolder",
            "provenance": "assumed",
        }
    ]
    return stones, logs, heat


def _tree(x: float, y: float, radius: float, height: float) -> dict[str, Any]:
    return {"x": x, "y": y, "radius": radius, "height": height, "collision": True}


def _canopy(x: float, y: float, z: float, radius: float) -> dict[str, Any]:
    return {"x": x, "y": y, "z": z, "radius": radius, "collision": False}


def detailed_forest_props(seed: int, density: float = 1.0) -> dict[str, list[dict[str, Any]]]:
    """Reproducible trees/roots/mounds. Corridor near +X stays mostly open."""
    rng = np.random.default_rng(int(seed))
    n_trees = max(6, min(14, int(round(10 * density))))
    # Fixed anchors so the scene looks like a stand, plus seeded jitter.
    anchors = [
        (-0.55, 0.42),
        (-0.72, -0.38),
        (0.88, 0.58),
        (1.18, -0.52),
        (0.38, 0.78),
        (-0.22, 0.90),
        (1.45, 0.22),
        (0.62, -0.82),
        (-0.95, 0.08),
        (1.62, 0.68),
        (-0.40, -0.72),
        (0.10, -1.02),
        (1.05, 0.95),
        (-1.05, 0.55),
    ]
    trees: list[dict[str, Any]] = []
    canopy: list[dict[str, Any]] = []
    roots: list[dict[str, Any]] = []
    for i, (ax, ay) in enumerate(anchors[:n_trees]):
        jx = float(rng.normal(0.0, 0.03))
        jy = float(rng.normal(0.0, 0.03))
        x, y = ax + jx, ay + jy
        radius = float(np.clip(0.045 + 0.015 * rng.normal(), 0.032, 0.07))
        height = float(np.clip(0.42 + 0.08 * rng.normal(), 0.32, 0.58))
        trees.append(_tree(x, y, radius, height))
        canopy.append(_canopy(x, y, height + 0.02, radius * 3.2))
        if i % 2 == 0:
            roots.append(
                {
                    "x": x + 0.06,
                    "y": y - 0.04,
                    "size": (0.04, 0.018, 0.012),
                    "yaw": float(rng.uniform(0, math.pi)),
                }
            )
    mounds = [
        {"x": 0.70, "y": 0.32, "size": (0.08, 0.06, 0.012)},
        {"x": 0.18, "y": -0.22, "size": (0.05, 0.07, 0.010)},
        {"x": 1.05, "y": -0.18, "size": (0.07, 0.05, 0.014)},
    ]
    branches = [
        {"x": 0.75, "y": 0.22, "length": 0.16, "radius": 0.008, "yaw": -0.6, "z": 0.04},
        {"x": 0.50, "y": -0.32, "length": 0.14, "radius": 0.007, "yaw": 0.9, "z": 0.035},
    ]
    extra_logs = [
        {"x": 1.10, "y": 0.10, "length": 0.28, "radius": 0.020, "yaw": -0.3, "hinged": False},
        {"x": -0.15, "y": 0.35, "length": 0.18, "radius": 0.014, "yaw": 1.1, "hinged": False},
    ]
    extra_rocks = [
        {"x": 0.15, "y": 0.22, "size": (0.03, 0.025, 0.02), "movable": False},
        {"x": 0.95, "y": -0.28, "size": (0.04, 0.03, 0.025), "movable": False},
        {"x": 0.05, "y": -0.18, "size": (0.02, 0.02, 0.015), "movable": True, "mass": 0.03},
    ]
    return {
        "trees": trees,
        "canopy": canopy,
        "roots": roots,
        "mounds": mounds,
        "branches": branches,
        "logs": extra_logs,
        "stones": extra_rocks,
    }


def wildfire_props(seed: int) -> dict[str, Any]:
    forest = detailed_forest_props(seed, density=1.0)
    # Smolder inspection target ahead (sensor-visible). Flame offset to +Y so
    # a straight approach can still reach the smolder if the robot steers.
    heat = [
        {
            "x": 0.92,
            "y": 0.04,
            "z": 0.02,
            "peak_c": 160.0,
            "sigma": 0.16,
            "kind": "smolder",
            "provenance": "assumed",
            "role": "inspection_target",
        },
        {
            "x": 0.55,
            "y": 0.48,
            "z": 0.04,
            "peak_c": 380.0,
            "sigma": 0.10,
            "kind": "flame",
            "provenance": "assumed",
            "role": "exclusion",
            "exclusion_r": 0.16,
        },
        {
            "x": 1.25,
            "y": -0.40,
            "z": 0.03,
            "peak_c": 90.0,
            "sigma": 0.12,
            "kind": "ember",
            "provenance": "assumed",
            "role": "ember_field",
        },
    ]
    flames = [
        {
            "x": 0.55,
            "y": 0.48,
            "height": 0.16,
            "radius": 0.035,
            "kind": "flame",
            "exclusion_r": 0.16,
            "provenance": "scenario_model_not_wildfire_forecast",
        }
    ]
    smoke = [
        {"x": 0.55, "y": 0.48, "z": 0.22, "radius": 0.09, "opacity": 0.28},
        {"x": 0.62, "y": 0.52, "z": 0.30, "radius": 0.11, "opacity": 0.20},
        {"x": 1.25, "y": -0.40, "z": 0.18, "radius": 0.07, "opacity": 0.18},
    ]
    return {
        **forest,
        "heat": heat,
        "flames": flames,
        "smoke": smoke,
        "return_xy": (0.0, 0.0),
        "mission_bound": {"xmin": -0.4, "xmax": 1.6, "ymin": -1.1, "ymax": 1.1},
        "flame_rule": "static_scenario_markers; not a spreading-fire forecast",
    }


def engineering_world() -> WorldSpec:
    return WorldSpec(
        kind="engineering",
        wall_x=ENGINEERING_WALL_X,
        wall_thickness=ENGINEERING_WALL_THICKNESS,
        ramp_angle_deg=ENGINEERING_RAMP_DEG,
        ramp_length=0.70,
        ramp_width=0.28,
        ramp_start_x=0.22,
        logs=[dict(ENGINEERING_LOG)],
        stones=[
            {
                "x": 0.70,
                "y": -0.10,
                "size": (0.025, 0.025, 0.018),
                "movable": False,
                "optional_loose": True,
            }
        ],
        extras={
            "metre_marks": True,
            "ramp_y": 0.38,
            "step": {
                "x": ENGINEERING_STEP_X,
                "y": ENGINEERING_STEP_Y,
                "height": ENGINEERING_STEP_HEIGHT,
                "size": (0.08, 0.10, 0.5 * ENGINEERING_STEP_HEIGHT),
            },
            "known_wall_face_x": ENGINEERING_WALL_X - 0.5 * ENGINEERING_WALL_THICKNESS,
            "mode": "engineering",
        },
    )


def forest_world(seed: int = 2, *, detailed: bool = True) -> WorldSpec:
    stones, logs, heat = default_forest_clutter()
    extras: dict[str, Any] = {"mode": "forest", "seed": seed}
    world = WorldSpec(kind="forest", stones=list(stones), logs=list(logs), heat=list(heat), extras=extras)
    if detailed:
        props = detailed_forest_props(seed)
        world.stones = list(world.stones) + list(props["stones"])
        world.logs = list(world.logs) + list(props["logs"])
        world.extras["trees"] = props["trees"]
        world.extras["canopy"] = props["canopy"]
        world.extras["roots"] = props["roots"]
        world.extras["mounds"] = props["mounds"]
        world.extras["branches"] = props["branches"]
        world.extras["detailed_forest"] = True
        world.extras["visual_canopy_no_collision"] = True
        world.extras["rigid_terrain_only"] = True
    return world


def wildfire_world(seed: int = 7) -> WorldSpec:
    world = forest_world(seed=seed, detailed=True)
    world.kind = "wildfire"
    props = wildfire_props(seed)
    world.heat = list(props["heat"])
    world.extras["trees"] = props["trees"]
    world.extras["canopy"] = props["canopy"]
    world.extras["roots"] = props["roots"]
    world.extras["mounds"] = props["mounds"]
    world.extras["branches"] = props["branches"]
    world.extras["flames"] = props["flames"]
    world.extras["smoke"] = props["smoke"]
    world.extras["return_xy"] = props["return_xy"]
    world.extras["mission_bound"] = props["mission_bound"]
    world.extras["flame_rule"] = props["flame_rule"]
    world.extras["mode"] = "wildfire"
    world.extras["smoke_does_not_affect_lidar"] = True
    world.extras["active_flame_exclusion_provisional"] = True
    return world


def course_case(name: str = "showcase") -> WorldSpec:
    """Rigid obstacle cases. Sizes are metres and multiples of roller radius R=0.03 m.

    Chassis clearance on the proxy is about 0.016 m. A stop or rollover is a
    valid outcome. Nothing here bends, breaks, or becomes soil.
    """
    r = 0.03
    case = name.replace("-", "_")
    extras: dict[str, Any] = {"mode": "course", "case": case, "roller_radius_m": r, "clearance_m": 0.016}
    logs: list[dict[str, Any]] = []
    stones: list[dict[str, Any]] = []
    if case in ("showcase", "branch"):
        logs.append(
            {
                "x": 0.34,
                "y": 0.0,
                "length": 0.22,
                "radius": 0.010,
                "yaw": math.pi / 2.0,
                "secured": True,
                "note": "0.33 R, height 0.020 m vs clearance 0.016 m",
            }
        )
    if case in ("showcase", "log"):
        logs.append(
            {
                "x": 0.72 if case == "showcase" else 0.36,
                "y": 0.0,
                "length": 0.26,
                "radius": 0.028,
                "yaw": math.pi / 2.0,
                "secured": True,
                "note": "0.93 R secured log; stall is a valid result",
            }
        )
    if case in ("showcase", "oblique"):
        logs.append(
            {
                "x": 1.08 if case == "showcase" else 0.38,
                "y": 0.02,
                "length": 0.28,
                "radius": 0.012,
                "yaw": 0.70,
                "secured": True,
                "note": "oblique branch, 0.40 R",
            }
        )
    if case in ("showcase", "roll"):
        stones.append(
            {
                "x": 0.42 if case != "showcase" else 1.55,
                "y": 0.055,
                "size": (0.16, 0.035, 0.004),
                "movable": False,
                "note": "one-sided board, top 0.008 m (0.27 R), +y side only",
            }
        )
    if case in ("showcase", "slope"):
        extras["include_ramp"] = True
        extras["cross_slope"] = {"deg": 10.0, "x": 1.15 if case == "showcase" else 0.55, "y": -0.32, "length": 0.45, "width": 0.26}
    if case in ("showcase", "loose"):
        logs.append(
            {
                "x": 1.85 if case == "showcase" else 0.36,
                "y": 0.0,
                "length": 0.18,
                "radius": 0.010,
                "yaw": math.pi / 2.0,
                "free": True,
                "mass": 0.04,
                "note": "loose rigid branch, mass 0.04 kg ASSUMED",
            }
        )
    world = WorldSpec(
        kind="course",
        ramp_angle_deg=8.0,
        ramp_length=0.55,
        ramp_width=0.26,
        ramp_start_x=1.55 if case == "showcase" else 0.40,
        logs=logs,
        stones=stones,
        extras=extras,
    )
    return world


def grove_world(seed: int = 2) -> WorldSpec:
    """Larger trunks and a clearing. Shrubs block the robot. Canopy does not.

    Trunk radii are about 3–4× the 0.03 m roller. Heights are assumed scenery,
    not a surveyed stand. Fire markers are not combustion.
    """
    rng = np.random.default_rng(int(seed))
    anchors = [
        (-0.85, 0.70),
        (-1.05, -0.55),
        (1.35, 0.85),
        (1.70, -0.75),
        (0.15, 1.15),
        (-0.35, -1.20),
        (2.05, 0.25),
        (0.95, -1.15),
    ]
    trees = []
    canopy = []
    for ax, ay in anchors:
        jx = float(rng.normal(0.0, 0.04))
        jy = float(rng.normal(0.0, 0.04))
        radius = float(np.clip(0.10 + 0.02 * rng.normal(), 0.08, 0.14))
        height = float(np.clip(1.15 + 0.12 * rng.normal(), 0.95, 1.55))
        trees.append(_tree(ax + jx, ay + jy, radius, height))
        canopy.append(_canopy(ax + jx, ay + jy, height + 0.05, radius * 2.4))
    shrubs = [
        {"x": -0.55, "y": 0.25, "z": 0.07, "radius": 0.06},
        {"x": 0.85, "y": 0.55, "z": 0.06, "radius": 0.05},
        {"x": 1.15, "y": -0.45, "z": 0.08, "radius": 0.07},
    ]
    stones, logs, heat = default_forest_clutter()
    logs = list(logs) + [
        {"x": 0.95, "y": 0.05, "length": 0.42, "radius": 0.03, "yaw": 0.4, "hinged": False},
    ]
    stones = list(stones) + [
        {"x": 0.48, "y": 0.22, "size": (0.05, 0.04, 0.03), "movable": False},
        {"x": 1.25, "y": -0.18, "size": (0.04, 0.035, 0.025), "movable": False},
    ]
    heat = [
        {
            "x": 1.55,
            "y": 0.15,
            "z": 0.04,
            "peak_c": 140.0,
            "sigma": 0.22,
            "kind": "smolder",
            "provenance": "assumed",
        },
        {
            "x": 1.85,
            "y": -0.55,
            "z": 0.05,
            "peak_c": 320.0,
            "sigma": 0.12,
            "kind": "flame",
            "provenance": "assumed",
            "role": "exclusion",
        },
    ]
    return WorldSpec(
        kind="grove",
        stones=stones,
        logs=logs,
        heat=heat,
        seed=int(seed),
        extras={
            "mode": "grove",
            "seed": int(seed),
            "trees": trees,
            "canopy": canopy,
            "shrubs": shrubs,
            "mounds": [
                {"x": 0.62, "y": -0.28, "size": (0.10, 0.08, 0.015)},
                {"x": 1.05, "y": 0.32, "size": (0.12, 0.07, 0.018)},
            ],
            "flames": [{"x": 1.85, "y": -0.55, "height": 0.22, "radius": 0.04}],
            "smoke": [
                {"x": 1.85, "y": -0.55, "z": 0.32, "radius": 0.12, "opacity": 0.22},
                {"x": 1.55, "y": 0.15, "z": 0.18, "radius": 0.08, "opacity": 0.16},
            ],
            "visual_canopy_no_collision": True,
            "shrubs_visual_only": False,
            "fire_is_not_combustion": True,
        },
    )


def world_for_mode(mode: str, seed: int = 2) -> WorldSpec:
    key = str(mode).strip().lower().replace("_", "-")
    if key in {"engineering", "eng", "test"}:
        return engineering_world()
    if key in {"wildfire", "wildfire-mission", "mission"}:
        return wildfire_world(seed=seed)
    if key in {"basic", "flat", "default"}:
        return WorldSpec(kind="flat", seed=seed)
    if key in {"forest", "explore"}:
        return forest_world(seed=seed, detailed=True)
    if key in {"grove", "rich-forest", "rich_forest"}:
        return grove_world(seed=seed)
    if key in {"course", "obstacle", "obstacles"}:
        return course_case("showcase")
    if key.startswith("course-"):
        return course_case(key.removeprefix("course-"))
    # Legacy kinds: flat/wall/ramp/bumps/split_friction/forest-yaml
    return WorldSpec(kind=key if key else "flat")


def materialize_world(world: WorldSpec) -> WorldSpec:
    """Fill default clutter for kinds that need it. Mutates `world` lists."""
    if world.kind == "forest" and not world.stones and not world.logs:
        stones, logs, heat = default_forest_clutter()
        world.stones = stones
        world.logs = logs
        if not world.heat:
            world.heat = heat
    if world.kind == "engineering":
        eng = engineering_world()
        if not world.logs:
            world.logs = list(eng.logs)
        if not world.stones:
            world.stones = list(eng.stones)
        world.wall_x = world.wall_x or eng.wall_x
        merged = dict(eng.extras)
        merged.update(world.extras or {})
        world.extras = merged
    if world.kind == "wildfire":
        seeded = int(world.extras.get("seed", 7)) if world.extras else 7
        filled = wildfire_world(seed=seeded)
        if not world.stones:
            world.stones = list(filled.stones)
        if not world.logs:
            world.logs = list(filled.logs)
        if not world.heat:
            world.heat = list(filled.heat)
        merged = dict(filled.extras)
        merged.update(world.extras or {})
        world.extras = merged
    return world
