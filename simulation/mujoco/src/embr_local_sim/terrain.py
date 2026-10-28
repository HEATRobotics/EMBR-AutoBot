"""Rough ground for one body. Heights are fractions of that body's wheel radius.

Not a surveyed burn site. Heat is a synthetic field: an ember under a rock
stays faint until the sampler is close, and a flame marker sits beside it.
"""

from __future__ import annotations

from embr_local_sim.model import RobotParams, WorldSpec


def _roller_pose(morphology: str) -> tuple[float, float, float, str]:
    """World-plan x, y, radius, and the body name of the wheel that should rise."""
    if morphology == "cad":
        from embr_local_sim.cad_model import load_cad_geometry

        geo = load_cad_geometry()
        roller = next(item for item in geo["rollers"] if item["joint"] == "front_left_maxon")
        x, y, _z = (float(v) for v in roller["body_xyz_m"])
        return x, y, float(geo["roller_radius_m"]), "front_left"
    params = RobotParams()
    return params.wheelbase_half(), params.track_half(), params.wheel_radius, "wheel_fl"


def rough_world(morphology: str, seed: int = 1) -> WorldSpec:
    x, y, radius, body = _roller_pose(morphology)
    # Ahead of the spawn, not under it. Short enough that a horizontal lidar
    # ray misses them, so the robot drives over them instead of treating them as walls.
    step = max(0.55, 4.0 * radius)
    rise = min(0.045, 0.50 * radius)
    foot = max(0.05, 0.50 * radius)
    half_h = rise / 2.0
    climb = {
        "x": x + step,
        "y": y,
        "size": (foot, foot * 0.75, half_h),
        "role": "left_climb",
    }
    right = {
        "x": x + 2.0 * step,
        "y": -y,
        "size": (foot, foot * 0.75, half_h),
        "role": "right_climb",
    }
    crest = {
        "x": x + 2.6 * step,
        "y": 0.0,
        "size": (foot * 1.3, max(foot, abs(y) * 0.85), half_h * 0.55),
        "role": "crest",
    }
    ember_x = x + 3.6 * step
    cover = {
        "x": ember_x,
        "y": 0.0,
        "size": (0.55 * radius, 0.55 * radius, half_h * 0.7),
        "role": "ember_cover",
    }
    heat = [
        {
            "x": ember_x,
            "y": 0.0,
            "z": 0.0,
            "peak_c": 150.0,
            "sigma": max(0.10, 0.9 * radius),
            "kind": "ember",
            "buried": True,
            "reveal_m": max(0.22, 1.6 * radius),
            "leak": 0.08,
            "provenance": "synthetic_ember_under_rock",
        },
        {
            "x": ember_x + 0.35,
            "y": max(0.45, 3.0 * radius),
            "z": 0.02,
            "peak_c": 340.0,
            "sigma": max(0.08, 0.5 * radius),
            "kind": "flame",
            "provenance": "synthetic_flame_marker",
        },
    ]
    return WorldSpec(
        kind="rough",
        stones=[climb, right, crest, cover],
        heat=heat,
        seed=seed,
        extras={
            "floor_half": 12.0,
            "climb_body": body,
            "first_rock_x": climb["x"],
            "layout_basis": (
                "Rock height is a fraction of this body's wheel radius. "
                "Heat is synthetic. Not a surveyed site and not a fire forecast."
            ),
        },
    )
