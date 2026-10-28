"""Which robot is in the sim.

Mission, scenes, and the viewer ask this module. They do not import a
morphology file. A new body adds a `BodySpec` here and keeps its geometry
in its own module.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

KNOWN_BODIES = ("wheels", "tracks", "articulated", "cad")

_PROXY_CAMERA = (0.42, -0.48, 0.28)
_PROXY_LOOK = (0.08, 0.0, 0.03)


@dataclass(frozen=True)
class BodySpec:
    """What the rest of the sim needs from a body. Not a mass or a joint list."""

    name: str
    fidelity: str
    layout_scale: float
    lidar_overhang_m: float
    view_span_m: float
    view_min_m: float
    camera_position: tuple[float, float, float]
    camera_look_at: tuple[float, float, float]
    grid_m: float


_WHEELS = BodySpec(
    name="wheels",
    fidelity="Four-wheel skid-steer proxy — not EMBR tracks.",
    layout_scale=1.0,
    lidar_overhang_m=0.0,
    view_span_m=1.4,
    view_min_m=-0.2,
    camera_position=_PROXY_CAMERA,
    camera_look_at=_PROXY_LOOK,
    grid_m=4.0,
)


def resolve_body(morphology: str) -> BodySpec:
    name = morphology or "wheels"
    if name == "cad":
        from embr_local_sim.cad_model import cad_body_spec

        return cad_body_spec()
    if name == "tracks":
        from embr_local_sim.track_approx import FIDELITY

        return replace(_WHEELS, name="tracks", fidelity=FIDELITY)
    if name == "articulated":
        return replace(
            _WHEELS,
            name="articulated",
            fidelity="Four-pod roller approximation — not a belt. Hardware validation pending.",
        )
    return replace(_WHEELS, name=name)


def apply_body(params):
    """Fill geometry for bodies that carry their own record. Proxies pass through."""
    if getattr(params, "morphology", "") == "cad":
        from embr_local_sim.cad_model import apply_cad_params

        return apply_cad_params(params)
    return params


def pod_drive(morphology: str):
    """Joint helpers for podded bodies. None means the four named wheel hinges."""
    if morphology == "tracks":
        from embr_local_sim.track_approx import N_ROLLERS, all_drive_joints, drive_siblings

        return drive_siblings, N_ROLLERS, all_drive_joints
    if morphology == "articulated":
        from embr_local_sim.articulated import ROLLER_OFFSETS_M, all_drive_joints, drive_siblings

        return drive_siblings, len(ROLLER_OFFSETS_M), all_drive_joints
    return None
