"""The CAD rover a teammate launches.

`embr-local-sim launch` stays the grove track proxy. This session is the
converted Onshape mesh in that same grove, so a teammate can drive it by hand
and work from there. Geometry stays in cad_model.
"""

from __future__ import annotations

from embr_local_sim.body import apply_body
from embr_local_sim.model import RobotParams, WorldSpec

MORPHOLOGY = "cad"
SCENE = "grove"
SEED = 1


def open_session(seed: int | None = None) -> tuple[RobotParams, WorldSpec, str, int]:
    """Build the robot and the grove. Does not open a window."""
    from embr_local_sim.worlds import prepare_world

    use_seed = SEED if seed is None else int(seed)
    params = apply_body(RobotParams(morphology=MORPHOLOGY))
    world = prepare_world(SCENE, use_seed, MORPHOLOGY)
    return params, world, SCENE, use_seed
