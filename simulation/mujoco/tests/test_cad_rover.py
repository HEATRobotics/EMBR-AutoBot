"""The teammate session is the CAD mesh in the yard, not the grove proxy."""

from embr_local_sim.cad_model import cad_mesh_path
from embr_local_sim.cad_rover import MORPHOLOGY, SCENE, SEED, open_session
from embr_local_sim.physics import Simulator
from embr_local_sim.types import DriveCommand


def test_cad_command_opens_the_yard_mesh():
    params, world, scene, seed = open_session()
    assert (MORPHOLOGY, SCENE, SEED) == ("cad", "yard", 1)
    assert params.morphology == "cad"
    assert scene == "yard" and seed == 1
    assert world.kind == "yard"
    mesh = cad_mesh_path()
    assert mesh.is_file() and mesh.stat().st_size > 100_000

    sim = Simulator(params=params, world=world, seed=seed)
    sim.reset()
    sim.run_for(0.35)
    x0 = sim.snapshot()[0].root_pos[0]
    moved = sim.run_for(0.6, DriveCommand(0.25, 0.0, 0.0, "test"))
    assert sim.finite()
    assert moved.root_pos[2] > 0.05
    assert moved.root_pos[0] - x0 > 0.02
