"""One CAD roller rises on a rock. A buried ember stays faint until the sampler is close."""

from embr_local_sim.mission import mission_for_robot
from embr_local_sim.model import RobotParams
from embr_local_sim.physics import Simulator
from embr_local_sim.terrain import rough_world
from embr_local_sim.thermal import ThermalField, sources_from_world


def test_buried_ember_is_faint_until_close():
    world = rough_world("cad")
    ember = next(item for item in world.heat if item["kind"] == "ember")
    field = ThermalField(sources=sources_from_world(world.heat))
    near = field.temperature_c((ember["x"], ember["y"], 0.02), 0.0)
    far = field.temperature_c((ember["x"] + 3.0, ember["y"], 0.02), 0.0)
    assert near > 80.0
    assert far < 40.0
    assert near > far + 40.0


def test_cad_drives_across_the_rocks():
    from embr_local_sim.types import DriveCommand

    world = rough_world("cad")
    rock_x = float(world.extras["first_rock_x"])
    sim = Simulator(params=RobotParams(morphology="cad"), world=world, seed=1)
    sim.reset()
    sim.run_for(0.3)
    x0 = sim.snapshot()[0].root_pos[0]
    peak = 0.0
    for _ in range(14):
        sim.run_for(0.45, DriveCommand(0.55, 0.0, 0.0, "test"))
        left = float(sim.data.xpos[int(sim.model.body("front_left").id)][2])
        right = float(sim.data.xpos[int(sim.model.body("front_right").id)][2])
        peak = max(peak, left - right)
    gt = sim.snapshot()[0]
    assert sim.finite()
    assert gt.root_pos[0] - x0 > rock_x - 0.15
    assert peak > 0.012


def test_rough_mission_searches_without_reading_the_map():
    world = rough_world("cad")
    sim = Simulator(params=RobotParams(morphology="cad"), world=world, seed=1)
    sim.reset()
    sim.start_mission(mission_for_robot(sim.params, world, 4))
    x0 = sim.snapshot()[0].root_pos[0]
    sim.run_for(2.5)
    assert sim.finite()
    assert sim.snapshot()[0].root_pos[0] > x0 + 0.15
    assert sim.mission.state in ("SEARCH", "APPROACH")
    assert sim.mission.used_ground_truth is False
