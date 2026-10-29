"""Follow instructions stays on the scene the operator selected."""

from embr_local_sim.mission import instruction_markdown, mission_for_robot, scene_to_open_for_follow
from embr_local_sim.model import RobotParams
from embr_local_sim.physics import Simulator
from embr_local_sim.worlds import prepare_world
from embr_local_sim.yard import yard_world


def test_follow_does_not_replace_the_selected_scene():
    params = RobotParams(morphology="cad")
    for mode in ("engineering", "wildfire", "grove", "forest", "yard"):
        world = prepare_world(mode, 1, "cad")
        assert scene_to_open_for_follow(world.kind) is None
        sim = Simulator(params=params, world=world, seed=1)
        sim.reset()
        sim.start_mission(mission_for_robot(sim.params, sim.world, 4))
        sim.run_for(0.2)
        assert sim.world.kind == world.kind
        assert sim.mission.state != "IDLE"
        if world.kind != "yard":
            assert not sim.mission.config.route


def test_instruction_text_matches_the_scene():
    yard = yard_world("cad")
    engineering = prepare_world("engineering", 1, "cad")
    wildfire = prepare_world("wildfire", 1, "cad")
    yard_text = instruction_markdown(yard)
    assert "Line up with the ramps" in yard_text
    for world in (engineering, wildfire):
        text = instruction_markdown(world)
        assert world.kind in text
        assert "Line up with the ramps" not in text
