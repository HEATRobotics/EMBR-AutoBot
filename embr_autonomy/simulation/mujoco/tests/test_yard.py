"""Burn-scar yard is sized for the CAD body. The proxy grove stays in proxy metres."""

from embr_local_sim.mission import mission_for_robot
from embr_local_sim.model import RobotParams
from embr_local_sim.physics import Simulator
from embr_local_sim.thermal import ThermalField, sources_from_world
from embr_local_sim.worlds import grove_world
from embr_local_sim.yard import yard_world


def test_cad_trunks_are_stand_sized_and_the_gap_clears_the_tracks():
    world = yard_world("cad")
    trees = world.extras["trees"]
    assert all(0.10 <= tree["radius"] <= 0.25 for tree in trees)
    assert all(tree["height"] >= 4.0 for tree in trees)
    gap = min(abs(tree["y"]) - tree["radius"] for tree in trees)
    assert gap > 0.70
    assert world.extras["flames"][0]["height"] > 0.8
    assert grove_world(2).extras["trees"][0]["x"] < 2.0


def test_yard_robot_sees_a_trunk_and_the_smolder_is_hot():
    world = yard_world("cad")
    sim = Simulator(params=RobotParams(morphology="cad"), world=world, seed=1)
    sim.reset()
    sim.run_for(0.4)
    gt, obs = sim.snapshot()
    assert sim.finite()
    assert gt.root_pos[2] > 0.05
    assert min(obs.lidar_ranges) < 6.0
    smolder = next(item for item in world.heat if item["kind"] == "smolder")
    field = ThermalField(sources=sources_from_world(world.heat))
    assert field.temperature_c((smolder["x"], smolder["y"], 0.1), 0.0) > 100.0


def test_yard_mission_logs_the_smolder_and_comes_back():
    world = yard_world("cad")
    sim = Simulator(params=RobotParams(morphology="cad"), world=world, seed=1)
    sim.reset()
    sim.start_mission(mission_for_robot(sim.params, world, 4))
    furthest = 0.0
    highest = 0.0
    widest = 0.0
    for _ in range(40):
        sim.run_for(2.0)
        gt = sim.snapshot()[0]
        furthest = max(furthest, gt.root_pos[0])
        highest = max(highest, gt.root_pos[2])
        widest = max(widest, abs(gt.root_pos[1]))
        if sim.mission.state in ("COMPLETE", "FAULT"):
            break
    gt = sim.snapshot()[0]
    assert sim.finite()
    assert furthest > 1.6
    assert highest > 0.15
    assert widest < 0.25
    assert sim.mission.state == "COMPLETE"
    assert sim.mission.reason == "returned"
    assert len(sim.mission.hotspots) == 1
    assert sim.mission.hotspots[0].measured_c >= 60.0
    assert sim.mission.hotspots[0].estimated_x > 1.4
    assert gt.root_pos[0] < 0.8
