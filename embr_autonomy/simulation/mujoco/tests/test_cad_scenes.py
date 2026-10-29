"""CAD scenes are the proxy layouts stretched by the plate length. Proxy scenes stay put."""

from embr_local_sim.cad_model import cad_lidar_overhang_m
from embr_local_sim.mission import mission_for_robot
from embr_local_sim.model import RobotParams, WorldSpec
from embr_local_sim.physics import Simulator
from embr_local_sim.worlds import grove_world, prepare_world, scale_world, world_for_mode


def test_proxy_grove_is_not_scaled():
    world = world_for_mode("grove", seed=2)
    raw = grove_world(seed=2)
    assert world.extras["trees"][2]["x"] == raw.extras["trees"][2]["x"]
    assert world.extras["trees"][2]["x"] < 2.0


def test_cad_grove_is_longer_than_the_robot():
    world = prepare_world("grove", 2, "cad")
    scale = world.extras["layout_scale"]
    assert scale > 4.0
    assert world.extras["trees"][2]["x"] > 4.0
    assert world.heat[0]["peak_c"] == grove_world(2).heat[0]["peak_c"]
    assert world.extras["floor_half"] >= 8.0


def test_every_launch_scene_builds_for_cad():
    for name in (
        "basic",
        "course",
        "grove",
        "forest",
        "engineering",
        "wildfire",
        "ramp",
        "wall",
        "split-friction",
        "stones",
    ):
        world = prepare_world(name, 1, "cad")
        assert world.extras["layout_scale"] > 4.0
        sim = Simulator(params=RobotParams(morphology="cad"), world=world, seed=1)
        sim.reset()
        assert sim.finite()


def test_cad_hotspot_mission_starts_searching():
    world = prepare_world("wildfire", 1, "cad")
    sim = Simulator(params=RobotParams(morphology="cad"), world=world, seed=1)
    sim.reset()
    cfg = mission_for_robot(sim.params, world, 4)
    assert cfg.blocked_range > cad_lidar_overhang_m()
    assert cfg.search_bound["xmax"] > 4.0
    sim.start_mission(cfg)
    sim.run_for(1.2)
    assert sim.finite()
    assert sim.mission.state in ("SEARCH", "APPROACH")
    assert abs(sim.mission.last_tele.velocity) + abs(sim.mission.last_tele.turn) > 0.0


def test_proxy_mission_numbers_stay_small():
    cfg = mission_for_robot(RobotParams(), WorldSpec(kind="flat"), 4)
    assert cfg.blocked_range == 0.14
    assert cfg.timeout_s == 18.0
