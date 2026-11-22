"""Room 3: the histogram takes a door and reads the hidden spot.

The chart weights are unchanged. The straight line from the corner hits a trunk.
"""

import math

from embr_local_sim.body import apply_body
from embr_local_sim.hot_room import begin_hotspot_mission
from embr_local_sim.model import RobotParams
from embr_local_sim.physics import Simulator
from embr_local_sim.planners.vfh import VfhPlanner
from embr_local_sim.worlds import prepare_world


def _sight_is_blocked(world) -> bool:
    spawn = world.extras["spawn"]
    spot = world.extras["spot"]
    x0, y0 = float(spawn["x"]), float(spawn["y"])
    x1, y1 = float(spot[0]), float(spot[1])
    trunks = [(float(t["x"]), float(t["y"]), float(t["radius"])) for t in world.extras["trees"]]
    for i in range(1, 80):
        t = i / 80.0
        x = x0 + (x1 - x0) * t
        y = y0 + (y1 - y0) * t
        if any(math.hypot(x - tx, y - ty) < radius for tx, ty, radius in trunks):
            return True
    return False


def test_the_histogram_finds_the_hidden_spot():
    params = apply_body(RobotParams(morphology="cad"))
    world = prepare_world("heat-track", 1, "cad")
    assert world.kind == "heat-track"
    assert "layout_scale" not in (world.extras or {})
    assert len(world.extras["trees"]) >= 40
    assert world.extras.get("skip_pass") is True
    assert world.extras.get("aim_at_spot") is True
    assert len(world.extras["waypoints"]) == 3
    assert world.extras["track_ramps"]["angle_deg"] < 2.0
    assert _sight_is_blocked(world)
    sim = Simulator(params=params, world=world, seed=1)
    sim.reset()
    begin_hotspot_mission(sim, histogram=True)
    assert isinstance(sim.mission.planner, VfhPlanner)
    assert sim.mission.config.goal_x is not None
    assert sim.mission.config.skip_pass is True
    hx, hy = (float(v) for v in world.extras["spot"])
    while sim.data.time < 200.0 and sim.mission.state not in ("SAMPLE", "COMPLETE", "FAULT"):
        sim.step_control_tick()
        assert sim.finite()
    gt, obs = sim.snapshot()
    dist = math.hypot(hx - gt.root_pos[0], hy - gt.root_pos[1])
    assert sim.mission.state in ("SAMPLE", "COMPLETE")
    assert sim.mission.reason in ("probe_ready", "sampled")
    assert obs.probe_c >= 70.0
    assert dist < 1.2
