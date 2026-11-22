"""The team command starts both rooms on the CAD body with the histogram.

The chart weights are not changed here. The model slot stays empty.
"""

from __future__ import annotations

import math

from embr_local_sim.body import apply_body
from embr_local_sim.hot_room import begin_hotspot_mission
from embr_local_sim.model import RobotParams
from embr_local_sim.physics import Simulator
from embr_local_sim.planners.vfh import VfhPlanner
from embr_local_sim.hot_room import DEFAULT_PULL, chase_camera
from embr_local_sim.two_rooms import PORT_1, PORT_2, PORT_3, ROOM_1, ROOM_2, ROOM_3, launch_command
from embr_local_sim.worlds import prepare_world


def test_the_wide_camera_sits_farther_back_than_the_tight_one():
    tight, _near = chase_camera(0.0, 0.0, 0.0, 0.0, 4.7, distance_scale=1.0)
    wide, _far = chase_camera(0.0, 0.0, 0.0, 0.0, 4.7, distance_scale=DEFAULT_PULL)
    assert wide[2] > tight[2] * 2.0
    assert abs(wide[0]) > abs(tight[0])


def test_the_team_command_names_both_rooms_and_the_histogram():
    first = launch_command(ROOM_1, PORT_1)
    second = launch_command(ROOM_2, PORT_2)
    third = launch_command(ROOM_3, PORT_3)
    assert "--scene" in first and ROOM_1 in first
    assert "--scene" in second and ROOM_2 in second
    assert "--scene" in third and ROOM_3 in third
    assert "--histogram" in first and "--histogram" in second and "--histogram" in third
    assert str(PORT_1) in first and str(PORT_2) in second and str(PORT_3) in third
    assert len({PORT_1, PORT_2, PORT_3}) == 3
    assert "43150" not in first and "43150" not in second and "43150" not in third


def _connected(scene: str) -> Simulator:
    params = apply_body(RobotParams(morphology="cad"))
    world = prepare_world(scene, 1, params.morphology)
    sim = Simulator(params=params, world=world, seed=1)
    sim.reset()
    begin_hotspot_mission(sim, histogram=True)
    return sim


def test_both_rooms_connect_the_cad_body_to_the_histogram():
    room_1 = _connected(ROOM_1)
    room_2 = _connected(ROOM_2)

    for sim in (room_1, room_2):
        assert sim.params.morphology == "cad"
        assert sim.steer_planner is None
        assert isinstance(sim.mission.planner, VfhPlanner)
        assert sim.params.thermal_n_bins == 9
        assert sim.mission.state == "IDLE"
        for _ in range(8):
            sim.step_control_tick()
        assert sim.finite()
        assert sim.mission.state == "SEARCH"
        assert sim.mission.reason == "vfh"

    assert not (room_1.world.extras or {}).get("blocks")
    blocks = (room_2.world.extras or {}).get("blocks") or []
    assert len(blocks) == 2
    heat_1 = room_1.world.heat[0]
    heat_2 = room_2.world.heat[0]
    assert (heat_1["x"], heat_1["y"]) != (heat_2["x"], heat_2["y"])


def test_the_second_room_goes_around_the_box_instead_of_staying_on_it():
    """The body has to reach the ball, and it cannot sit on a box."""
    sim = _connected(ROOM_2)
    heat = sim.world.heat[0]
    hx, hy = float(heat["x"]), float(heat["y"])
    mark = 0.0
    last = sim.snapshot()[0].root_pos
    while sim.data.time < 140.0 and sim.mission.state not in ("SAMPLE", "COMPLETE", "FAULT"):
        sim.step_control_tick()
        gt, _ = sim.snapshot()
        dist = math.hypot(hx - gt.root_pos[0], hy - gt.root_pos[1])
        if gt.sim_time - mark >= 15.0 and dist > 4.0:
            moved = math.hypot(gt.root_pos[0] - last[0], gt.root_pos[1] - last[1])
            assert moved > 0.6
            mark = float(gt.sim_time)
            last = gt.root_pos
    gt, _ = sim.snapshot()
    dist = math.hypot(hx - gt.root_pos[0], hy - gt.root_pos[1])
    assert sim.mission.state in ("SAMPLE", "COMPLETE")
    assert dist < 2.0
