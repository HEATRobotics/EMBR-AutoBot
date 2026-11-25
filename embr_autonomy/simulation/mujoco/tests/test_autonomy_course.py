"""Autonomy test course: two routes, the histogram takes the one it can climb."""

from __future__ import annotations

import math

import pytest

from embr_local_sim.autonomy_course import (
    CLIMB_LIMIT_DEG,
    DIVIDER_X,
    MID_Y,
    RAMP_DECK,
    RAMP_LENGTH,
    RAMPS,
    SPAWN,
    SPOT,
)
from embr_local_sim.body import apply_body
from embr_local_sim.hot_room import begin_hotspot_mission
from embr_local_sim.model import RobotParams, WorldSpec, lane_ramp_geoms
from embr_local_sim.physics import Simulator
from embr_local_sim.planners.vfh import VfhPlanner
from embr_local_sim.worlds import prepare_world

HUMP_RUN = 2.0 * RAMP_LENGTH + RAMP_DECK


def _cad() -> RobotParams:
    return apply_body(RobotParams(morphology="cad"))


def test_the_course_has_two_routes_and_one_is_too_steep() -> None:
    world = prepare_world("autonomy-course", 1, "cad")
    assert len(world.stones) == 5
    assert len(world.heat) == 1
    steep = [angle for _x, angle, lane in RAMPS if lane == "left"]
    gentle = [angle for _x, angle, lane in RAMPS if lane == "right"]
    assert steep and min(steep) > CLIMB_LIMIT_DEG
    assert gentle and max(gentle) < CLIMB_LIMIT_DEG
    humps = [g for g in lane_ramp_geoms(world.extras) if g["name"].startswith("hump_")]
    assert len(humps) == 3 * len(RAMPS)
    # Floor-like slopes are hidden from the flat fan. The steep one is an obstacle.
    groups = {g["group"] for g in humps if g["pos"][1] > MID_Y}
    assert groups == {0}
    groups = {g["group"] for g in humps if g["pos"][1] < MID_Y}
    assert groups == {2}


def _drive_straight_at(angle_deg: float) -> float:
    toe = 1.5
    world = WorldSpec(
        kind="hot-room",
        extras={
            "floor_half": 20.0,
            "spawn": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "ramps": [
                {"x": toe, "y": 0.0, "width": 3.0, "length": RAMP_LENGTH, "deck": RAMP_DECK, "angle_deg": angle_deg}
            ],
        },
    )
    sim = Simulator(params=_cad(), world=world, seed=1)
    sim.reset()
    while sim.data.time < 20.0:
        sim.drive(1.0, 0.0)
        sim.step_control_tick()
    return float(sim.snapshot()[0].root_pos[0]) - (toe + HUMP_RUN)


@pytest.mark.parametrize("lane", ["left", "right"])
def test_only_the_steep_route_is_physically_blocked(lane: str) -> None:
    angle = max(a for _x, a, side in RAMPS if side == lane)
    past_the_hump = _drive_straight_at(angle)
    if lane == "left":
        assert past_the_hump < -HUMP_RUN + RAMP_LENGTH, f"{angle} degrees should stall on the slope"
    else:
        assert past_the_hump > 0.3, f"{angle} degrees should be climbable"


def test_the_lidar_sits_on_top_and_sees_around_the_body() -> None:
    sim = Simulator(params=_cad(), world=prepare_world("autonomy-course", 1, "cad"), seed=1)
    sim.reset()
    for _ in range(50):
        sim.step_control_tick()
    gt, obs = sim.snapshot()
    site = sim.data.site_xpos[sim.model.site("lidar").id]
    assert site[0] > gt.root_pos[0] + 0.2
    assert site[2] - gt.root_pos[2] > 0.1
    span = max(obs.lidar_angles) - min(obs.lidar_angles)
    assert span > math.radians(350)
    blocked = [a for a, own in zip(obs.lidar_angles, obs.lidar_occluded) if own]
    assert blocked, "the track tops should shadow some beams"
    assert all(abs(a) > math.radians(60) for a in blocked)
    # The planner gets the whole ring, minus the beams the body blocks.
    ranges, angles = sim._planner_scan(obs)
    assert len(ranges) == len(obs.lidar_ranges) - len(blocked)
    assert min(angles) < math.radians(-170) and max(angles) > math.radians(170)


@pytest.mark.parametrize("morphology", ["cad", "urdf"])
def test_the_histogram_takes_the_climbable_route_to_the_hotspot(morphology: str) -> None:
    params = apply_body(RobotParams(morphology=morphology))
    sim = Simulator(params=params, world=prepare_world("autonomy-course", 1, morphology), seed=1)
    sim.reset()
    begin_hotspot_mission(sim, histogram=True)
    assert isinstance(sim.mission.planner, VfhPlanner)
    gt, _ = sim.snapshot()
    assert math.hypot(gt.root_pos[0] - SPAWN[0], gt.root_pos[1] - SPAWN[1]) < 0.1

    gentle = [(x, a) for x, a, lane in RAMPS if lane == "right"]
    rest_z = float(gt.root_pos[2])
    peak_z_on = {x: 0.0 for x, _a in gentle}
    entered_steep_lane = False
    while sim.data.time < 200.0 and sim.mission.state not in ("SAMPLE", "COMPLETE", "FAULT"):
        sim.step_control_tick()
        assert sim.finite()
        gt, _ = sim.snapshot()
        px, py, pz = (float(v) for v in gt.root_pos)
        rest_z = min(rest_z, pz)
        if DIVIDER_X[0] < px < DIVIDER_X[1] and py > MID_Y:
            entered_steep_lane = True
        for toe, _a in gentle:
            if toe <= px <= toe + HUMP_RUN:
                peak_z_on[toe] = max(peak_z_on[toe], pz)

    gt, obs = sim.snapshot()
    dist = math.hypot(SPOT[0] - gt.root_pos[0], SPOT[1] - gt.root_pos[1])
    assert sim.mission.state in ("SAMPLE", "COMPLETE"), sim.mission.reason
    assert obs.probe_c >= 65.0
    assert dist < 1.2
    assert not entered_steep_lane
    for toe, angle in gentle:
        rise = RAMP_LENGTH * math.sin(math.radians(angle))
        assert peak_z_on[toe] - rest_z > 0.7 * rise, f"did not climb the {angle} degree ramp"
