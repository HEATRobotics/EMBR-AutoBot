"""Follow instructions turns toward an open lidar gap instead of stopping on the nose."""

import math

import mujoco
import pytest

from embr_local_sim.autonomy import way_around
from embr_local_sim.mission import MissionConfig, MissionView
from embr_local_sim.model import RobotParams, WorldSpec, _box_diag_inertia, build_mjcf
from embr_local_sim.physics import Simulator
from embr_local_sim.worlds import scale_world


def _scan(ranges: list[float]) -> MissionView:
    n = len(ranges)
    angles = [(-math.pi / 3.0) + (2.0 * math.pi / 3.0) * i / (n - 1) for i in range(n)]
    return MissionView(
        sim_time=1.0,
        stamp_wall=1.0,
        odom_x=0.0,
        odom_y=0.0,
        odom_yaw=0.0,
        lidar_ranges=ranges,
        lidar_angles=angles,
        lidar_stamp_sim=1.0,
        lidar_max_range=8.0,
        thermal_c=[20.0],
        thermal_angles=[0.0],
        probe_c=20.0,
        probe_stamp_sim=1.0,
        probe_frame="probe",
    )


def test_vfh_turns_toward_the_open_side_nearest_the_goal():
    from embr_local_sim.autonomy import vfh_steer

    # Angles run right to left. The goal is left, and only the left side is open.
    ranges = [0.35] * 14 + [4.0] * 7
    tele, why = vfh_steer(_scan(ranges), 0.6, wall=1.0, cruise=0.5, blocked_range=0.16, half_width=0.08)
    assert why == "vfh"
    assert tele.velocity > 0.0
    assert tele.turn < 0.0


def test_vfh_backs_up_when_the_fan_is_closed():
    from embr_local_sim.autonomy import vfh_steer

    tele, why = vfh_steer(_scan([0.2] * 21), 0.0, wall=1.0, cruise=0.5, blocked_range=0.16)
    assert why == "backing_up"
    assert tele.velocity < 0.0


def test_open_scan_does_not_invent_a_turn():
    view = _scan([4.0] * 21)
    assert way_around(view, 0.0, wall=1.0) is None


def test_a_side_gap_is_chosen_over_the_hit_ahead():
    # Angles run from right to left. The long returns are on the right.
    ranges = [4.0] * 7 + [0.45] * 7 + [0.50] * 7
    tele, why = way_around(_scan(ranges), 0.0, wall=1.0)
    assert why in ("aim_at_gap", "gap_steer")
    assert tele.turn > 0.0


def test_a_wall_that_fills_the_fan_is_not_a_gap():
    view = _scan([0.30] * 21)
    assert way_around(view, 0.0, wall=1.0) is None


def test_a_heavy_stone_against_one_wheel_makes_the_rover_turn():
    world = WorldSpec(
        kind="flat",
        stones=[{"x": 0.20, "y": 0.045, "size": (0.025, 0.025, 0.02), "movable": True, "mass": 3.0}],
    )
    sim = Simulator(params=RobotParams(), world=world, seed=1)
    sim.reset()
    sim.run_for(0.2)
    sim.start_mission(MissionConfig(level=4, detect_c=400.0, sample_c=400.0, timeout_s=30.0))
    turned = False
    for _ in range(int(5.0 * sim.params.control_hz)):
        gt = sim.step_control_tick()
        if abs(gt.yaw) > 0.25 or abs(sim.mission.last_tele.turn) > 0.4:
            turned = True
            break
    assert sim.finite()
    assert sim.mission.state != "FAULT"
    assert turned


def test_search_goes_around_a_trunk_instead_of_stopping():
    world = WorldSpec(
        kind="flat",
        extras={"trees": [{"x": 0.55, "y": 0.0, "radius": 0.05, "height": 0.6}]},
    )
    sim = Simulator(params=RobotParams(), world=world, seed=1)
    sim.reset()
    sim.run_for(0.3)
    sim.start_mission(MissionConfig(level=4, detect_c=400.0, sample_c=400.0, timeout_s=30.0))
    reasons: set[str] = set()
    for _ in range(int(8.0 * sim.params.control_hz)):
        sim.step_control_tick()
        reasons.add(str(sim.mission.reason))
    gt, _obs = sim.snapshot()
    assert sim.finite()
    assert sim.mission.state != "FAULT"
    assert reasons & {"aim_at_gap", "gap_steer", "vfh"}
    assert gt.root_pos[0] > 0.55
    assert abs(gt.root_pos[1]) > 0.06


def test_scaled_stone_keeps_its_density_and_stops_the_push():
    authored = {
        "x": 0.36,
        "y": 0.0,
        "size": (0.02, 0.02, 0.015),
        "movable": True,
        "mass": 0.04,
    }
    grown = scale_world(WorldSpec(kind="flat", stones=[dict(authored)]), 2.0)
    stone = grown.stones[0]
    assert stone["mass"] == 0.04 * 8.0
    assert stone["x"] == 0.72
    assert stone["size"][0] == 0.04

    xml = build_mjcf(RobotParams(), grown)
    model = mujoco.MjModel.from_xml_string(xml)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "stone_0")
    mass = float(model.body_mass[body])
    assert mass == 0.04 * 8.0
    sx, sy, sz = stone["size"]
    expect = _box_diag_inertia(mass, 2.0 * sx, 2.0 * sy, 2.0 * sz)
    got = [float(v) for v in model.body_inertia[body]]
    assert got == pytest.approx(list(expect), rel=1e-4)

    def shove(item: dict) -> float:
        sim = Simulator(params=RobotParams(), world=WorldSpec(kind="flat", stones=[item]), seed=1)
        sim.reset()
        sim.run_for(0.25)
        adr = int(sim.model.joint("stone_0_free").qposadr[0])
        x0 = float(sim.data.qpos[adr])
        from embr_local_sim.types import DriveCommand

        sim.run_for(1.4, DriveCommand(0.30, 0.0, 0.0, "test"))
        assert sim.finite()
        return float(sim.data.qpos[adr]) - x0

    heavy = shove({**stone, "x": 0.20})
    light = shove({**stone, "x": 0.20, "mass": 0.04})
    assert heavy < 0.08
    assert light > heavy + 0.05
