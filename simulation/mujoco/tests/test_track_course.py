"""Track approximation and obstacle course. These checks are about contact, not appearance."""

from __future__ import annotations

import math
import time
from pathlib import Path

import pytest

from embr_local_sim.model import RobotParams, WorldSpec, build_mjcf
from embr_local_sim.physics import Simulator
from embr_local_sim.track_approx import FIDELITY, N_ROLLERS, all_drive_joints
from embr_local_sim.types import DriveCommand
from embr_local_sim.worlds import course_case, grove_world


def _drive(sim: Simulator, seconds: float, vx: float = 0.22) -> None:
    sim.run_for(seconds, DriveCommand(vx, 0.0, time.time(), source="test"))


def test_tracks_label_and_visual_tread_does_not_collide():
    xml = build_mjcf(RobotParams(morphology="tracks"), WorldSpec(kind="flat"))
    assert "Articulated EMBR approximation" in FIDELITY
    assert "tread_visual" in xml
    tread = xml.split("tread_visual")[1][:400]
    assert 'contype="0"' in tread
    assert len(all_drive_joints()) == 4 * N_ROLLERS
    assert N_ROLLERS == 5


def test_tracks_settle_and_drive_with_bounded_torque():
    sim = Simulator(params=RobotParams(morphology="tracks"), world=WorldSpec(kind="flat"), seed=1)
    sim.reset()
    sim.run_for(0.45)
    z = sim.snapshot()[0].root_pos[2]
    assert 0.01 < z < 0.12
    x0 = sim.snapshot()[0].root_pos[0]
    _drive(sim, 1.2, 0.20)
    gt = sim.snapshot()[0]
    assert sim.finite()
    assert gt.root_pos[0] - x0 > 0.05
    limit = sim.params.max_wheel_torque / N_ROLLERS + 1e-6
    assert all(abs(v) <= limit for v in gt.applied_torque.values())
    names = {c.geom1 for c in gt.contacts} | {c.geom2 for c in gt.contacts}
    assert not any("tread" in name for name in names)


def test_pitch_moves_through_the_actuator():
    sim = Simulator(params=RobotParams(morphology="tracks"), seed=0)
    sim.reset()
    sim.run_for(0.3)
    sim.set_art_targets({"art_fl": -0.40, "art_fr": -0.40, "art_rl": 0.15, "art_rr": 0.15})
    for _ in range(int(1.2 * sim.params.control_hz)):
        sim.step_control_tick()
    angle = float(sim.data.qpos[sim._art_qpos["art_fl"]])
    assert angle < -0.12
    assert abs(angle) <= 0.55 + 0.05
    text = Path(__file__).resolve().parents[1].joinpath("src/embr_local_sim/physics.py").read_text(encoding="utf-8")
    apply = text.split("def _apply_control")[1].split("def _contacts")[0]
    assert "qpos[adr : adr + 3]" not in apply


def _until(sim: Simulator, seconds: float, vx: float = 0.22) -> tuple[bool, float, float]:
    """Drive and return (touched log or stone, peak |roll|, peak |pitch|)."""
    touched = False
    peak_roll = 0.0
    peak_pitch = 0.0
    ticks = int(seconds * sim.params.control_hz)
    for _ in range(ticks):
        sim.drive(vx, 0.0, source="test")
        gt = sim.step_control_tick()
        names = {c.geom1 for c in gt.contacts} | {c.geom2 for c in gt.contacts}
        touched = touched or bool(names & {"log_0", "stone_0"})
        peak_roll = max(peak_roll, abs(gt.roll))
        peak_pitch = max(peak_pitch, abs(gt.pitch))
    return touched, peak_roll, peak_pitch


def test_small_branch_splits_the_two_contact_models():
    """Wheel proxy crests a 10 mm branch. Rollers contact it and stall.

    The stall is the missing belt: a front cylinder cannot wrap the branch.
    """
    world = course_case("branch")
    branch_x = float(world.logs[0]["x"])
    outcomes = {}
    for morph in ("wheels", "tracks"):
        sim = Simulator(params=RobotParams(morphology=morph), world=world, seed=2)
        sim.reset()
        sim.run_for(0.3)
        touched, _roll, pitch = _until(sim, 3.2)
        gt = sim.snapshot()[0]
        assert sim.finite()
        assert touched
        assert all(int(sim.data.warning[i].number) == 0 for i in range(7))
        outcomes[morph] = (gt.root_pos[0], pitch)
    assert outcomes["wheels"][0] > branch_x
    assert outcomes["tracks"][0] < branch_x
    assert outcomes["wheels"][1] > 0.05


def test_secured_log_stops_both_and_loose_branch_rests():
    world = course_case("log")
    log_x = float(world.logs[0]["x"])
    for morph in ("wheels", "tracks"):
        sim = Simulator(params=RobotParams(morphology=morph), world=world, seed=2)
        sim.reset()
        sim.run_for(0.3)
        touched, _r, _p = _until(sim, 2.5)
        gt = sim.snapshot()[0]
        assert sim.finite() and touched
        assert gt.root_pos[0] < log_x

    loose = course_case("loose")
    sim = Simulator(params=RobotParams(morphology="tracks"), world=loose, seed=1)
    sim.reset()
    sim.run_for(0.35)
    bid = int(sim.model.body("log_0").id)
    rest = [float(v) for v in sim.data.xpos[bid]]
    assert rest[2] > 0.005
    sim.run_for(2.0, DriveCommand(0.22, 0.0, time.time(), source="test"))
    moved = [float(v) for v in sim.data.xpos[bid]]
    assert moved[2] > 0.005
    assert abs(moved[0] - rest[0]) > 0.01
    assert sim.finite()


def test_one_sided_board_rolls_the_wheel_proxy():
    world = course_case("roll")
    peaks = {}
    for morph in ("wheels", "tracks"):
        sim = Simulator(params=RobotParams(morphology=morph), world=world, seed=1)
        sim.reset()
        sim.run_for(0.25)
        touched, roll, _pitch = _until(sim, 3.0)
        assert sim.finite() and touched
        peaks[morph] = roll
    assert peaks["wheels"] > 0.04
    # Rollers yaw into the board and do not mount it. That is recorded, not hidden.
    assert peaks["tracks"] < peaks["wheels"]


def test_tracks_skid_steer_both_directions():
    """A and D yaw in place. They must not drive the robot forward."""
    for wz, sign in ((1.5, 1.0), (-1.5, -1.0)):
        sim = Simulator(params=RobotParams(morphology="tracks"), world=WorldSpec(kind="flat"), seed=1)
        sim.reset()
        sim.run_for(0.3)
        start = sim.snapshot()[0]
        gt = sim.run_for(2.0, DriveCommand(0.0, wz, time.time(), source="test"))
        dyaw = math.atan2(math.sin(gt.yaw - start.yaw), math.cos(gt.yaw - start.yaw))
        moved = math.hypot(gt.root_pos[0] - start.root_pos[0], gt.root_pos[1] - start.root_pos[1])
        assert dyaw * sign > 0.15
        assert moved < 0.08
        assert max(abs(v) for v in gt.applied_torque.values()) <= 0.080 / 5 + 1e-9
        assert sim.finite()


def test_tracks_turn_while_moving():
    """Forward or back plus a turn follows an arc. Single-axis drive is unchanged."""
    # +wz is left. Forward yaw follows wz. Reverse yaw flips so the path
    # still bends to that side: D goes right going forward or backward.
    cases = (
        (0.18, 1.5),
        (0.18, -1.5),
        (-0.18, 1.5),
        (-0.18, -1.5),
    )
    for vx, wz in cases:
        sim = Simulator(params=RobotParams(morphology="tracks"), world=WorldSpec(kind="flat"), seed=1)
        sim.reset()
        sim.run_for(0.3)
        start = sim.snapshot()[0]
        gt = sim.run_for(2.0, DriveCommand(vx, wz, time.time(), source="test"))
        dyaw = math.atan2(math.sin(gt.yaw - start.yaw), math.cos(gt.yaw - start.yaw))
        dx = gt.root_pos[0] - start.root_pos[0]
        dy = gt.root_pos[1] - start.root_pos[1]
        move_sign = 1.0 if vx > 0.0 else -1.0
        yaw_sign = (1.0 if wz > 0.0 else -1.0) * move_sign
        side = 1.0 if wz > 0.0 else -1.0
        assert dx * move_sign > 0.12
        assert dyaw * yaw_sign > 0.25
        assert dy * side > 0.04
        assert max(abs(v) for v in gt.applied_torque.values()) <= 0.080 / 5 + 1e-9
        assert sim.finite()


def test_grove_loads_with_collision_and_visual_only_plants():
    world = grove_world(seed=3)
    xml = build_mjcf(RobotParams(morphology="tracks"), world)
    assert "shrub_0" in xml and 'name="shrub_0"' in xml
    shrub = xml.split('name="shrub_0"')[1][:240]
    assert 'contype="1"' in shrub
    assert 'conaffinity="2"' in shrub
    canopy = xml.split('name="canopy_0"')[1][:220]
    assert 'contype="0"' in canopy
    assert "tree_0" in xml
    sim = Simulator(params=RobotParams(morphology="tracks"), world=world, seed=3)
    sim.reset()
    sim.run_for(0.4)
    assert sim.finite()
    assert sim.snapshot()[0].root_pos[2] > 0.01


def test_wheels_proxy_remains_the_default():
    assert RobotParams().morphology == "wheels"
    xml = build_mjcf(RobotParams(), WorldSpec(kind="flat"))
    assert "tread_visual" not in xml
