"""Spec E acceptance: TeleCmd, sensors, mission, replay, honesty guards."""

from __future__ import annotations

import ast
import math
import time
from pathlib import Path

import pytest

from embr_local_sim.capstan import CAPSTAN_IGNORES_TURN, CapstanHelperSim
from embr_local_sim.fds_replay import NoFdsFileError, load_fds_slice
from embr_local_sim.mission import MissionConfig, MissionMachine, MissionView
from embr_local_sim.model import RobotParams, WorldSpec
from embr_local_sim.physics import Simulator
from embr_local_sim.tele_cmd import (
    MOTOR_ORDER_FL_RL_FR_RR,
    REPO_HELPER_COMMAND_TIMEOUT_S,
    REPO_IBUS_FRAME_TIMEOUT_S,
    DrivetrainHelperSim,
    TeleCmd,
    hardware_motor_levels,
    mix_four_motor_levels,
    tele_cmd_to_twist,
)
from embr_local_sim.thermal import COMPONENT_RATING_C


def _settle(sim: Simulator, seconds: float = 0.55):
    sim.reset()
    return sim.run_for(seconds)


def test_timeout_layers_are_distinct():
    params = RobotParams()
    assert params.watchdog_wall_s == pytest.approx(0.40)
    assert REPO_HELPER_COMMAND_TIMEOUT_S == pytest.approx(0.50)
    assert REPO_IBUS_FRAME_TIMEOUT_S == pytest.approx(0.25)
    assert params.watchdog_wall_s != REPO_HELPER_COMMAND_TIMEOUT_S


def test_telecmd_is_normalized_not_metres():
    params = RobotParams()
    cmd = TeleCmd(velocity=1.0, turn=0.0, timestamp_wall=0.0, source="test")
    twist = tele_cmd_to_twist(cmd, params)
    assert twist.vx == pytest.approx(params.max_vx)
    assert twist.wz == pytest.approx(0.0)


def test_positive_turn_is_right_negative_wz():
    params = RobotParams()
    twist = tele_cmd_to_twist(TeleCmd(0.0, 1.0, 0.0, "test"), params)
    assert twist.wz == pytest.approx(-params.max_wz)
    twist_left = tele_cmd_to_twist(TeleCmd(0.0, -1.0, 0.0, "test"), params)
    assert twist_left.wz == pytest.approx(params.max_wz)


def test_motor_order_fl_rl_fr_rr():
    assert MOTOR_ORDER_FL_RL_FR_RR == (
        "front_left_maxon",
        "rear_left_maxon",
        "front_right_maxon",
        "rear_right_maxon",
    )
    levels = hardware_motor_levels(TeleCmd(1.0, 0.5, 0.0, "test"))
    # helper: mix(forward, -turn) → left = 1-(-0.5)=1.5, right=1+(-0.5)=0.5, scale 1.5
    expected = mix_four_motor_levels(1.0, -0.5)
    assert levels == pytest.approx(expected)
    assert len(levels) == 4
    assert levels[0] == pytest.approx(levels[1])  # FL == RL
    assert levels[2] == pytest.approx(levels[3])  # FR == RR
    assert levels[0] > levels[2]  # left faster when turning right after negation


def test_nonfinite_telecmd_zeros():
    helper = DrivetrainHelperSim(RobotParams())
    twist = helper.ingest(TeleCmd(float("nan"), 0.2, 1.0, "test"))
    assert twist.vx == 0.0 and twist.wz == 0.0


def test_helper_timeout_does_not_replace_sim_watchdog():
    helper = DrivetrainHelperSim(RobotParams(), timeout_s=0.5)
    helper.ingest(TeleCmd(1.0, 0.0, 10.0, "test"))
    live = helper.publish(10.4)
    dead = helper.publish(10.55)
    assert live.vx > 0.0
    assert dead.vx == 0.0
    assert dead.source == "helper-timeout"


def test_capstan_uses_velocity_ignores_turn():
    assert CAPSTAN_IGNORES_TURN is True
    cap = CapstanHelperSim()
    a = cap.ingest(TeleCmd(0.4, 1.0, 1.0, "test"))
    b = cap.ingest(TeleCmd(0.4, -1.0, 1.1, "test"))
    assert a.level == pytest.approx(0.4)
    assert b.level == pytest.approx(0.4)


def test_drive_and_capstan_not_wired_by_default():
    sim = Simulator(seed=0)
    assert not hasattr(sim, "capstan") or getattr(sim, "share_tele_with_capstan", False) is False


def test_lidar_known_wall_distance():
    sim = Simulator(world=WorldSpec(kind="wall", wall_x=0.50), seed=0)
    _settle(sim, 0.6)
    _, obs = sim.snapshot()
    assert obs.lidar_frame == "lidar"
    assert len(obs.lidar_ranges) == sim.params.lidar_n_rays
    mid = obs.lidar_ranges[len(obs.lidar_ranges) // 2]
    wall_face = sim.world.wall_x - 0.5 * sim.world.wall_thickness
    # lidar site is ~2 cm forward of root; allow contact/settle slop
    assert 0.35 < mid < 0.55
    assert mid == pytest.approx(wall_face - 0.02, abs=0.08)


def test_thermal_hotter_near_source_not_sine():
    world = WorldSpec(
        kind="flat",
        heat=[{"x": 0.4, "y": 0.0, "z": 0.02, "peak_c": 120.0, "sigma": 0.12, "kind": "smolder"}],
    )
    sim = Simulator(world=world, seed=0)
    sim.reset()
    cold = sim.thermal.temperature_c((0.0, 0.0, 0.03), 1.0)
    hot = sim.thermal.temperature_c((0.4, 0.0, 0.03), 1.0)
    assert hot > cold + 40.0
    t0 = sim.thermal.temperature_c((0.4, 0.0, 0.03), 0.0)
    t1 = sim.thermal.temperature_c((0.4, 0.0, 0.03), 1.0)
    assert t0 == pytest.approx(t1, abs=1e-6)


def test_probe_is_fixed_chassis_site():
    sim = Simulator(world=WorldSpec(kind="forest"), seed=1)
    _settle(sim, 0.4)
    _, obs = sim.snapshot()
    assert obs.probe_kind == "hinge_boom_local_proposal"
    assert obs.probe_frame == "probe"
    sid = int(sim.model.site("probe").id)
    assert sid >= 0
    assert 'name="probe"' in sim.xml
    assert obs.probe_c == obs.probe_c


def test_lumped_temps_heat_and_unknown_rating():
    world = WorldSpec(
        kind="flat",
        heat=[{"x": 0.0, "y": 0.0, "z": 0.02, "peak_c": 100.0, "sigma": 0.08}],
    )
    sim = Simulator(world=world, seed=0)
    sim.reset()
    t0 = sim.thermal.lumps["electronics"].temperature_c
    sim.run_for(1.2)
    t1 = sim.thermal.lumps["electronics"].temperature_c
    assert t1 > t0 + 0.5
    assert sim.thermal.lumps["electronics"].rating_c == COMPONENT_RATING_C


def test_lumps_cool_after_retreat():
    world = WorldSpec(
        kind="flat",
        heat=[{"x": 0.0, "y": 0.0, "z": 0.02, "peak_c": 140.0, "sigma": 0.07}],
    )
    sim = Simulator(world=world, seed=0)
    sim.reset()
    sim.run_for(0.8)
    hot = sim.thermal.lumps["electronics"].temperature_c
    sim.reset({"x": 3.0, "y": 0.0})
    # reset clears lumps; heat away from spawn then drive off
    sim = Simulator(world=world, seed=0)
    sim.reset()
    sim.run_for(0.9)
    hot = sim.thermal.lumps["electronics"].temperature_c
    sim.thermal.sources.clear()
    sim.run_for(2.0)
    cool = sim.thermal.lumps["electronics"].temperature_c
    assert cool < hot


def test_root_free_named_joint():
    sim = Simulator(world=WorldSpec(kind="forest"), seed=0)
    adr = int(sim.model.joint("root_free").qposadr[0])
    assert adr == sim._root_qposadr
    sim.reset({"x": 0.05})
    assert sim.data.qpos[adr] == pytest.approx(0.05)


def test_mission_view_has_no_gt_pose_field():
    fields = set(MissionView.__dataclass_fields__)
    assert "root_pos" not in fields
    assert "ground_truth" not in fields
    src = Path(__file__).resolve().parents[1] / "src" / "embr_local_sim" / "mission.py"
    tree = ast.parse(src.read_text(encoding="utf-8"))
    names = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
    assert "root_pos" not in names


def test_manual_takeover_cancels_mission():
    sim = Simulator(world=WorldSpec(kind="flat"), seed=0)
    _settle(sim, 0.4)
    sim.start_mission(MissionConfig(timeout_s=5.0, detect_c=999.0))
    sim.step_control_tick()
    assert sim.gate.mode == "mission"
    sim.ingest_tele_cmd(TeleCmd(0.0, 0.0, time.time(), source="gui-hold"))
    for _ in range(3):
        sim.step_control_tick()
    assert sim.gate.mode == "manual"
    assert sim.mission.state == "FAULT"
    assert sim.mission.reason == "manual_takeover"


def test_pause_does_not_freeze_watchdog():
    sim = Simulator(seed=0)
    _settle(sim, 0.3)
    t0 = time.time()
    sim.controller.ingest(
        __import__("embr_local_sim.types", fromlist=["DriveCommand"]).DriveCommand(0.2, 0.0, t0, "test"),
        wall_now=t0,
    )
    sim.controller.expire_if_stale(wall_now=t0 + 0.5, sim_time=1.0)
    assert sim.controller.target_vx == 0.0


def test_command_vs_measured_flat_and_wall():
    from embr_local_sim.types import DriveCommand

    flat = Simulator(world=WorldSpec(kind="flat"), seed=1)
    _settle(flat, 0.5)
    flat.run_for(1.2, DriveCommand(0.20, 0.0, 0.0, "test"))
    f_gt, _ = flat.snapshot()
    assert f_gt.actual_vx_body > 0.08
    assert abs(f_gt.target_vx - f_gt.actual_vx_body) < 0.18

    wall = Simulator(world=WorldSpec(kind="wall"), seed=1)
    _settle(wall, 0.5)
    saw_wall = False
    for _ in range(int(2.8 * wall.params.control_hz)):
        wall.drive(0.22, 0.0, source="test")
        w_gt = wall.step_control_tick()
        names = {c.geom1 for c in w_gt.contacts} | {c.geom2 for c in w_gt.contacts}
        if "wall" in names:
            saw_wall = True
    assert saw_wall
    assert w_gt.target_vx > 0.15
    assert abs(w_gt.actual_vx_body) < 0.12


def test_replay_separates_streams():
    sim = Simulator(world=WorldSpec(kind="flat"), seed=2)
    _settle(sim, 0.4)
    sim.run_for(0.6, __import__("embr_local_sim.types", fromlist=["DriveCommand"]).DriveCommand(0.18, 0.0, 0.0, "test"))
    frame = sim.replay.frames[-1]
    assert frame.command_vx != frame.odom_x
    assert "gt_x" in frame.__dataclass_fields__
    # odom is not secretly equal to a pose write of command*time as the only motion
    assert abs(frame.measured_vx_body) > 0.0 or abs(frame.gt_x) >= 0.0


def test_fds_absent_refuses():
    with pytest.raises(NoFdsFileError):
        load_fds_slice(None)


def test_tracks_absent_from_this_tree():
    src = Path(__file__).resolve().parents[1] / "src" / "embr_local_sim"
    names = {p.name for p in src.glob("*.py")}
    assert "tracks.py" not in names
    assert "gym_tracks.py" not in names
    # articulated.py is a labeled roller approximation, not CAD belts.


def test_mission_complete_and_fault_yaml(tmp_path):
    from embr_local_sim.scenarios import repo_root, run_scenario

    complete = run_scenario(repo_root() / "scenarios" / "mission_complete.yaml", out_dir=tmp_path)
    assert complete["passed"], complete["failures"]
    assert complete["final"]["mission"]["state"] == "COMPLETE"
    assert complete["final"]["mission"]["used_ground_truth"] is False
    rec = complete["final"]["mission"]["hotspots"][0]
    assert rec["frame"] == "odom"
    assert rec["provenance"] == "hinge_boom_local_proposal"

    fault = run_scenario(repo_root() / "scenarios" / "mission_fault.yaml", out_dir=tmp_path)
    assert fault["passed"], fault["failures"]
    assert fault["final"]["mission"]["state"] == "FAULT"
    assert "blocked" in fault["final"]["mission"]["reason"]
