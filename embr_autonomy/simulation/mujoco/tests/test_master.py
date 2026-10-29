"""Master-plan gates: named root, probe SM, RL leakage, faults."""

from __future__ import annotations

from pathlib import Path

import pytest

from embr_local_sim.model import RobotParams, WorldSpec, build_mjcf
from embr_local_sim.physics import Simulator
from embr_local_sim.probe import ProbeMachine
from embr_local_sim.scenarios import repo_root, run_scenario


def test_named_root_survives_obstacle_qpos_order():
    world = WorldSpec(
        kind="flat",
        stones=[{"x": 0.4, "y": 0.0, "size": (0.03, 0.03, 0.03), "movable": True, "mass": 0.05}],
    )
    xml = build_mjcf(RobotParams(), world)
    assert xml.find("stone_0_free") < xml.find("root_free")
    sim = Simulator(world=world, seed=0)
    sim.reset({"x": 0.11, "y": 0.02})
    stone_adr = int(sim.model.joint("stone_0_free").qposadr[0])
    root_adr = int(sim.model.joint("root_free").qposadr[0])
    assert stone_adr < root_adr
    assert sim._root_qposadr == root_adr
    assert sim.snapshot()[0].root_pos[0] == pytest.approx(0.11, abs=1e-6)
    xpos = sim.data.xpos[sim._root_id]
    assert float(xpos[0]) == pytest.approx(0.11, abs=1e-5)


def test_probe_sm_deploy_sample_stow_and_stuck():
    sim = Simulator(world=WorldSpec(kind="flat"), seed=1)
    sim.reset()
    sim.run_for(0.3)
    assert sim.probe.state == "STOWED"
    sim.probe.request_deploy(float(sim.data.time))
    saw = False
    for _ in range(int(2.5 * sim.params.control_hz)):
        sim.step_control_tick()
        if sim.probe.state == "SAMPLING":
            saw = True
            break
    assert saw
    sim.probe.request_stow(float(sim.data.time))
    for _ in range(int(2.5 * sim.params.control_hz)):
        sim.step_control_tick()
        if sim.probe.state == "STOWED":
            break
    assert sim.probe.state == "STOWED"
    stuck = ProbeMachine()
    stuck.config.stuck = True
    stuck.request_deploy(0.0)
    stuck.tick(sim_time=0.5, angle=0.0)
    assert stuck.state == "FAULT"


def test_probe_and_drive_are_separate_actuators():
    sim = Simulator(seed=0)
    assert "probe_hinge" in sim.model.actuator("probe_hinge").name
    assert "front_left_maxon" in sim._act_id
    assert sim._probe_act != sim._act_id["front_left_maxon"]


def test_lidar_dropout_and_probe_stuck_yaml(tmp_path):
    drop = run_scenario(repo_root() / "scenarios" / "lidar_dropout.yaml", out_dir=tmp_path)
    assert drop["passed"], drop["failures"]
    stuck = run_scenario(repo_root() / "scenarios" / "probe_stuck.yaml", out_dir=tmp_path)
    assert stuck["passed"], stuck["failures"]


def test_rl_training_is_not_in_this_baseline():
    src = Path(__file__).resolve().parents[1] / "src" / "embr_local_sim"
    names = {path.name for path in src.glob("*.py")}
    assert "gym_env.py" not in names
    assert "train_rl.py" not in names
    assert "eval_rl.py" not in names
