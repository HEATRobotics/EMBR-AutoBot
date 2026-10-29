"""CLI helpers for make verify-* targets."""

from __future__ import annotations

import json

from embr_local_sim.capstan import CapstanHelperSim
from embr_local_sim.model import RobotParams, WorldSpec
from embr_local_sim.physics import Simulator
from embr_local_sim.probe import ProbeMachine
from embr_local_sim.scenarios import repo_root, run_scenario
from embr_local_sim.tele_cmd import TeleCmd
from embr_local_sim.types import DriveCommand


def verify_drive() -> int:
    sim = Simulator(world=WorldSpec(kind="flat"), seed=0)
    sim.reset()
    sim.run_for(0.5)
    x0 = sim.snapshot()[0].root_pos[0]
    sim.run_for(1.2, DriveCommand(0.2, 0.0, 0.0, "verify"))
    x1 = sim.snapshot()[0].root_pos[0]
    ok = x1 - x0 > 0.08
    print(json.dumps({"morphology": "wheels", "forward_dx": x1 - x0, "passed": ok}))
    return 0 if ok else 1


def verify_probe() -> int:
    sim = Simulator(world=WorldSpec(kind="flat"), seed=0)
    sim.reset()
    sim.run_for(0.3)
    sim.probe.request_deploy(float(sim.data.time))
    for _ in range(int(2.0 * sim.params.control_hz)):
        sim.step_control_tick()
        if sim.probe.state == "SAMPLING":
            break
    sampling = sim.probe.state == "SAMPLING"
    sim.probe.request_stow(float(sim.data.time))
    for _ in range(int(2.0 * sim.params.control_hz)):
        sim.step_control_tick()
        if sim.probe.state == "STOWED":
            break
    stowed = sim.probe.state == "STOWED"
    stuck = ProbeMachine()
    stuck.config.stuck = True
    stuck.request_deploy(0.0)
    stuck.tick(sim_time=0.2, angle=0.0)
    print(
        json.dumps(
            {
                "kind": "hinge_boom_local_proposal",
                "sampling": sampling,
                "stowed": stowed,
                "stuck_fault": stuck.state == "FAULT",
                "not_capstan": True,
                "passed": sampling and stowed and stuck.state == "FAULT",
            }
        )
    )
    return 0 if sampling and stowed else 1


def verify_arms() -> int:
    """Parametric 4-pod benches. CAD meshes were not supplied."""
    cap = CapstanHelperSim()
    a = cap.ingest(TeleCmd(0.3, 1.0, 0.0, "test"))
    payload: dict = {
        "cad_meshes": "UNKNOWN — no STL/OBJ/STEP/URDF of the four-pod in this drop",
        "kind": "four_pod_pitch_hinge_roller_approx",
        "capstan_ignores_turn": a.level == 0.3,
        "not_a_belt": True,
        "skipped_track_bench": False,
    }
    sim = Simulator(params=RobotParams(morphology="articulated"), world=WorldSpec(kind="flat"), seed=0)
    sim.reset()
    sim.run_for(0.5)
    x0 = sim.snapshot()[0].root_pos[0]
    sim.run_for(1.2, DriveCommand(0.18, 0.0, 0.0, "verify-arms"))
    x1 = sim.snapshot()[0].root_pos[0]
    dx = x1 - x0
    payload["flat_drive_dx"] = dx
    drive_ok = dx > 0.06

    sim2 = Simulator(params=RobotParams(morphology="articulated"), world=WorldSpec(kind="flat"), seed=1)
    sim2.reset()
    sim2.run_for(0.4)
    sim2.set_art_targets({"art_fl": -0.40, "art_fr": -0.40, "art_rl": 0.0, "art_rr": 0.0})
    for _ in range(int(1.6 * sim2.params.control_hz)):
        sim2.step_control_tick()
    ang = float(sim2.data.qpos[sim2._art_qpos["art_fl"]])
    payload["art_fl_after_command"] = ang
    art_ok = abs(ang) > 0.12

    sim3 = Simulator(params=RobotParams(morphology="articulated"), seed=2)
    sim3.reset()
    sim3.set_supervisor_mode("fault",)
    sim3.step_control_tick()
    sim3.drive(0.2, 0.0, source="should-be-blocked")
    sim3.step_control_tick()
    payload["fault_holds_zero_drive"] = abs(sim3.controller.target_vx) < 1e-6
    payload["passed"] = bool(drive_ok and art_ok and payload["fault_holds_zero_drive"] and payload["capstan_ignores_turn"])
    print(json.dumps(payload, indent=2))
    return 0 if payload["passed"] else 1


def verify_mission() -> int:
    complete = run_scenario(repo_root() / "scenarios" / "mission_complete.yaml")
    fault = run_scenario(repo_root() / "scenarios" / "mission_fault.yaml")
    stuck = run_scenario(repo_root() / "scenarios" / "probe_stuck.yaml")
    drop = run_scenario(repo_root() / "scenarios" / "lidar_dropout.yaml")
    results = [complete, fault, stuck, drop]
    print(json.dumps({r["name"]: {"passed": r["passed"], "failures": r["failures"]} for r in results}, indent=2))
    return 0 if all(r["passed"] for r in results) else 1
