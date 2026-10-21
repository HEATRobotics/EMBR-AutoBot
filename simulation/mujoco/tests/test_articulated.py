"""Four-pod pitch-hinge prototype. CAD meshes were not in this drop."""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from embr_local_sim.articulated import ART_JOINTS, KIND, all_drive_joints
from embr_local_sim.model import RobotParams, WorldSpec, build_mjcf
from embr_local_sim.observation import RobotObservation
from embr_local_sim.physics import Simulator
from embr_local_sim.scenarios import repo_root, run_scenario
from embr_local_sim.supervisor import CommandSupervisor
from embr_local_sim.types import DriveCommand


def test_default_morphology_remains_wheels():
    assert RobotParams().morphology == "wheels"
    xml = build_mjcf(RobotParams(), WorldSpec(kind="flat"))
    assert "wheel_fl" in xml
    assert "art_fl" not in xml


def test_articulated_xml_has_four_pitch_hinges_and_rollers():
    xml = build_mjcf(RobotParams(morphology="articulated"), WorldSpec(kind="flat"))
    for name in ART_JOINTS:
        assert f'name="{name}"' in xml
    assert "roller_fl_1" in xml
    assert "root_free" in xml
    assert "four_pod" in xml or "roller_approx" in xml
    assert KIND == "four_pod_pitch_hinge_roller_approx"


def test_no_cad_mesh_files_claimed():
    root = Path(__file__).resolve().parents[1]
    found = []
    for folder in (root / "models", root / "src"):
        for ext in (".stl", ".obj", ".step", ".stp"):
            found.extend(folder.rglob(f"*{ext}"))
    assert found == []
    src = (root / "src/embr_local_sim/articulated.py").read_text(encoding="utf-8")
    assert "Not imported CAD" in src
    assert "not a moving belt" in src.lower() or "not a belt" in src.lower()


def test_robot_observation_has_no_gt_pose():
    fields = set(RobotObservation.__dataclass_fields__)
    assert "root_pos" not in fields
    src = Path(__file__).resolve().parents[1] / "src" / "embr_local_sim" / "observation.py"
    names = {n.attr for n in ast.walk(ast.parse(src.read_text(encoding="utf-8"))) if isinstance(n, ast.Attribute)}
    assert "root_pos" not in names


def test_articulated_named_root_and_drive_contact():
    sim = Simulator(params=RobotParams(morphology="articulated"), world=WorldSpec(kind="flat"), seed=0)
    assert sim._root_qposadr == int(sim.model.joint("root_free").qposadr[0])
    sim.reset()
    sim.run_for(0.5)
    x0 = sim.snapshot()[0].root_pos[0]
    sim.run_for(1.3, DriveCommand(0.20, 0.0, 0.0, "test"))
    gt, _ = sim.snapshot()
    assert gt.root_pos[0] - x0 > 0.05
    assert sim.finite()
    assert len(all_drive_joints()) == 12


def test_articulation_tracks_command_not_teleport():
    sim = Simulator(params=RobotParams(morphology="articulated"), seed=1)
    sim.reset()
    sim.run_for(0.3)
    sim.set_art_targets({"art_fl": -0.45, "art_fr": -0.45, "art_rl": 0.0, "art_rr": 0.0})
    for _ in range(int(1.4 * sim.params.control_hz)):
        sim.step_control_tick()
    ang = float(sim.data.qpos[sim._art_qpos["art_fl"]])
    assert ang < -0.12
    src = Path(__file__).resolve().parents[1] / "src" / "embr_local_sim" / "physics.py"
    text = src.read_text(encoding="utf-8")
    assert "data.qpos[adr : adr + 3]" in text  # reset/teleport only
    # locomotion path must not assign root qpos each step
    apply = text.split("def _apply_control")[1].split("def _contacts")[0]
    assert "qpos[adr : adr + 3]" not in apply


def test_supervisor_rejects_nan_and_holds_art():
    sup = CommandSupervisor()
    from embr_local_sim.observation import ControllerOutput, RobotObservation

    obs = RobotObservation(
        sim_time=0.0,
        stamp_wall=0.0,
        imu_quat_wxyz=(1, 0, 0, 0),
        imu_angvel_body=(0, 0, 0),
        imu_specific_force_body=(0, 0, 9.81),
        roll_rad=0.0,
        pitch_rad=0.0,
    )
    out = ControllerOutput(0.2, 0.0, {"art_fl": float("nan")}, source="test")
    res = sup.approve(out, obs)
    assert res.rejected
    assert res.approved_drive_vx == 0.0
    assert sup.mode == "fault"


def test_articulated_yaml_scenarios(tmp_path):
    rest = run_scenario(repo_root() / "scenarios" / "articulated_rest.yaml", out_dir=tmp_path)
    assert rest["passed"], rest["failures"]
    drive = run_scenario(repo_root() / "scenarios" / "articulated_drive.yaml", out_dir=tmp_path)
    assert drive["passed"], drive["failures"]
    log = run_scenario(repo_root() / "scenarios" / "articulated_log.yaml", out_dir=tmp_path)
    assert log["passed"], log["failures"]


def test_propulsion_not_wired_to_art_actuator():
    sim = Simulator(params=RobotParams(morphology="articulated"), seed=0)
    assert sim._act_id["front_left_maxon"] != sim._art_act["art_fl"]
    assert "probe_hinge" not in sim._act_id
