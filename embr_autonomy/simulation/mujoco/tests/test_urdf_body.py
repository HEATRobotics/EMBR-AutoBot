"""URDF mock bot: a standard primitive-only URDF, four corner arms, and height/pitch/roll from them."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from embr_local_sim.arm import ARM_JOINTS, ARM_LENGTH_M, IDLE, REACHED, arm_angles
from embr_local_sim.body import apply_body
from embr_local_sim.lidar import _rotate_body_to_world
from embr_local_sim.model import RobotParams, WorldSpec
from embr_local_sim.physics import Simulator
from embr_local_sim.urdf_model import DRIVE_JOINTS, urdf_path


def _sim() -> Simulator:
    sim = Simulator(params=apply_body(RobotParams(morphology="urdf")), world=WorldSpec(kind="flat"), seed=1)
    sim.reset()
    return sim


def _settle(sim: Simulator, ticks: int = 300) -> tuple[float, float, float]:
    """Chassis height (m), pitch (deg, + nose down), roll (deg, + left up)."""
    for _ in range(ticks):
        sim.step_control_tick()
    gt, _ = sim.snapshot()
    q = np.array(gt.root_quat_wxyz)
    fwd = _rotate_body_to_world(q, np.array([1.0, 0.0, 0.0]))
    left = _rotate_body_to_world(q, np.array([0.0, 1.0, 0.0]))
    return float(gt.root_pos[2]), math.degrees(math.asin(-fwd[2])), math.degrees(math.asin(left[2]))


def test_the_urdf_is_primitive_only_and_loads_natively() -> None:
    tree = ET.parse(urdf_path())
    assert not tree.findall(".//mesh")
    joints = {j.get("name"): j for j in tree.findall("joint")}
    for name in ARM_JOINTS:
        assert joints[name].get("type") == "revolute"
        limit = joints[name].find("limit")
        assert float(limit.get("lower")) == pytest.approx(-math.pi / 2, abs=1e-3)
        assert float(limit.get("upper")) == pytest.approx(math.pi / 2, abs=1e-3)
        # Each arm hinges on the chassis, and its drive cylinder hangs off the far end.
        assert joints[name].find("parent").get("link") == "root"
    for name in DRIVE_JOINTS:
        assert joints[name].get("type") == "continuous"
        assert joints[name].find("parent").get("link").startswith("arm_")
    chassis = tree.find("link[@name='root']/collision/geometry/box")
    assert chassis.get("size").split()[:2] == ["0.706", "0.508"]
    model = mujoco.MjModel.from_xml_path(str(urdf_path()))
    assert model.njnt == len(joints)


def test_the_sim_body_keeps_the_drive_contract() -> None:
    model = _sim().model
    for site in ("imu", "lidar", "probe"):
        model.site(site)
    for actuator in (*DRIVE_JOINTS, *ARM_JOINTS, "probe_hinge"):
        model.actuator(actuator)
    contact = [model.geom(i).name for i in range(model.ngeom) if model.geom_contype[i] == 2]
    assert sorted(contact) == sorted(["chassis", "front_left", "front_right", "rear_left", "rear_right"])


def test_the_pose_mapping() -> None:
    level = arm_angles(0.0, 0.0, 0.0)
    assert all(a == pytest.approx(0.0) for a in level.values())
    assert all(a == pytest.approx(-math.pi / 2) for a in arm_angles(1.0, 0.0, 0.0).values())
    assert all(a == pytest.approx(math.pi / 2) for a in arm_angles(-1.0, 0.0, 0.0).values())
    nose_down = arm_angles(0.5, 0.3, 0.0)
    # Rear arms go further down than the front ones.
    assert nose_down["arm_rear_left"] < nose_down["arm_front_left"]
    left_up = arm_angles(0.5, 0.0, 0.3)
    assert left_up["arm_front_left"] < left_up["arm_front_right"]
    assert all(abs(a) <= math.pi / 2 + 1e-9 for a in arm_angles(1.0, -1.0, 1.0).values())


def test_height_raises_the_chassis_by_the_arm_drop() -> None:
    sim = _sim()
    level_z, _p, _r = _settle(sim, 100)
    sim.set_arm(0.5, 0.0, 0.0)
    z, pitch, roll = _settle(sim)
    assert sim.arm_status().status == REACHED
    assert z - level_z == pytest.approx(0.5 * ARM_LENGTH_M, abs=0.02)
    assert abs(pitch) < 1.0 and abs(roll) < 1.0


@pytest.mark.parametrize(("cmd", "axis"), [((0.5, 0.4, 0.0), "pitch"), ((0.5, 0.0, 0.4), "roll")])
def test_pitch_and_roll_tilt_the_chassis_the_ros_way(cmd: tuple[float, float, float], axis: str) -> None:
    sim = _sim()
    assert sim.arm_status().status == IDLE
    sim.set_arm(*cmd)
    _z, pitch, roll = _settle(sim)
    assert sim.arm_status().status == REACHED
    tilt, other = (pitch, roll) if axis == "pitch" else (roll, pitch)
    assert tilt > 5.0
    assert abs(other) < 1.0
    assert sim.finite()


def test_extreme_poses_stay_upright() -> None:
    for cmd in [(1.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.5, -0.5, 0.5)]:
        sim = _sim()
        sim.set_arm(*cmd)
        _z, pitch, roll = _settle(sim)
        assert abs(pitch) < 45.0 and abs(roll) < 45.0, cmd


def test_bodies_without_arms_ignore_arm_commands() -> None:
    sim = Simulator(params=apply_body(RobotParams(morphology="cad")), world=WorldSpec(kind="flat"), seed=1)
    sim.reset()
    sim.set_arm(1.0, 1.0, 1.0)
    sim.step_control_tick()
    assert sim.arm is None and sim.arm_status() is None
