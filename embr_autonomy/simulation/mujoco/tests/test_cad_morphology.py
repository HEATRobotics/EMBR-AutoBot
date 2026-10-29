"""CAD morphology. Geometry is from the STEP tessellation; mass and torque are not."""

from __future__ import annotations

import math

from embr_local_sim.cad_model import load_cad_geometry
from embr_local_sim.model import RobotParams, WorldSpec
from embr_local_sim.physics import Simulator
from embr_local_sim.types import DriveCommand, WHEEL_JOINTS


def _cad() -> Simulator:
    sim = Simulator(params=RobotParams(morphology="cad"), world=WorldSpec(kind="flat"), seed=0)
    sim.reset()
    return sim


def test_cad_loads_finite_and_clears_the_ground():
    geo = load_cad_geometry()
    sim = _cad()
    assert sim.params.wheel_radius == geo["roller_radius_m"]
    assert abs(sim.params.wheel_separation - geo["wheel_separation_m"]) < 1e-9
    gt = sim.run_for(0.8)
    assert sim.finite()
    assert gt.root_pos[2] > 0.05
    plate_bottom = gt.root_pos[2] + geo["plate"]["body_center_m"][2] - 0.5 * sim.params.chassis_size[2]
    assert plate_bottom > 0.01
    assert gt.kinetic_energy < 1.5
    for name in WHEEL_JOINTS:
        assert name in gt.joint_pos
    probe = float(sim.data.qpos[sim.model.joint("probe_hinge").qposadr[0]])
    assert 0.0 <= probe <= 1.2


def test_cad_forward_reverse_and_yaw():
    sim = _cad()
    sim.run_for(0.5)
    x0 = sim.snapshot()[0].root_pos[0]
    sim.run_for(1.8, DriveCommand(0.25, 0.0, 0.0, "test"))
    forward = sim.snapshot()[0].root_pos[0] - x0
    assert forward > 0.05

    sim.reset()
    sim.run_for(0.5)
    x0 = sim.snapshot()[0].root_pos[0]
    sim.run_for(1.8, DriveCommand(-0.25, 0.0, 0.0, "test"))
    assert sim.snapshot()[0].root_pos[0] - x0 < -0.05

    sim.reset()
    sim.run_for(0.5)
    yaw0 = sim.snapshot()[0].yaw
    sim.run_for(1.6, DriveCommand(0.0, 0.8, 0.0, "test"))
    yaw1 = sim.snapshot()[0].yaw
    dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
    assert dyaw > 0.05
    assert sim.finite()


def test_cad_stop_reduces_speed_and_stays_deterministic():
    def distance() -> float:
        sim = Simulator(params=RobotParams(morphology="cad"), world=WorldSpec(kind="flat"), seed=4)
        sim.reset()
        sim.run_for(0.4)
        x0 = sim.snapshot()[0].root_pos[0]
        sim.run_for(0.8, DriveCommand(0.25, 0.0, 0.0, "test"))
        moving = abs(sim.snapshot()[0].actual_vx_body)
        sim.run_for(0.8, DriveCommand(0.0, 0.0, 0.0, "test"))
        stopped = abs(sim.snapshot()[0].actual_vx_body)
        return sim.snapshot()[0].root_pos[0] - x0, moving, stopped

    dx_a, moving, stopped = distance()
    dx_b, _, _ = distance()
    assert moving > stopped
    assert abs(dx_a - dx_b) < 1e-6


def test_cad_wall_contact_stays_finite():
    sim = Simulator(
        params=RobotParams(morphology="cad"),
        world=WorldSpec(kind="wall", wall_x=1.05),
        seed=5,
    )
    sim.reset()
    sim.run_for(0.4)
    saw_wall = False
    for _ in range(int(2.5 * sim.params.control_hz)):
        sim.drive(0.25, 0.0, source="test")
        gt = sim.step_control_tick()
        names = {c.geom1 for c in gt.contacts} | {c.geom2 for c in gt.contacts}
        saw_wall = saw_wall or "wall" in names
    assert sim.finite()
    assert saw_wall
    assert 0.02 < sim.snapshot()[0].root_pos[2] < 0.4


def test_proxy_default_is_unchanged():
    sim = Simulator(world=WorldSpec(kind="flat"), seed=0)
    assert sim.params.morphology == "wheels"
    assert sim.params.wheel_radius == 0.03
