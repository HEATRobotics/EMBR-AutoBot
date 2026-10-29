"""Headless contact-dynamics tests. Assertions are derived from the brief gates."""

from __future__ import annotations

import math

import pytest

from embr_local_sim.model import RobotParams, WorldSpec
from embr_local_sim.physics import Simulator
from embr_local_sim.scenarios import list_scenarios, load_scenario, run_scenario
from embr_local_sim.types import DriveCommand


def _settle(sim: Simulator, seconds: float = 0.8):
    sim.reset()
    return sim.run_for(seconds)


def test_drop_settle_no_explosion():
    sim = Simulator(seed=0)
    gt = _settle(sim, 0.9)
    assert sim.finite()
    assert gt.kinetic_energy < 0.05
    assert 0.026 <= gt.root_pos[2] <= 0.045
    worst = min((c.dist for c in gt.contacts), default=0.0)
    assert worst > -0.008


def test_at_rest_no_unintended_propulsion():
    sim = Simulator(seed=1)
    a = _settle(sim, 0.8)
    b = sim.run_for(1.0)
    assert abs(b.root_pos[0] - a.root_pos[0]) < 0.01
    assert abs(b.actual_vx_body) < 0.03


def test_drive_and_reverse_signs():
    sim = Simulator(seed=2)
    _settle(sim, 0.6)
    x0 = sim.snapshot()[0].root_pos[0]
    sim.run_for(1.6, DriveCommand(0.2, 0.0, 0.0, "test"))
    x1 = sim.snapshot()[0].root_pos[0]
    assert x1 - x0 > 0.08
    sim.reset()
    _settle(sim, 0.6)
    x0 = sim.snapshot()[0].root_pos[0]
    sim.run_for(1.6, DriveCommand(-0.2, 0.0, 0.0, "test"))
    x1 = sim.snapshot()[0].root_pos[0]
    assert x1 - x0 < -0.08


def test_turn_yaw_sign():
    sim = Simulator(seed=3)
    _settle(sim, 0.6)
    yaw0 = sim.snapshot()[0].yaw
    sim.run_for(1.6, DriveCommand(0.10, 1.2, 0.0, "test"))
    yaw1 = sim.snapshot()[0].yaw
    dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
    assert dyaw > 0.15


def test_wall_blocks_and_torque_bounded():
    sim = Simulator(world=WorldSpec(kind="wall"), seed=4)
    _settle(sim, 0.5)
    peak_tau = 0.0
    max_x = -1e9
    saw_wall = False
    sim.drive(0.25, 0.0, source="test")
    for _ in range(int(3.0 * sim.params.control_hz)):
        gt = sim.step_control_tick()
        peak_tau = max(peak_tau, max(abs(v) for v in gt.applied_torque.values()))
        max_x = max(max_x, gt.root_pos[0])
        names = {c.geom1 for c in gt.contacts} | {c.geom2 for c in gt.contacts}
        if "wall" in names:
            saw_wall = True
    assert saw_wall
    wall_face = sim.world.wall_x - 0.5 * sim.world.wall_thickness
    chassis_half = 0.5 * sim.params.chassis_size[0]
    assert max_x + chassis_half < wall_face + 0.012
    assert peak_tau <= sim.params.max_wheel_torque + 1e-6


def test_watchdog_zeros_target_without_teleport():
    sim = Simulator(seed=5)
    _settle(sim, 0.5)
    t_wall = 10.0
    sim.controller.ingest(
        DriveCommand(0.2, 0.0, t_wall, "test", timestamp_sim=sim.data.time),
        wall_now=t_wall,
    )
    sim.controller.expire_if_stale(wall_now=t_wall + 0.5, sim_time=float(sim.data.time))
    assert sim.controller.target_vx == 0.0
    x_before = sim.snapshot()[0].root_pos[0]
    sim.step_control_tick()
    x_after = sim.snapshot()[0].root_pos[0]
    # Zero target is not an instant pose reset.
    assert abs(x_after - x_before) < 0.05


def test_friction_affects_traction():
    grip = Simulator(seed=6)
    ice = Simulator(
        params=RobotParams(wheel_friction=(0.04, 0.001, 0.0001), floor_friction=(0.04, 0.001, 0.0001)),
        seed=6,
    )
    _settle(grip, 0.5)
    _settle(ice, 0.5)
    grip.run_for(1.4, DriveCommand(0.30, 0.0, 0.0, "test"))
    ice.run_for(1.4, DriveCommand(0.30, 0.0, 0.0, "test"))
    grip_gt, grip_obs = grip.snapshot()
    ice_gt, ice_obs = ice.snapshot()
    ice_slip = abs(ice_obs.wheel_odom_x - ice_gt.root_pos[0])
    grip_slip = abs(grip_obs.wheel_odom_x - grip_gt.root_pos[0])
    assert ice_gt.root_pos[0] < grip_gt.root_pos[0]
    assert ice_slip > grip_slip + 0.02


def test_torque_affects_acceleration():
    strong = Simulator(params=RobotParams(max_wheel_torque=0.025), world=WorldSpec(kind="ramp"), seed=7)
    weak = Simulator(params=RobotParams(max_wheel_torque=0.002), world=WorldSpec(kind="ramp"), seed=7)
    _settle(strong, 0.5)
    _settle(weak, 0.5)
    strong.run_for(3.2, DriveCommand(0.22, 0.0, 0.0, "test"))
    weak.run_for(3.2, DriveCommand(0.22, 0.0, 0.0, "test"))
    assert strong.snapshot()[0].root_pos[2] > weak.snapshot()[0].root_pos[2] + 0.02
    assert strong.snapshot()[0].root_pos[0] > weak.snapshot()[0].root_pos[0] + 0.08


def test_ramp_bounded_response():
    sim = Simulator(world=WorldSpec(kind="ramp", ramp_angle_deg=8.0), seed=8)
    _settle(sim, 0.5)
    gt = sim.run_for(3.5, DriveCommand(0.22, 0.0, 0.0, "test"))
    assert sim.finite()
    assert gt.root_pos[0] > 0.35
    assert gt.root_pos[2] > 0.04


def test_reproducible_same_seed():
    def traj():
        sim = Simulator(seed=11)
        _settle(sim, 0.4)
        sim.run_for(1.0, DriveCommand(0.18, 0.4, 0.0, "test"))
        return sim.snapshot()[0].root_pos

    a, b = traj(), traj()
    assert a[0] == pytest.approx(b[0], abs=1e-5)
    assert a[1] == pytest.approx(b[1], abs=1e-5)


def test_timestep_convergence_reportable():
    def end_x(dt: float) -> float:
        steps_per_ctrl = max(1, int(round(0.02 / dt)))
        sim = Simulator(params=RobotParams(physics_dt=dt), seed=9)
        sim._control_period_steps = steps_per_ctrl
        _settle(sim, 0.4)
        sim.run_for(1.0, DriveCommand(0.2, 0.0, 0.0, "test"))
        return sim.snapshot()[0].root_pos[0]

    x1 = end_x(0.001)
    x2 = end_x(0.0005)
    # Sensitivity is reported, not required to be bitwise identical.
    assert abs(x1 - x2) < 0.08


def test_reset_clears_sensor_and_controller_state():
    sim = Simulator(seed=10)
    _settle(sim, 0.4)
    sim.run_for(0.8, DriveCommand(0.2, 0.0, 0.0, "test"))
    odom_busy = sim.snapshot()[1].wheel_odom_x
    assert abs(odom_busy) > 0.02
    sim.reset()
    assert sim.controller.target_vx == 0.0
    assert sim.snapshot()[1].wheel_odom_x == pytest.approx(0.0)
    assert sim.snapshot()[0].sim_time == pytest.approx(0.0)


def test_five_yaml_scenarios(tmp_path):
    paths = list_scenarios()
    assert len(paths) >= 5
    failures = []
    for path in paths:
        result = run_scenario(path, out_dir=tmp_path)
        if not result["passed"]:
            failures.append((result["name"], result["failures"]))
    assert failures == []


def test_yaml_pass_criteria_are_predeclared():
    wall = load_scenario(next(p for p in list_scenarios() if p.stem == "wall"))
    assert wall.pass_criteria.get("wall_blocked") is True
    assert "min_final_x" in load_scenario(next(p for p in list_scenarios() if p.stem == "forward")).pass_criteria
