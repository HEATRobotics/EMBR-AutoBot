"""Unit tests for command mapping, saturation, watchdog, and resets."""

from __future__ import annotations

import math
import time

import pytest

from embr_local_sim.controls import (
    InvalidCommandError,
    VelocityController,
    reject_if_invalid,
    saturate_twist,
    twist_to_wheel_speeds,
)
from embr_local_sim.model import RobotParams
from embr_local_sim.types import LEFT_WHEELS, RIGHT_WHEELS, DriveCommand


def test_symmetric_forward_wheel_speeds():
    params = RobotParams()
    speeds = twist_to_wheel_speeds(0.2, 0.0, params)
    left = {speeds[n] for n in LEFT_WHEELS}
    right = {speeds[n] for n in RIGHT_WHEELS}
    assert len(left) == 1 and len(right) == 1
    assert speeds["front_left_maxon"] == pytest.approx(speeds["rear_left_maxon"])
    assert speeds["front_right_maxon"] == pytest.approx(speeds["rear_right_maxon"])
    assert speeds["front_left_maxon"] == pytest.approx(speeds["front_right_maxon"])
    expected = params.hinge_from_ground_speed * 0.2 / params.wheel_radius
    assert speeds["front_left_maxon"] == pytest.approx(expected)


def test_left_turn_right_wheels_faster_forward():
    params = RobotParams()
    speeds = twist_to_wheel_speeds(0.0, 1.0, params)
    # +wz is CCW. Right track ground speed is + (track/2); hinge sign is -1.
    assert abs(speeds["front_right_maxon"]) == pytest.approx(
        (params.wheel_separation * 0.5) / params.wheel_radius
    )
    ground_right = speeds["front_right_maxon"] * params.hinge_from_ground_speed * params.wheel_radius
    ground_left = speeds["front_left_maxon"] * params.hinge_from_ground_speed * params.wheel_radius
    assert ground_right > ground_left


def test_saturation_and_nan_rejected():
    params = RobotParams()
    vx, wz = saturate_twist(10.0, -10.0, params)
    assert vx == params.max_vx
    assert wz == -params.max_wz
    with pytest.raises(InvalidCommandError):
        reject_if_invalid(float("nan"), 0.0)
    with pytest.raises(InvalidCommandError):
        reject_if_invalid(0.0, float("inf"))


def test_invalid_command_zeros_target():
    ctrl = VelocityController(RobotParams())
    ctrl.ingest(DriveCommand(float("nan"), 0.1, time.time(), "test"), wall_now=time.time())
    assert ctrl.target_vx == 0.0 and ctrl.target_wz == 0.0


def test_watchdog_expires_on_wall_clock():
    params = RobotParams(watchdog_wall_s=0.2)
    ctrl = VelocityController(params)
    t0 = 1000.0
    ctrl.ingest(DriveCommand(0.2, 0.0, t0, "test"), wall_now=t0)
    assert ctrl.expire_if_stale(wall_now=t0 + 0.05, sim_time=0.05) is False
    assert ctrl.target_vx == pytest.approx(0.2)
    assert ctrl.expire_if_stale(wall_now=t0 + 0.25, sim_time=0.25) is True
    assert ctrl.target_vx == 0.0
    assert ctrl.watchdog_tripped is True


def test_torque_is_bounded_and_slew_limited():
    params = RobotParams(max_wheel_torque=0.05, max_wheel_accel=5.0, velocity_kp=1.0)
    ctrl = VelocityController(params)
    ctrl.ingest(DriveCommand(0.3, 0.0, time.time(), "test"), wall_now=time.time())
    zeros = {n: 0.0 for n in ("front_left_maxon", "front_right_maxon", "rear_left_maxon", "rear_right_maxon")}
    tau = ctrl.torque_command(zeros, dt=0.02)
    assert all(abs(v) <= params.max_wheel_torque + 1e-9 for v in tau.values())
    # One 20 ms tick cannot reach the full 0.3 m/s wheel speed.
    assert abs(ctrl.slewed_wheel["front_left_maxon"]) < abs(
        params.hinge_from_ground_speed * 0.3 / params.wheel_radius
    )


def test_reset_clears_integrator():
    ctrl = VelocityController(RobotParams())
    ctrl.ingest(DriveCommand(0.2, 0.1, time.time(), "test"), wall_now=time.time())
    ctrl.torque_command({n: 0.0 for n in ctrl.slewed_wheel}, dt=0.02)
    ctrl.reset()
    assert ctrl.target_vx == 0.0
    assert all(v == 0.0 for v in ctrl.slewed_wheel.values())
    assert ctrl.last_command is None


def test_quaternion_yaw_convention():
    # +90 deg about Z → yaw +pi/2. Matches MuJoCo wxyz.
    w, x, y, z = math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    assert yaw == pytest.approx(math.pi / 2)
