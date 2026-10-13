"""Differential-drive command mapping, slew limits, watchdog, torque loop.

Differential-drive kinematics produce wheel-speed *targets* only. Applied
motion comes from bounded motor torques and contact in the physics engine.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from embr_local_sim.model import RobotParams
from embr_local_sim.types import LEFT_WHEELS, RIGHT_WHEELS, WHEEL_JOINTS, DriveCommand


class InvalidCommandError(ValueError):
    pass


def reject_if_invalid(vx: float, wz: float) -> None:
    if not math.isfinite(vx) or not math.isfinite(wz):
        raise InvalidCommandError(f"non-finite command vx={vx} wz={wz}")


def saturate_twist(vx: float, wz: float, params: RobotParams) -> tuple[float, float]:
    vx = max(-params.max_vx, min(params.max_vx, vx))
    wz = max(-params.max_wz, min(params.max_wz, wz))
    return vx, wz


def twist_to_wheel_speeds(vx: float, wz: float, params: RobotParams) -> dict[str, float]:
    """Map body twist to hinge angular velocities (rad/s).

    Right-hand hinge axis is body +Y. On this MJCF, positive hinge velocity
    produces +X chassis travel (verified by a bounded torque sweep). +wz is yaw
    about +Z (CCW): the right track then needs a larger forward ground speed
    than the left.
    """
    reject_if_invalid(vx, wz)
    vx, wz = saturate_twist(vx, wz, params)
    track = params.wheel_separation
    v_left = vx - wz * (track * 0.5)
    v_right = vx + wz * (track * 0.5)
    scale = params.hinge_from_ground_speed / params.wheel_radius
    out: dict[str, float] = {}
    for name in LEFT_WHEELS:
        out[name] = v_left * scale
    for name in RIGHT_WHEELS:
        out[name] = v_right * scale
    return out


def _slew(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


@dataclass
class VelocityController:
    """Finite-gain wheel-speed loop with accel limit, torque cap, and deadman."""

    params: RobotParams
    target_vx: float = 0.0
    target_wz: float = 0.0
    last_command: DriveCommand | None = None
    last_command_wall: float = 0.0
    slewed_wheel: dict[str, float] = field(default_factory=dict)
    watchdog_tripped: bool = False
    zero_target_sim_time: float | None = None

    def __post_init__(self) -> None:
        if not self.slewed_wheel:
            self.slewed_wheel = {name: 0.0 for name in WHEEL_JOINTS}

    def reset(self) -> None:
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.last_command = None
        self.last_command_wall = 0.0
        self.slewed_wheel = {name: 0.0 for name in WHEEL_JOINTS}
        self.watchdog_tripped = False
        self.zero_target_sim_time = None

    def ingest(self, command: DriveCommand, *, wall_now: float) -> None:
        try:
            reject_if_invalid(command.vx, command.wz)
        except InvalidCommandError:
            self.target_vx = 0.0
            self.target_wz = 0.0
            self.last_command = DriveCommand(0.0, 0.0, wall_now, source="invalid")
            self.last_command_wall = wall_now
            return
        vx, wz = saturate_twist(command.vx, command.wz, self.params)
        self.target_vx = vx
        self.target_wz = wz
        self.last_command = command
        self.last_command_wall = command.timestamp_wall
        self.watchdog_tripped = False
        if vx == 0.0 and wz == 0.0 and self.zero_target_sim_time is None:
            self.zero_target_sim_time = command.timestamp_sim

    def expire_if_stale(self, *, wall_now: float, sim_time: float) -> bool:
        """Wall-clock supervisor. A stale command zeros the speed target.

        Pause itself is handled by Simulator, which clears targets immediately
        and drops drive input until resume.
        """
        age = wall_now - self.last_command_wall if self.last_command is not None else math.inf
        if age > self.params.watchdog_wall_s:
            if self.target_vx != 0.0 or self.target_wz != 0.0:
                if self.zero_target_sim_time is None:
                    self.zero_target_sim_time = sim_time
            self.target_vx = 0.0
            self.target_wz = 0.0
            self.watchdog_tripped = True
            return True
        return False

    def torque_command(
        self,
        joint_vel: dict[str, float],
        *,
        dt: float,
    ) -> dict[str, float]:
        desired = twist_to_wheel_speeds(self.target_vx, self.target_wz, self.params)
        max_delta = self.params.max_wheel_accel * dt
        tau: dict[str, float] = {}
        limit = self.params.max_wheel_torque
        kp = self.params.velocity_kp
        for name in WHEEL_JOINTS:
            slewed = _slew(self.slewed_wheel[name], desired[name], max_delta)
            self.slewed_wheel[name] = slewed
            err = slewed - joint_vel.get(name, 0.0)
            t = max(-limit, min(limit, kp * err))
            tau[name] = t
        return tau

    def watchdog_age(self, wall_now: float) -> float:
        if self.last_command is None:
            return math.inf
        return wall_now - self.last_command_wall
