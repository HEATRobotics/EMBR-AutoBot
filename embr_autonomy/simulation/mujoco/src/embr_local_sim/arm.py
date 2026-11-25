"""Arm actuation: four corner arms that set chassis height, pitch, and roll.

Matches the Autonomy Test Deliverable interface. Input is height, pitch,
roll, each normalised to [-1, 1]. Output is an ArmStatus code and a message:
IDLE 0, MOVING 1, REACHED 2, FAULT 3.

Each arm is a hinge at a chassis corner with its drive cylinder at the far
end. Lowering a cylinder by d lifts that corner by d. So a chassis pose is a
lift per corner, and each lift is one arm angle:

    lift_i = L * clip(height - front_i * pitch + left_i * roll, -1, 1)
    angle_i = -asin(lift_i / L)        (positive angle = cylinder up)

front_i and left_i are +1 or -1. Pitch follows ROS REP 103 (positive is nose
down), roll too (positive is left side up). height +1 is all arms 90 degrees
down (chassis highest), -1 is all arms 90 degrees up (chassis on its belly).
Gains and tolerances are ASSUMED.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco

ARM_LENGTH_M = 0.243
# Joint name, front (+1) or rear (-1), left (+1) or right (-1).
ARMS: tuple[tuple[str, int, int], ...] = (
    ("arm_front_left", 1, 1),
    ("arm_front_right", 1, -1),
    ("arm_rear_left", -1, 1),
    ("arm_rear_right", -1, -1),
)
ARM_JOINTS = tuple(name for name, _f, _l in ARMS)
AXES = ("height", "pitch", "roll")

IDLE, MOVING, REACHED, FAULT = 0, 1, 2, 3
STATUS_NAMES = {IDLE: "IDLE", MOVING: "MOVING", REACHED: "REACHED", FAULT: "FAULT"}

# Setpoints move toward the target at this rate. A position servo jumping
# straight to a far target flings the chassis over. ASSUMED.
ARM_RATE_RAD_S = 1.0
POS_TOL_RAD = 0.04
VEL_TOL = 0.08
TIMEOUT_S = 4.0


def arm_angles(height: float, pitch: float, roll: float) -> dict[str, float]:
    """Normalised pose to the four arm angles, radians, positive is cylinder up."""
    out = {}
    for name, front, left in ARMS:
        share = max(-1.0, min(1.0, float(height) - front * float(pitch) + left * float(roll)))
        out[name] = -math.asin(share)
    return out


@dataclass
class ArmStatus:
    status: int
    message: str

    @property
    def name(self) -> str:
        return STATUS_NAMES[self.status]


class ArmController:
    def __init__(self, model: mujoco.MjModel) -> None:
        self._qpos = {j: int(model.joint(j).qposadr[0]) for j in ARM_JOINTS}
        self._dof = {j: int(model.joint(j).dofadr[0]) for j in ARM_JOINTS}
        self._act = {j: int(model.actuator(j).id) for j in ARM_JOINTS}
        self.reset()

    @classmethod
    def from_model(cls, model: mujoco.MjModel) -> "ArmController | None":
        try:
            return cls(model)
        except KeyError:
            return None

    def reset(self) -> None:
        self.targets = {j: 0.0 for j in ARM_JOINTS}
        self.setpoints = {j: 0.0 for j in ARM_JOINTS}
        self.commanded_at: float | None = None
        self.normalised = {a: 0.0 for a in AXES}

    def command(self, height: float, pitch: float, roll: float, sim_time: float) -> dict[str, float]:
        self.normalised = {"height": float(height), "pitch": float(pitch), "roll": float(roll)}
        self.targets = arm_angles(height, pitch, roll)
        self.commanded_at = float(sim_time)
        return dict(self.targets)

    def apply(self, data: mujoco.MjData, dt: float) -> None:
        step = ARM_RATE_RAD_S * max(float(dt), 0.0)
        for joint, target in self.targets.items():
            current = self.setpoints[joint]
            self.setpoints[joint] = current + max(-step, min(step, target - current))
            data.ctrl[self._act[joint]] = self.setpoints[joint]

    def joint_positions(self, data: mujoco.MjData) -> dict[str, float]:
        return {j: float(data.qpos[adr]) for j, adr in self._qpos.items()}

    def status(self, data: mujoco.MjData) -> ArmStatus:
        if self.commanded_at is None:
            return ArmStatus(IDLE, "no command yet")
        errors = {j: abs(float(data.qpos[self._qpos[j]]) - self.targets[j]) for j in ARM_JOINTS}
        still = all(abs(float(data.qvel[self._dof[j]])) < VEL_TOL for j in ARM_JOINTS)
        if all(e <= POS_TOL_RAD for e in errors.values()) and still:
            return ArmStatus(REACHED, "at target")
        # The ramp alone can take up to pi seconds; time out only after it.
        limit = TIMEOUT_S + math.pi / ARM_RATE_RAD_S
        if float(data.time) - self.commanded_at > limit:
            worst = max(errors, key=lambda j: errors[j])
            return ArmStatus(FAULT, f"{worst} is {math.degrees(errors[worst]):.0f} deg off after {limit:.0f} s")
        return ArmStatus(MOVING, "moving to target")
