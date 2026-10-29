"""Typed observation / command structs for the native control loop.

RobotObservation is the deployable bundle: IMU, joints, drive request, validity.
It must not carry ground-truth chassis pose or hidden map geometry.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class RobotObservation:
    sim_time: float
    stamp_wall: float
    imu_quat_wxyz: tuple[float, float, float, float]
    imu_angvel_body: tuple[float, float, float]
    imu_specific_force_body: tuple[float, float, float]
    roll_rad: float
    pitch_rad: float
    joint_pos: dict[str, float] = field(default_factory=dict)
    joint_vel: dict[str, float] = field(default_factory=dict)
    drive_vx: float = 0.0
    drive_wz: float = 0.0
    art_pos: dict[str, float] = field(default_factory=dict)
    art_vel: dict[str, float] = field(default_factory=dict)
    imu_valid: bool = True
    joints_valid: bool = True
    command_fresh: bool = True
    watchdog_active: bool = False


@dataclass
class ControllerOutput:
    drive_vx: float
    drive_wz: float
    art_targets: dict[str, float]
    source: str
    status: str = "ok"


@dataclass
class SupervisorResult:
    mode: str
    approved_drive_vx: float
    approved_drive_wz: float
    approved_art: dict[str, float]
    reason: str
    rejected: bool = False
