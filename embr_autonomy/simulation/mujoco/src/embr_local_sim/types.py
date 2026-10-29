"""Shared command and state types. Ground truth is never treated as odometry."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping


WHEEL_JOINTS: tuple[str, ...] = (
    "front_left_maxon",
    "front_right_maxon",
    "rear_left_maxon",
    "rear_right_maxon",
)
LEFT_WHEELS: tuple[str, ...] = ("front_left_maxon", "rear_left_maxon")
RIGHT_WHEELS: tuple[str, ...] = ("front_right_maxon", "rear_right_maxon")

# Drive frame: +X forward, +Y left, +Z up. Base body is named `root` to match
# the EMBR simple URDF (do not silently rename it to base_link).
ROOT_BODY = "root"


@dataclass(frozen=True)
class DriveCommand:
    """Operator command. UI threads enqueue this; they never write poses."""

    vx: float  # m/s, +X forward in the body/drive frame
    wz: float  # rad/s, +Z left/CCW yaw
    timestamp_wall: float
    source: str
    timestamp_sim: float | None = None


@dataclass
class ContactHit:
    geom1: str
    geom2: str
    dist: float  # MuJoCo: negative is penetration
    pos: tuple[float, float, float]


@dataclass
class GroundTruthState:
    """Simulator truth. Navigation must not secretly consume this as odometry."""

    sim_time: float
    wall_time: float
    root_pos: tuple[float, float, float]
    root_quat_wxyz: tuple[float, float, float, float]
    root_linvel_world: tuple[float, float, float]
    root_angvel_body: tuple[float, float, float]
    yaw: float
    joint_pos: dict[str, float] = field(default_factory=dict)
    joint_vel: dict[str, float] = field(default_factory=dict)
    commanded_wheel_vel: dict[str, float] = field(default_factory=dict)
    applied_torque: dict[str, float] = field(default_factory=dict)
    contacts: list[ContactHit] = field(default_factory=list)
    ncon: int = 0
    kinetic_energy: float = 0.0
    potential_energy: float = 0.0
    watchdog_active: bool = False
    watchdog_age_wall: float = 0.0
    paused: bool = False
    target_vx: float = 0.0
    target_wz: float = 0.0
    actual_vx_body: float = 0.0
    actual_wz: float = 0.0
    real_time_factor: float = 0.0
    physics_step_latency_s: float = 0.0
    saturated: bool = False
    command_source: str = "none"
    roll: float = 0.0
    pitch: float = 0.0
    supervisor_mode: str = "n/a"
    supervisor_reason: str = ""
    art_angles: dict[str, float] = field(default_factory=dict)


@dataclass
class SensorObservations:
    """Synthetic measurements. Separate from GroundTruthState on purpose."""

    sim_time: float
    stamp_wall: float
    encoder_rad: dict[str, float] = field(default_factory=dict)
    encoder_vel: dict[str, float] = field(default_factory=dict)
    imu_quat_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    imu_angvel_body: tuple[float, float, float] = (0.0, 0.0, 0.0)
    imu_specific_force_body: tuple[float, float, float] = (0.0, 0.0, 9.81)
    wheel_odom_x: float = 0.0
    wheel_odom_y: float = 0.0
    wheel_odom_yaw: float = 0.0
    lidar_frame: str = "lidar"
    lidar_ranges: list[float] = field(default_factory=list)
    lidar_angles: list[float] = field(default_factory=list)
    lidar_hit_x: list[float] = field(default_factory=list)
    lidar_hit_y: list[float] = field(default_factory=list)
    lidar_hit_z: list[float] = field(default_factory=list)
    lidar_max_range: float = 2.0
    lidar_stamp_sim: float = 0.0
    thermal_frame: str = "thermal"
    thermal_c: list[float] = field(default_factory=list)
    thermal_angles: list[float] = field(default_factory=list)
    probe_c: float = 20.0
    probe_stamp_sim: float = 0.0
    probe_frame: str = "probe"
    probe_kind: str = "hinge_boom_local_proposal"
    probe_state: str = "STOWED"
    probe_angle: float = 0.0
    probe_valid: bool = True
    lidar_valid: bool = True
    ambient_c: float = 20.0
    lump_c: dict[str, float] = field(default_factory=dict)
    lump_rating_c: dict[str, str] = field(default_factory=dict)


def mapping_finite(values: Mapping[str, float]) -> bool:
    return all(v == v and abs(v) != float("inf") for v in values.values())
