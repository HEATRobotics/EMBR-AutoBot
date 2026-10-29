"""Synthetic sensors. Observations are not ground-truth pose."""

from __future__ import annotations

import math

import numpy as np

from embr_local_sim.lidar import LidarFan
from embr_local_sim.model import RobotParams
from embr_local_sim.thermal import ThermalField
from embr_local_sim.types import LEFT_WHEELS, RIGHT_WHEELS, WHEEL_JOINTS, GroundTruthState, SensorObservations


class SensorModel:
    """Wheel encoders + IMU + wheel odom + 2D LiDAR + world-backed thermal/probe."""

    def __init__(self, params: RobotParams, seed: int = 0) -> None:
        self.params = params
        self.rng = np.random.default_rng(seed)
        self.encoder_noise_std = 0.0
        self.imu_gyro_noise_std = 0.0
        self.imu_acc_noise_std = 0.0
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._last_enc: dict[str, float] | None = None
        self._last_time: float | None = None
        self.lidar = LidarFan(params)
        self.lidar_dropout = False
        # When set, ranges are withheld once simulation time reaches this value.
        # None leaves the scan alone. Reset clears the frozen stamp, not this arm.
        self.lidar_dropout_after: float | None = None
        self._lidar_frozen_stamp: float | None = None
        half = 0.5 * math.radians(params.thermal_fov_deg)
        n = params.thermal_n_bins
        self.thermal_angles = np.linspace(-half, half, n) if n > 1 else np.array([0.0])

    def reset(self, seed: int | None = None) -> None:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._last_enc = None
        self._last_time = None
        self.lidar_dropout = False
        self.lidar_dropout_after = None
        self._lidar_frozen_stamp = None

    def _quantize(self, rad: float) -> float:
        ticks = self.params.encoder_ticks
        step = 2.0 * math.pi / ticks
        noisy = rad + float(self.rng.normal(0.0, self.encoder_noise_std))
        return round(noisy / step) * step

    def _thermal_bins(
        self,
        gt: GroundTruthState,
        thermal: ThermalField | None,
        lidar_scan: dict,
    ) -> tuple[list[float], list[float]]:
        if thermal is None:
            amb = 20.0
            return [amb for _ in self.thermal_angles], [float(a) for a in self.thermal_angles]
        qw, qx, qy, qz = gt.root_quat_wxyz
        # Body +X/+Y in world.
        x_axis = _rotate_body_to_world(np.array([qw, qx, qy, qz]), np.array([1.0, 0.0, 0.0]))
        y_axis = _rotate_body_to_world(np.array([qw, qx, qy, qz]), np.array([0.0, 1.0, 0.0]))
        origin = np.array(gt.root_pos, dtype=np.float64)
        ranges = lidar_scan.get("ranges") or []
        angles_l = lidar_scan.get("angles") or []
        bins: list[float] = []
        for ang in self.thermal_angles:
            direction = math.cos(ang) * x_axis + math.sin(ang) * y_axis
            sample_r = self.params.thermal_sample_range
            if ranges and angles_l:
                nearest = min(range(len(angles_l)), key=lambda i: abs(angles_l[i] - float(ang)))
                sample_r = min(sample_r, float(ranges[nearest]))
            pos = origin + direction * sample_r
            bins.append(thermal.temperature_c((float(pos[0]), float(pos[1]), float(pos[2])), gt.sim_time))
        return bins, [float(a) for a in self.thermal_angles]

    def observe(
        self,
        gt: GroundTruthState,
        *,
        model=None,
        data=None,
        thermal: ThermalField | None = None,
    ) -> SensorObservations:
        enc = {name: self._quantize(gt.joint_pos[name]) for name in WHEEL_JOINTS}
        enc_vel = {name: float(gt.joint_vel[name]) for name in WHEEL_JOINTS}
        if self._last_enc is not None and self._last_time is not None:
            dt = gt.sim_time - self._last_time
            if dt > 0:
                scale = self.params.hinge_from_ground_speed * self.params.wheel_radius
                d_left = scale * 0.5 * sum(enc[n] - self._last_enc[n] for n in LEFT_WHEELS)
                d_right = scale * 0.5 * sum(enc[n] - self._last_enc[n] for n in RIGHT_WHEELS)
                ds = 0.5 * (d_left + d_right)
                # Heading uses the synthetic gyro, not the wheel difference.
                # On this skid-steer proxy the wheels scrub, so encoder yaw runs
                # ahead of the body and then keeps integrating after the body
                # has stopped turning. The gyro below is the physics body rate
                # plus the configured noise (zero unless a test sets it). It is
                # not a hardware IMU and it is not a live pose correction:
                # only this rate is integrated.
                dyaw = float(gt.root_angvel_body[2]) * dt
                yaw_mid = self._odom_yaw + 0.5 * dyaw
                self._odom_x += ds * math.cos(yaw_mid)
                self._odom_y += ds * math.sin(yaw_mid)
                self._odom_yaw += dyaw
        self._last_enc = dict(enc)
        self._last_time = gt.sim_time

        g_world = np.array([0.0, 0.0, self.params.gravity])
        # IMU frame is `root`. Specific force at rest is approximately +Z * g.
        qw, qx, qy, qz = gt.root_quat_wxyz
        q_inv = np.array([qw, -qx, -qy, -qz], dtype=np.float64)
        w, x, y, z = q_inv
        qv = np.array([x, y, z])
        uv = np.cross(qv, g_world)
        uuv = np.cross(qv, uv)
        g_body = g_world + 2.0 * (w * uv + uuv)
        specific = g_body + self.rng.normal(0.0, self.imu_acc_noise_std, size=3)
        gyro = np.array(gt.root_angvel_body) + self.rng.normal(0.0, self.imu_gyro_noise_std, size=3)

        lidar_scan: dict = {
            "frame": "lidar",
            "ranges": [],
            "angles": [],
            "hit_x": [],
            "hit_y": [],
            "hit_z": [],
            "max_range": self.params.lidar_max_range,
            "stamp_sim": gt.sim_time,
        }
        if model is not None and data is not None:
            lidar_scan = self.lidar.scan(model, data)
        lidar_valid = True
        dropout_now = self.lidar_dropout or (
            self.lidar_dropout_after is not None and gt.sim_time >= self.lidar_dropout_after
        )
        if dropout_now:
            # Withhold the scan. Do not substitute max-range, which would look free.
            lidar_valid = False
            if self._lidar_frozen_stamp is None:
                self._lidar_frozen_stamp = float(gt.sim_time)
            lidar_scan["ranges"] = []
            lidar_scan["angles"] = []
            lidar_scan["hit_x"] = []
            lidar_scan["hit_y"] = []
            lidar_scan["hit_z"] = []
            lidar_scan["stamp_sim"] = self._lidar_frozen_stamp

        thermal_c, thermal_ang = self._thermal_bins(gt, thermal, lidar_scan)
        ambient = thermal.ambient_c if thermal is not None else 20.0
        probe_c = ambient
        probe_stamp = gt.sim_time
        if thermal is not None and model is not None and data is not None:
            try:
                sid = int(model.site("probe").id)
                p = data.site_xpos[sid]
                probe_c = thermal.temperature_c((float(p[0]), float(p[1]), float(p[2])), gt.sim_time)
            except Exception:
                probe_c = thermal.temperature_c(gt.root_pos, gt.sim_time)
        lumps = thermal.lump_snapshot() if thermal is not None else {}

        return SensorObservations(
            sim_time=gt.sim_time,
            stamp_wall=gt.wall_time,
            encoder_rad=enc,
            encoder_vel=enc_vel,
            imu_quat_wxyz=gt.root_quat_wxyz,
            imu_angvel_body=(float(gyro[0]), float(gyro[1]), float(gyro[2])),
            imu_specific_force_body=(float(specific[0]), float(specific[1]), float(specific[2])),
            wheel_odom_x=self._odom_x,
            wheel_odom_y=self._odom_y,
            wheel_odom_yaw=self._odom_yaw,
            lidar_frame=str(lidar_scan.get("frame", "lidar")),
            lidar_ranges=list(lidar_scan.get("ranges") or []),
            lidar_angles=list(lidar_scan.get("angles") or []),
            lidar_hit_x=list(lidar_scan.get("hit_x") or []),
            lidar_hit_y=list(lidar_scan.get("hit_y") or []),
            lidar_hit_z=list(lidar_scan.get("hit_z") or []),
            lidar_max_range=float(lidar_scan.get("max_range", self.params.lidar_max_range)),
            lidar_stamp_sim=float(lidar_scan.get("stamp_sim", gt.sim_time)),
            thermal_frame="thermal",
            thermal_c=thermal_c,
            thermal_angles=thermal_ang,
            probe_c=probe_c,
            probe_stamp_sim=probe_stamp,
            probe_frame="probe",
            probe_kind="hinge_boom_local_proposal",
            lidar_valid=lidar_valid,
            ambient_c=ambient,
            lump_c={k: float(v["temperature_c"]) for k, v in lumps.items()},
            lump_rating_c={k: str(v["rating_c"]) for k, v in lumps.items()},
        )


def _rotate_body_to_world(quat_wxyz: np.ndarray, vec: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(v) for v in quat_wxyz)
    qvec = np.array([x, y, z], dtype=np.float64)
    uv = np.cross(qvec, vec)
    uuv = np.cross(qvec, uv)
    return vec + 2.0 * (w * uv + uuv)
