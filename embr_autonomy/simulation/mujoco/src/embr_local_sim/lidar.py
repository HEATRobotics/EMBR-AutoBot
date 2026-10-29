"""Labeled 2D LiDAR ray fan via MuJoCo `mj_ray`. Not a Velodyne driver."""

from __future__ import annotations

import math

import mujoco
import numpy as np

from embr_local_sim.model import RobotParams


def _rotate_body_to_world(quat_wxyz: np.ndarray, vec: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(v) for v in quat_wxyz)
    qvec = np.array([x, y, z], dtype=np.float64)
    uv = np.cross(qvec, vec)
    uuv = np.cross(qvec, uv)
    return vec + 2.0 * (w * uv + uuv)


class LidarFan:
    """Horizontal fan in the `lidar` site frame (body +X forward, +Y left)."""

    def __init__(self, params: RobotParams) -> None:
        self.n_rays = int(params.lidar_n_rays)
        self.fov = math.radians(params.lidar_fov_deg)
        self.max_range = float(params.lidar_max_range)
        self.min_range = float(params.lidar_min_range)
        self.site_name = "lidar"
        half = 0.5 * self.fov
        if self.n_rays == 1:
            self.angles = np.array([0.0], dtype=np.float64)
        else:
            self.angles = np.linspace(-half, half, self.n_rays)

    def scan(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, list[float] | str | float]:
        try:
            sid = int(model.site(self.site_name).id)
        except Exception:
            return {
                "frame": "lidar",
                "ranges": [self.max_range] * self.n_rays,
                "angles": [float(a) for a in self.angles],
                "hit_x": [],
                "hit_y": [],
                "hit_z": [],
                "max_range": self.max_range,
                "stamp_sim": float(data.time),
            }
        pnt = np.array(data.site_xpos[sid], dtype=np.float64)
        # xmat maps site-local vectors into the world and is stored row-major.
        xmat = np.array(data.site_xmat[sid], dtype=np.float64).reshape(3, 3)
        geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        ranges: list[float] = []
        hx: list[float] = []
        hy: list[float] = []
        hz: list[float] = []
        geomid = np.zeros(1, dtype=np.int32)
        for ang in self.angles:
            local = np.array([math.cos(ang), math.sin(ang), 0.0], dtype=np.float64)
            world_dir = xmat @ local
            geomid[0] = -1
            dist = float(
                mujoco.mj_ray(model, data, pnt, world_dir, geomgroup, 1, -1, geomid)
            )
            if not math.isfinite(dist) or dist < 0.0:
                r = self.max_range
            else:
                r = min(self.max_range, max(self.min_range, dist))
            ranges.append(r)
            hit = pnt + world_dir * r
            hx.append(float(hit[0]))
            hy.append(float(hit[1]))
            hz.append(float(hit[2]))
        return {
            "frame": "lidar",
            "ranges": ranges,
            "angles": [float(a) for a in self.angles],
            "hit_x": hx,
            "hit_y": hy,
            "hit_z": hz,
            "max_range": self.max_range,
            "stamp_sim": float(data.time),
        }
