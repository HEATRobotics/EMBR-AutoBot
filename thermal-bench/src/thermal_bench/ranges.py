"""Distance clouds the lab can swap without touching the thermal frame.

`phone` is a forward depth grid, about the reach of a phone lidar.
`unitree-l2` is the roof dome: 360° around, 90° up, blind inside 5 cm.
Both return metres in the world, plus a reflectance channel that is not Celsius.

A real scan dropped in `inbox/phone.npy` or `inbox/unitree-l2.npy` replaces the
rays. The file is Nx3 metres in the sensor frame, +x forward and +z up.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

PHONE = "phone"
UNITREE_L2 = "unitree-l2"

PHONE_MAX_M = 5.0
PHONE_MIN_M = 0.15
L2_MAX_M = 15.0
L2_MIN_M = 0.05


@dataclass(frozen=True)
class RangeCloud:
    name: str
    xyz: np.ndarray
    ranges_m: np.ndarray
    reflectance: np.ndarray
    forward_m: float

    @property
    def nearest_m(self) -> float:
        if self.ranges_m.size == 0:
            return float("inf")
        return float(np.min(self.ranges_m))


def inbox_cloud(name: str, folder: Path | None = None) -> np.ndarray | None:
    """Nx3 sensor-frame metres, or None when Tuesday's file is not here yet."""
    root = Path(folder) if folder is not None else Path("inbox")
    path = root / f"{name}.npy"
    if not path.is_file():
        return None
    arr = np.load(path)
    if arr.ndim != 2 or arr.shape[1] != 3:
        return None
    return np.asarray(arr, dtype=np.float64)


def _phone_dirs() -> np.ndarray:
    hfov, vfov = math.radians(60.0), math.radians(45.0)
    us = np.linspace(-math.tan(hfov / 2), math.tan(hfov / 2), 28)
    vs = np.linspace(-math.tan(vfov / 2), math.tan(vfov / 2), 18)
    dirs = np.array([[1.0, 0.0, 0.0]] + [[1.0, u, v] for v in vs for u in us], dtype=np.float64)
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    return dirs


def _l2_dirs() -> np.ndarray:
    az = np.linspace(0.0, 2.0 * math.pi, 48, endpoint=False)
    el = np.linspace(0.0, math.pi / 2, 8)
    dirs = []
    for e in el:
        ce, se = math.cos(e), math.sin(e)
        for a in az:
            dirs.append((ce * math.cos(a), ce * math.sin(a), se))
    return np.asarray(dirs, dtype=np.float64)


def _cast(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    origin: np.ndarray,
    directions: np.ndarray,
    *,
    min_m: float,
    max_m: float,
    bodyexclude: int,
) -> tuple[np.ndarray, np.ndarray]:
    geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
    geomid = np.zeros(1, dtype=np.int32)
    pts: list[np.ndarray] = []
    ranges: list[float] = []
    for direction in directions:
        geomid[0] = -1
        dist = float(mujoco.mj_ray(model, data, origin, direction, geomgroup, 1, bodyexclude, geomid))
        if not math.isfinite(dist) or dist < min_m or dist > max_m:
            continue
        pts.append(origin + direction * dist)
        ranges.append(dist)
    if not pts:
        return np.zeros((0, 3)), np.zeros((0,))
    return np.vstack(pts), np.asarray(ranges, dtype=np.float64)


def _to_world(local: np.ndarray, origin: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    return local @ rotation.T + origin


def scan(
    kind: str,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    site: str,
    inbox: Path | None = None,
) -> RangeCloud:
    """Cast `kind` from a MuJoCo site, or load that site's inbox cloud."""
    if kind not in (PHONE, UNITREE_L2):
        raise ValueError(f"Unknown range sensor {kind!r}. Use {PHONE!r} or {UNITREE_L2!r}.")
    sid = int(model.site(site).id)
    origin = np.array(data.site_xpos[sid], dtype=np.float64)
    rotation = np.array(data.site_xmat[sid], dtype=np.float64).reshape(3, 3)
    bodyexclude = int(model.site_bodyid[sid])
    loaded = inbox_cloud(kind, inbox)
    if loaded is not None:
        xyz = _to_world(loaded, origin, rotation)
        ranges = np.linalg.norm(loaded, axis=1)
        ahead = loaded[:, 0] > 0
        forward = float(np.min(loaded[ahead, 0])) if np.any(ahead) else float("inf")
        return RangeCloud(kind, xyz, ranges, np.full(len(ranges), 0.5), forward)
    if kind == PHONE:
        local = _phone_dirs()
        min_m, max_m = PHONE_MIN_M, PHONE_MAX_M
    else:
        local = _l2_dirs()
        min_m, max_m = L2_MIN_M, L2_MAX_M
    world_dirs = local @ rotation.T
    xyz, ranges = _cast(
        model, data, origin, world_dirs, min_m=min_m, max_m=max_m, bodyexclude=bodyexclude
    )
    forward_dir = rotation @ np.array([1.0, 0.0, 0.0])
    hit, hit_range = _cast(
        model,
        data,
        origin,
        forward_dir.reshape(1, 3),
        min_m=min_m,
        max_m=max_m,
        bodyexclude=bodyexclude,
    )
    forward = float(hit_range[0]) if hit_range.size else float("inf")
    return RangeCloud(kind, xyz, ranges, np.full(ranges.shape, 0.35), forward)
