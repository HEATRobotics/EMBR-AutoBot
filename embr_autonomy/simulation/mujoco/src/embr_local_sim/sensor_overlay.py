"""Draw what autonomy sees: lidar beams, the VFH histogram, and the thermal bins.

3D overlays sit in the scene around the robot. Two small images sit in the
GUI panel: the thermal strip (a 1 x N "camera" frame) and the VFH bars.
Everything here reads observations and planner memory. Nothing feeds back
into control.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

# Ironbow-style stops: cold black-purple to hot yellow-white.
_HEAT_STOPS = np.array(
    [[0, 0, 20], [90, 0, 140], [210, 40, 60], [255, 150, 0], [255, 255, 200]],
    dtype=np.float64,
)
# Open green, half-blocked yellow, blocked red.
_DENSITY_STOPS = np.array([[40, 200, 90], [240, 210, 40], [220, 40, 40]], dtype=np.float64)


def _ramp(stops: np.ndarray, t: np.ndarray) -> np.ndarray:
    t = np.clip(np.asarray(t, dtype=np.float64), 0.0, 1.0)
    xs = np.linspace(0.0, 1.0, len(stops))
    return np.stack([np.interp(t, xs, stops[:, c]) for c in range(3)], axis=-1).astype(np.uint8)


def heat_rgb(temps_c: list[float] | np.ndarray, lo_c: float, hi_c: float) -> np.ndarray:
    span = max(hi_c - lo_c, 1e-6)
    return _ramp(_HEAT_STOPS, (np.asarray(temps_c, dtype=np.float64) - lo_c) / span)


def density_rgb(density: list[float] | np.ndarray) -> np.ndarray:
    return _ramp(_DENSITY_STOPS, np.asarray(density, dtype=np.float64))


def _fan(origin: np.ndarray, xmat: np.ndarray, angles: list[float], lengths: list[float]) -> np.ndarray:
    """(N, 2, 3) segments from origin along body-frame headings."""
    local = np.stack([np.cos(angles), np.sin(angles), np.zeros(len(angles))], axis=1)
    ends = origin + (local * np.asarray(lengths)[:, None]) @ xmat.T
    return np.stack([np.tile(origin, (len(angles), 1)), ends], axis=1)


def lidar_beams(origin: np.ndarray, obs: Any) -> tuple[np.ndarray, np.ndarray] | None:
    """Origin-to-hit segments. Cyan hit something, grey reached max range, orange hit the robot itself."""
    if not obs.lidar_hit_x:
        return None
    hits = np.stack([obs.lidar_hit_x, obs.lidar_hit_y, obs.lidar_hit_z], axis=1).astype(np.float64)
    ranges = np.asarray(obs.lidar_ranges, dtype=np.float64)
    occluded = np.asarray(getattr(obs, "lidar_occluded", None) or [False] * len(hits), dtype=bool)
    segs = np.stack([np.tile(origin, (len(hits), 1)), hits], axis=1)
    max_range = float(getattr(obs, "lidar_max_range", 0.0) or max(obs.lidar_ranges))
    rgb = np.where((ranges < 0.98 * max_range)[:, None], [[80, 220, 255]], [[110, 110, 110]])
    rgb = np.where(occluded[:, None], [[255, 140, 30]], rgb).astype(np.uint8)
    return segs, np.repeat(rgb[:, None, :], 2, axis=1)


def vfh_fan(origin: np.ndarray, xmat: np.ndarray, debug: dict, length: float) -> tuple[np.ndarray, np.ndarray]:
    """One spoke per sector (long = open, short = blocked), then goal (blue) and chosen heading (white)."""
    centers = list(debug["centers"])
    density = np.asarray(debug["density"], dtype=np.float64)
    lengths = list(length * (1.0 - 0.8 * density))
    angles = centers + [float(debug["goal"])]
    lengths += [length * 1.15]
    rgb = list(density_rgb(density)) + [np.array([60, 120, 255], dtype=np.uint8)]
    if debug.get("heading") is not None:
        angles.append(float(debug["heading"]))
        lengths.append(length * 1.3)
        rgb.append(np.array([255, 255, 255], dtype=np.uint8))
    segs = _fan(origin, xmat, angles, lengths)
    return segs, np.repeat(np.asarray(rgb, dtype=np.uint8)[:, None, :], 2, axis=1)


def thermal_points(origin: np.ndarray, xmat: np.ndarray, obs: Any, sample_range: float) -> np.ndarray:
    """Where each thermal bin samples, on the floor plane at the sample range."""
    n = len(obs.thermal_angles)
    pts = _fan(origin, xmat, list(obs.thermal_angles), [sample_range] * n)[:, 1, :]
    pts[:, 2] = 0.02
    return pts


def thermal_strip_image(obs: Any, lo_c: float, hi_c: float, cell: int = 24) -> np.ndarray:
    """The thermal bins as a camera frame. Left of the image is the robot's left."""
    temps = list(reversed(obs.thermal_c)) if obs.thermal_c else [lo_c]
    row = heat_rgb(temps, lo_c, hi_c)
    return np.repeat(np.repeat(row[None, :, :], cell, axis=0), cell, axis=1)


def vfh_bars_image(debug: dict, height: int = 72, bar: int = 14) -> np.ndarray:
    """Polar histogram unrolled: one bar per sector, tall = blocked. Left of the image is the robot's left."""
    centers = list(reversed(debug["centers"]))
    density = np.asarray(list(reversed(debug["density"])), dtype=np.float64)
    img = np.full((height, bar * len(centers), 3), 24, dtype=np.uint8)
    colors = density_rgb(density)
    for i, d in enumerate(density):
        top = int(round((1.0 - d) * (height - 4)))
        img[top:, i * bar + 1 : (i + 1) * bar - 1] = colors[i]
    width = (centers[0] - centers[-1]) or 1.0

    def _col(rad: float) -> int:
        return int(np.clip((centers[0] - rad) / width * (len(centers) - 1) * bar + 0.5 * bar, 0, img.shape[1] - 1))

    img[:, max(_col(float(debug["goal"])) - 1, 0) : _col(float(debug["goal"])) + 1] = (60, 120, 255)
    if debug.get("heading") is not None:
        c = _col(float(debug["heading"]))
        img[:, max(c - 1, 0) : c + 1] = (255, 255, 255)
    return img


class SensorOverlay:
    """Owns the viser handles. Call `sync` each render tick, `reset` on a scene rebuild."""

    def __init__(self, server: Any) -> None:
        self.server = server
        self._h: dict[str, Any] = {}
        with server.gui.add_folder("Sensor views", expand_by_default=True):
            self.show_beams = server.gui.add_checkbox("LiDAR beams", initial_value=True)
            self.show_vfh = server.gui.add_checkbox(
                "VFH sectors",
                initial_value=True,
                hint="Green open, red blocked. Blue is the goal heading, white the chosen one.",
            )
            self.show_thermal = server.gui.add_checkbox("Thermal bins", initial_value=True)
            self.thermal_label = server.gui.add_markdown("**Thermal** —")
            self.thermal_img = server.gui.add_image(np.zeros((24, 216, 3), dtype=np.uint8), label="thermal")
            self.vfh_label = server.gui.add_markdown("**VFH** — waiting for the mission")
            self.vfh_img = server.gui.add_image(np.zeros((72, 210, 3), dtype=np.uint8), label="vfh")

    def reset(self) -> None:
        for handle in self._h.values():
            try:
                handle.remove()
            except Exception:
                pass
        self._h.clear()

    def _lines(self, key: str, data: tuple[np.ndarray, np.ndarray] | None, visible: bool, thickness: float) -> None:
        handle = self._h.get(key)
        if data is None or not visible:
            if handle is not None:
                handle.visible = False
            return
        segs, colors = data
        if handle is not None and handle.points.shape == segs.shape:
            handle.points = segs.astype(np.float32)
            handle.colors = colors
            handle.visible = True
            return
        if handle is not None:
            handle.remove()
        self._h[key] = self.server.scene.add_line_segments(
            f"/sensors/{key}", points=segs.astype(np.float32), colors=colors, thickness=thickness
        )

    def sync(self, sim: Any, obs: Any) -> None:
        try:
            sid = int(sim.model.site("lidar").id)
        except Exception:
            return
        origin = np.array(sim.data.site_xpos[sid], dtype=np.float64)
        xmat = np.array(sim.data.site_xmat[sid], dtype=np.float64).reshape(3, 3)

        self._lines("lidar_beams", lidar_beams(origin, obs), bool(self.show_beams.value), 0.012)

        debug = getattr(getattr(sim, "mission", None), "_gap", {}).get("vfh_debug")
        reach = 0.35 * float(sim.params.lidar_max_range)
        fan = vfh_fan(origin + np.array([0.0, 0.0, 0.03]), xmat, debug, reach) if debug else None
        self._lines("vfh", fan, bool(self.show_vfh.value), 0.03)
        if debug:
            self.vfh_img.image = vfh_bars_image(debug)
            heading = debug.get("heading")
            chosen = "backing up" if heading is None else f"{math.degrees(heading):+.0f}°"
            self.vfh_label.content = f"**VFH** — goal `{math.degrees(debug['goal']):+.0f}°`, chosen `{chosen}`"

        if obs.thermal_c:
            lo = float(obs.ambient_c)
            hi = max(lo + 40.0, 90.0)
            self.thermal_img.image = thermal_strip_image(obs, lo, hi)
            self.thermal_label.content = (
                f"**Thermal** — max `{max(obs.thermal_c):.1f} °C`, probe `{obs.probe_c:.1f} °C` (scale {lo:.0f}–{hi:.0f})"
            )
            pts = thermal_points(origin, xmat, obs, float(sim.params.thermal_sample_range))
            handle = self._h.get("thermal")
            colors = heat_rgb(obs.thermal_c, lo, hi)
            visible = bool(self.show_thermal.value)
            if handle is None:
                self._h["thermal"] = self.server.scene.add_point_cloud(
                    "/sensors/thermal", points=pts.astype(np.float32), colors=colors, point_size=0.12
                )
            else:
                handle.points = pts.astype(np.float32)
                handle.colors = colors
                handle.visible = visible
