"""160×120 thermal image for the table bench.

This is the frame shape in GitHub issue 70 (FLIR Lepton 3.1 virtual twin).
The numbers are the same kind of assumed Gaussian the rover sim already uses.
They are not a reading from the PureThermal board.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

HEIGHT = 120
WIDTH = 160
AMBIENT_C = 20.0
# Square 12 µm pixels: vertical field follows the horizontal field.
# 57° is the Lepton 3 / 3.0 lens. 95° is the Lepton 3.1R lens.
HFOV_IMAGING_DEG = 57.0
HFOV_3_1R_DEG = 95.0


@dataclass(frozen=True)
class HeatBlob:
    """A hot point in the room. peak_c is extra degrees above ambient."""

    x: float
    y: float
    z: float
    peak_c: float
    sigma_m: float


def vertical_fov_rad(hfov_deg: float, height: int = HEIGHT, width: int = WIDTH) -> float:
    half = np.tan(np.deg2rad(hfov_deg) / 2.0) * (height / width)
    return float(2.0 * np.arctan(half))


def render_frame(
    cam_pos: tuple[float, float, float],
    *,
    yaw: float = 0.0,
    pitch: float = 0.0,
    hfov_deg: float = HFOV_IMAGING_DEG,
    sources: list[HeatBlob],
    ambient_c: float = AMBIENT_C,
    max_range_m: float = 1.4,
    n_steps: int = 80,
) -> np.ndarray:
    """Hottest assumed °C along each pixel ray. Shape (120, 160).

    Yaw 0 looks along +x. Pitch 0 is level. Positive yaw turns left.
    """
    hfov = np.deg2rad(hfov_deg)
    vfov = vertical_fov_rad(hfov_deg)
    u = ((np.arange(WIDTH) + 0.5) / WIDTH) * 2.0 - 1.0
    v = ((np.arange(HEIGHT) + 0.5) / HEIGHT) * 2.0 - 1.0
    uu, vv = np.meshgrid(u, v)
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    forward = np.array([cp * cy, cp * sy, sp], dtype=float)
    right = np.array([-sy, cy, 0.0], dtype=float)
    up = np.cross(right, forward)
    dirs = (
        forward
        + uu[..., None] * np.tan(hfov / 2.0) * right
        - vv[..., None] * np.tan(vfov / 2.0) * up
    )
    dirs /= np.linalg.norm(dirs, axis=-1, keepdims=True)
    ts = np.linspace(0.03, max_range_m, n_steps)
    pts = np.asarray(cam_pos, dtype=float) + ts[:, None, None, None] * dirs[None, ...]
    temp = np.full(pts.shape[:-1], ambient_c, dtype=float)
    for src in sources:
        delta = pts - np.array([src.x, src.y, src.z], dtype=float)
        r2 = np.sum(delta * delta, axis=-1)
        sig2 = src.sigma_m * src.sigma_m
        temp += src.peak_c * np.exp(-r2 / (2.0 * sig2))
    return temp.max(axis=0)


# Picture scale: room temperature is 0, the Lepton high-gain top (140 °C) is 1.
# The same Celsius value keeps the same color when the phone moves.
DISPLAY_SPAN_C = 120.0


def normalize_unit(frame_c: np.ndarray) -> np.ndarray:
    """Stretch this one frame to 0..1. Display only. Do not train on this."""
    lo = float(frame_c.min())
    hi = float(frame_c.max())
    span = hi - lo
    if span < 1e-6:
        return np.zeros_like(frame_c)
    return (frame_c - lo) / span


def sample_temperature(
    points: np.ndarray,
    sources: list[HeatBlob],
    ambient_c: float = AMBIENT_C,
) -> np.ndarray:
    """Celsius at each 3D point. Same Gaussian the thermal frame uses."""
    pts = np.asarray(points, dtype=float)
    temp = np.full(pts.shape[:-1], ambient_c, dtype=float)
    for src in sources:
        delta = pts - np.array([src.x, src.y, src.z], dtype=float)
        r2 = np.sum(delta * delta, axis=-1)
        temp = temp + src.peak_c * np.exp(-r2 / (2.0 * src.sigma_m * src.sigma_m))
    return temp


def normalize_fixed(
    frame_c: np.ndarray,
    ambient_c: float = AMBIENT_C,
    span_c: float = DISPLAY_SPAN_C,
) -> np.ndarray:
    """Degrees above the room, divided by a fixed span, clipped to 0..1."""
    return np.clip((frame_c - ambient_c) / span_c, 0.0, 1.0)


def to_tlinear_counts(frame_c: np.ndarray) -> np.ndarray:
    """Kelvin × 100, the Lepton 3.1R / 3.5 TLinear packing.

    Use this only as a label for the simulated field. A PureThermal color
    frame is AGC and is not this packing. A non-radiometric Lepton 3 has
    no published conversion from raw counts to Celsius.
    """
    counts = np.rint((frame_c + 273.15) * 100.0)
    return np.clip(counts, 0, 65535).astype(np.uint16)


def colorize(frame_c: np.ndarray) -> np.ndarray:
    """uint8 RGB. Room temperature stays dark. 140 °C is full brightness."""
    x = normalize_fixed(frame_c)
    r = np.clip(1.6 * x, 0.0, 1.0)
    g = np.clip(1.8 * x - 0.45, 0.0, 1.0)
    b = np.clip(0.55 - 1.4 * x, 0.0, 1.0) + np.clip(x - 0.75, 0.0, 1.0)
    rgb = np.stack([r, g, b], axis=-1)
    return (rgb * 255.0).astype(np.uint8)


def hot_pixel_count(frame_c: np.ndarray, above_c: float) -> int:
    return int(np.count_nonzero(frame_c >= above_c))
