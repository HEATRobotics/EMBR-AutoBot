"""World-backed temperature field and lumped chassis heating.

Not FDS, not a radiometric FLIR, not a certified withstand model.
Peak temperatures in YAML are ASSUMED. Missing component ratings stay UNKNOWN;
tests must not fail a made-up °C limit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


AMBIENT_C_ASSUMED = 20.0
COMPONENT_RATING_C = "UNKNOWN"


@dataclass
class HeatSource:
    x: float
    y: float
    z: float
    peak_c: float  # additional °C above ambient at the center; ASSUMED
    sigma: float  # Gaussian spatial scale, metres; ASSUMED
    kind: str = "smolder"  # flame | smolder | ember | cooling
    grow_s: float = 0.0
    hold_s: float = 1.0e9
    decay_s: float = 30.0
    provenance: str = "assumed"


def _peak_scale(src: HeatSource, sim_time: float) -> float:
    t = max(0.0, sim_time)
    if src.grow_s > 0.0 and t < src.grow_s:
        return t / src.grow_s
    t_hold_end = src.grow_s + src.hold_s
    if t <= t_hold_end:
        return 1.0
    if src.decay_s <= 0.0:
        return 0.0
    return math.exp(-(t - t_hold_end) / src.decay_s)


@dataclass
class ThermalLump:
    name: str
    temperature_c: float = AMBIENT_C_ASSUMED
    # First-order coupling. ASSUMED; not a measured housing model.
    absorb: float = 0.35
    cool: float = 0.12
    rating_c: str = COMPONENT_RATING_C


@dataclass
class ThermalField:
    ambient_c: float = AMBIENT_C_ASSUMED
    sources: list[HeatSource] = field(default_factory=list)
    lumps: dict[str, ThermalLump] = field(default_factory=dict)
    noise_std: float = 0.0
    rng: np.random.Generator = field(default_factory=lambda: np.random.default_rng(0))

    def __post_init__(self) -> None:
        if not self.lumps:
            self.lumps = {
                "electronics": ThermalLump("electronics"),
                "battery": ThermalLump("battery"),
                "motors": ThermalLump("motors", absorb=0.28, cool=0.18),
                "camera": ThermalLump("camera", absorb=0.22, cool=0.15),
            }

    def reset(self, seed: int | None = None) -> None:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        for lump in self.lumps.values():
            lump.temperature_c = self.ambient_c

    def temperature_c(self, pos: tuple[float, float, float] | np.ndarray, sim_time: float) -> float:
        """Analytic field T = ambient + Σ peak(t) exp(-r² / (2σ²))."""
        x, y, z = (float(pos[0]), float(pos[1]), float(pos[2]))
        extra = 0.0
        for src in self.sources:
            scale = _peak_scale(src, sim_time)
            dx = x - src.x
            dy = y - src.y
            dz = z - src.z
            r2 = dx * dx + dy * dy + dz * dz
            extra += src.peak_c * scale * math.exp(-r2 / (2.0 * src.sigma * src.sigma))
        t = self.ambient_c + extra
        if self.noise_std > 0.0:
            t += float(self.rng.normal(0.0, self.noise_std))
        return t

    def step_lumps(self, body_pos: tuple[float, float, float], sim_time: float, dt: float) -> None:
        field_t = self.temperature_c(body_pos, sim_time)
        for lump in self.lumps.values():
            heating = lump.absorb * (field_t - lump.temperature_c)
            cooling = lump.cool * (lump.temperature_c - self.ambient_c)
            lump.temperature_c += dt * (heating - cooling)

    def lump_snapshot(self) -> dict[str, dict[str, str | float]]:
        out: dict[str, dict[str, str | float]] = {}
        for name, lump in self.lumps.items():
            out[name] = {
                "temperature_c": lump.temperature_c,
                "rating_c": lump.rating_c,
            }
        return out


def sources_from_world(raw: list[dict] | None) -> list[HeatSource]:
    sources: list[HeatSource] = []
    for item in raw or []:
        sources.append(
            HeatSource(
                x=float(item.get("x", 0.0)),
                y=float(item.get("y", 0.0)),
                z=float(item.get("z", 0.02)),
                peak_c=float(item.get("peak_c", 80.0)),
                sigma=float(item.get("sigma", 0.18)),
                kind=str(item.get("kind", "smolder")),
                grow_s=float(item.get("grow_s", 0.0)),
                hold_s=float(item.get("hold_s", 1.0e9)),
                decay_s=float(item.get("decay_s", 30.0)),
                provenance=str(item.get("provenance", "assumed")),
            )
        )
    return sources
