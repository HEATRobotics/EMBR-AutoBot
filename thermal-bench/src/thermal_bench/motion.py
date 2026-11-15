"""How far the phone moved along the rail, toward the iron or away from it."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PhoneMotion:
    distance_m: float
    delta_m: float
    direction: str


class PhoneRail:
    def __init__(self) -> None:
        self._last: float | None = None

    def update(self, distance_m: float) -> PhoneMotion:
        if self._last is None:
            delta = 0.0
            direction = "still"
        else:
            delta = float(distance_m) - self._last
            if delta > 1e-4:
                direction = "farther"
            elif delta < -1e-4:
                direction = "closer"
            else:
                direction = "still"
        self._last = float(distance_m)
        return PhoneMotion(float(distance_m), delta, direction)
