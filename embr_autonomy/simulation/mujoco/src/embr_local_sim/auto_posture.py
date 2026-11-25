"""Automatic arm posture while a mission drives.

Reads only deployable signals: IMU roll and pitch, wheel encoder speed, and
the mission's own jam reaction. Outputs a normalised height, pitch, roll for the arm
controller (see arm.py). Three rules:

1. Level: integrate pitch and roll commands against the measured tilt, so
   the chassis stays near level on slopes. 1 degree deadband.
2. Slope clearance: while tilted past 2.5 degrees, and a while after, lift to
   0.35 so the belly clears ramp crests and toes.
3. Unstick: when the mission starts backing up from a jam, or the wheel
   encoders show a stall (turning under a quarter of the commanded speed for
   a second), lift to 0.8 and hold it while it retries.

Cruise height on flat ground is 0. A taller body lifts the flat lidar over
low walls (the hot rooms use 0.22 m walls). All gains are ASSUMED.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class AutoPostureConfig:
    cruise_height: float = 0.0
    slope_height: float = 0.35
    stuck_height: float = 0.8
    slope_enter_rad: float = math.radians(2.5)
    slope_hold_s: float = 3.0
    stuck_hold_s: float = 6.0
    level_gain: float = 3.0  # normalised command per radian of tilt per second
    deadband_rad: float = math.radians(1.0)
    tilt_cmd_limit: float = 0.4
    stall_ratio: float = 0.25
    stall_after_s: float = 1.0


class AutoPosture:
    def __init__(self, config: AutoPostureConfig | None = None) -> None:
        self.config = config or AutoPostureConfig()
        self.enabled = True
        self.reset()

    def reset(self) -> None:
        self.height = self.config.cruise_height
        self.pitch = 0.0
        self.roll = 0.0
        self.mode = "cruise"
        self._slope_until = -math.inf
        self._stuck_until = -math.inf
        self._stall_since: float | None = None

    def stalled(self, t: float, commanded_mps: float, wheel_mps: float) -> bool:
        """Encoders turning far slower than commanded, for long enough. Real drives report both."""
        cfg = self.config
        if commanded_mps > 0.1 and wheel_mps < cfg.stall_ratio * commanded_mps:
            if self._stall_since is None:
                self._stall_since = t
            return t - self._stall_since >= cfg.stall_after_s
        self._stall_since = None
        return False

    def _level(self, measured: float, command: float, dt: float) -> float:
        cfg = self.config
        if abs(measured) <= cfg.deadband_rad:
            return command
        # + pitch command is nose down, + measured pitch is nose down: push against it.
        command -= cfg.level_gain * measured * dt
        return max(-cfg.tilt_cmd_limit, min(cfg.tilt_cmd_limit, command))

    def update(
        self,
        *,
        t: float,
        roll_rad: float,
        pitch_rad: float,
        reason: str,
        dt: float,
        commanded_mps: float = 0.0,
        wheel_mps: float = 0.0,
    ) -> tuple[float, float, float]:
        cfg = self.config
        self.pitch = self._level(pitch_rad, self.pitch, dt)
        self.roll = self._level(roll_rad, self.roll, dt)
        if max(abs(pitch_rad), abs(roll_rad)) > cfg.slope_enter_rad:
            self._slope_until = t + cfg.slope_hold_s
        if reason == "backing_up" or self.stalled(t, commanded_mps, wheel_mps):
            self._stuck_until = t + cfg.stuck_hold_s
        if t < self._stuck_until:
            self.mode, self.height = "unstick", cfg.stuck_height
        elif t < self._slope_until:
            self.mode, self.height = "slope", cfg.slope_height
        else:
            self.mode, self.height = "cruise", cfg.cruise_height
            # Flat again: bleed the levelling back to zero.
            self.pitch *= 0.97
            self.roll *= 0.97
        return self.height, self.pitch, self.roll
