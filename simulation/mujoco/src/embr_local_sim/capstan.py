"""Single-axis capstan helper equivalent.

Verified against `node_capstan_helper.py` @ 1054c0ef: uses TeleCmd.velocity,
**ignores turn**, timeout 0.5 s, one axis. This is a test-stand path, not a
four-pod controller.

Default sandbox routing does **not** feed the same TeleCmd into drive and
capstan. Sharing that topic is an integration hazard the tests document.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from embr_local_sim.tele_cmd import REPO_HELPER_COMMAND_TIMEOUT_S, TeleCmd

CAPSTAN_IGNORES_TURN = True
REPO_CAPSTAN_MAX_VELOCITY = 10.0  # helper default; not a measured rating


@dataclass
class CapstanCommand:
    level: float
    sim_velocity: float
    timestamp_wall: float
    source: str


class CapstanHelperSim:
    def __init__(self, timeout_s: float = REPO_HELPER_COMMAND_TIMEOUT_S, max_velocity: float = 1.0) -> None:
        self.timeout_s = timeout_s
        # Local experiments use a unit-scale axis. The helper's 10.0 is a ROS
        # controller unit, not a MuJoCo joint we own in this proxy.
        self.max_velocity = max_velocity
        self._level = 0.0
        self._last_wall: float | None = None

    def reset(self) -> None:
        self._level = 0.0
        self._last_wall = None

    def ingest(self, cmd: TeleCmd) -> CapstanCommand:
        if math.isfinite(cmd.velocity):
            self._level = max(-1.0, min(1.0, cmd.velocity))
        else:
            self._level = 0.0
        self._last_wall = cmd.timestamp_wall
        return self.publish(cmd.timestamp_wall, source=cmd.source)

    def publish(self, wall_now: float, source: str = "capstan") -> CapstanCommand:
        if self._last_wall is None or (wall_now - self._last_wall) > self.timeout_s:
            self._level = 0.0
        return CapstanCommand(
            level=self._level,
            sim_velocity=self._level * self.max_velocity,
            timestamp_wall=wall_now,
            source=source,
        )
