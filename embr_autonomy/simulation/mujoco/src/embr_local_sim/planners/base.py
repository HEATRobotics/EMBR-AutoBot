"""The only type the mission needs in order to steer.

A planner sees `SteerContext` and returns a `TeleCmd` plus a short reason.
It does not write poses, read ground truth, or own mission states. Manual
WASD never calls a planner. Replace `VfhPlanner` by passing another object
with the same `steer` method into `MissionMachine` or `Simulator.steer_planner`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

from embr_local_sim.mission import MissionView
from embr_local_sim.tele_cmd import TeleCmd


@dataclass
class SteerContext:
    """One control tick of sensor data, plus the heading the mission wants.

    `goal_heading` is in the lidar frame. Positive is left, zero is straight
    ahead. `cruise`, `blocked_range`, and `half_width` are body-scale numbers
    from the mission config. `memory` is a dict the planner may store its
    last choice in. The mission clears it on reset.
    """

    view: MissionView
    goal_heading: float
    wall: float
    cruise: float
    blocked_range: float
    half_width: float
    memory: dict = field(default_factory=dict)


class SteerPlanner(Protocol):
    """Swap point for follow-instructions steering.

    `steer` returns `(command, reason)` or `None`. `None` means this scan
    is not usable, and the mission falls back to `LocalPolicy`.
    """

    name: str

    def steer(self, ctx: SteerContext) -> tuple[TeleCmd, str] | None:
        """Pick a velocity and turn for this tick. +turn is right."""
