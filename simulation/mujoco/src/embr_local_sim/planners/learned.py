"""Empty slot for a model a teammate trains outside this package.

This file does not import a learning library and does not load a checkpoint.
Subclass `LearnedSteerPlanner` and implement `act`, then pass an instance as
`Simulator.steer_planner` before `start_mission`. Manual WASD does not call it.

`act` receives sensor fields only:

- `ctx.view.lidar_ranges`, `ctx.view.lidar_angles` (radians, positive is left)
- `ctx.view.thermal_c`, `ctx.view.thermal_angles`
- `ctx.view.probe_c`, `ctx.view.odom_x`, `ctx.view.odom_y`, `ctx.view.odom_yaw`
- `ctx.goal_heading` (radians, positive is left, zero is straight ahead)

Return `(velocity, turn)` in [-1, 1]. Positive turn is right. Those sticks
are the same `TeleCmd` the keyboard uses. They are not metres per second.
"""

from __future__ import annotations

from embr_local_sim.planners.base import SteerContext
from embr_local_sim.tele_cmd import TeleCmd


class LearnedSteerPlanner:
    """Raises until `act` is replaced with a real model."""

    name = "learned"

    def act(self, ctx: SteerContext) -> tuple[float, float]:
        raise NotImplementedError(
            "LearnedSteerPlanner.act is the training slot. "
            "Return (velocity, turn) in [-1, 1]. +turn is right. "
            "See the module docstring for the sensor fields on SteerContext."
        )

    def steer(self, ctx: SteerContext) -> tuple[TeleCmd, str] | None:
        velocity, turn = self.act(ctx)
        velocity = max(-1.0, min(1.0, float(velocity)))
        turn = max(-1.0, min(1.0, float(turn)))
        return TeleCmd(velocity, turn, ctx.wall, source="mission"), self.name
