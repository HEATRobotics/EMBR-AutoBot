"""Local steering used when a scan has no width, plus goto and flame avoidance.

Search and approach arcs come from `planners.SteerPlanner` (`VfhPlanner` by
default). Pass another planner into `MissionMachine` or set
`Simulator.steer_planner` before `start_mission`. A planner returns velocity
and turn. It does not write poses. Manual WASD does not call either object.
"""

from __future__ import annotations

from typing import Protocol

from embr_local_sim.tele_cmd import TeleCmd


class LocalPolicy(Protocol):
    name: str

    def search(self, *, cruise: float, front: float, slow_range: float) -> tuple[float, float]:
        """Velocity and turn while looking, before a hotspot is chosen."""

    def approach(
        self,
        *,
        cruise: float,
        bearing: float,
        front: float,
        slow_range: float,
        blocked_range: float,
    ) -> tuple[float, float]:
        """Velocity and turn while facing the hottest bin."""

    def flame(self, flame_ang: float, wall: float) -> TeleCmd:
        """Steer off a bin the mission has marked as exclusion."""

    def goto(self, view, target_x: float, target_y: float, *, arrive_m: float, wall: float, avoid: bool, cruise: float):
        """Command, arrived flag, and a short reason."""


class ClassicalLocalPolicy:
    """The steering already used by the mission. Numbers stay here, not in the state machine."""

    name = "classical"

    def search(self, *, cruise: float, front: float, slow_range: float) -> tuple[float, float]:
        velocity = cruise
        turn = 0.0
        if front < slow_range:
            velocity = 0.15
            # Obstacle on the nose: positive turn is right. Yaw left to try a gap.
            turn = -0.55
        elif front < slow_range + 0.12:
            velocity = 0.28
        return velocity, turn

    def approach(
        self,
        *,
        cruise: float,
        bearing: float,
        front: float,
        slow_range: float,
        blocked_range: float,
    ) -> tuple[float, float]:
        # Bin angles are +left. TeleCmd turn is +right, so the bearing is negated.
        aligned = abs(bearing) < 0.25
        # Already pointed at the hotspot. A small turn on a slope walks the rollers off the lane.
        turn = 0.0 if aligned else max(-1.0, min(1.0, -bearing * 1.6))
        velocity = cruise if aligned else 0.18
        if front < slow_range:
            velocity = min(velocity, 0.12)
            if front < blocked_range + 0.02:
                turn = -0.7
        return velocity, turn

    def flame(self, flame_ang: float, wall: float) -> TeleCmd:
        turn_away = max(-1.0, min(1.0, (1.0 if flame_ang >= 0 else -1.0) * 0.85))
        return TeleCmd(velocity=0.12, turn=turn_away, timestamp_wall=wall, source="mission")

    def goto(self, view, target_x: float, target_y: float, *, arrive_m: float, wall: float, avoid: bool, cruise: float):
        from embr_local_sim.autonomy import goto_command

        return goto_command(
            view, target_x, target_y, arrive_m=arrive_m, wall=wall, avoid=avoid, cruise=cruise
        )
