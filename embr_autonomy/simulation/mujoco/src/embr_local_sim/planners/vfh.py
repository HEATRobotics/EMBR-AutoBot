"""Vector-field histogram on the live lidar fan.

Borenstein and Koren, 1991. The scan is already polar, so there is no map.
This module does not train anything. A teammate replaces the class, not
these weights, when they bring their own model.
"""

from __future__ import annotations

import math

from embr_local_sim.mission import MissionView
from embr_local_sim.planners.base import SteerContext
from embr_local_sim.tele_cmd import TeleCmd


def _wrap(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def vfh_steer(
    view: MissionView,
    goal_heading: float,
    *,
    wall: float,
    cruise: float,
    blocked_range: float = 0.16,
    half_width: float = 0.08,
    memory: dict | None = None,
) -> tuple[TeleCmd, str] | None:
    """Pick the open sector nearest `goal_heading`.

    Returns None when the fan has no angular width, so the caller keeps its
    own command. A fan with no open sector backs up. A gap that is only a
    bit tight for the body backs up on an arc toward that gap.
    """
    if not view.lidar_valid or not view.lidar_ranges or not view.lidar_angles:
        return None
    angles = [float(a) for a in view.lidar_angles]
    ranges = [float(r) for r in view.lidar_ranges]
    if max(angles) - min(angles) < 0.4:
        return None
    fov = max(abs(a) for a in angles)
    n = 15
    width = (2.0 * fov) / n
    centers = [-fov + width * (i + 0.5) for i in range(n)]
    # Each bin keeps the nearest return that actually falls in it. A short
    # return beside the nose must not paint the open center as blocked.
    density = [1.0] * n
    seen = [False] * n
    limit = max(blocked_range + 0.05, 0.2)
    span = max(0.05, 3.0 - limit)

    def _cost(rng: float) -> float:
        if not math.isfinite(rng) or rng <= 0.05:
            return 1.0
        if rng >= 3.0:
            return 0.0
        return min(1.0, (3.0 - rng) / span)

    for ang, rng in zip(angles, ranges):
        index = min(range(n), key=lambda i: abs(_wrap(ang - centers[i])))
        if abs(_wrap(ang - centers[index])) > width:
            continue
        cost = _cost(rng)
        if not seen[index] or cost > density[index]:
            density[index] = cost
            seen[index] = True
    # A near miss raises the next bin only when that bin has no ray of its own.
    owned = density[:]
    for i, value in enumerate(owned):
        if value < 0.75:
            continue
        for j in (i - 1, i + 1):
            if 0 <= j < n and not seen[j]:
                density[j] = max(density[j], 0.7)

    def _hits_body(heading: float) -> bool:
        """True when this heading would clip a return, given the body half-width."""
        for ang, rng in zip(angles, ranges):
            if not math.isfinite(rng) or rng <= 0.05 or _cost(rng) < 0.55:
                continue
            pad = math.atan2(max(half_width, 0.05), max(rng, 0.08))
            if abs(_wrap(ang - heading)) <= pad:
                return True
        return False

    goal = max(-fov, min(fov, goal_heading))
    previous_h = 0.0 if memory is None else float(memory.get("vfh", 0.0))
    best_h: float | None = None
    best_cost = 1e9
    best_d = 1.0
    for i, (center, raw) in enumerate(zip(centers, density)):
        d = raw if seen[i] or raw < 1.0 else 1.0
        if d > 0.78 or _hits_body(center):
            continue
        # Blocked sectors lose first. Then prefer the goal. The last term
        # keeps the winner from flipping between two equal gaps every tick.
        cost = 3.0 * d + 1.7 * abs(_wrap(center - goal)) + 0.45 * abs(_wrap(center - previous_h))
        if cost < best_cost - 0.05 or (
            best_h is not None and abs(cost - best_cost) <= 0.05 and abs(_wrap(center - goal)) < abs(_wrap(best_h - goal))
        ):
            best_cost = cost
            best_h = center
            best_d = d
    if best_h is None:
        fallback = min(range(n), key=lambda i: density[i])
        if density[fallback] > 0.78:
            return TeleCmd(-0.45, 0.0, wall, source="mission"), "backing_up"
        best_h = centers[fallback]
        if memory is not None:
            memory["vfh"] = best_h
        turn = max(-1.0, min(1.0, -best_h * 2.4))
        return TeleCmd(-0.40, turn, wall, source="mission"), "backing_up"
    if memory is not None:
        memory["vfh"] = best_h
    # This chassis arcs slowly. A half-stick turn reaches a trunk before the
    # body has yawed clear of it, so an off-center valley gets a fuller turn
    # and a shorter speed. +heading is left, +turn is right.
    turn = max(-1.0, min(1.0, -best_h * 2.4))
    velocity = cruise if best_d < 0.35 and abs(best_h) < 0.25 else min(cruise, 0.40)
    if abs(best_h) > 0.22:
        velocity = min(velocity, 0.28)
    if abs(best_h) > 0.45:
        velocity = min(velocity, 0.20)
    return TeleCmd(max(0.18, velocity), turn, wall, source="mission"), "vfh"


class VfhPlanner:
    """Default follow-instructions planner. No weights and no training."""

    name = "vfh"

    def steer(self, ctx: SteerContext) -> tuple[TeleCmd, str] | None:
        return vfh_steer(
            ctx.view,
            ctx.goal_heading,
            wall=ctx.wall,
            cruise=ctx.cruise,
            blocked_range=ctx.blocked_range,
            half_width=ctx.half_width,
            memory=ctx.memory,
        )
