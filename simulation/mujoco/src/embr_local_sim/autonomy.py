"""Conventional autonomy helpers. No hidden simulator ground-truth pose.

Levels (prompt §9):
  1 manual + supervisor
  2 goto an operator-selected odom location
  3 avoid / replan from the 2D lidar fan
  4 probe inspection + return (MissionMachine)
  5 bounded search from odom + thermal bins

A collision-free 2D path is not a guarantee that rollers can climb the terrain.
Flame exclusion uses thermal bins (sensor) unless an explicit labeled oracle map
is enabled.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from embr_local_sim.mission import MissionView
from embr_local_sim.tele_cmd import TeleCmd


def wrap_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def heading_turn(view: MissionView, target_x: float, target_y: float) -> tuple[float, float, float]:
    """Return (distance_m, heading_error_rad, tele_turn). +turn is right."""
    dx = target_x - view.odom_x
    dy = target_y - view.odom_y
    dist = math.hypot(dx, dy)
    bearing = math.atan2(dy, dx)
    err = wrap_angle(bearing - view.odom_yaw)
    turn = max(-1.0, min(1.0, -err * 1.4))
    return dist, err, turn


def lidar_gap_turn(view: MissionView) -> float:
    """Steer toward the longest lidar ray. +turn right."""
    if not view.lidar_ranges or not view.lidar_angles:
        return 0.0
    idx = max(range(len(view.lidar_ranges)), key=lambda i: view.lidar_ranges[i])
    ang = view.lidar_angles[idx]
    # LiDAR +ang is left in the body frame; TeleCmd.turn is +right.
    return max(-1.0, min(1.0, -ang * 1.3))


def front_range(view: MissionView) -> float:
    if not view.lidar_ranges:
        return view.lidar_max_range
    n = len(view.lidar_ranges)
    lo = max(0, n // 2 - 2)
    hi = min(n, n // 2 + 3)
    return min(view.lidar_ranges[lo:hi])


def hottest_bearing(view: MissionView) -> tuple[float, float]:
    if not view.thermal_c:
        return view.ambient_c, 0.0
    idx = max(range(len(view.thermal_c)), key=lambda i: view.thermal_c[i])
    ang = view.thermal_angles[idx] if idx < len(view.thermal_angles) else 0.0
    return view.thermal_c[idx], ang


def flame_bearing(view: MissionView, flame_c: float) -> float | None:
    """Sensor-only: bearing of a bin hotter than the demonstration flame threshold."""
    if not view.thermal_c:
        return None
    hot = [(c, a) for c, a in zip(view.thermal_c, view.thermal_angles) if c >= flame_c]
    if not hot:
        return None
    _c, ang = max(hot, key=lambda t: t[0])
    return ang


@dataclass
class ProgressWatch:
    last_x: float = 0.0
    last_y: float = 0.0
    last_t: float = 0.0
    stalled_s: float = 0.0

    def update(self, view: MissionView, commanded: bool, thresh_s: float) -> bool:
        dx = view.odom_x - self.last_x
        dy = view.odom_y - self.last_y
        dt = max(1e-3, view.sim_time - self.last_t) if self.last_t else 0.02
        if self.last_t == 0.0:
            self.last_x, self.last_y, self.last_t = view.odom_x, view.odom_y, view.sim_time
            return False
        moved = math.hypot(dx, dy)
        if commanded and moved < 0.008:
            self.stalled_s += dt
        else:
            self.stalled_s = 0.0
        self.last_x, self.last_y, self.last_t = view.odom_x, view.odom_y, view.sim_time
        return self.stalled_s >= thresh_s


def lawnmower_target(view: MissionView, bound: dict[str, float], lane: int) -> tuple[float, float, int]:
    xmin = float(bound.get("xmin", -0.2))
    xmax = float(bound.get("xmax", 1.2))
    ymin = float(bound.get("ymin", -0.4))
    ymax = float(bound.get("ymax", 0.4))
    n_lanes = 5
    ys = [ymin + (ymax - ymin) * i / (n_lanes - 1) for i in range(n_lanes)]
    lane = max(0, min(n_lanes - 1, lane))
    y = ys[lane]
    going_pos_x = lane % 2 == 0
    x_goal = xmax if going_pos_x else xmin
    if abs(view.odom_x - x_goal) < 0.10 and abs(view.odom_y - y) < 0.12:
        lane = min(n_lanes - 1, lane + 1)
        y = ys[lane]
        going_pos_x = lane % 2 == 0
        x_goal = xmax if going_pos_x else xmin
    return x_goal, y, lane


def goto_command(
    view: MissionView,
    target_x: float,
    target_y: float,
    *,
    arrive_m: float = 0.10,
    cruise: float = 0.45,
    wall: float,
    avoid: bool = True,
    blocked_range: float = 0.14,
    slow_range: float = 0.22,
) -> tuple[TeleCmd, bool, str]:
    dist, err, turn = heading_turn(view, target_x, target_y)
    if dist <= arrive_m:
        return TeleCmd(0.0, 0.0, wall, source="goto"), True, "arrived"
    velocity = cruise if abs(err) < 0.45 else 0.18
    front = front_range(view)
    reason = "goto"
    if avoid and front < slow_range:
        velocity = min(velocity, 0.12)
        reason = "slow_for_lidar"
        if front < blocked_range:
            turn = lidar_gap_turn(view)
            velocity = 0.08
            reason = "lidar_replan"
    return TeleCmd(velocity, turn, wall, source="goto"), False, reason
