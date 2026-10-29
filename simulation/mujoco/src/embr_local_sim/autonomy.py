"""Conventional autonomy helpers. No hidden simulator ground-truth pose.

Levels (prompt §9):
  1 manual + supervisor
  2 goto an operator-selected odom location
  3 reactive gap steering from the 2D lidar fan (not a route planner)
  4 probe inspection + return (MissionMachine)
  5 bounded search from odom + thermal bins

A collision-free 2D path is not a guarantee that rollers can climb the terrain.
Flame exclusion uses thermal bins (sensor) unless an explicit labeled oracle map
is enabled.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

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


# Software-experiment footprint for reactive steering. The chassis is 0.07 m
# wide and the roller span is about 0.12 m. The corridor is wider than that
# span, plus a margin, so one long ray is not treated as a passage.
CORRIDOR_HALF_WIDTH_M = 0.07
CORRIDOR_MARGIN_M = 0.05
# A corridor shorter than this is the face of an obstacle, not a way past it.
PASS_CLEAR_M = 0.75


def corridor_clearance(view: MissionView, heading: float, half_width: float) -> float | None:
    """Forward clearance (m) of a robot-width corridor around a body heading.

    Heading is in the sensor frame: positive is left. Returns None when the
    corridor is not covered by fresh, finite ranges. A max-range return is
    observed free space out to that range. An empty or invalid scan is not
    free space.
    """
    if not view.lidar_valid or not view.lidar_ranges or not view.lidar_angles:
        return None
    span = math.atan2(max(half_width, 1e-3), 0.22)
    pairs: list[tuple[float, float]] = []
    for rng, ang in zip(view.lidar_ranges, view.lidar_angles):
        if abs(wrap_angle(ang - heading)) <= span:
            pairs.append((float(ang), float(rng)))
    if len(pairs) < 3:
        return None
    angs = [ang for ang, _rng in pairs]
    if min(angs) > heading - span + 0.12 or max(angs) < heading + span - 0.12:
        return None
    clear = float(view.lidar_max_range)
    for ang, rng in pairs:
        if not math.isfinite(rng) or rng <= 0.0:
            return None
        delta = wrap_angle(ang - heading)
        forward = rng * math.cos(delta)
        lateral = abs(rng * math.sin(delta))
        if forward <= 0.0:
            continue
        if lateral <= half_width:
            clear = min(clear, forward)
    return clear


def select_gap(
    view: MissionView,
    preferred_heading: float,
    half_width: float,
    *,
    prefer_sign: float = 0.0,
    hold_heading: float | None = None,
) -> tuple[float | None, float, str]:
    """Pick a heading inside the observed fan with corridor clearance.

    This is reactive gap selection, not a route planner. The heading closest
    to the goal wins when its corridor is wide enough for the robot. A held
    heading stays only while it is still clear, so a new obstacle can change
    the choice. Missing ranges are not treated as free space.
    """
    if not view.lidar_valid or not view.lidar_ranges or not view.lidar_angles:
        return None, 0.0, "range_unobserved"
    fov = max(abs(float(ang)) for ang in view.lidar_angles)
    best_heading: float | None = None
    best_clear = 0.0
    best_score = -1e9
    for step in range(11):
        heading = -fov + (2.0 * fov) * step / 10.0
        clear = corridor_clearance(view, heading, half_width)
        if clear is None:
            continue
        # A short corridor toward the goal is the obstacle itself. An open
        # side gap has to win, or the robot drives into the post.
        if clear < PASS_CLEAR_M:
            score = -1.0 + 0.25 * clear
        else:
            score = min(clear, 1.2) - 0.70 * abs(wrap_angle(heading - preferred_heading))
        score += 0.15 * prefer_sign * heading
        if score > best_score:
            best_score = score
            best_heading = heading
            best_clear = clear
    if best_heading is None:
        return None, 0.0, "no_observed_corridor"
    if hold_heading is not None and abs(hold_heading) <= fov:
        held = corridor_clearance(view, hold_heading, half_width)
        if (
            held is not None
            and held >= PASS_CLEAR_M
            and abs(wrap_angle(hold_heading - best_heading)) < 0.45
        ):
            return hold_heading, held, "gap_selected"
    return best_heading, best_clear, "gap_selected"


def clearer_side(view: MissionView) -> float:
    """Body heading of the more open half of the fan. 0 when the two sides match.

    Positive is left. Used when the rover is pushing and not moving: the center
    rays can still look clear while one wheel is against a rock.
    """
    if not view.lidar_valid or not view.lidar_ranges or not view.lidar_angles:
        return 0.0
    left: list[float] = []
    right: list[float] = []
    for ang, rng in zip(view.lidar_angles, view.lidar_ranges):
        if not math.isfinite(rng):
            continue
        if ang > 0.20:
            left.append(float(rng))
        elif ang < -0.20:
            right.append(float(rng))
    if len(left) < 2 or len(right) < 2:
        return 0.0
    gap = (sum(left) / len(left)) - (sum(right) / len(right))
    if abs(gap) < 0.25:
        return 0.0
    return 0.80 if gap > 0.0 else -0.80


def way_around(
    view: MissionView,
    preferred_heading: float,
    *,
    wall: float,
    memory: dict | None = None,
    blocked_range: float = 0.14,
) -> tuple[TeleCmd, str] | None:
    """Turn toward the longest lidar gap when it is longer than the hit ahead.

    None means no heading is meaningfully more open than straight ahead, so the
    caller keeps its own command. A wall that fills the fan returns None: there
    is no observed way past it, and the mission's blocked timer still applies.
    """
    half = CORRIDOR_HALF_WIDTH_M + CORRIDOR_MARGIN_M
    ahead = corridor_clearance(view, 0.0, half)
    if ahead is None or not view.lidar_angles:
        return None
    fov = max(abs(float(ang)) for ang in view.lidar_angles)
    best_heading: float | None = None
    best_clear = 0.0
    best_score = -1.0
    for step in range(11):
        heading = -fov + (2.0 * fov) * step / 10.0
        clear = corridor_clearance(view, heading, half)
        if clear is None:
            continue
        # Length of the gap decides. A small penalty keeps two equal gaps
        # pointed near the way the caller already wanted to go.
        score = clear - 0.35 * abs(wrap_angle(heading - preferred_heading))
        if score > best_score:
            best_score = score
            best_heading = heading
            best_clear = clear
    if best_heading is None or best_clear <= ahead + 0.35 or best_clear < blocked_range:
        if memory is not None:
            memory.pop("heading", None)
        return None
    hold = None if memory is None else memory.get("heading")
    if hold is not None and abs(hold) <= fov:
        held = corridor_clearance(view, float(hold), half)
        if held is not None and held >= best_clear - 0.40 and held > ahead + 0.35:
            best_heading = float(hold)
            best_clear = held
    if memory is not None:
        memory["heading"] = best_heading
    if abs(best_heading) > 0.25:
        return TeleCmd(0.0, _spin_cmd(best_heading), wall, source="mission"), "aim_at_gap"
    turn = max(-0.45, min(0.45, -best_heading * 1.2))
    velocity = 0.34 if best_clear >= PASS_CLEAR_M else 0.18
    return TeleCmd(velocity, turn, wall, source="mission"), "gap_steer"


# Histogram steering lives in planners.vfh. This name stays so existing
# callers and tests do not need to know which module owns it.
from embr_local_sim.planners.vfh import vfh_steer as vfh_steer


def lidar_gap_turn(view: MissionView) -> float:
    """Reactive steering toward an observed corridor. +turn is right.

    A single longest ray is not used. The heading has to clear the robot
    footprint. Missing ranges produce no turn.
    """
    half = CORRIDOR_HALF_WIDTH_M + CORRIDOR_MARGIN_M
    heading, _clear, _why = select_gap(view, 0.0, half)
    if heading is None:
        return 0.0
    return max(-1.0, min(1.0, -heading * 1.4))


def front_range(view: MissionView) -> float:
    """Shortest range near the nose. Empty or invalid scans are not free space."""
    if not view.lidar_valid or not view.lidar_ranges:
        return 0.0
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


class WindowProgress:
    """Progress over a fixed simulation-time window, not one control tick.

    A tick that moves a few millimetres is not a stall, and a tick that moves
    a lot is not progress by itself. The window has to show either motion or a
    drop in distance-to-goal. Turning in place is not a stall when `active` is
    false. The same trajectory sampled at two control rates uses the same rule.
    """

    def __init__(self) -> None:
        self.samples: list[tuple[float, float, float, float | None]] = []

    def clear(self) -> None:
        self.samples.clear()

    def stalled(
        self,
        *,
        sim_time: float,
        x: float,
        y: float,
        dist_to_goal: float | None,
        window_s: float,
        active: bool,
        min_motion_m: float = 0.04,
        min_dist_drop_m: float = 0.03,
    ) -> bool:
        if not active or window_s <= 0.0:
            self.samples.clear()
            return False
        self.samples.append((sim_time, x, y, dist_to_goal))
        horizon = sim_time - window_s
        self.samples = [sample for sample in self.samples if sample[0] >= horizon - 1e-9]
        if sim_time - self.samples[0][0] < 0.95 * window_s:
            return False
        _t0, x0, y0, d0 = self.samples[0]
        moved = math.hypot(x - x0, y - y0)
        dist_drop = 0.0
        if dist_to_goal is not None and d0 is not None:
            dist_drop = d0 - dist_to_goal
        if moved >= min_motion_m or dist_drop >= min_dist_drop_m:
            return False
        return True


@dataclass
class ProgressWatch:
    window: WindowProgress = field(default_factory=WindowProgress)

    def update(self, view: MissionView, commanded: bool, thresh_s: float) -> bool:
        return self.window.stalled(
            sim_time=view.sim_time,
            x=view.odom_x,
            y=view.odom_y,
            dist_to_goal=None,
            window_s=thresh_s,
            active=commanded,
        )


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


def _spin_cmd(heading: float) -> float:
    """In-place yaw toward a body heading. +heading is left, +turn is right.

    Tracks do not break static friction on a small turn command, so a yaw
    that is meant to happen in place uses a fixed command.
    """
    if abs(heading) < 1e-3:
        return 0.0
    return math.copysign(0.70, -heading)


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
    prefer_sign: float = 0.0,
    memory: dict | None = None,
) -> tuple[TeleCmd, bool, str]:
    dist, err, turn = heading_turn(view, target_x, target_y)
    if dist <= arrive_m:
        return TeleCmd(0.0, 0.0, wall, source="goto"), True, "arrived"
    half = CORRIDOR_HALF_WIDTH_M + CORRIDOR_MARGIN_M
    hold = None if memory is None else memory.get("heading")
    if not avoid:
        if abs(err) > 0.35:
            return TeleCmd(0.0, _spin_cmd(err), wall, source="goto"), False, "turn_to_face_goal"
        velocity = cruise if abs(err) < 0.40 else 0.22
        return TeleCmd(velocity, turn, wall, source="goto"), False, "goto"
    ahead = corridor_clearance(view, 0.0, half)
    if ahead is None:
        return TeleCmd(0.0, 0.0, wall, source="goto"), False, "range_unobserved"
    # Keep the sample inside the forward fan. A goal behind the robot is not a
    # corridor the sensor can measure, and clamping to the fan edge makes the
    # corridor look uncovered.
    goal_heading = max(-0.55, min(0.55, err))
    toward_goal = corridor_clearance(view, goal_heading, half)
    if toward_goal is None:
        return TeleCmd(0.0, 0.0, wall, source="goto"), False, "range_unobserved"
    # Spin in place only when the way toward the goal is actually open.
    # Otherwise follow the open side the current scan can see.
    way_open = toward_goal >= PASS_CLEAR_M and ahead >= blocked_range
    if ahead < slow_range or toward_goal < PASS_CLEAR_M:
        heading, clear, why = select_gap(
            view, err, half, prefer_sign=prefer_sign, hold_heading=hold
        )
        if heading is None:
            return TeleCmd(0.0, 0.0, wall, source="goto"), False, why
        if memory is not None:
            memory["heading"] = heading
        if clear < PASS_CLEAR_M:
            return TeleCmd(0.0, 0.0, wall, source="goto"), False, "no_passage_stop"
        # Point at the open corridor before driving into the obstacle.
        if abs(heading) > 0.25:
            return TeleCmd(0.0, _spin_cmd(heading), wall, source="goto"), False, "aim_at_gap"
        velocity = 0.34 if clear > 0.9 else 0.20
        turn = max(-0.45, min(0.45, -heading * 1.2))
        return TeleCmd(velocity, turn, wall, source="goto"), False, "gap_steer"
    # The forward fan cannot see a post already beside the body. After a gap,
    # drive straight for a short distance before turning back toward the goal,
    # so the corner does not swing into the obstacle on the way past.
    if memory is not None and memory.get("heading") is not None and memory.get("commit_xy") is None:
        memory["commit_xy"] = (view.odom_x, view.odom_y)
        memory["commit_h"] = memory.get("heading")
    if memory is not None and memory.get("commit_xy") is not None:
        cx, cy = memory["commit_xy"]
        if math.hypot(view.odom_x - cx, view.odom_y - cy) < 0.34:
            held = float(memory.get("commit_h") or 0.0)
            hold_turn = max(-0.12, min(0.12, -held * 0.8))
            return TeleCmd(0.26, hold_turn, wall, source="goto"), False, "clear_the_obstacle"
        memory["commit_xy"] = None
        memory["heading"] = None
    elif memory is not None:
        memory["heading"] = None
    if abs(err) > 0.35 and way_open:
        return TeleCmd(0.0, _spin_cmd(err), wall, source="goto"), False, "turn_to_face_goal"
    velocity = cruise if abs(err) < 0.40 else 0.22
    reason = "goto"
    if ahead < slow_range + 0.12:
        velocity = min(velocity, 0.28)
        reason = "slow_for_corridor"
    return TeleCmd(velocity, turn, wall, source="goto"), False, reason
