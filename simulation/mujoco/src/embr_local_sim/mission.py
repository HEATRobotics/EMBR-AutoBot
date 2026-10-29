"""Sensor-driven mission state machine.

States: IDLE, SEARCH, APPROACH, SAMPLE, RETREAT, COMPLETE, FAULT.

The controller sees MissionView only (odom + lidar + thermal + probe).
Ground-truth pose is not a field on that view. Probe is a **one-DOF hinge boom**
(local proposal), not a track arm and not a capstan.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from embr_local_sim.policy import ClassicalLocalPolicy, LocalPolicy
from embr_local_sim.tele_cmd import TeleCmd


STATES = (
    "IDLE",
    "GOTO",
    "SEARCH_AREA",
    "SEARCH",
    "ROUTE",
    "APPROACH",
    "SAMPLE",
    "RETREAT",
    "COMPLETE",
    "FAULT",
)


@dataclass
class MissionView:
    """Deployable observations. Do not add root_pos / ground-truth pose here."""

    sim_time: float
    stamp_wall: float
    odom_x: float
    odom_y: float
    odom_yaw: float
    lidar_ranges: list[float]
    lidar_angles: list[float]
    lidar_stamp_sim: float
    thermal_c: list[float]
    thermal_angles: list[float]
    probe_c: float
    probe_stamp_sim: float
    probe_frame: str
    probe_state: str = "STOWED"
    probe_valid: bool = True
    lidar_valid: bool = True
    watchdog_active: bool = False
    lidar_max_range: float = 2.0
    ambient_c: float = 20.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    imu_valid: bool = True


@dataclass
class HotspotRecord:
    timestamp_sim: float
    timestamp_wall: float
    frame: str
    origin: str
    estimated_x: float
    estimated_y: float
    measured_c: float
    uncertainty_m: float
    quality: str
    provenance: str
    thermal_bearing_rad: float


@dataclass
class MissionConfig:
    timeout_s: float = 18.0
    detect_c: float = 48.0
    sample_c: float = 70.0
    sample_dwell_s: float = 0.40
    retreat_s: float = 2.0
    search_velocity: float = 0.55
    approach_velocity: float = 0.40
    blocked_range: float = 0.14
    slow_range: float = 0.22
    blocked_hold_s: float = 1.20
    sensor_stale_s: float = 0.30
    probe_frame: str = "probe"
    origin: str = "local_metres_odom"
    complete_retreat_x: float = 0.12
    # Autonomy extras. Demonstration tilt limit is PROVISIONAL, not a hardware rating.
    level: int = 4
    goto_x: float | None = None
    goto_y: float | None = None
    arrive_m: float = 0.10
    flame_avoid_c: float = 220.0
    max_tilt_rad: float = 0.55
    immobilized_s: float = 2.8
    search_bound: dict | None = None
    oracle: bool = False
    oracle_exclusion: list | None = None
    # Ordered instructions. Each step is x, y, action (drive, sample, return), name.
    route: list | None = None


def scene_to_open_for_follow(kind: str) -> str | None:
    """L4 stays on the scene already showing. None means do not rebuild."""
    return None


def instruction_markdown(world: Any) -> str:
    """Steps for the scene on screen. A scene without a route does not quote the yard."""
    extras = getattr(world, "extras", None) or {}
    steps = extras.get("route") or []
    if not steps:
        kind = getattr(world, "kind", "this scene")
        return (
            f"**Instructions**\n\n"
            f"Search this {kind} scene for a hotspot. "
            f"The lidar fan picks the open direction nearest the heat. "
            f"Follow instructions does not load another scene."
        )
    lines = "\n".join(
        f"{i + 1}. {step.get('name', step.get('action'))}" for i, step in enumerate(steps)
    )
    return f"**Instructions**\n\n{lines}"


def mission_for_robot(params: Any, world: Any, level: int) -> MissionConfig:
    """Mission numbers for this body. A body at proxy scale keeps the small defaults."""
    from embr_local_sim.body import resolve_body

    cfg = MissionConfig(level=level)
    bound = None
    extras = getattr(world, "extras", None) or {}
    if extras.get("route"):
        cfg.route = [dict(step) for step in extras["route"]]
    if extras.get("mission_bound"):
        bound = dict(extras["mission_bound"])
    spec = resolve_body(getattr(params, "morphology", "wheels"))
    if spec.layout_scale == 1.0:
        cfg.search_bound = bound
        return cfg
    scale = float(extras.get("layout_scale", spec.layout_scale))
    overhang = spec.lidar_overhang_m
    if bound is None:
        bound = {
            "xmin": -0.4 * scale,
            "xmax": 1.6 * scale,
            "ymin": -1.1 * scale,
            "ymax": 1.1 * scale,
        }
    cfg.timeout_s = 20.0 * max(scale, 1.0)
    cfg.blocked_range = overhang + 0.20
    cfg.slow_range = overhang + 0.70
    cfg.search_velocity = 0.45
    cfg.approach_velocity = 0.50
    cfg.retreat_s = 30.0
    cfg.complete_retreat_x = 0.40
    cfg.arrive_m = max(0.35, 0.10 * scale)
    cfg.immobilized_s = 5.0
    cfg.search_bound = bound
    return cfg


class MissionMachine:
    def __init__(
        self,
        config: MissionConfig | None = None,
        local: LocalPolicy | None = None,
        planner: Any | None = None,
    ) -> None:
        """`local` covers goto and the no-width-fan fallback. `planner` picks the arc.

        `planner` is anything with `steer(SteerContext) -> (TeleCmd, reason) | None`.
        None uses the vector-field histogram. Manual driving does not call it.
        """
        self.config = config or MissionConfig()
        self.local: LocalPolicy = local or ClassicalLocalPolicy()
        if planner is None:
            from embr_local_sim.planners.vfh import VfhPlanner

            planner = VfhPlanner()
        self.planner = planner
        self.state = "IDLE"
        self.reason = "idle"
        self.started_sim: float | None = None
        self.state_entered: float = 0.0
        self.blocked_since: float | None = None
        self.sample_started: float | None = None
        self.hotspots: list[HotspotRecord] = []
        self.last_tele = TeleCmd(0.0, 0.0, 0.0, source="mission")
        self.used_ground_truth = False
        self.cancel_requested = False
        self.probe_command: str | None = None
        self.retreat_drive_started: float | None = None
        self.level = int(self.config.level)
        self.search_lane = 0
        self.route_i = 0
        self._progress = None
        self._gap: dict = {}
        self._stuck: list[tuple[float, float, float, float]] = []
        self._escape_until: float | None = None
        self._escape_side = 0.0
        self._backup_until: float | None = None
        self._reaim_until: float | None = None
        self._reaim_turn = 0.0
        self._recoveries = 0
        self._gave_up_at: float | None = None
        self._gave_up_xy = (0.0, 0.0)
        self._vfh_motion: list[tuple[float, float, float]] = []
        self._unstick = 0
        self._jam_latched = False
        self._backup_turn = 0.0
        self._avoid_until: float | None = None
        self._avoid_heading = 0.0

    def reset(self) -> None:
        self.state = "IDLE"
        self.reason = "idle"
        self.started_sim = None
        self.state_entered = 0.0
        self.blocked_since = None
        self.sample_started = None
        self.hotspots.clear()
        self.last_tele = TeleCmd(0.0, 0.0, 0.0, source="mission")
        self.used_ground_truth = False
        self.cancel_requested = False
        self.probe_command = None
        self.retreat_drive_started = None
        self.level = int(self.config.level)
        self.search_lane = 0
        self.route_i = 0
        self._progress = None
        self._gap = {}
        self._stuck = []
        self._escape_until = None
        self._escape_side = 0.0
        self._backup_until = None
        self._reaim_until = None
        self._reaim_turn = 0.0
        self._recoveries = 0
        self._gave_up_at = None
        self._gave_up_xy = (0.0, 0.0)
        self._vfh_motion = []
        self._unstick = 0
        self._jam_latched = False
        self._backup_turn = 0.0
        self._avoid_until = None
        self._avoid_heading = 0.0

    def _look_around(self, view: MissionView, preferred: float, wall: float) -> TeleCmd | None:
        """A command that turns toward a longer lidar gap, or None to keep driving."""
        from embr_local_sim.autonomy import way_around

        found = way_around(
            view,
            preferred,
            wall=wall,
            memory=self._gap,
            blocked_range=self.config.blocked_range,
        )
        if found is None:
            return None
        tele, why = found
        self.reason = why
        return tele

    def _if_stuck(self, view: MissionView, wall: float, tele: TeleCmd) -> TeleCmd:
        """Yaw off a wheel that is pushing and not moving the body.

        Wheel odom can climb while a heavy rock holds the chassis. A forward
        range that stays short is that case. A fan that is short on every
        heading is a wall, and the blocked timer still stops the mission.
        """
        from embr_local_sim.autonomy import _spin_cmd, clearer_side

        if self.state in ("SAMPLE", "RETREAT", "COMPLETE", "FAULT", "IDLE"):
            self._stuck.clear()
            self._escape_until = None
            return tele
        if self._escape_until is not None and view.sim_time < self._escape_until:
            self.reason = "gap_steer"
            self.blocked_since = None
            return TeleCmd(0.0, _spin_cmd(self._escape_side), wall, source="mission")
        if self._escape_until is not None:
            self._escape_until = None
            self._stuck.clear()
        if abs(tele.velocity) < 0.15:
            self._stuck.clear()
            return tele
        front = self._front_range(view)
        self._stuck.append((view.sim_time, view.odom_x, view.odom_y, front))
        horizon = view.sim_time - 0.80
        self._stuck = [sample for sample in self._stuck if sample[0] >= horizon]
        if view.sim_time - self._stuck[0][0] < 0.70:
            return tele
        _t0, x0, y0, front0 = self._stuck[0]
        moved = math.hypot(view.odom_x - x0, view.odom_y - y0)
        range_change = abs(front - front0)
        against = front < 0.80
        slipping = moved >= 0.012 and range_change < 0.03 and against
        if moved >= 0.012 and not slipping:
            return tele
        if not against and moved >= 0.012:
            return tele
        ranges = [float(r) for r in view.lidar_ranges if math.isfinite(r)]
        if ranges and (sum(ranges) / len(ranges)) < 0.50:
            return tele
        side = clearer_side(view)
        if side == 0.0:
            side = -self._escape_side if self._escape_side else 0.80
        self.reason = "gap_steer"
        self.blocked_since = None
        self._stuck.clear()
        self._escape_until = view.sim_time + 2.2
        self._escape_side = side
        return TeleCmd(0.0, _spin_cmd(side), wall, source="mission")

    def _choose_reaim(self, view: MissionView, bearing: float) -> float:
        """Turn toward the hotspot, or toward the clearer side of the fan."""
        from embr_local_sim.autonomy import _spin_cmd, clearer_side

        if self.state == "APPROACH" and abs(bearing) > 0.35:
            return _spin_cmd(bearing)
        side = clearer_side(view)
        if side == 0.0:
            side = 0.80
        return _spin_cmd(side)

    def _vfh(self, view: MissionView, wall: float, goal: float, t: float) -> TeleCmd | None:
        """Open sector nearest the goal. A closed fan backs up, then faults."""
        if self._backup_until is not None and t < self._backup_until:
            self.reason = "backing_up"
            self.blocked_since = None
            return TeleCmd(-0.45, self._backup_turn, wall, source="mission")
        if self._backup_until is not None:
            self._backup_until = None
        if self._avoid_until is not None and t < self._avoid_until:
            goal = self._avoid_heading
        elif self._avoid_until is not None:
            self._avoid_until = None
        from embr_local_sim.planners.base import SteerContext

        half = 0.07 if self.config.blocked_range < 0.25 else 0.22
        found = self.planner.steer(
            SteerContext(
                view=view,
                goal_heading=goal,
                wall=wall,
                cruise=self.config.approach_velocity if self.state == "APPROACH" else self.config.search_velocity,
                blocked_range=self.config.blocked_range,
                half_width=half,
                memory=self._gap,
            )
        )
        if found is None:
            return None
        tele, why = found
        if why != "backing_up":
            self.reason = why
            self.blocked_since = None
            if tele.velocity > 0.15 and self._jammed(view, t):
                self._unstick += 1
                if self._unstick >= 6:
                    self._enter("FAULT", t, "blocked")
                    return TeleCmd(0.0, 0.0, wall, source="mission")
                self._backup_until = t + (0.70 if self.config.blocked_range < 0.25 else 1.1)
                self._vfh_motion.clear()
                self._jam_latched = False
                self.reason = "backing_up"
                heading = float(self._gap.get("vfh", goal))
                if abs(heading) < 0.25:
                    from embr_local_sim.autonomy import clearer_side

                    heading = clearer_side(view) or (0.7 if goal >= 0.0 else -0.7)
                turn = max(-1.0, min(1.0, -heading * 2.4))
                self._backup_turn = turn
                if abs(turn) > 0.2:
                    self._avoid_heading = max(-1.0, min(1.0, -turn / 2.4))
                    self._avoid_until = self._backup_until + 2.0
                return TeleCmd(-0.45, turn, wall, source="mission")
            return tele
        if self._recoveries >= 2:
            self._enter("FAULT", t, "blocked")
            return TeleCmd(0.0, 0.0, wall, source="mission")
        self._recoveries += 1
        self._backup_until = t + (0.70 if self.config.blocked_range < 0.25 else 1.1)
        self._backup_turn = max(-1.0, min(1.0, tele.turn))
        if abs(self._backup_turn) > 0.2:
            self._avoid_heading = max(-1.0, min(1.0, -self._backup_turn / 2.4))
            self._avoid_until = self._backup_until + 2.0
        self.blocked_since = None
        self.reason = "backing_up"
        return TeleCmd(-0.45, self._backup_turn, wall, source="mission")

    def _jammed(self, view: MissionView, t: float) -> bool:
        """True once, when the nose range stays short while a forward command is on.

        Wheel odom keeps climbing when a trunk holds the chassis, so odom
        alone never says stuck. The latch fires one backup, then waits for
        a fresh window.
        """
        front = self._front_range(view)
        self._vfh_motion.append((t, view.odom_yaw, front))
        horizon = t - 1.05
        self._vfh_motion = [sample for sample in self._vfh_motion if sample[0] >= horizon]
        touching = front < self.config.blocked_range + 0.10
        if t - self._vfh_motion[0][0] < (0.45 if touching else 0.85):
            return False
        _t0, yaw0, front0 = self._vfh_motion[0]
        turning = abs(math.atan2(math.sin(view.odom_yaw - yaw0), math.cos(view.odom_yaw - yaw0))) > 0.06
        if turning and not touching:
            self._jam_latched = False
            return False
        # A max-range fan is open ground, and the range stays put while the
        # body moves. A nearer return that does not get closer, while the
        # heading is also stuck, means the wheels are turning and the chassis is not.
        frozen_nose = front < view.lidar_max_range * 0.90 and abs(front - front0) < 0.04
        if not frozen_nose:
            self._jam_latched = False
            return False
        if self._jam_latched:
            return False
        self._jam_latched = True
        return True

    def _recovery_command(
        self, view: MissionView, wall: float, front: float, bearing: float, t: float
    ) -> TeleCmd | None:
        """Back up off a block, yaw, then let the search drive again.

        One recovery is allowed. A nose that is still blocked after that
        enters FAULT blocked. Manual drive does not use this path.
        """
        cfg = self.config
        if self._backup_until is not None:
            if t < self._backup_until:
                self.reason = "backing_up"
                self.blocked_since = None
                return TeleCmd(-0.55, 0.0, wall, source="mission")
            self._backup_until = None
            self._reaim_until = t + 1.60
            self._reaim_turn = self._choose_reaim(view, bearing)
        if self._reaim_until is not None:
            if t < self._reaim_until:
                self.reason = "reaim"
                self.blocked_since = None
                return TeleCmd(0.0, self._reaim_turn, wall, source="mission")
            self._reaim_until = None
            self.blocked_since = None
            if self._recoveries >= 2:
                self._gave_up_at = t
                self._gave_up_xy = (view.odom_x, view.odom_y)
        if self._gave_up_at is not None and self._backup_until is None and self._reaim_until is None:
            if t - self._gave_up_at >= 1.0:
                moved = math.hypot(view.odom_x - self._gave_up_xy[0], view.odom_y - self._gave_up_xy[1])
                if moved < 0.10:
                    self._enter("FAULT", t, "blocked")
                    return TeleCmd(0.0, 0.0, wall, source="mission")
                self._gave_up_at = None
        if front >= cfg.blocked_range:
            self.blocked_since = None
            return None
        if self.blocked_since is None:
            self.blocked_since = t
            return None
        held = t - self.blocked_since
        if held >= 0.40 and self._recoveries < 2:
            self._recoveries += 1
            self._backup_until = t + 0.90
            self.blocked_since = None
            self.reason = "backing_up"
            return TeleCmd(-0.55, 0.0, wall, source="mission")
        if held >= cfg.blocked_hold_s and self._recoveries >= 2:
            self._enter("FAULT", t, "blocked")
            return TeleCmd(0.0, 0.0, wall, source="mission")
        return None

    def _route_name(self) -> str:
        steps = self.config.route or []
        if self.route_i >= len(steps):
            return "route_done"
        step = steps[self.route_i]
        return str(step.get("name") or step.get("action") or "route")

    def start(self, sim_time: float) -> None:
        self.reset()
        self.started_sim = sim_time
        self.state_entered = sim_time
        if self.level <= 1:
            self.state = "IDLE"
            self.reason = "manual"
        elif self.level in (2, 3):
            self.state = "GOTO"
            self.reason = "goto_operator_target"
        elif self.level == 5:
            self.state = "SEARCH_AREA"
            self.reason = "bounded_search"
        elif self.config.route:
            self.state = "ROUTE"
            self.reason = self._route_name()
        else:
            self.state = "SEARCH"
            self.reason = "searching"

    def cancel(self, reason: str, sim_time: float) -> None:
        if self.state in ("COMPLETE", "FAULT"):
            return
        self.state = "FAULT"
        self.reason = reason
        self.state_entered = sim_time
        self.cancel_requested = True
        self.last_tele = TeleCmd(0.0, 0.0, 0.0, source="mission")

    def _enter(self, state: str, sim_time: float, reason: str) -> None:
        self.state = state
        self.reason = reason
        self.state_entered = sim_time

    def _front_range(self, view: MissionView) -> float:
        if not view.lidar_ranges:
            return view.lidar_max_range
        n = len(view.lidar_ranges)
        lo = max(0, n // 2 - 2)
        hi = min(n, n // 2 + 3)
        return min(view.lidar_ranges[lo:hi])

    def _hottest(self, view: MissionView) -> tuple[int, float, float]:
        if not view.thermal_c:
            return 0, view.ambient_c, 0.0
        # A bin at or above the flame limit is an exclusion, not a sample target.
        # The side flame must not pull the search off the residual hotspot.
        limit = self.config.flame_avoid_c
        ranked = [i for i, temp in enumerate(view.thermal_c) if temp < limit]
        pool = ranked or list(range(len(view.thermal_c)))
        # A ramp beside the nose shortens those rays and can look hotter than
        # the source straight ahead. Steer on the forward bins when they are warm.
        forward = [
            i
            for i in pool
            if i < len(view.thermal_angles) and abs(view.thermal_angles[i]) <= 0.30
        ]
        # Steer on the forward bins. A ramp beside the nose shortens those
        # side rays and makes them look like a hotter target.
        if forward:
            pool = forward
        idx = max(pool, key=lambda i: (view.thermal_c[i], -abs(view.thermal_angles[i] if i < len(view.thermal_angles) else 0.0)))
        ang = view.thermal_angles[idx] if idx < len(view.thermal_angles) else 0.0
        return idx, view.thermal_c[idx], ang

    def _warmest(self, view: MissionView) -> tuple[float, float]:
        """Hottest bin in the whole fan, excluding a flame."""
        if not view.thermal_c:
            return view.ambient_c, 0.0
        limit = self.config.flame_avoid_c
        pool = [i for i, temp in enumerate(view.thermal_c) if temp < limit]
        if not pool:
            return view.ambient_c, 0.0
        idx = max(pool, key=lambda i: view.thermal_c[i])
        ang = view.thermal_angles[idx] if idx < len(view.thermal_angles) else 0.0
        return float(view.thermal_c[idx]), float(ang)

    def _stale(self, view: MissionView) -> str | None:
        if view.sim_time - view.lidar_stamp_sim > self.config.sensor_stale_s:
            return "stale_lidar"
        if view.sim_time - view.probe_stamp_sim > self.config.sensor_stale_s:
            return "stale_probe"
        return None

    def tick(self, view: MissionView, *, wall: float) -> TeleCmd:
        cfg = self.config
        t = view.sim_time
        zero = TeleCmd(0.0, 0.0, wall, source="mission")
        self.probe_command = None

        if self.state in ("COMPLETE", "FAULT", "IDLE"):
            self.last_tele = zero
            return zero

        if not view.imu_valid:
            self._enter("FAULT", t, "imu_invalid")
            self.last_tele = zero
            return zero
        if max(abs(view.roll_rad), abs(view.pitch_rad)) > cfg.max_tilt_rad:
            self._enter("FAULT", t, "excessive_tilt")
            self.last_tele = zero
            return zero

        if not view.probe_valid:
            self._enter("FAULT", t, "probe_fault")
            self.last_tele = zero
            return zero
        if not view.lidar_valid:
            self._enter("FAULT", t, "lidar_dropout")
            self.last_tele = zero
            return zero

        if view.watchdog_active and self.state not in ("IDLE", "SEARCH") and (t - self.state_entered) > 0.5:
            self._enter("FAULT", t, "watchdog")
            self.last_tele = zero
            return zero

        stale = self._stale(view)
        if stale:
            self._enter("FAULT", t, stale)
            self.last_tele = zero
            return zero

        if self.started_sim is not None and (t - self.started_sim) > cfg.timeout_s:
            self._enter("FAULT", t, "timeout")
            self.last_tele = zero
            return zero

        from embr_local_sim.autonomy import ProgressWatch, flame_bearing, lawnmower_target

        if self._progress is None:
            self._progress = ProgressWatch()
        commanded_later = self.state not in ("SAMPLE",)
        # Stall check uses previous command; updated after we pick this tick's tele.

        front = self._front_range(view)
        idx, hot_c, bearing = self._hottest(view)
        flame_ang = flame_bearing(view, cfg.flame_avoid_c)
        if flame_ang is not None and abs(flame_ang) < 0.45 and self.state in (
            "SEARCH",
            "APPROACH",
            "GOTO",
            "SEARCH_AREA",
        ):
            # Sensor-only exclusion: do not drive into a bin hotter than the
            # demonstration flame threshold. Not a hardware survive-fire rating.
            tele = self.local.flame(flame_ang, wall)
            self.reason = "flame_exclusion_sensor"
            self.last_tele = tele
            return tele

        if cfg.oracle and cfg.oracle_exclusion:
            self.used_ground_truth = True
            self.reason = "oracle_exclusion_map"

        if self.state == "GOTO":
            tx = cfg.goto_x if cfg.goto_x is not None else 0.8
            ty = cfg.goto_y if cfg.goto_y is not None else 0.0
            tele, arrived, why = self.local.goto(
                view, tx, ty, arrive_m=cfg.arrive_m, wall=wall, avoid=self.level >= 3, cruise=0.45
            )
            self.reason = why
            if arrived:
                self._enter("COMPLETE", t, "goto_arrived")
                tele = zero
            else:
                tele = self._if_stuck(view, wall, tele)
            self.last_tele = tele
            if self._progress.update(view, abs(tele.velocity) > 0.05, cfg.immobilized_s):
                self._enter("FAULT", t, "suspected_immobilization")
                self.last_tele = zero
                return zero
            return tele

        if self.state == "ROUTE":
            steps = cfg.route or []
            if self.route_i >= len(steps):
                self._enter("COMPLETE", t, "returned")
                self.last_tele = zero
                return zero
            step = steps[self.route_i]
            # Face the point, then drive. avoid=False does not cut a diagonal.
            tele, arrived, why = self.local.goto(
                view,
                float(step["x"]),
                float(step["y"]),
                arrive_m=0.30,
                wall=wall,
                avoid=False,
                cruise=cfg.approach_velocity,
            )
            action = str(step.get("action", "drive"))
            self.reason = str(step.get("name") or why)
            dist = math.hypot(float(step["x"]) - view.odom_x, float(step["y"]) - view.odom_y)
            # Sample on the object, not when a warm reading shows up on the way there.
            if action == "sample" and view.probe_c >= cfg.sample_c and dist <= 0.28 and abs(view.odom_x) > 0.05:
                self._enter("SAMPLE", t, "probe_ready")
                self.sample_started = t
                self.last_tele = zero
                return zero
            if action in ("return", "reverse"):
                # The point is behind the nose. A spin on the slope walks the
                # tracks sideways. Back straight down and hold the heading.
                if view.probe_state != "STOWED":
                    self.probe_command = "stow"
                    self.last_tele = zero
                    return zero
                hold = max(-0.40, min(0.40, view.odom_yaw * 2.5))
                tele = TeleCmd(-0.55, hold, wall, source="mission")
                if math.hypot(float(step["x"]) - view.odom_x, float(step["y"]) - view.odom_y) <= 0.35:
                    self._enter("COMPLETE", t, "returned")
                    self.last_tele = zero
                    return zero
                self.last_tele = tele
                return tele
            if action == "drive":
                detour = self._look_around(view, 0.0, wall)
                if detour is not None:
                    tele = detour
            if arrived and action == "drive":
                self.route_i += 1
                self.reason = self._route_name()
            elif arrived and action == "sample":
                tele = TeleCmd(0.35, 0.0, wall, source="mission")
                self.reason = "creep_to_sample"
            tele = self._if_stuck(view, wall, tele)
            self.last_tele = tele
            return tele

        if self.state == "SEARCH_AREA":
            bound = cfg.search_bound or {"xmin": -0.15, "xmax": 1.1, "ymin": -0.35, "ymax": 0.35}
            gx, gy, self.search_lane = lawnmower_target(view, bound, self.search_lane)
            if hot_c >= cfg.detect_c:
                self._enter("APPROACH", t, "heat_candidate")
            else:
                tele, _arrived, why = self.local.goto(
                    view, gx, gy, arrive_m=0.12, wall=wall, avoid=True, cruise=cfg.search_velocity
                )
                self.reason = why
                if self._progress.update(view, abs(tele.velocity) > 0.05, cfg.immobilized_s):
                    self._enter("FAULT", t, "suspected_immobilization")
                    self.last_tele = zero
                    return zero
                tele = self._if_stuck(view, wall, tele)
                self.last_tele = tele
                return tele
        _ = commanded_later, idx

        around = None
        if self.state in ("SEARCH", "APPROACH"):
            peak, aim = self._warmest(view)
            _idx, forward_c, forward_ang = self._hottest(view)
            if peak >= forward_c + 12.0 and peak >= view.ambient_c + 8.0:
                goal = aim
            elif forward_c >= view.ambient_c + 8.0:
                goal = forward_ang
            else:
                goal = 0.0
            steered = self._vfh(view, wall, goal, t)
            if self.state == "FAULT":
                self.last_tele = zero
                return zero
            if steered is not None:
                around = steered

        velocity = 0.0
        turn = 0.0

        if self.state == "SEARCH":
            if view.probe_c >= cfg.sample_c and abs(view.odom_x) > 0.05:
                # The body is already in the hotspot even if the forward bin has passed it.
                self._enter("SAMPLE", t, "probe_ready")
                self.sample_started = t
                velocity = 0.0
                turn = 0.0
            elif self._warmest(view)[0] >= cfg.detect_c:
                self._enter("APPROACH", t, "heat_candidate")
            else:
                if around is not None:
                    velocity, turn = around.velocity, around.turn
                else:
                    velocity, turn = self.local.search(
                        cruise=cfg.search_velocity, front=front, slow_range=cfg.slow_range
                    )

        if self.state == "APPROACH":
            if self._warmest(view)[0] < cfg.detect_c - 8.0 and (t - self.state_entered) > 1.5:
                # A lawnmower keeps its lanes after a miss. A hotspot search goes back to driving the corridor.
                back = "SEARCH_AREA" if self.level == 5 else "SEARCH"
                self._enter(back, t, "lost_candidate")
            else:
                if around is not None:
                    velocity, turn = around.velocity, around.turn
                else:
                    velocity, turn = self.local.approach(
                        cruise=cfg.approach_velocity,
                        bearing=bearing,
                        front=front,
                        slow_range=cfg.slow_range,
                        blocked_range=cfg.blocked_range,
                    )
                    self.reason = "heat_candidate"
                if view.probe_c >= cfg.sample_c and abs(view.odom_x) > 0.05:
                    self._enter("SAMPLE", t, "probe_ready")
                    self.sample_started = t
                    velocity = 0.0
                    turn = 0.0

        if self.state == "SAMPLE":
            velocity = 0.0
            turn = 0.0
            self.probe_command = "deploy"
            if view.probe_state == "FAULT":
                self._enter("FAULT", t, "probe_fault")
                self.last_tele = zero
                return zero
            if view.probe_state != "SAMPLING":
                self.last_tele = zero
                return zero
            started = self.sample_started if self.sample_started is not None else t
            if self.sample_started is None or view.probe_state != "SAMPLING":
                self.sample_started = t
                started = t
            if view.probe_c < cfg.sample_c - 15.0:
                self._enter("FAULT", t, "probe_reading_mismatch")
                self.last_tele = zero
                return zero
            if (t - started) >= cfg.sample_dwell_s:
                rec = HotspotRecord(
                    timestamp_sim=t,
                    timestamp_wall=wall,
                    frame="odom",
                    origin=cfg.origin,
                    estimated_x=view.odom_x + 0.12 * math.cos(view.odom_yaw + bearing),
                    estimated_y=view.odom_y + 0.12 * math.sin(view.odom_yaw + bearing),
                    measured_c=view.probe_c,
                    uncertainty_m=0.15,
                    quality="probe_hinge_local_proposal",
                    provenance="hinge_boom_local_proposal",
                    thermal_bearing_rad=bearing,
                )
                self.hotspots.append(rec)
                self.probe_command = "stow"
                if self.config.route and self.route_i + 1 < len(self.config.route):
                    self.route_i += 1
                    self._enter("ROUTE", t, self._route_name())
                else:
                    self._enter("RETREAT", t, "sampled")

        if self.state == "RETREAT":
            self.probe_command = "stow"
            if view.probe_state != "STOWED":
                velocity = 0.0
                turn = 0.0
            else:
                if self.retreat_drive_started is None:
                    self.retreat_drive_started = t
                velocity = -0.55
                turn = 0.0
                if (t - self.retreat_drive_started) >= cfg.retreat_s or view.odom_x <= cfg.complete_retreat_x:
                    self._enter("COMPLETE", t, "retreated")
                    velocity = 0.0

        command = TeleCmd(velocity=velocity, turn=turn, timestamp_wall=wall, source="mission")
        # A planner owns its command, including a teammate's model. The stuck
        # yaw is only for the classical fallback, which does not return a command.
        if around is None:
            command = self._if_stuck(view, wall, command)
        self.last_tele = command
        # Stall watchdog is for goto/search levels. Level 4 keeps the original
        # blocked_hold FAULT so Milestone A mission_complete stays valid.
        if self.level in (2, 3, 5) and self._progress is not None and self._progress.update(
            view, abs(velocity) > 0.08, cfg.immobilized_s
        ):
            self._enter("FAULT", t, "suspected_immobilization")
            self.last_tele = zero
            return zero
        return self.last_tele

    def snapshot(self) -> dict[str, Any]:
        return {
            "state": self.state,
            "reason": self.reason,
            "used_ground_truth": self.used_ground_truth,
            "level": self.level,
            "oracle": self.config.oracle,
            "probe_command": self.probe_command,
            "hotspots": [
                {
                    "timestamp_sim": h.timestamp_sim,
                    "frame": h.frame,
                    "origin": h.origin,
                    "estimated_x": h.estimated_x,
                    "estimated_y": h.estimated_y,
                    "measured_c": h.measured_c,
                    "uncertainty_m": h.uncertainty_m,
                    "quality": h.quality,
                    "provenance": h.provenance,
                }
                for h in self.hotspots
            ],
        }
