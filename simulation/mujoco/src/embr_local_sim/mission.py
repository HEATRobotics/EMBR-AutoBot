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

from embr_local_sim.tele_cmd import TeleCmd


STATES = (
    "IDLE",
    "GOTO",
    "SEARCH_AREA",
    "SEARCH",
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


class MissionMachine:
    def __init__(self, config: MissionConfig | None = None) -> None:
        self.config = config or MissionConfig()
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
        self._progress = None

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
        self._progress = None

    def start(self, sim_time: float) -> None:
        self.reset()
        self.started_sim = sim_time
        self.state_entered = sim_time
        if self.level <= 1:
            self.state = "IDLE"
            self.reason = "manual"
        elif self.level == 2:
            self.state = "GOTO"
            self.reason = "goto_operator_target"
        elif self.level == 5:
            self.state = "SEARCH_AREA"
            self.reason = "bounded_search"
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
        idx = max(range(len(view.thermal_c)), key=lambda i: view.thermal_c[i])
        ang = view.thermal_angles[idx] if idx < len(view.thermal_angles) else 0.0
        return idx, view.thermal_c[idx], ang

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

        from embr_local_sim.autonomy import ProgressWatch, flame_bearing, goto_command, lawnmower_target

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
            turn_away = max(-1.0, min(1.0, (1.0 if flame_ang >= 0 else -1.0) * 0.85))
            tele = TeleCmd(velocity=0.12, turn=turn_away, timestamp_wall=wall, source="mission")
            self.reason = "flame_exclusion_sensor"
            self.last_tele = tele
            return tele

        if cfg.oracle and cfg.oracle_exclusion:
            self.used_ground_truth = True
            self.reason = "oracle_exclusion_map"

        if self.state == "GOTO":
            tx = cfg.goto_x if cfg.goto_x is not None else 0.8
            ty = cfg.goto_y if cfg.goto_y is not None else 0.0
            tele, arrived, why = goto_command(
                view, tx, ty, arrive_m=cfg.arrive_m, wall=wall, avoid=self.level >= 3
            )
            self.reason = why
            if arrived:
                self._enter("COMPLETE", t, "goto_arrived")
                tele = zero
            self.last_tele = tele
            if self._progress.update(view, abs(tele.velocity) > 0.05, cfg.immobilized_s):
                self._enter("FAULT", t, "suspected_immobilization")
                self.last_tele = zero
                return zero
            return tele

        if self.state == "SEARCH_AREA":
            bound = cfg.search_bound or {"xmin": -0.15, "xmax": 1.1, "ymin": -0.35, "ymax": 0.35}
            gx, gy, self.search_lane = lawnmower_target(view, bound, self.search_lane)
            if hot_c >= cfg.detect_c:
                self._enter("APPROACH", t, "heat_candidate")
            else:
                tele, _arrived, why = goto_command(
                    view, gx, gy, arrive_m=0.12, wall=wall, avoid=True, cruise=cfg.search_velocity
                )
                self.reason = why
                if self._progress.update(view, abs(tele.velocity) > 0.05, cfg.immobilized_s):
                    self._enter("FAULT", t, "suspected_immobilization")
                    self.last_tele = zero
                    return zero
                self.last_tele = tele
                return tele
        _ = commanded_later, idx

        if front < cfg.blocked_range:
            if self.blocked_since is None:
                self.blocked_since = t
            elif (t - self.blocked_since) >= cfg.blocked_hold_s and self.state in ("SEARCH", "APPROACH"):
                self._enter("FAULT", t, "blocked")
                self.last_tele = zero
                return zero
        else:
            self.blocked_since = None

        velocity = 0.0
        turn = 0.0

        if self.state == "SEARCH":
            if hot_c >= cfg.detect_c:
                self._enter("APPROACH", t, "heat_candidate")
            else:
                velocity = cfg.search_velocity
                if front < cfg.slow_range:
                    velocity = 0.15
                    # Obstacle on the nose: positive TeleCmd.turn is right.
                    # Yaw left (negative turn) to try a gap.
                    turn = -0.55
                elif front < cfg.slow_range + 0.12:
                    velocity = 0.28

        if self.state == "APPROACH":
            if hot_c < cfg.detect_c - 8.0 and (t - self.state_entered) > 1.5:
                self._enter("SEARCH", t, "lost_candidate")
            else:
                # Face the hottest bin. Bin angles are +left in the body frame;
                # TeleCmd.turn is positive-right, so negate.
                turn = max(-1.0, min(1.0, -bearing * 1.6))
                aligned = abs(bearing) < 0.25
                velocity = cfg.approach_velocity if aligned else 0.18
                if front < cfg.slow_range:
                    velocity = min(velocity, 0.12)
                    if front < cfg.blocked_range + 0.02:
                        turn = -0.7
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

        self.last_tele = TeleCmd(velocity=velocity, turn=turn, timestamp_wall=wall, source="mission")
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
