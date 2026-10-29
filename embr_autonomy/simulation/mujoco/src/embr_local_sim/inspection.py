"""Operator inspection mission. Separate from the heat-search mission.

Select a target, drive, stop, deploy the hinge probe, measure, retract,
return to the pose recorded at start, then stop. The controller reads
MissionView only: odometry, ranges, IMU tilt, joint-backed probe status, and
the synthetic probe temperature. It does not read chassis pose, obstacle
coordinates, or hidden hotspot positions.

A cold reading can still finish the mission. Temperature labels below are
provisional labels for this synthetic probe, not a hardware rating and not a
wildfire measurement.

Timeouts, a blocked corridor, a stale scan, and a probe fault are failures
or incomplete outcomes. They are not success.

Clock: mission timers and sensor freshness use simulation time. Pausing the
simulator stops that clock. The existing 0.40 s command watchdog stays on
wall time and is an operator/network timeout, not a mission timer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from embr_local_sim.autonomy import (
    CORRIDOR_HALF_WIDTH_M,
    CORRIDOR_MARGIN_M,
    WindowProgress,
    goto_command,
    heading_turn,
)
from embr_local_sim.mission import MissionView
from embr_local_sim.tele_cmd import TeleCmd


# Provisional synthetic-probe labels. Not component ratings.
COLD_BELOW_C = 40.0
HOT_AT_OR_ABOVE_C = 70.0

STATES = (
    "IDLE",
    "NAVIGATE_TO_TARGET",
    "ALIGN_FOR_INSPECTION",
    "STOP_FOR_INSPECTION",
    "DEPLOY_PROBE",
    "SAMPLE",
    "RETRACT_PROBE",
    "RETURN_TO_START",
    "COMPLETE",
    "BLOCKED",
    "FAULT",
    "CANCELLED",
)

TERMINAL = ("COMPLETE", "BLOCKED", "FAULT", "CANCELLED")

STATE_HELP = {
    "IDLE": "Waiting for Start.",
    "NAVIGATE_TO_TARGET": "Driving toward the operator target with odometry and forward range.",
    "ALIGN_FOR_INSPECTION": "Turning in place to face the target.",
    "STOP_FOR_INSPECTION": "Waiting until estimated motion stays still.",
    "DEPLOY_PROBE": "Holding position while the hinge boom extends.",
    "SAMPLE": "Recording the probe temperature while the boom is in the sampling pose.",
    "RETRACT_PROBE": "Holding position until the boom is confirmed stowed.",
    "RETURN_TO_START": "Facing the recorded start, then driving there with the forward sensor.",
    "COMPLETE": "Stopped at the recorded start with the probe stowed.",
    "BLOCKED": "No progress in the ranges observed from here. Not a claim the target is unreachable.",
    "FAULT": "Stopped on a sensor, probe, clearance, or timing fault.",
    "CANCELLED": "Stop or manual takeover. The mission does not resume on its own.",
}


def classify_probe_c(temp_c: float) -> str:
    if temp_c >= HOT_AT_OR_ABOVE_C:
        return "hot"
    if temp_c >= COLD_BELOW_C:
        return "warm"
    return "cold"


def world_point_to_estimated(
    spawn_x: float,
    spawn_y: float,
    spawn_yaw: float,
    world_x: float,
    world_y: float,
) -> tuple[float, float]:
    """Express an operator world target in the odometry frame zeroed at spawn.

    Reset sets wheel odometry to (0, 0, 0) even when the chassis faces
    spawn_yaw in the world. This rotation is applied once, by the operator
    side, when Start is pressed. The controller then tracks that fixed goal
    with odometry. It does not receive a continuous ground-truth correction,
    and the click or the numbers are not a hidden map.
    """
    dx = world_x - spawn_x
    dy = world_y - spawn_y
    c = math.cos(spawn_yaw)
    s = math.sin(spawn_yaw)
    return c * dx + s * dy, -s * dx + c * dy


@dataclass
class InspectionConfig:
    goal_x: float = 0.6
    goal_y: float = 0.0
    # Software-experiment tolerances, not hardware specifications.
    # 0.12 m is a bit under one chassis length (0.15 m) in the estimated frame.
    arrive_m: float = 0.08
    return_arrive_m: float = 0.06
    heading_tol_rad: float = 0.35
    # 0.40 s matches the existing sample dwell. The clock starts only once the
    # probe reports SAMPLING, not when deploy was requested.
    sample_dwell_s: float = 0.40
    # Estimated speed must stay under this for stop_dwell_s before deploy or completion.
    stop_speed_mps: float = 0.03
    stop_dwell_s: float = 0.25
    # No motion and no distance-to-goal drop across this simulation-time window.
    progress_window_s: float = 2.5
    mission_timeout_s: float = 55.0
    align_timeout_s: float = 8.0
    sensor_stale_s: float = 0.30
    cruise: float = 0.50
    slow_range: float = 0.40
    blocked_range: float = 0.28
    max_tilt_rad: float = 0.55


@dataclass
class InspectionMachine:
    config: InspectionConfig = field(default_factory=InspectionConfig)
    state: str = "IDLE"
    reason: str = "idle"
    started_sim: float | None = None
    state_entered: float = 0.0
    start_x: float | None = None
    start_y: float | None = None
    start_yaw: float | None = None
    sample_started: float | None = None
    measurement_c: float | None = None
    measurement_class: str | None = None
    probe_command: str | None = None
    last_tele: TeleCmd = field(default_factory=lambda: TeleCmd(0.0, 0.0, 0.0, source="inspection"))
    transitions: list[dict[str, Any]] = field(default_factory=list)
    dist_to_target_m: float | None = None
    dist_to_start_m: float | None = None
    sensor_note: str = "no scan yet"
    _progress: WindowProgress = field(default_factory=WindowProgress)
    _odom_window: list[tuple[float, float, float]] = field(default_factory=list)
    _recoveries: int = 0
    _prefer_sign: float = 1.0
    _passage_bad_since: float | None = None
    _steer: dict = field(default_factory=dict)
    _goal_x: float = 0.0
    _goal_y: float = 0.0

    def __post_init__(self) -> None:
        self._goal_x = float(self.config.goal_x)
        self._goal_y = float(self.config.goal_y)

    @property
    def active(self) -> bool:
        return self.state not in ("IDLE", *TERMINAL)

    def reset(self) -> None:
        cfg = self.config
        self.state = "IDLE"
        self.reason = "idle"
        self.started_sim = None
        self.state_entered = 0.0
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.sample_started = None
        self.measurement_c = None
        self.measurement_class = None
        self.probe_command = None
        self.last_tele = TeleCmd(0.0, 0.0, 0.0, source="inspection")
        self.transitions.clear()
        self.dist_to_target_m = None
        self.dist_to_start_m = None
        self.sensor_note = "no scan yet"
        self._progress = WindowProgress()
        self._odom_window.clear()
        self._recoveries = 0
        self._prefer_sign = 1.0
        self._passage_bad_since = None
        self._steer = {}
        self._goal_x = float(cfg.goal_x)
        self._goal_y = float(cfg.goal_y)

    def start(self, sim_time: float) -> None:
        self.reset()
        self.started_sim = sim_time
        self._enter("NAVIGATE_TO_TARGET", sim_time, "operator_start")

    def cancel(self, reason: str, sim_time: float) -> None:
        if self.state in TERMINAL or self.state == "IDLE":
            return
        self._enter("CANCELLED", sim_time, reason)
        self.last_tele = TeleCmd(0.0, 0.0, 0.0, source="inspection")
        self.probe_command = None

    def help_text(self) -> str:
        return STATE_HELP.get(self.state, "")

    def snapshot(self) -> dict[str, Any]:
        elapsed = None
        if self.started_sim is not None:
            elapsed = None  # filled by the caller that knows the current sim time
        return {
            "state": self.state,
            "reason": self.reason,
            "help": self.help_text(),
            "started_sim": self.started_sim,
            "state_entered": self.state_entered,
            "start_pose_odom": [self.start_x, self.start_y, self.start_yaw],
            "goal_odom": [self._goal_x, self._goal_y],
            "dist_to_target_m": self.dist_to_target_m,
            "dist_to_start_m": self.dist_to_start_m,
            "sensor": self.sensor_note,
            "probe_command": self.probe_command,
            "measurement_c": self.measurement_c,
            "measurement_class": self.measurement_class,
            "elapsed_sim_s": elapsed,
            "transitions": list(self.transitions),
            "used_ground_truth": False,
        }

    def _enter(self, state: str, sim_time: float, reason: str) -> None:
        self.state = state
        self.reason = reason
        self.state_entered = sim_time
        self._progress.clear()
        self._odom_window.clear()
        self.transitions.append({"t": sim_time, "state": state, "reason": reason})

    def _remember_start(self, view: MissionView) -> None:
        if self.start_x is None:
            self.start_x = view.odom_x
            self.start_y = view.odom_y
            self.start_yaw = view.odom_yaw
            self.started_sim = view.sim_time

    def _speed(self, view: MissionView) -> float | None:
        self._odom_window.append((view.sim_time, view.odom_x, view.odom_y))
        horizon = view.sim_time - self.config.stop_dwell_s
        self._odom_window = [row for row in self._odom_window if row[0] >= horizon - 1e-9]
        if view.sim_time - self._odom_window[0][0] < 0.9 * self.config.stop_dwell_s:
            return None
        t0, x0, y0 = self._odom_window[0]
        dt = max(1e-3, view.sim_time - t0)
        return math.hypot(view.odom_x - x0, view.odom_y - y0) / dt

    def _sensor_fault(self, view: MissionView) -> str | None:
        if not view.lidar_valid:
            self.sensor_note = "invalid"
            return "sensor_invalid"
        age = view.sim_time - view.lidar_stamp_sim
        if age > self.config.sensor_stale_s:
            self.sensor_note = f"stale {age:.2f}s"
            return "stale_lidar"
        if not view.lidar_ranges:
            self.sensor_note = "missing"
            return "sensor_invalid"
        self.sensor_note = f"fresh age {age:.2f}s"
        return None

    def _zero(self, wall: float) -> TeleCmd:
        tele = TeleCmd(0.0, 0.0, wall, source="inspection")
        self.last_tele = tele
        return tele

    def _navigate(self, view: MissionView, goal_x: float, goal_y: float, wall: float, arrive_m: float) -> tuple[TeleCmd, bool, str]:
        tele, arrived, why = goto_command(
            view,
            goal_x,
            goal_y,
            arrive_m=arrive_m,
            cruise=self.config.cruise,
            wall=wall,
            avoid=True,
            blocked_range=self.config.blocked_range,
            slow_range=self.config.slow_range,
            prefer_sign=self._prefer_sign,
            memory=self._steer,
        )
        tele = TeleCmd(tele.velocity, tele.turn, wall, source="inspection")
        return tele, arrived, why

    def tick(self, view: MissionView, *, wall: float) -> TeleCmd:
        cfg = self.config
        t = view.sim_time
        self.probe_command = None
        if self.state in ("IDLE", *TERMINAL):
            return self._zero(wall)

        self._remember_start(view)
        assert self.start_x is not None and self.start_y is not None
        self.dist_to_target_m = math.hypot(self._goal_x - view.odom_x, self._goal_y - view.odom_y)
        self.dist_to_start_m = math.hypot(self.start_x - view.odom_x, self.start_y - view.odom_y)

        if not view.imu_valid or max(abs(view.roll_rad), abs(view.pitch_rad)) > cfg.max_tilt_rad:
            self._enter("FAULT", t, "excessive_tilt" if view.imu_valid else "imu_invalid")
            return self._zero(wall)

        sensor_fault = self._sensor_fault(view)
        if sensor_fault and self.state not in ("DEPLOY_PROBE", "SAMPLE", "RETRACT_PROBE"):
            # Sampling may finish a dwell already underway, but driving requires a live scan.
            self._enter("FAULT", t, sensor_fault)
            return self._zero(wall)
        if sensor_fault and self.state in ("DEPLOY_PROBE", "SAMPLE", "RETRACT_PROBE"):
            self._enter("FAULT", t, sensor_fault)
            return self._zero(wall)

        if self.started_sim is not None and (t - self.started_sim) > cfg.mission_timeout_s:
            self._enter("FAULT", t, "timeout")
            return self._zero(wall)

        if self.state == "NAVIGATE_TO_TARGET":
            tele, arrived, why = self._navigate(view, self._goal_x, self._goal_y, wall, cfg.arrive_m)
            self.reason = why
            if arrived:
                self._enter("ALIGN_FOR_INSPECTION", t, "near_target")
                return self._zero(wall)
            if why in ("range_unobserved", "no_observed_corridor", "no_passage_stop"):
                if self._passage_bad_since is None:
                    self._passage_bad_since = t
                elif t - self._passage_bad_since > 0.8:
                    self._enter("BLOCKED", t, "no_observed_passage")
                    return self._zero(wall)
            else:
                self._passage_bad_since = None
            # Aiming or turning in place is not a stall. Only driving that
            # fails to move, or fails to get closer, counts.
            stalled = self._progress.stalled(
                sim_time=t,
                x=view.odom_x,
                y=view.odom_y,
                dist_to_goal=self.dist_to_target_m,
                window_s=cfg.progress_window_s,
                active=abs(tele.velocity) > 0.05,
            )
            if stalled:
                self._recoveries += 1
                self._prefer_sign = -1.0 if self._prefer_sign >= 0.0 else 1.0
                self._progress.clear()
                if self._recoveries >= 2:
                    self._enter("BLOCKED", t, "no_progress_local_sensing")
                    return self._zero(wall)
                self.reason = "gap_recovery"
            self.last_tele = tele
            return tele

        if self.state == "ALIGN_FOR_INSPECTION":
            _dist, err, turn = heading_turn(view, self._goal_x, self._goal_y)
            if abs(err) <= cfg.heading_tol_rad:
                self._enter("STOP_FOR_INSPECTION", t, "aligned")
                return self._zero(wall)
            if (t - self.state_entered) > cfg.align_timeout_s:
                self._enter("FAULT", t, "insufficient_clearance_to_align")
                return self._zero(wall)
            tele = TeleCmd(0.0, turn, wall, source="inspection")
            self.reason = "align_heading"
            self.last_tele = tele
            return tele

        if self.state == "STOP_FOR_INSPECTION":
            speed = self._speed(view)
            self.reason = "waiting_for_stop" if speed is None or speed > cfg.stop_speed_mps else "stopped"
            if speed is not None and speed <= cfg.stop_speed_mps:
                self._enter("DEPLOY_PROBE", t, "stable_stop")
            elif (t - self.state_entered) > 3.0:
                self._enter("FAULT", t, "did_not_stop")
            return self._zero(wall)

        if self.state == "DEPLOY_PROBE":
            self.probe_command = "deploy"
            self.reason = f"probe_{view.probe_state.lower()}"
            if view.probe_state == "FAULT":
                self._enter("FAULT", t, "probe_fault")
                return self._zero(wall)
            if view.probe_state == "SAMPLING":
                # Dwell starts only once the boom is actually sampling.
                self.sample_started = t
                self._enter("SAMPLE", t, "probe_reached_sampling")
            elif (t - self.state_entered) > 3.0:
                self._enter("FAULT", t, "deploy_timeout")
            return self._zero(wall)

        if self.state == "SAMPLE":
            self.probe_command = "deploy"
            if view.probe_state == "FAULT":
                self._enter("FAULT", t, "probe_fault")
                return self._zero(wall)
            if view.probe_state != "SAMPLING" or self.sample_started is None:
                self._enter("FAULT", t, "left_sampling_early")
                return self._zero(wall)
            self.reason = "dwelling"
            if (t - self.sample_started) >= cfg.sample_dwell_s:
                self.measurement_c = float(view.probe_c)
                self.measurement_class = classify_probe_c(self.measurement_c)
                self._enter("RETRACT_PROBE", t, f"measured_{self.measurement_class}")
            return self._zero(wall)

        if self.state == "RETRACT_PROBE":
            self.probe_command = "stow"
            self.reason = f"probe_{view.probe_state.lower()}"
            if view.probe_state == "FAULT":
                self._enter("FAULT", t, "probe_retract_failed")
                return self._zero(wall)
            if view.probe_state == "STOWED":
                self._enter("RETURN_TO_START", t, "probe_stowed")
                return self._zero(wall)
            if (t - self.state_entered) > 3.0:
                self._enter("FAULT", t, "probe_retract_failed")
            return self._zero(wall)

        if self.state == "RETURN_TO_START":
            if view.probe_state != "STOWED":
                self._enter("FAULT", t, "drove_before_stow")
                return self._zero(wall)
            tele, arrived, why = self._navigate(
                view, self.start_x, self.start_y, wall, cfg.return_arrive_m
            )
            self.reason = why
            if why in ("range_unobserved", "no_observed_corridor", "no_passage_stop"):
                if self._passage_bad_since is None:
                    self._passage_bad_since = t
                elif t - self._passage_bad_since > 0.8:
                    self._enter("BLOCKED", t, "no_observed_passage_on_return")
                    return self._zero(wall)
            else:
                self._passage_bad_since = None
            if arrived:
                # Heading is enforced while approaching (goto turns before it
                # drives). Once the estimated position is inside the return
                # tolerance, bearing to that point is no longer meaningful.
                speed = self._speed(view)
                if speed is not None and speed <= cfg.stop_speed_mps:
                    self._enter("COMPLETE", t, "returned_and_stopped")
                    return self._zero(wall)
                self.reason = "stopping_at_start"
                return self._zero(wall)
            # Turning to face the start does not move the chassis. That is not a stall.
            stalled = self._progress.stalled(
                sim_time=t,
                x=view.odom_x,
                y=view.odom_y,
                dist_to_goal=self.dist_to_start_m,
                window_s=cfg.progress_window_s,
                active=abs(tele.velocity) > 0.05,
            )
            if stalled:
                self._enter("BLOCKED", t, "no_progress_on_return")
                return self._zero(wall)
            self.last_tele = tele
            return tele

        return self._zero(wall)


def elapsed_sim(machine: InspectionMachine, sim_time: float) -> float | None:
    if machine.started_sim is None:
        return None
    return float(sim_time) - float(machine.started_sim)
