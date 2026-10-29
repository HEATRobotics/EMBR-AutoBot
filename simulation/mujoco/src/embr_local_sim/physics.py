"""MuJoCo physics owner. This thread is the only writer of simulator state."""

from __future__ import annotations

import math
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np

from embr_local_sim.articulated import ART_JOINTS
from embr_local_sim.controls import VelocityController
from embr_local_sim.model import RobotParams, WorldSpec, build_mjcf
from embr_local_sim.inspection import InspectionConfig, InspectionMachine
from embr_local_sim.mission import MissionConfig, MissionMachine, MissionView
from embr_local_sim.observation import ControllerOutput, RobotObservation
from embr_local_sim.posture import PostureController
from embr_local_sim.probe import PROBE_ACTUATOR, PROBE_JOINT, ProbeMachine
from embr_local_sim.replay import ReplayFrame, ReplayLog
from embr_local_sim.sensors import SensorModel
from embr_local_sim.supervisor import CommandSupervisor
from embr_local_sim.tele_cmd import CommandGate, DrivetrainHelperSim, TeleCmd, twist_to_drive
from embr_local_sim.thermal import ThermalField, sources_from_world
from embr_local_sim.types import (
    LEFT_WHEELS,
    RIGHT_WHEELS,
    ROOT_BODY,
    WHEEL_JOINTS,
    ContactHit,
    DriveCommand,
    GroundTruthState,
    SensorObservations,
)


def _boost_track_yaw(
    tau: dict[str, float], *, limit: float, vx: float, wz: float, max_wz: float
) -> dict[str, float]:
    """A and D yaw. W and S move. Together they arc.

    A or D alone used to add torque on one side only, so the key drove the
    robot forward. In place, the sides push in opposite directions and the
    robot yaws without driving off.     With W held, the inside side keeps that drive and the outside side gets
    extra torque. Positive wz speeds the right side (turn left). With S held,
    the opposite side works harder so A still bends left and D still bends right.
    """
    from embr_local_sim.track_approx import TURN_CORNER_CAP_NM, TURN_OUTSIDE_EXTRA_NM

    left = 0.5 * sum(tau[name] for name in LEFT_WHEELS)
    right = 0.5 * sum(tau[name] for name in RIGHT_WHEELS)
    common = max(-limit, min(limit, 0.5 * (left + right)))
    yaw_frac = 0.0 if max_wz <= 0.0 else max(-1.0, min(1.0, wz / max_wz))
    cap = TURN_CORNER_CAP_NM
    if abs(vx) < 0.03:
        spin = cap * yaw_frac
        left_torque, right_torque = -spin, spin
    else:
        # Full steering difference while a drive key is held. Scaling this by
        # the stick left W+D almost straight. The extra follows the drive
        # direction, so S+D keeps reversing instead of cancelling it.
        extra = TURN_OUTSIDE_EXTRA_NM * max(abs(yaw_frac), 0.85)
        if vx >= 0.0:
            # Outside track pushes harder forward. +wz speeds the right side.
            if yaw_frac >= 0.0:
                left_torque, right_torque = common, common + extra
            else:
                left_torque, right_torque = common + extra, common
        else:
            # Backing up flips which side works harder. D still bends the path
            # to the right, and A to the left, the same as driving forward.
            if yaw_frac >= 0.0:
                left_torque, right_torque = common, common - extra
            else:
                left_torque, right_torque = common - extra, common
    out: dict[str, float] = {}
    for name in LEFT_WHEELS:
        out[name] = max(-cap, min(cap, left_torque))
    for name in RIGHT_WHEELS:
        out[name] = max(-cap, min(cap, right_torque))
    return out


def _yaw_from_wxyz(q: np.ndarray) -> float:
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _rotate_world_to_body(quat_wxyz: np.ndarray, vec_world: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_wxyz
    # Inverse rotation (body from world).
    q_inv = np.array([w, -x, -y, -z], dtype=np.float64)
    return _rotate_body_to_world(q_inv, vec_world)


def _rotate_body_to_world(quat_wxyz: np.ndarray, vec: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(v) for v in quat_wxyz)
    qvec = np.array([x, y, z], dtype=np.float64)
    uv = np.cross(qvec, vec)
    uuv = np.cross(qvec, uv)
    return vec + 2.0 * (w * uv + uuv)


@dataclass
class UiRequest:
    kind: str
    command: DriveCommand | None = None
    tele: TeleCmd | None = None
    teleport: dict[str, Any] | None = None
    preset: str | None = None
    art_targets: dict[str, float] | None = None
    mode: str | None = None
    goto: tuple[float, float] | None = None
    level: int | None = None


class Simulator:
    """Contact-based rover. Poses are written only on explicit reset/teleport."""

    def __init__(
        self,
        params: RobotParams | None = None,
        world: WorldSpec | None = None,
        seed: int = 0,
    ) -> None:
        from embr_local_sim.body import apply_body, pod_drive

        self.params = apply_body(params or RobotParams())
        self.world = world or WorldSpec()
        self.seed = seed
        self.xml = build_mjcf(self.params, self.world)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.controller = VelocityController(self.params)
        self.sensors = SensorModel(self.params, seed=seed)
        self.helper = DrivetrainHelperSim(self.params)
        self.gate = CommandGate()
        self.mission = MissionMachine()
        # None keeps the built-in histogram. Set this before start_mission to
        # swap in another steer() implementation. Manual drive does not read it.
        self.steer_planner = None
        self.inspection = InspectionMachine()
        self.inspection_trace: list[dict[str, Any]] = []
        self.spawn_pose = (0.0, 0.0, 0.0)
        self.probe = ProbeMachine()
        self.replay = ReplayLog()
        self.thermal = ThermalField(
            ambient_c=float(self.world.ambient_c),
            sources=sources_from_world(self.world.heat),
            rng=np.random.default_rng(seed),
        )
        self._last_tele = TeleCmd(0.0, 0.0, 0.0, source="none")
        self.requests: queue.Queue[UiRequest] = queue.Queue()
        self._state_lock = threading.Lock()
        self._latest: GroundTruthState | None = None
        self._latest_obs: SensorObservations | None = None
        self._paused = False
        self._stop = False
        self._single_step = False
        self._wall_start = time.perf_counter()
        self._sim_wall_anchor = self._wall_start
        self._rtf = 0.0
        self._step_latency = 0.0
        self._steps = 0
        self._physics_thread: threading.Thread | None = None
        self._control_period_steps = max(1, int(round(1.0 / (self.params.control_hz * self.params.physics_dt))))
        layout = pod_drive(self.params.morphology)
        self._podded = layout is not None
        self._siblings = layout[0] if layout else None
        self._n_roll = layout[1] if layout else 1
        drive_names = list(layout[2]()) if layout else list(WHEEL_JOINTS)
        self._drive_names = drive_names
        self._joint_adr = {name: int(self.model.joint(name).qposadr[0]) for name in drive_names}
        self._joint_dof = {name: int(self.model.joint(name).dofadr[0]) for name in drive_names}
        self._act_id = {name: int(self.model.actuator(name).id) for name in drive_names}
        self._art_qpos = {}
        self._art_act = {}
        if self._podded:
            self._art_qpos = {name: int(self.model.joint(name).qposadr[0]) for name in ART_JOINTS}
            self._art_act = {name: int(self.model.actuator(name).id) for name in ART_JOINTS}
        self.posture = PostureController()
        self.supervisor = CommandSupervisor()
        if not self._podded:
            self.supervisor.mode = "n/a"
        self._probe_act = int(self.model.actuator(PROBE_ACTUATOR).id)
        self._probe_qpos = int(self.model.joint(PROBE_JOINT).qposadr[0])
        self._root_id = int(self.model.body(ROOT_BODY).id)
        self._root_qposadr = int(self.model.joint("root_free").qposadr[0])
        self._root_dofadr = int(self.model.joint("root_free").dofadr[0])
        mujoco.mj_forward(self.model, self.data)
        self._publish()

    def enqueue(self, request: UiRequest) -> None:
        self.requests.put(request)

    def drive(self, vx: float, wz: float, source: str = "api") -> None:
        self.enqueue(
            UiRequest(
                kind="command",
                command=DriveCommand(vx=vx, wz=wz, timestamp_wall=time.time(), source=source),
            )
        )

    def ingest_tele_cmd(self, tele: TeleCmd) -> None:
        self.enqueue(UiRequest(kind="tele_cmd", tele=tele))

    def set_posture_preset(self, name: str) -> None:
        self.enqueue(UiRequest(kind="preset", preset=name))

    def set_art_targets(self, targets: dict[str, float]) -> None:
        self.enqueue(UiRequest(kind="art_targets", art_targets=targets))

    def set_supervisor_mode(self, mode: str) -> None:
        self.enqueue(UiRequest(kind="supervisor_mode", mode=mode))

    def start_inspection(self, config: InspectionConfig | None = None) -> None:
        """Start the inspection mission. Does not start the heat-search mission."""
        if config is not None:
            self.inspection = InspectionMachine(config)
        self.enqueue(UiRequest(kind="inspection_start"))

    def start_mission(self, config: MissionConfig | None = None) -> None:
        # A planner assigned after construction has to be picked up here.
        # start_mission() with no config is the script in the README.
        if config is not None or self.steer_planner is not None:
            cfg = config if config is not None else self.mission.config
            self.mission = MissionMachine(cfg, local=self.mission.local, planner=self.steer_planner)
        self.enqueue(UiRequest(kind="mission_start"))

    def start_goto(self, x: float, y: float, *, level: int = 3) -> None:
        from embr_local_sim.mission import mission_for_robot

        cfg = mission_for_robot(self.params, self.world, level)
        cfg.goto_x = x
        cfg.goto_y = y
        self.mission = MissionMachine(cfg, planner=self.steer_planner)
        self.enqueue(UiRequest(kind="mission_start"))

    def start_search(self, bound: dict | None = None) -> None:
        from embr_local_sim.mission import mission_for_robot

        cfg = mission_for_robot(self.params, self.world, 5)
        if bound is not None:
            cfg.search_bound = bound
        self.mission = MissionMachine(cfg, planner=self.steer_planner)
        self.enqueue(UiRequest(kind="mission_start"))

    def cancel_mission(self, reason: str = "cancelled") -> None:
        self.enqueue(UiRequest(kind="mission_cancel", teleport={"reason": reason}))

    def request_reset(self, teleport: dict[str, Any] | None = None) -> None:
        self.enqueue(UiRequest(kind="reset", teleport=teleport))

    def snapshot(self) -> tuple[GroundTruthState, SensorObservations]:
        with self._state_lock:
            assert self._latest is not None and self._latest_obs is not None
            return self._latest, self._latest_obs

    def mjcf_hash(self) -> str:
        import hashlib

        return hashlib.sha256(self.xml.encode("utf-8")).hexdigest()[:16]

    def reset(self, teleport: dict[str, Any] | None = None) -> None:
        """Labeled teleport / initial condition. Not used for locomotion."""
        mujoco.mj_resetData(self.model, self.data)
        z = float(teleport.get("z", self.params.spawn_height())) if teleport else self.params.spawn_height()
        x = float(teleport["x"]) if teleport and "x" in teleport else 0.0
        y = float(teleport["y"]) if teleport and "y" in teleport else 0.0
        yaw = float(teleport["yaw"]) if teleport and "yaw" in teleport else 0.0
        self.spawn_pose = (x, y, yaw)
        adr = self._root_qposadr
        self.data.qpos[adr : adr + 3] = (x, y, z)
        half = 0.5 * yaw
        self.data.qpos[adr + 3 : adr + 7] = (math.cos(half), 0.0, 0.0, math.sin(half))
        for name in self._drive_names:
            self.data.qpos[self._joint_adr[name]] = 0.0
        for name, adr in self._art_qpos.items():
            self.data.qpos[adr] = 0.0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.controller.reset()
        self.sensors.reset(seed=self.seed)
        self.helper.reset()
        self.gate.reset()
        self.mission.reset()
        self.inspection.reset()
        self.probe.reset()
        self.probe.config.retract_fault = False
        self.inspection_trace.clear()
        self.posture.reset()
        self.supervisor.reset()
        if not self._podded:
            self.supervisor.mode = "n/a"
        self.replay.reset()
        self.thermal.reset(seed=self.seed)
        self._last_tele = TeleCmd(0.0, 0.0, 0.0, source="none")
        self._publish()

    def _drain_requests(self, wall_now: float) -> None:
        while True:
            try:
                req = self.requests.get_nowait()
            except queue.Empty:
                break
            if self._paused and not self._single_step and req.kind in ("command", "tele_cmd"):
                # Drop drive input while paused so a held button cannot queue
                # motion that appears on resume.
                continue
            if req.kind == "command" and req.command is not None:
                cmd = req.command
                if cmd.timestamp_sim is None:
                    cmd = DriveCommand(
                        vx=cmd.vx,
                        wz=cmd.wz,
                        timestamp_wall=cmd.timestamp_wall,
                        source=cmd.source,
                        timestamp_sim=float(self.data.time),
                    )
                self.controller.ingest(cmd, wall_now=wall_now)
            elif req.kind == "tele_cmd" and req.tele is not None:
                self._apply_tele(req.tele, wall_now)
            elif req.kind == "mission_start":
                self.gate.start_mission()
                self.controller.watchdog_tripped = False
                self.controller.last_command_wall = wall_now
                self.inspection.reset()
                self.mission.start(float(self.data.time))
            elif req.kind == "inspection_start":
                self.gate.start_mission()
                self.controller.watchdog_tripped = False
                self.controller.last_command_wall = wall_now
                self.mission.reset()
                self.inspection.start(float(self.data.time))
            elif req.kind == "mission_cancel":
                reason = "cancelled"
                if req.teleport and "reason" in req.teleport:
                    reason = str(req.teleport["reason"])
                self.gate.mode = "manual"
                if self.inspection.active:
                    self.inspection.cancel(reason, float(self.data.time))
                elif self.mission.state not in ("IDLE", "COMPLETE", "FAULT"):
                    self.mission.cancel(reason, float(self.data.time))
                self.controller.ingest(
                    DriveCommand(0.0, 0.0, wall_now, source="mission-cancel"),
                    wall_now=wall_now,
                )
            elif req.kind == "probe_deploy":
                self.probe.request_deploy(float(self.data.time))
            elif req.kind == "probe_stow":
                self.probe.request_stow(float(self.data.time))
            elif req.kind == "preset" and req.preset:
                self.posture.set_preset(req.preset)
                self.supervisor.set_mode("assisted", "preset")
            elif req.kind == "art_targets" and req.art_targets:
                self.posture.set_manual(req.art_targets)
                self.supervisor.set_mode("assisted", "manual_legs")
            elif req.kind == "supervisor_mode" and req.mode:
                self.supervisor.set_mode(req.mode)
            elif req.kind == "reset":
                self.reset(req.teleport)
            elif req.kind == "pause":
                self._paused = True
                self._zero_drive(wall_now, "pause")
            elif req.kind == "resume":
                self._paused = False
                self._zero_drive(wall_now, "resume")
            elif req.kind == "step":
                self._single_step = True
            elif req.kind == "stop":
                self._stop = True

    def _zero_drive(self, wall_now: float, source: str) -> None:
        """Clear latched speed targets. Does not write the root pose."""
        self.helper.reset()
        self.controller.ingest(
            DriveCommand(
                vx=0.0,
                wz=0.0,
                timestamp_wall=wall_now,
                source=source,
                timestamp_sim=float(self.data.time),
            ),
            wall_now=wall_now,
        )
        self.controller.slewed_wheel = {name: 0.0 for name in self.controller.slewed_wheel}
        self.data.ctrl[:] = 0.0
        self._last_tele = TeleCmd(0.0, 0.0, wall_now, source=source)

    def _apply_tele(self, tele: TeleCmd, wall_now: float) -> None:
        owner = self.gate.on_tele(tele)
        if owner == "rejected":
            return
        if self.gate.takeover and self.inspection.active:
            self.inspection.cancel("manual_takeover", float(self.data.time))
            self.gate.takeover = False
        elif self.gate.takeover and self.mission.state not in ("IDLE", "COMPLETE", "FAULT"):
            self.mission.cancel("manual_takeover", float(self.data.time))
            self.gate.takeover = False
        self._last_tele = tele
        twist = self.helper.ingest(tele)
        drive = twist_to_drive(twist)
        if drive.timestamp_sim is None:
            drive = DriveCommand(
                vx=drive.vx,
                wz=drive.wz,
                timestamp_wall=drive.timestamp_wall,
                source=drive.source,
                timestamp_sim=float(self.data.time),
            )
        self.controller.ingest(drive, wall_now=wall_now)

    def _joint_vel(self) -> dict[str, float]:
        return {name: float(self.data.qvel[adr]) for name, adr in self._joint_dof.items()}

    def _joint_pos(self) -> dict[str, float]:
        return {name: float(self.data.qpos[adr]) for name, adr in self._joint_adr.items()}

    def _roll_pitch(self, quat: np.ndarray) -> tuple[float, float]:
        w, x, y, z = (float(v) for v in quat)
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        return roll, pitch

    def _robot_observation(self) -> RobotObservation:
        _gt, obs = self.snapshot()
        quat = np.array(obs.imu_quat_wxyz, dtype=np.float64)
        roll, pitch = self._roll_pitch(quat)
        art_pos = {n: float(self.data.qpos[adr]) for n, adr in self._art_qpos.items()}
        art_vel = {}
        for n in self._art_qpos:
            art_vel[n] = float(self.data.qvel[int(self.model.joint(n).dofadr[0])])
        return RobotObservation(
            sim_time=obs.sim_time,
            stamp_wall=obs.stamp_wall,
            imu_quat_wxyz=obs.imu_quat_wxyz,
            imu_angvel_body=obs.imu_angvel_body,
            imu_specific_force_body=obs.imu_specific_force_body,
            roll_rad=roll,
            pitch_rad=pitch,
            joint_pos=dict(obs.encoder_rad),
            joint_vel=dict(obs.encoder_vel),
            drive_vx=self.controller.target_vx,
            drive_wz=self.controller.target_wz,
            art_pos=art_pos,
            art_vel=art_vel,
            imu_valid=True,
            joints_valid=True,
            command_fresh=not self.controller.watchdog_tripped,
            watchdog_active=self.controller.watchdog_tripped,
        )

    def _apply_control(self, dt: float, wall_now: float) -> dict[str, float]:
        self.controller.expire_if_stale(wall_now=wall_now, sim_time=float(self.data.time))
        if self._podded:
            rob = self._robot_observation()
            out = self.posture.step(rob, self.controller.target_vx, self.controller.target_wz)
            approved = self.supervisor.approve(out, rob)
            if approved.rejected or self.supervisor.mode in ("fault", "paused"):
                self.controller.target_vx = approved.approved_drive_vx
                self.controller.target_wz = approved.approved_drive_wz
            for name, target in approved.approved_art.items():
                self.data.ctrl[self._art_act[name]] = target
        tau = self.controller.torque_command(self._joint_vel(), dt=dt)
        if self.params.morphology == "tracks" and abs(self.controller.target_wz) > 1e-4:
            tau = _boost_track_yaw(
                tau,
                limit=self.params.max_wheel_torque,
                vx=self.controller.target_vx,
                wz=self.controller.target_wz,
                max_wz=self.params.max_wz,
            )
        if self._podded:
            n_roll = self._n_roll
            applied: dict[str, float] = {}
            for primary in WHEEL_JOINTS:
                share = tau.get(primary, 0.0) / n_roll
                for sib in self._siblings(primary):
                    self.data.ctrl[self._act_id[sib]] = share
                    applied[sib] = share
            tau = applied
        else:
            for name, value in tau.items():
                self.data.ctrl[self._act_id[name]] = value
        angle = float(self.data.qpos[self._probe_qpos])
        target = self.probe.tick(sim_time=float(self.data.time), angle=angle)
        if self.probe.config.stuck:
            pass
        elif self.probe.state == "FAULT":
            # Hold the hinge. A retraction fault must not keep driving it shut.
            self.data.ctrl[self._probe_act] = angle
        else:
            self.data.ctrl[self._probe_act] = target
        return tau

    def _contacts(self) -> list[ContactHit]:
        hits: list[ContactHit] = []
        for i in range(int(self.data.ncon)):
            c = self.data.contact[i]
            g1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom1)) or f"geom{c.geom1}"
            g2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom2)) or f"geom{c.geom2}"
            hits.append(
                ContactHit(
                    geom1=g1,
                    geom2=g2,
                    dist=float(c.dist),
                    pos=(float(c.pos[0]), float(c.pos[1]), float(c.pos[2])),
                )
            )
        return hits

    def _publish(self, tau: dict[str, float] | None = None, *, integrate_thermal: bool = True) -> GroundTruthState:
        adr = self._root_qposadr
        dof = self._root_dofadr
        quat = np.array(self.data.qpos[adr + 3 : adr + 7], dtype=np.float64)
        lin_world = np.array(self.data.qvel[dof : dof + 3], dtype=np.float64)
        ang_body = np.array(self.data.qvel[dof + 3 : dof + 6], dtype=np.float64)
        lin_body = _rotate_world_to_body(quat, lin_world)
        roll, pitch = self._roll_pitch(quat)
        mujoco.mj_energyPos(self.model, self.data)
        mujoco.mj_energyVel(self.model, self.data)
        applied = tau or {name: float(self.data.ctrl[self._act_id[name]]) for name in WHEEL_JOINTS}
        gt = GroundTruthState(
            sim_time=float(self.data.time),
            wall_time=time.time(),
            root_pos=(
                float(self.data.qpos[adr]),
                float(self.data.qpos[adr + 1]),
                float(self.data.qpos[adr + 2]),
            ),
            root_quat_wxyz=(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
            root_linvel_world=(float(lin_world[0]), float(lin_world[1]), float(lin_world[2])),
            root_angvel_body=(float(ang_body[0]), float(ang_body[1]), float(ang_body[2])),
            yaw=_yaw_from_wxyz(quat),
            joint_pos=self._joint_pos(),
            joint_vel=self._joint_vel(),
            commanded_wheel_vel=dict(self.controller.slewed_wheel),
            applied_torque=applied,
            contacts=self._contacts(),
            ncon=int(self.data.ncon),
            kinetic_energy=float(self.data.energy[1]),
            potential_energy=float(self.data.energy[0]),
            watchdog_active=self.controller.watchdog_tripped,
            watchdog_age_wall=self.controller.watchdog_age(time.time()),
            paused=self._paused,
            target_vx=self.controller.target_vx,
            target_wz=self.controller.target_wz,
            actual_vx_body=float(lin_body[0]),
            actual_wz=float(ang_body[2]),
            real_time_factor=self._rtf,
            physics_step_latency_s=self._step_latency,
            saturated=any(abs(v) >= 0.98 * (self.params.max_wheel_torque / float(self._n_roll) if self._podded else self.params.max_wheel_torque) for v in applied.values()),
            command_source=self.controller.last_command.source if self.controller.last_command else "none",
            roll=roll,
            pitch=pitch,
            supervisor_mode=self.supervisor.mode,
            supervisor_reason=self.supervisor.reason,
            art_angles={n: float(self.data.qpos[adr]) for n, adr in self._art_qpos.items()},
        )
        if integrate_thermal:
            self.thermal.step_lumps(gt.root_pos, gt.sim_time, dt=float(self.model.opt.timestep))
        obs = self.sensors.observe(gt, model=self.model, data=self.data, thermal=self.thermal)
        obs.probe_kind = "hinge_boom_local_proposal"
        obs.probe_state = self.probe.state
        obs.probe_angle = float(self.data.qpos[self._probe_qpos])
        obs.probe_valid = self.probe.state != "FAULT"
        self._record_replay(gt, obs)
        with self._state_lock:
            self._latest = gt
            self._latest_obs = obs
        return gt

    def _record_replay(self, gt: GroundTruthState, obs: SensorObservations) -> None:
        thermal_max = max(obs.thermal_c) if obs.thermal_c else obs.ambient_c
        lidar_min = min(obs.lidar_ranges) if obs.lidar_ranges else obs.lidar_max_range
        self.replay.append(
            ReplayFrame(
                sim_time=gt.sim_time,
                tele_velocity=self._last_tele.velocity,
                tele_turn=self._last_tele.turn,
                command_vx=gt.target_vx,
                command_wz=gt.target_wz,
                measured_vx_body=gt.actual_vx_body,
                measured_wz=gt.actual_wz,
                odom_x=obs.wheel_odom_x,
                odom_y=obs.wheel_odom_y,
                odom_yaw=obs.wheel_odom_yaw,
                gt_x=gt.root_pos[0],
                gt_y=gt.root_pos[1],
                gt_yaw=gt.yaw,
                mission_state=self.inspection.state if self.inspection.state != "IDLE" else self.mission.state,
                probe_c=obs.probe_c,
                thermal_max_c=thermal_max,
                lidar_min=lidar_min,
                source=gt.command_source,
                watchdog=gt.watchdog_active,
            )
        )

    def _mission_view(self, obs: SensorObservations, gt: GroundTruthState) -> MissionView:
        return MissionView(
            sim_time=obs.sim_time,
            stamp_wall=obs.stamp_wall,
            odom_x=obs.wheel_odom_x,
            odom_y=obs.wheel_odom_y,
            odom_yaw=obs.wheel_odom_yaw,
            lidar_ranges=list(obs.lidar_ranges),
            lidar_angles=list(obs.lidar_angles),
            lidar_stamp_sim=obs.lidar_stamp_sim,
            thermal_c=list(obs.thermal_c),
            thermal_angles=list(obs.thermal_angles),
            probe_c=obs.probe_c,
            probe_stamp_sim=obs.probe_stamp_sim,
            probe_frame=obs.probe_frame,
            probe_state=obs.probe_state,
            probe_valid=obs.probe_valid,
            lidar_valid=obs.lidar_valid,
            watchdog_active=gt.watchdog_active,
            lidar_max_range=obs.lidar_max_range,
            ambient_c=obs.ambient_c,
            roll_rad=self._roll_pitch(np.array(obs.imu_quat_wxyz, dtype=np.float64))[0],
            pitch_rad=self._roll_pitch(np.array(obs.imu_quat_wxyz, dtype=np.float64))[1],
            imu_valid=True,
        )

    def _trace_inspection(self, gt: GroundTruthState, obs: SensorObservations, tele: TeleCmd) -> None:
        """Evaluation stream. The inspection controller does not read this log."""
        if self.inspection.state == "IDLE":
            return
        if self.inspection_trace:
            last_t = float(self.inspection_trace[-1]["sim_time"])
            if gt.sim_time - last_t < 0.10 and self.inspection.state == self.inspection_trace[-1]["mission_state"]:
                return
        hits = [
            {"geom1": c.geom1, "geom2": c.geom2, "dist": c.dist}
            for c in gt.contacts
            if c.dist < 0.002
        ]
        self.inspection_trace.append(
            {
                "sim_time": gt.sim_time,
                "mission_state": self.inspection.state,
                "reason": self.inspection.reason,
                "tele_velocity": tele.velocity,
                "tele_turn": tele.turn,
                "odom_x": obs.wheel_odom_x,
                "odom_y": obs.wheel_odom_y,
                "odom_yaw": obs.wheel_odom_yaw,
                "lidar_valid": obs.lidar_valid,
                "lidar_stamp_sim": obs.lidar_stamp_sim,
                "lidar_n": len(obs.lidar_ranges),
                "probe_state": obs.probe_state,
                "probe_c": obs.probe_c,
                "gt_x": gt.root_pos[0],
                "gt_y": gt.root_pos[1],
                "gt_yaw": gt.yaw,
                "gt_speed": math.hypot(gt.root_linvel_world[0], gt.root_linvel_world[1]),
                "contacts": hits,
            }
        )

    def _tick_mission(self, wall_now: float) -> None:
        if self.gate.mode != "mission":
            return
        gt, obs = self.snapshot()
        view = self._mission_view(obs, gt)
        if self.inspection.active or self.inspection.state in ("COMPLETE", "BLOCKED", "FAULT", "CANCELLED"):
            if self.inspection.state in ("COMPLETE", "BLOCKED", "FAULT", "CANCELLED"):
                already = self.inspection_trace and self.inspection_trace[-1]["mission_state"] in (
                    "COMPLETE",
                    "BLOCKED",
                    "FAULT",
                    "CANCELLED",
                )
                if not already:
                    self._trace_inspection(gt, obs, TeleCmd(0.0, 0.0, wall_now, source="inspection"))
                return
            tele = self.inspection.tick(view, wall=wall_now)
            if self.inspection.probe_command == "deploy":
                self.probe.request_deploy(float(self.data.time))
            elif self.inspection.probe_command == "stow":
                self.probe.request_stow(float(self.data.time))
            if self.probe.extended or self.inspection.state in (
                "STOP_FOR_INSPECTION",
                "DEPLOY_PROBE",
                "SAMPLE",
                "RETRACT_PROBE",
            ):
                tele = TeleCmd(0.0, 0.0, tele.timestamp_wall, source=tele.source)
            self._trace_inspection(gt, obs, tele)
            self._apply_tele(tele, wall_now)
            return
        if self.mission.state in ("IDLE", "COMPLETE", "FAULT"):
            return
        tele = self.mission.tick(view, wall=wall_now)
        if self.mission.probe_command == "deploy":
            self.probe.request_deploy(float(self.data.time))
        elif self.mission.probe_command == "stow":
            self.probe.request_stow(float(self.data.time))
        if self.probe.extended:
            tele = TeleCmd(0.0, tele.turn, tele.timestamp_wall, source=tele.source)
        self._apply_tele(tele, wall_now)

    def step_physics(self, n: int = 1) -> GroundTruthState:
        """Integrate n fixed steps. Never skips steps to fake real-time.

        While paused, simulation time does not advance and drive commands are
        discarded. A single-step request integrates one control burst.
        """
        wall_now = time.time()
        self._drain_requests(wall_now)
        take_step = self._single_step
        self._single_step = False
        if self._paused and not take_step:
            return self._publish(integrate_thermal=False)
        self._tick_mission(wall_now)
        dt = float(self.model.opt.timestep)
        tau = {name: 0.0 for name in WHEEL_JOINTS}
        t0 = time.perf_counter()
        for _ in range(n):
            tau = self._apply_control(dt=dt, wall_now=wall_now)
            with self._state_lock:
                mujoco.mj_step(self.model, self.data)
                self._steps += 1
        self._step_latency = time.perf_counter() - t0
        elapsed_wall = max(time.perf_counter() - self._sim_wall_anchor, 1e-9)
        self._rtf = float(self.data.time) / elapsed_wall if elapsed_wall > 0 else 0.0
        return self._publish(tau)

    def step_control_tick(self) -> GroundTruthState:
        return self.step_physics(self._control_period_steps)

    def run_for(self, duration_s: float, command: DriveCommand | None = None) -> GroundTruthState:
        ticks = int(math.ceil(duration_s * self.params.control_hz))
        gt = self.snapshot()[0]
        for _ in range(ticks):
            if command is not None:
                self.drive(command.vx, command.wz, source=command.source)
            gt = self.step_control_tick()
        return gt

    def start_background(self) -> None:
        if self._physics_thread and self._physics_thread.is_alive():
            return
        self._stop = False
        self._sim_wall_anchor = time.perf_counter()

        def _loop() -> None:
            next_tick = time.perf_counter()
            period = 1.0 / self.params.control_hz
            while not self._stop:
                was_paused = self._paused
                self.step_control_tick()
                if self._paused:
                    time.sleep(0.02)
                    next_tick = time.perf_counter()
                    continue
                if was_paused:
                    next_tick = time.perf_counter()
                next_tick += period
                sleep_for = next_tick - time.perf_counter()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    # Behind real time: do not skip integration; catch up.
                    next_tick = time.perf_counter()

        self._physics_thread = threading.Thread(target=_loop, name="embr-physics", daemon=True)
        self._physics_thread.start()

    def stop_background(self) -> None:
        self._stop = True
        if self._physics_thread:
            self._physics_thread.join(timeout=2.0)

    def finite(self) -> bool:
        return bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all())
