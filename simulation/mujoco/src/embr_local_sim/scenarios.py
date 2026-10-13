"""YAML scenarios, headless runner, and pass-criteria evaluation."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from embr_local_sim.metrics import RunLog, append_csv, write_json, write_markdown_report
from embr_local_sim.model import RobotParams, WorldSpec
from embr_local_sim.paths import repo_root, scenarios_dir
from embr_local_sim.physics import Simulator
from embr_local_sim.types import DriveCommand, GroundTruthState

__all__ = ["repo_root", "scenarios_dir"]


@dataclass
class Scenario:
    name: str
    seed: int
    duration_s: float
    settle_s: float
    world: WorldSpec
    params_overrides: dict[str, Any]
    commands: list[dict[str, Any]]
    pass_criteria: dict[str, Any]
    notes: str = ""
    raw: dict[str, Any] = field(default_factory=dict)


def load_scenario(path: Path) -> Scenario:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    world_raw = raw.get("world") or {}
    kind = world_raw.get("kind", "flat")
    extras = dict(world_raw.get("extras") or {})
    if world_raw.get("detailed"):
        extras["detailed_forest"] = True
    world = WorldSpec(
        kind=kind,
        wall_x=float(world_raw.get("wall_x", 0.50)),
        ramp_angle_deg=float(world_raw.get("ramp_angle_deg", 8.0)),
        ramp_length=float(world_raw.get("ramp_length", 0.70)),
        ramp_start_x=float(world_raw.get("ramp_start_x", 0.28)),
        left_friction=world_raw.get("left_friction"),
        right_friction=world_raw.get("right_friction"),
        stones=list(world_raw.get("stones") or []),
        logs=list(world_raw.get("logs") or []),
        heat=list(world_raw.get("heat") or []),
        ambient_c=float(world_raw.get("ambient_c", 20.0)),
        extras=extras,
        seed=int(raw.get("seed", 0)),
    )
    return Scenario(
        name=str(raw.get("name", path.stem)),
        seed=int(raw.get("seed", 0)),
        duration_s=float(raw.get("duration_s", 3.0)),
        settle_s=float(raw.get("settle_s", 0.6)),
        world=world,
        params_overrides=dict(raw.get("params") or {}),
        commands=list(raw.get("commands") or []),
        pass_criteria=dict(raw.get("pass") or {}),
        notes=str(raw.get("notes") or ""),
        raw=raw,
    )


def list_scenarios() -> list[Path]:
    return sorted(scenarios_dir().glob("*.yaml"))


def _command_at(commands: list[dict[str, Any]], sim_t: float, wall: float) -> DriveCommand:
    active = DriveCommand(0.0, 0.0, wall, source="scenario", timestamp_sim=sim_t)
    for item in commands:
        t0 = float(item.get("t", 0.0))
        t1 = t0 + float(item.get("duration", 0.0))
        if t0 <= sim_t < t1:
            active = DriveCommand(
                vx=float(item.get("vx", 0.0)),
                wz=float(item.get("wz", 0.0)),
                timestamp_wall=wall,
                source="scenario",
                timestamp_sim=sim_t,
            )
    return active


def evaluate(criteria: dict[str, Any], samples: list[GroundTruthState], sim: Simulator) -> list[str]:
    failures: list[str] = []
    if not samples:
        return ["no samples"]
    first, last = samples[0], samples[-1]
    xs = [s.root_pos[0] for s in samples]
    ys = [s.root_pos[1] for s in samples]
    zs = [s.root_pos[2] for s in samples]
    speeds = [math.hypot(s.actual_vx_body, s.actual_wz * sim.params.wheel_separation) for s in samples]
    if criteria.get("no_nan", True):
        if not all(sim.finite() for _ in [0]) or not all(
            math.isfinite(s.root_pos[0]) and math.isfinite(s.kinetic_energy) for s in samples
        ):
            failures.append("NaN or inf in trajectory")
    if "max_abs_penetration" in criteria:
        worst = min((c.dist for s in samples for c in s.contacts), default=0.0)
        if worst < -float(criteria["max_abs_penetration"]):
            failures.append(f"penetration {worst:.4f} deeper than allowed")
    if "max_final_speed" in criteria and abs(last.actual_vx_body) > float(criteria["max_final_speed"]):
        failures.append(f"final vx {last.actual_vx_body:.3f} exceeds rest bound")
    if "stable_z_range" in criteria:
        lo, hi = criteria["stable_z_range"]
        if not (float(lo) <= last.root_pos[2] <= float(hi)):
            failures.append(f"final z {last.root_pos[2]:.4f} outside [{lo}, {hi}]")
    if "min_final_x" in criteria and last.root_pos[0] < float(criteria["min_final_x"]):
        failures.append(f"final x {last.root_pos[0]:.3f} < {criteria['min_final_x']}")
    if "max_final_x" in criteria and last.root_pos[0] > float(criteria["max_final_x"]):
        failures.append(f"final x {last.root_pos[0]:.3f} > {criteria['max_final_x']}")
    if "max_abs_final_y" in criteria and abs(last.root_pos[1]) > float(criteria["max_abs_final_y"]):
        failures.append(f"final |y| {last.root_pos[1]:.3f} too large")
    if "min_displacement_x" in criteria:
        dx = last.root_pos[0] - first.root_pos[0]
        if dx < float(criteria["min_displacement_x"]):
            failures.append(f"dx {dx:.3f} < {criteria['min_displacement_x']}")
    if "max_displacement_x" in criteria:
        dx = last.root_pos[0] - first.root_pos[0]
        if dx > float(criteria["max_displacement_x"]):
            failures.append(f"dx {dx:.3f} > {criteria['max_displacement_x']}")
    if "sign_dx" in criteria:
        dx = last.root_pos[0] - first.root_pos[0]
        if math.copysign(1.0, dx) != math.copysign(1.0, float(criteria["sign_dx"])) or abs(dx) < 0.02:
            failures.append(f"dx sign {dx:.3f} did not match {criteria['sign_dx']}")
    if criteria.get("wall_blocked"):
        wall_face = sim.world.wall_x - 0.5 * sim.world.wall_thickness
        chassis_half = 0.5 * sim.params.chassis_size[0]
        front = max(xs) + chassis_half
        if front > wall_face + 0.012:
            failures.append(
                f"chassis crossed wall envelope (front={front:.3f} wall_face={wall_face:.3f})"
            )
        names = {c.geom1 for s in samples for c in s.contacts} | {c.geom2 for s in samples for c in s.contacts}
        if "wall" not in names:
            failures.append("never contacted wall geom")
    if "min_peak_ncon" in criteria and max(s.ncon for s in samples) < int(criteria["min_peak_ncon"]):
        failures.append(f"peak ncon {max(s.ncon for s in samples)} < {criteria['min_peak_ncon']}")
    if "min_peak_z" in criteria and max(zs) < float(criteria["min_peak_z"]):
        failures.append(f"peak z {max(zs):.4f} < {criteria['min_peak_z']}")
    if "max_kinetic_after_settle" in criteria:
        k = first.kinetic_energy
        if k > float(criteria["max_kinetic_after_settle"]):
            failures.append(f"kinetic after settle {k:.4f}")
    if "min_abs_art" in criteria and sim._art_qpos:
        peak = max(abs(float(sim.data.qpos[adr])) for adr in sim._art_qpos.values())
        if peak < float(criteria["min_abs_art"]):
            failures.append(f"peak |art| {peak:.3f} < {criteria['min_abs_art']}")
    if "max_abs_roll" in criteria:
        rolls = [abs(s.roll) for s in samples]
        if max(rolls) > float(criteria["max_abs_roll"]):
            failures.append(f"peak |roll| {max(rolls):.3f} exceeds {criteria['max_abs_roll']}")
    _ = speeds, ys
    return failures


def _mission_config(raw: dict[str, Any]):
    from embr_local_sim.mission import MissionConfig

    m = dict(raw.get("mission") or {})
    cfg = MissionConfig()
    for field in (
        "timeout_s",
        "detect_c",
        "sample_c",
        "sample_dwell_s",
        "retreat_s",
        "search_velocity",
        "approach_velocity",
        "blocked_range",
        "slow_range",
        "blocked_hold_s",
        "sensor_stale_s",
        "complete_retreat_x",
        "arrive_m",
        "flame_avoid_c",
        "max_tilt_rad",
        "immobilized_s",
    ):
        if field in m:
            setattr(cfg, field, float(m[field]))
    if "level" in m:
        cfg.level = int(m["level"])
    if "goto_x" in m:
        cfg.goto_x = float(m["goto_x"])
    if "goto_y" in m:
        cfg.goto_y = float(m["goto_y"])
    if "oracle" in m:
        cfg.oracle = bool(m["oracle"])
    if "search_bound" in m:
        cfg.search_bound = dict(m["search_bound"])
    return cfg


def run_mission_scenario(path: Path, out_dir: Path | None = None) -> dict[str, Any]:
    from embr_local_sim.mission import MissionMachine

    scenario = load_scenario(path)
    params = RobotParams().with_overrides(**scenario.params_overrides)
    sim = Simulator(params=params, world=scenario.world, seed=scenario.seed)
    sim.reset()
    ticks_settle = int(round(scenario.settle_s * params.control_hz))
    for _ in range(ticks_settle):
        sim.step_control_tick()
    sim.mission = MissionMachine(_mission_config(scenario.raw))
    if scenario.raw.get("probe_stuck"):
        sim.probe.config.stuck = True
    if scenario.raw.get("lidar_dropout"):
        sim.sensors.lidar_dropout = True
    sim.start_mission()
    elapsed = 0.0
    dt = 1.0 / params.control_hz
    samples = []
    while elapsed < scenario.duration_s:
        samples.append(sim.step_control_tick())
        elapsed += dt
        if sim.mission.state in ("COMPLETE", "FAULT") and elapsed > 0.4:
            # Continue a short coast so replay has the terminal state.
            if elapsed > sim.mission.state_entered + 0.3:
                break
    failures = evaluate(scenario.pass_criteria, samples, sim)
    wanted = scenario.pass_criteria.get("mission_end_state")
    if wanted and sim.mission.state != str(wanted):
        failures.append(f"mission ended in {sim.mission.state} ({sim.mission.reason}), want {wanted}")
    min_hot = scenario.pass_criteria.get("min_hotspots")
    if min_hot is not None and len(sim.mission.hotspots) < int(min_hot):
        failures.append(f"hotspots {len(sim.mission.hotspots)} < {min_hot}")
    if scenario.pass_criteria.get("autonomy_used_gt") is False and sim.mission.used_ground_truth:
        failures.append("mission used ground-truth pose")
    needle = scenario.pass_criteria.get("fault_reason_contains")
    if needle and needle not in sim.mission.reason:
        failures.append(f"fault reason {sim.mission.reason!r} missing {needle!r}")
    last = samples[-1] if samples else sim.snapshot()[0]
    result = {
        "name": scenario.name,
        "path": str(path),
        "seed": scenario.seed,
        "passed": not failures,
        "failures": failures,
        "final": {
            "t": last.sim_time,
            "x": last.root_pos[0],
            "y": last.root_pos[1],
            "z": last.root_pos[2],
            "yaw": last.yaw,
            "vx": last.actual_vx_body,
            "ncon": last.ncon,
            "rtf": last.real_time_factor,
            "mission": sim.mission.snapshot(),
        },
        "model_hash": sim.mjcf_hash(),
        "notes": scenario.notes,
        "mode": "mission",
    }
    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        replay_path = out_dir / f"{scenario.name}.replay.jsonl"
        sim.replay.dump(replay_path)
        from embr_local_sim.metrics import RunLog, append_csv, write_json, write_markdown_report

        log = RunLog.from_run(sim, scenario, samples, result)
        write_json(out_dir / f"{scenario.name}.json", log.as_dict())
        append_csv(out_dir / f"{scenario.name}.csv", samples)
        write_markdown_report(out_dir / f"{scenario.name}.md", log)
        result["replay"] = str(replay_path)
    return result


def run_scenario(path: Path, out_dir: Path | None = None) -> dict[str, Any]:
    scenario = load_scenario(path)
    if str(scenario.raw.get("mode", "drive")) == "mission":
        return run_mission_scenario(path, out_dir=out_dir)
    params = RobotParams().with_overrides(**scenario.params_overrides)
    sim = Simulator(params=params, world=scenario.world, seed=scenario.seed)
    sim.reset()
    samples: list[GroundTruthState] = []
    import time as _time

    # Settle with zero command.
    ticks_settle = int(round(scenario.settle_s * params.control_hz))
    for _ in range(ticks_settle):
        samples.append(sim.step_control_tick())
    post_settle = samples[-1] if samples else sim.snapshot()[0]

    run_samples = [post_settle]
    elapsed = 0.0
    dt = 1.0 / params.control_hz
    while elapsed < scenario.duration_s:
        cmd = _command_at(scenario.commands, elapsed, _time.time())
        sim.drive(cmd.vx, cmd.wz, source=cmd.source)
        run_samples.append(sim.step_control_tick())
        elapsed += dt

    failures = evaluate(scenario.pass_criteria, run_samples, sim)
    result = {
        "name": scenario.name,
        "path": str(path),
        "seed": scenario.seed,
        "passed": not failures,
        "failures": failures,
        "final": {
            "t": run_samples[-1].sim_time,
            "x": run_samples[-1].root_pos[0],
            "y": run_samples[-1].root_pos[1],
            "z": run_samples[-1].root_pos[2],
            "yaw": run_samples[-1].yaw,
            "vx": run_samples[-1].actual_vx_body,
            "ncon": run_samples[-1].ncon,
            "rtf": run_samples[-1].real_time_factor,
        },
        "model_hash": sim.mjcf_hash(),
        "notes": scenario.notes,
    }
    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        log = RunLog.from_run(sim, scenario, run_samples, result)
        write_json(out_dir / f"{scenario.name}.json", log.as_dict())
        append_csv(out_dir / f"{scenario.name}.csv", run_samples)
        write_markdown_report(out_dir / f"{scenario.name}.md", log)
    return result
