"""Headless inspection scenarios, evaluation, and run reports.

The controller does not import this module. Evaluation uses the ground-truth
trace recorded beside the mission, not as an input to steering.

Tolerances below are software-experiment requirements for this approximate
robot and synthetic probe. They are not hardware specifications.

- Inspection position error 0.18 m. The estimated arrival radius is 0.12 m and
  the chassis is 0.15 m long, so this allows a small odometry disagreement.
- Return position error 0.22 m. Same idea, plus a little slip on the way back.
- Stopped means ground-truth planar speed under 0.08 m/s. The controller's own
  stop test is 0.03 m/s of wheel odometry.
- A safe failure (blocked, stale scan, retract fault) is a pass when that is
  the scenario's expected outcome. COMPLETE is not required for those cases.
"""

from __future__ import annotations

import hashlib
import json
import math
import subprocess
from pathlib import Path
from typing import Any

from embr_local_sim.inspection import InspectionConfig, world_point_to_estimated
from embr_local_sim.model import RobotParams, WorldSpec
from embr_local_sim.physics import Simulator

# Software-experiment thresholds. See the module docstring for the rationale.
INSPECT_POS_TOL_M = 0.18
RETURN_POS_TOL_M = 0.22
STOP_SPEED_MPS = 0.08
SAMPLE_DWELL_MIN_S = 0.35
DRIVE_WHILE_UNSTOWED = 0.05

_ROLLING = ("maxon", "roller", "wheel")
_BODY = ("chassis", "probe", "pod")
_OBSTACLE = ("stone", "wall", "log", "shrub", "tree", "branch")


def contact_kind(geom1: str, geom2: str) -> str:
    """Separate ground contact from a hit on an obstacle."""
    names = (geom1, geom2)

    def has(keys: tuple[str, ...], name: str) -> bool:
        return any(key in name for key in keys)

    if any("floor" in name for name in names):
        other = names[1] if "floor" in names[0] else names[0]
        if has(_ROLLING, other):
            return "terrain_rolling"
        if has(_BODY, other):
            return "terrain_body"
        return "terrain_other"
    robot = any(has(_ROLLING, name) or has(_BODY, name) for name in names)
    obstacle = any(has(_OBSTACLE, name) for name in names)
    if robot and obstacle:
        return "obstacle"
    return "other"


def _stone(x: float, y: float, sx: float, sy: float, sz: float) -> dict[str, Any]:
    return {"x": x, "y": y, "size": (sx, sy, sz), "movable": False}


def scenario_catalog() -> dict[str, dict[str, Any]]:
    """Five flat-ground cases. Expected outcomes are not motion scripts."""
    return {
        "clear": {
            "title": "Clear inspection",
            "seed": 7,
            "morphology": "tracks",
            "spawn": {"x": 0.0, "y": -0.05, "yaw": 0.40},
            "target_world": (0.55, 0.12),
            "stones": [],
            "lidar_dropout_after": None,
            "retract_fault": False,
            "expected_state": "COMPLETE",
            "note": "Open ground. Reach the target, sample, retract, return, stop.",
        },
        "detour": {
            "title": "Obstacle with a viable detour",
            "seed": 3,
            "morphology": "tracks",
            "spawn": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "target_world": (0.85, 0.05),
            "stones": [_stone(0.42, 0.0, 0.03, 0.045, 0.08)],
            "lidar_dropout_after": None,
            "retract_fault": False,
            "expected_state": "COMPLETE",
            "note": "One post on the straight line. Measured clearance remains on either side.",
        },
        "blocked": {
            "title": "Blocked route",
            "seed": 5,
            "morphology": "tracks",
            "spawn": {"x": 0.0, "y": 0.0, "yaw": 0.15},
            "target_world": (1.05, 0.0),
            "stones": [
                _stone(0.22, 0.18, 0.48, 0.02, 0.08),
                _stone(0.22, -0.18, 0.48, 0.02, 0.08),
                _stone(0.68, 0.0, 0.025, 0.20, 0.08),
            ],
            "lidar_dropout_after": None,
            "retract_fault": False,
            "expected_state": "BLOCKED",
            "note": "A closed corridor. Local sensing cannot see a robot-width exit.",
        },
        "dropout": {
            "title": "Sensor dropout",
            "seed": 11,
            "morphology": "tracks",
            "spawn": {"x": 0.0, "y": 0.0, "yaw": 0.20},
            "target_world": (0.70, 0.0),
            "stones": [],
            "lidar_dropout_after": 0.35,
            "retract_fault": False,
            "expected_state": "FAULT",
            "expected_reason_contains": ("sensor_invalid", "stale_lidar"),
            "note": "Range delivery stops at 0.35 s of simulation time. The scan is not replaced with free space.",
        },
        "retract": {
            "title": "Probe retraction failure",
            "seed": 13,
            "morphology": "tracks",
            "spawn": {"x": 0.10, "y": 0.05, "yaw": -0.25},
            "target_world": (0.48, 0.02),
            "stones": [],
            "lidar_dropout_after": None,
            "retract_fault": True,
            "expected_state": "FAULT",
            "expected_reason_contains": ("probe_retract_failed",),
            "note": "The hinge faults when retraction is requested, after a real sample.",
        },
    }


def _world(spec: dict[str, Any]) -> WorldSpec:
    return WorldSpec(kind="flat", stones=list(spec["stones"]), seed=int(spec["seed"]))


def _git_fingerprint(repo: Path) -> dict[str, str]:
    git = "/opt/homebrew/bin/git"
    try:
        commit = subprocess.check_output([git, "-C", str(repo), "rev-parse", "--short", "HEAD"], text=True).strip()
        dirty = subprocess.check_output([git, "-C", str(repo), "status", "--porcelain"], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return {"commit": "unknown", "dirty": "unknown"}
    return {"commit": commit, "dirty": "yes" if dirty else "no"}


def run_case(
    name: str,
    *,
    morphology: str | None = None,
    target_world: tuple[float, float] | None = None,
    spawn: dict[str, float] | None = None,
    seed: int | None = None,
    stones: list | None = None,
    timeout_s: float | None = None,
    out_dir: Path | None = None,
) -> dict[str, Any]:
    catalog = scenario_catalog()
    if name not in catalog:
        raise KeyError(name)
    spec = dict(catalog[name])
    if morphology is not None:
        spec["morphology"] = morphology
    if target_world is not None:
        spec["target_world"] = target_world
    if spawn is not None:
        spec["spawn"] = spawn
    if seed is not None:
        spec["seed"] = seed
    if stones is not None:
        spec["stones"] = stones
    morph = str(spec["morphology"])
    params = RobotParams(morphology=morph)  # type: ignore[arg-type]
    sim = Simulator(params=params, world=_world(spec), seed=int(spec["seed"]))
    sim.reset(dict(spec["spawn"]))
    sim.sensors.lidar_dropout_after = spec["lidar_dropout_after"]
    sim.probe.config.retract_fault = bool(spec["retract_fault"])
    sx, sy, syaw = sim.spawn_pose
    tx, ty = spec["target_world"]
    goal_x, goal_y = world_point_to_estimated(sx, sy, syaw, tx, ty)
    cfg = InspectionConfig(goal_x=goal_x, goal_y=goal_y)
    if timeout_s is not None:
        cfg.mission_timeout_s = timeout_s
    sim.start_inspection(cfg)
    limit = cfg.mission_timeout_s + 1.0
    dt = 1.0 / params.control_hz
    elapsed = 0.0
    while elapsed < limit:
        sim.step_control_tick()
        elapsed += dt
        if sim.inspection.state in ("COMPLETE", "BLOCKED", "FAULT", "CANCELLED"):
            # Let the stop command settle. One tick is not enough for the body to stop.
            for _ in range(int(0.8 * params.control_hz)):
                sim.step_control_tick()
            break
    if sim.inspection_trace:
        gt, _obs = sim.snapshot()
        sim.inspection_trace[-1]["gt_speed"] = math.hypot(gt.root_linvel_world[0], gt.root_linvel_world[1])
        sim.inspection_trace[-1]["gt_x"] = gt.root_pos[0]
        sim.inspection_trace[-1]["gt_y"] = gt.root_pos[1]
        sim.inspection_trace[-1]["gt_yaw"] = gt.yaw
    report = _report(sim, spec, goal_x, goal_y)
    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        path = out_dir / f"{name}-{morph}-seed{spec['seed']}.json"
        path.write_text(json.dumps(report, indent=2))
        report["report_path"] = str(path)
    return report


def _report(sim: Simulator, spec: dict[str, Any], goal_x: float, goal_y: float) -> dict[str, Any]:
    repo = Path(__file__).resolve().parents[4]
    trace = list(sim.inspection_trace)
    verdict = evaluate_trace(
        spec,
        trace,
        sim.inspection.measurement_c,
        sim.inspection.measurement_class,
        transitions=list(sim.inspection.transitions),
    )
    cfg_blob = json.dumps(
        {
            "scenario": spec,
            "goal_odom": [goal_x, goal_y],
            "tolerances": {
                "inspect_m": INSPECT_POS_TOL_M,
                "return_m": RETURN_POS_TOL_M,
                "stop_mps": STOP_SPEED_MPS,
            },
        },
        sort_keys=True,
        default=str,
    )
    machine = sim.inspection.snapshot()
    machine["elapsed_sim_s"] = None if not trace else trace[-1]["sim_time"] - (sim.inspection.started_sim or trace[0]["sim_time"])
    return {
        "scenario": spec["title"],
        "case": spec.get("title"),
        "seed": spec["seed"],
        "morphology": spec["morphology"],
        "spawn": spec["spawn"],
        "target_world": spec["target_world"],
        "goal_odom": [goal_x, goal_y],
        "faults": {
            "lidar_dropout_after": spec["lidar_dropout_after"],
            "retract_fault": spec["retract_fault"],
        },
        "note": spec["note"],
        "git": _git_fingerprint(repo),
        "model_hash": sim.mjcf_hash(),
        "config_hash": hashlib.sha256(cfg_blob.encode()).hexdigest()[:16],
        "controller": {
            "state": sim.inspection.state,
            "reason": sim.inspection.reason,
            "help": sim.inspection.help_text(),
            "dist_to_target_m": sim.inspection.dist_to_target_m,
            "dist_to_start_m": sim.inspection.dist_to_start_m,
            "measurement_c": sim.inspection.measurement_c,
            "measurement_class": sim.inspection.measurement_class,
            "transitions": machine["transitions"],
            "used_ground_truth": False,
        },
        "evaluator": verdict,
        "trace_samples": len(trace),
        "trace": trace,
    }


def _sample_dwell(trace: list[dict[str, Any]], transitions: list[dict[str, Any]] | None) -> float:
    if transitions:
        sample_t = next((row["t"] for row in transitions if row["state"] == "SAMPLE"), None)
        retract_t = next((row["t"] for row in transitions if row["state"] == "RETRACT_PROBE"), None)
        if sample_t is not None and retract_t is not None:
            return float(retract_t) - float(sample_t)
    sample_rows = [row for row in trace if row["mission_state"] == "SAMPLE"]
    if len(sample_rows) >= 2:
        return float(sample_rows[-1]["sim_time"] - sample_rows[0]["sim_time"])
    return 0.0


def evaluate_trace(
    spec: dict[str, Any],
    trace: list[dict[str, Any]],
    measurement_c: float | None,
    measurement_class: str | None,
    transitions: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    failures: list[str] = []
    if not trace:
        return {"pass": False, "failures": ["no evaluation trace"], "metrics": {}}
    start = trace[0]
    end = trace[-1]
    expected = str(spec["expected_state"])
    state = str(end["mission_state"])
    reason = str(end["reason"])
    if state != expected:
        failures.append(f"controller ended {state} ({reason}), expected {expected}")
    needles = spec.get("expected_reason_contains") or ()
    if needles and not any(needle in reason for needle in needles):
        failures.append(f"reason {reason!r} did not contain {needles}")
    if "unreachable" in reason:
        failures.append("reason claims the target is unreachable")

    sample_rows = [row for row in trace if row["mission_state"] == "SAMPLE" and row["probe_state"] == "SAMPLING"]
    deploy_rows = [row for row in trace if row["mission_state"] == "DEPLOY_PROBE"]
    inspect_error = None
    if sample_rows:
        sx, sy = spec["target_world"]
        row = sample_rows[0]
        inspect_error = math.hypot(row["gt_x"] - sx, row["gt_y"] - sy)
    dwell = _sample_dwell(trace, transitions)
    if expected == "COMPLETE" and sample_rows and dwell + 1e-6 < SAMPLE_DWELL_MIN_S:
        failures.append(f"sample dwell {dwell:.2f}s < {SAMPLE_DWELL_MIN_S}")
    elif expected == "COMPLETE" and not sample_rows:
        failures.append("never sampled")

    return_error = math.hypot(end["gt_x"] - start["gt_x"], end["gt_y"] - start["gt_y"])
    if expected == "COMPLETE":
        if inspect_error is None or inspect_error > INSPECT_POS_TOL_M:
            failures.append(f"inspection position error {inspect_error} > {INSPECT_POS_TOL_M}")
        if return_error > RETURN_POS_TOL_M:
            failures.append(f"return position error {return_error:.3f} > {RETURN_POS_TOL_M}")
        if end["probe_state"] != "STOWED":
            failures.append(f"probe {end['probe_state']} at completion")
        if end["gt_speed"] > STOP_SPEED_MPS:
            failures.append(f"still moving at completion ({end['gt_speed']:.3f} m/s)")
        obstacles = [
            hit
            for row in trace
            for hit in row["contacts"]
            if contact_kind(hit["geom1"], hit["geom2"]) == "obstacle"
        ]
        if obstacles:
            failures.append(f"obstacle contact {obstacles[0]}")
        if measurement_c is None or measurement_class not in ("cold", "warm", "hot"):
            failures.append("measurement was not classified")

    if deploy_rows and expected in ("COMPLETE", "FAULT") and spec.get("retract_fault"):
        pass
    if deploy_rows and expected == "COMPLETE":
        if deploy_rows[0]["gt_speed"] > STOP_SPEED_MPS:
            failures.append(f"moved during deploy ({deploy_rows[0]['gt_speed']:.3f} m/s)")

    drove_early = [
        row
        for row in trace
        if row["mission_state"] in ("RETRACT_PROBE", "DEPLOY_PROBE", "SAMPLE") and abs(row["tele_velocity"]) > DRIVE_WHILE_UNSTOWED
    ]
    if drove_early:
        failures.append("drive command while the probe was not confirmed stowed")

    if expected == "FAULT" and spec.get("retract_fault"):
        fault_rows = [row for row in trace if row["mission_state"] == "FAULT"]
        if fault_rows:
            fx, fy = fault_rows[0]["gt_x"], fault_rows[0]["gt_y"]
            slipped = math.hypot(end["gt_x"] - fx, end["gt_y"] - fy)
            if slipped > 0.08:
                failures.append(f"drove {slipped:.3f} m after the retract fault")
        if not any(row["probe_state"] == "SAMPLING" for row in trace):
            failures.append("retract fault fired before a sampling pose")

    if expected == "FAULT" and spec.get("lidar_dropout_after") is not None:
        if end["gt_speed"] > STOP_SPEED_MPS:
            failures.append(f"kept moving after the sensor fault ({end['gt_speed']:.3f} m/s)")

    if expected == "BLOCKED":
        tx, ty = spec["target_world"]
        reached = math.hypot(end["gt_x"] - tx, end["gt_y"] - ty)
        if reached < INSPECT_POS_TOL_M:
            failures.append("reported blocked but the chassis is at the target")
        if end["gt_speed"] > STOP_SPEED_MPS:
            failures.append(f"blocked but still moving ({end['gt_speed']:.3f} m/s)")

    return {
        "pass": not failures,
        "failures": failures,
        "metrics": {
            "controller_state": state,
            "controller_reason": reason,
            "inspection_position_error_m": inspect_error,
            "return_position_error_m": return_error,
            "sample_dwell_s": dwell,
            "final_speed_mps": end["gt_speed"],
            "measurement_c": measurement_c,
            "measurement_class": measurement_class,
            "final_gt": [end["gt_x"], end["gt_y"], end["gt_yaw"]],
            "start_gt": [start["gt_x"], start["gt_y"], start["gt_yaw"]],
        },
    }


def run_catalog(out_dir: Path) -> list[dict[str, Any]]:
    results = []
    for name in scenario_catalog():
        results.append(run_case(name, out_dir=out_dir))
    return results


def main() -> None:
    out = Path("embr-sim-runs") / "inspection"
    results = run_catalog(out)
    for item in results:
        ev = item["evaluator"]
        ctrl = item["controller"]
        print(
            f"{item['scenario']}: controller {ctrl['state']} ({ctrl['reason']}) "
            f"evaluator {'PASS' if ev['pass'] else 'FAIL'} {ev['failures']}"
        )


if __name__ == "__main__":
    main()
