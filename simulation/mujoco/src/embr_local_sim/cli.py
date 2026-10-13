"""Launch, headless checks, and tests for the MuJoCo baseline."""

from __future__ import annotations

import argparse
import dataclasses
import sys
import time
from pathlib import Path
from typing import Any

import yaml

from embr_local_sim.model import RobotParams, WorldSpec
from embr_local_sim.paths import config_dir, repo_root
from embr_local_sim.scenarios import list_scenarios, run_scenario

SCENES = ("basic", "course", "grove", "forest", "engineering", "wildfire")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="embr-local-sim",
        description=(
            "EMBR MuJoCo simulation baseline. `launch` opens the grove and the "
            "track-roller approximation, not the full EMBR and not a belt. "
            "Viser only displays this process; it is not a standalone website."
        ),
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    launch = sub.add_parser("launch", help="Open the Viser viewer for a scene")
    _add_common(launch)
    launch.add_argument("--host", default="127.0.0.1", help="Bind address (default: 127.0.0.1)")
    launch.add_argument("--port", type=int, default=43147, help="Browser server port (default: 43147)")
    launch.add_argument(
        "--mission",
        action="store_true",
        help="Start the optional sensor-driven mission after launch. This is a demonstration, not field autonomy.",
    )

    headless = sub.add_parser("headless", help="Run physics without a viewer or browser")
    _add_common(headless)
    headless.add_argument("--steps", type=int, default=None, help="Control ticks to integrate after settle")
    headless.add_argument(
        "--vx",
        type=float,
        default=0.20,
        help="Forward speed command in m/s during --steps (0 runs with no drive command)",
    )
    headless.add_argument("--scenario", default=None, help="Scenario stem or YAML path")
    headless.add_argument("--all", action="store_true", help="Run every packaged YAML scenario")
    headless.add_argument(
        "--out",
        default=None,
        help="Directory for scenario JSON/CSV logs (default: ./embr-sim-runs)",
    )

    sub.add_parser("test", help="Run pytest for this package")
    sub.add_parser("doctor", help="Report Python dependencies. ROS is not required.")
    write_model = sub.add_parser("write-model", help="Write generated MJCF to a file")
    write_model.add_argument("--out", default=None, help="Output XML path")
    return parser


def _add_common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--scene",
        default=None,
        choices=SCENES,
        help="launch opens grove. headless defaults to basic. Also course, forest, engineering, wildfire.",
    )
    parser.add_argument("--config", default=None, help="YAML config path. Overrides packaged defaults.")
    parser.add_argument("--seed", type=int, default=None, help="Seed for randomized scenes")
    parser.add_argument(
        "--morphology",
        default=None,
        choices=("wheels", "articulated", "tracks"),
        help="launch opens tracks (5 rollers per module, not a belt). headless defaults to wheels. articulated = 3-roller pods.",
    )


def _load_config(path: str | None) -> dict[str, Any]:
    if path is None:
        return {}
    file = Path(path)
    if not file.is_file():
        raise FileNotFoundError(
            f"Configuration file not found: {file}. "
            f"Packaged examples are in {config_dir()}."
        )
    raw = yaml.safe_load(file.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"Configuration must be a YAML mapping: {file}")
    return raw


def _resolve_run(args: argparse.Namespace) -> tuple[RobotParams, WorldSpec, str, int]:
    cfg = _load_config(args.config)
    if getattr(args, "cmd", None) == "launch":
        scene_default, morph_default = "grove", "tracks"
    else:
        scene_default, morph_default = "basic", "wheels"
    scene = args.scene or str(cfg.get("scene", scene_default))
    if scene not in SCENES:
        raise ValueError(f"Unknown scene {scene!r}. Choose one of: {', '.join(SCENES)}.")
    seed = int(args.seed if args.seed is not None else cfg.get("seed", 1))
    morphology = args.morphology or str(cfg.get("morphology", morph_default))
    if morphology not in ("wheels", "articulated", "tracks"):
        raise ValueError("morphology must be 'wheels', 'articulated', or 'tracks'")
    overrides = dict(cfg.get("params") or {})
    known = {field.name for field in dataclasses.fields(RobotParams)}
    unknown = sorted(set(overrides) - known)
    if unknown:
        raise ValueError(f"Unknown config params {unknown}. Known fields include {sorted(known)}.")
    params = RobotParams(morphology=morphology).with_overrides(**overrides)
    from embr_local_sim.worlds import world_for_mode

    world = world_for_mode(scene, seed=seed)
    return params, world, scene, seed


def _run_steps(args: argparse.Namespace) -> int:
    from embr_local_sim.physics import Simulator
    from embr_local_sim.types import DriveCommand

    params, world, scene, seed = _resolve_run(args)
    steps = int(args.steps)
    if steps < 1:
        print("--steps must be at least 1.", file=sys.stderr)
        return 2
    sim = Simulator(params=params, world=world, seed=seed)
    sim.reset()
    sim.run_for(0.40)
    x0 = float(sim.snapshot()[0].root_pos[0])
    t0 = float(sim.data.time)
    command = None
    if abs(float(args.vx)) > 0.0:
        command = DriveCommand(vx=float(args.vx), wz=0.0, timestamp_wall=time.time(), source="headless")
    gt = sim.run_for(steps / params.control_hz, command)
    dx = float(gt.root_pos[0]) - x0
    dt = float(gt.sim_time) - t0
    torque = max((abs(v) for v in gt.applied_torque.values()), default=0.0)
    finite = sim.finite()
    print(
        "headless "
        f"scene={scene} seed={seed} morphology={params.morphology} "
        f"steps={steps} sim_time_s={gt.sim_time:.4f} dt_s={dt:.4f} "
        f"x0_m={x0:.4f} x_m={gt.root_pos[0]:.4f} dx_m={dx:.4f} "
        f"target_vx_m_s={gt.target_vx:.4f} max_abs_torque_nm={torque:.6f} "
        f"finite={finite}"
    )
    if not finite:
        print("State is not finite.", file=sys.stderr)
        return 1
    if command is not None and scene == "basic" and dx <= 0.02:
        print(
            "Basic scene did not move forward under a positive drive command. "
            "Driving is expected to come from wheel torque and contact.",
            file=sys.stderr,
        )
        return 1
    if command is not None and torque <= 0.0 and dx <= 0.0:
        print("Drive command produced neither torque nor displacement.", file=sys.stderr)
        return 1
    return 0


def _run_scenarios(args: argparse.Namespace) -> int:
    out = Path(args.out) if args.out else Path.cwd() / "embr-sim-runs"
    if args.scenario:
        candidate = Path(args.scenario)
        if candidate.is_file():
            paths = [candidate]
        else:
            paths = [repo_root() / "scenarios" / f"{args.scenario}.yaml"]
        if not paths[0].is_file():
            names = ", ".join(path.stem for path in list_scenarios())
            print(f"Scenario file not found: {paths[0]}", file=sys.stderr)
            print(f"Packaged scenarios: {names}", file=sys.stderr)
            return 2
    else:
        paths = list_scenarios()
        if not paths:
            print("No packaged scenarios were found.", file=sys.stderr)
            return 2
    failed = 0
    for path in paths:
        print(f"Running {path.name} …")
        result = run_scenario(path, out_dir=out)
        status = "PASS" if result["passed"] else "FAIL"
        print(f"  {status}  {result['name']}  final x={result['final']['x']:.3f}")
        if result["failures"]:
            for item in result["failures"]:
                print(f"    - {item}")
        if not result["passed"]:
            failed += 1
    print(f"Wrote logs under {out}  ({failed} failed / {len(paths)})")
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.cmd == "launch":
            params, world, scene, seed = _resolve_run(args)
            from embr_local_sim.viewer import run_viewer

            try:
                run_viewer(
                    host=args.host,
                    port=args.port,
                    world=world,
                    start_mission=bool(args.mission),
                    params=params,
                    mode=scene,
                    seed=seed,
                )
            except RuntimeError as exc:
                print(exc, file=sys.stderr)
                return 1
            return 0
        if args.cmd == "headless":
            selected = sum(bool(x) for x in (args.steps, args.scenario, args.all))
            if selected != 1:
                print(
                    "Choose one of: --steps N, --scenario NAME, or --all.",
                    file=sys.stderr,
                )
                return 2
            if args.steps is not None:
                return _run_steps(args)
            return _run_scenarios(args)
        if args.cmd == "doctor":
            from embr_local_sim.doctor import main as doctor_main

            return doctor_main()
        if args.cmd == "write-model":
            from embr_local_sim.model import write_mjcf

            out = Path(args.out) if args.out else Path.cwd() / "embr-sim-runs" / "embr_simple_skidsteer.xml"
            write_mjcf(out, RobotParams(), WorldSpec(kind="flat"))
            print(out)
            return 0
        if args.cmd == "test":
            import pytest

            root = Path(__file__).resolve().parents[2]
            tests = root / "tests"
            if not tests.is_dir():
                print(
                    f"Tests are not inside the installed package ({tests}). "
                    "From simulation/mujoco run: python -m pytest",
                    file=sys.stderr,
                )
                return 2
            return int(pytest.main([str(tests)]))
    except (FileNotFoundError, ValueError) as exc:
        print(exc, file=sys.stderr)
        return 2
    print("Unknown command.", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
