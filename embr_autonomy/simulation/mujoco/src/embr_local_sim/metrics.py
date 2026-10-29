"""Run logging: JSON/CSV plus a short Markdown summary."""

from __future__ import annotations

import csv
import json
import platform
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from embr_local_sim.types import GroundTruthState


def host_profile() -> dict[str, Any]:
    info: dict[str, Any] = {
        "system": platform.system(),
        "release": platform.release(),
        "machine": platform.machine(),
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "processor": platform.processor(),
        "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    try:
        import mujoco
        import numpy
        import viser

        info["mujoco"] = mujoco.__version__
        info["numpy"] = numpy.__version__
        info["viser"] = viser.__version__
    except Exception as exc:  # pragma: no cover - optional in unit tests
        info["import_error"] = str(exc)
    return info


@dataclass
class RunLog:
    scenario: str
    seed: int
    model_hash: str
    passed: bool
    failures: list[str]
    host: dict[str, Any]
    assumptions: list[dict[str, Any]]
    final: dict[str, Any]
    n_samples: int
    timestep: float
    extras: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "scenario": self.scenario,
            "seed": self.seed,
            "model_hash": self.model_hash,
            "passed": self.passed,
            "failures": self.failures,
            "host": self.host,
            "assumptions": self.assumptions,
            "final": self.final,
            "n_samples": self.n_samples,
            "timestep": self.timestep,
            "extras": self.extras,
        }

    @classmethod
    def from_run(cls, sim, scenario, samples: list[GroundTruthState], result: dict[str, Any]) -> "RunLog":
        table = [
            {
                "value": rec.value,
                "unit": rec.unit,
                "provenance": rec.provenance,
                "notes": rec.notes,
            }
            for rec in sim.params.provenance_table()
        ]
        return cls(
            scenario=scenario.name,
            seed=scenario.seed,
            model_hash=result["model_hash"],
            passed=result["passed"],
            failures=result["failures"],
            host=host_profile(),
            assumptions=table,
            final=result["final"],
            n_samples=len(samples),
            timestep=sim.params.physics_dt,
        )


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, default=str) + "\n", encoding="utf-8")


def append_csv(path: Path, samples: list[GroundTruthState]) -> None:
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(
            [
                "sim_time",
                "x",
                "y",
                "z",
                "yaw",
                "vx_body",
                "wz",
                "target_vx",
                "target_wz",
                "ncon",
                "kinetic",
                "watchdog",
                "rtf",
            ]
        )
        for s in samples:
            writer.writerow(
                [
                    s.sim_time,
                    s.root_pos[0],
                    s.root_pos[1],
                    s.root_pos[2],
                    s.yaw,
                    s.actual_vx_body,
                    s.actual_wz,
                    s.target_vx,
                    s.target_wz,
                    s.ncon,
                    s.kinetic_energy,
                    int(s.watchdog_active),
                    s.real_time_factor,
                ]
            )


def write_markdown_report(path: Path, log: RunLog) -> None:
    status = "PASS" if log.passed else "FAIL"
    lines = [
        f"# Scenario `{log.scenario}`",
        "",
        f"- Result: **{status}**",
        f"- Seed: `{log.seed}`",
        f"- Model hash: `{log.model_hash}`",
        f"- Samples: {log.n_samples}",
        f"- Timestep: {log.timestep} s",
        f"- Final pose: x={log.final.get('x', 0):.4f} y={log.final.get('y', 0):.4f} "
        f"z={log.final.get('z', 0):.4f} yaw={log.final.get('yaw', 0):.4f}",
        "",
    ]
    if log.failures:
        lines.append("Failures:")
        lines.extend(f"- {item}" for item in log.failures)
        lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")
