"""Deterministic 12-second motor replay used by both the viewer and headless CLI."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict
from pathlib import Path

from embr_local_sim.motor import MotorSample, MotorTwin

DEMO_HZ = 50
DEMO_DURATION_S = 12
# Each stage lasts two seconds. Values are display speeds, not motor limits.
STAGES = (
    ("Forward slow", math.pi),
    ("Forward fast", 2 * math.pi),
    ("Stop", 0.0),
    ("Reverse slow", -math.pi),
    ("Reverse fast", -2 * math.pi),
    ("Finished / stopped", 0.0),
)


def demo_sample(timestamp_s: float) -> tuple[str, MotorSample]:
    """Return an analytical unwrapped position/speed pair at time 0..12 seconds."""
    if not math.isfinite(timestamp_s) or not 0 <= timestamp_s <= DEMO_DURATION_S:
        raise ValueError("Demo time must be between 0 and 12 seconds")
    stage = min(int(timestamp_s // 2), len(STAGES) - 1)
    position = sum(speed * 2 for _, speed in STAGES[:stage])
    label, speed = STAGES[stage]
    position += speed * (timestamp_s - stage * 2)
    return label, MotorSample(timestamp_s, position, speed)


def run_headless(twin: MotorTwin, out: str | Path | None = None) -> dict:
    """Replay all 601 samples, check applied values, optionally write CSV and JSON."""
    twin.reset()
    rows = []
    max_position_error = max_velocity_error = 0.0
    for tick in range(DEMO_DURATION_S * DEMO_HZ + 1):
        label, sample = demo_sample(tick / DEMO_HZ)
        state = twin.apply(sample)
        position = float(twin.data.joint("motor_shaft").qpos[0])
        velocity = float(twin.data.joint("motor_shaft").qvel[0])
        max_position_error = max(max_position_error, abs(position - sample.position_rad))
        max_velocity_error = max(max_velocity_error, abs(velocity - sample.velocity_rad_s))
        rows.append({"stage": label, **asdict(state), "qpos_rad": position, "qvel_rad_s": velocity})
    summary = {
        "model": "Maxon EC-i52 667065 telemetry twin",
        "source": "synthetic", "samples": len(rows), "sample_hz": DEMO_HZ,
        "duration_s": DEMO_DURATION_S,
        "final_position_rad": rows[-1]["qpos_rad"],
        "final_velocity_rad_s": rows[-1]["qvel_rad_s"],
        "max_position_error_rad": max_position_error,
        "max_velocity_error_rad_s": max_velocity_error,
        "passed": max_position_error <= 1e-12 and max_velocity_error <= 1e-12,
    }
    if out is not None:
        folder = Path(out)
        folder.mkdir(parents=True, exist_ok=True)
        with (folder / "motor_samples.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        (folder / "motor_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary
