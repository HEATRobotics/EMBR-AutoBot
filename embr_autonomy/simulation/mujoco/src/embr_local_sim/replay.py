"""Replay log that keeps command, measured motion, and odom distinct."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable


@dataclass
class ReplayFrame:
    sim_time: float
    tele_velocity: float
    tele_turn: float
    command_vx: float
    command_wz: float
    measured_vx_body: float
    measured_wz: float
    odom_x: float
    odom_y: float
    odom_yaw: float
    gt_x: float
    gt_y: float
    gt_yaw: float
    mission_state: str
    probe_c: float
    thermal_max_c: float
    lidar_min: float
    source: str
    watchdog: bool


class ReplayLog:
    def __init__(self) -> None:
        self.frames: list[ReplayFrame] = []

    def reset(self) -> None:
        self.frames.clear()

    def append(self, frame: ReplayFrame) -> None:
        self.frames.append(frame)

    def dump(self, path: Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as fh:
            for frame in self.frames:
                fh.write(json.dumps(asdict(frame)) + "\n")
        return path

    def command_vs_measured(self) -> dict[str, float]:
        if not self.frames:
            return {"n": 0, "mean_abs_vx_err": 0.0}
        err = [abs(f.command_vx - f.measured_vx_body) for f in self.frames]
        return {
            "n": float(len(err)),
            "mean_abs_vx_err": sum(err) / len(err),
            "max_abs_vx_err": max(err),
        }


def load_replay(path: Path) -> list[dict[str, Any]]:
    rows = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def summarize(frames: Iterable[ReplayFrame]) -> dict[str, Any]:
    items = list(frames)
    return {
        "n": len(items),
        "states": sorted({f.mission_state for f in items}),
        "command_vs_measured": ReplayLog.__new__(ReplayLog).command_vs_measured()
        if not items
        else {
            "n": len(items),
            "mean_abs_vx_err": sum(abs(f.command_vx - f.measured_vx_body) for f in items) / len(items),
        },
    }
