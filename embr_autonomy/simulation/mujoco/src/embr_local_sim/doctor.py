"""Host doctor: report missing pieces without pretending they passed."""

from __future__ import annotations

import json
import os
import platform
import shutil
import sys
from pathlib import Path
from typing import Any


def _try_import(name: str) -> tuple[bool, str]:
    try:
        mod = __import__(name)
        ver = getattr(mod, "__version__", "present")
        return True, str(ver)
    except Exception as exc:
        return False, str(exc)


def doctor() -> dict[str, Any]:
    info: dict[str, Any] = {
        "os": platform.platform(),
        "machine": platform.machine(),
        "python": sys.version.split()[0],
        "cwd": str(Path.cwd()),
        "gpu": Path("/dev/dri").exists(),
        "checks": {},
        "blockers": [],
        "ok": True,
    }
    checks = info["checks"]
    info["notes"] = []
    for name in ("mujoco", "numpy", "yaml", "viser"):
        ok, detail = _try_import(name if name != "yaml" else "yaml")
        checks[name] = {"ok": ok, "detail": detail}
        if not ok:
            info["ok"] = False
            info["blockers"].append(f"missing Python package {name}: {detail}")
    pytest_ok, pytest_detail = _try_import("pytest")
    checks["pytest"] = {"ok": pytest_ok, "detail": pytest_detail, "optional": "test"}
    if not pytest_ok:
        info["notes"].append("pytest is a development extra, not required to launch the viewer")
    rclpy_ok, rclpy_detail = _try_import("rclpy")
    checks["rclpy"] = {"ok": rclpy_ok, "detail": rclpy_detail, "required_for_simulator": False}
    if not rclpy_ok:
        info["notes"].append(
            "ROS 2 is not imported. This MuJoCo package does not need it. "
            "Use the team Ubuntu 22.04 or Docker workflow for embr_phys/ros2_ws."
        )
    for exe in ("ros2", "gz", "ign", "gazebo", "docker"):
        path = shutil.which(exe)
        checks[f"bin:{exe}"] = {"ok": path is not None, "detail": path or "not on PATH"}
    ram = shutil.disk_usage("/")
    info["disk_free_gb"] = round(ram.free / 1e9, 1)
    try:
        info["cpu_count"] = os.cpu_count()
    except Exception:
        info["cpu_count"] = None
    from embr_local_sim.assets import cad_status

    info["cad"] = cad_status()
    if info["cad"].get("blocker"):
        info["notes"].append(info["cad"]["blocker"])
    return info


def main() -> int:
    report = doctor()
    print(json.dumps(report, indent=2))
    # Native sim can proceed without ROS. Doctor is not a silent green skip.
    native = all(report["checks"][n]["ok"] for n in ("mujoco", "numpy", "yaml", "viser"))
    return 0 if native else 2


if __name__ == "__main__":
    raise SystemExit(main())
