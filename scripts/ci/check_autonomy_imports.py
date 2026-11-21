#!/usr/bin/env python3
"""Import each direct RL requirement and print a pass/fail table."""

from __future__ import annotations

import importlib
import re
import sys
from pathlib import Path


IMPORTS = {
    "torch": ("torch",),
    "torchvision": ("torchvision",),
    "numpy": ("numpy",),
    "opencv-python-headless": ("cv2",),
    "moviepy": ("moviepy",),
    "matplotlib": ("matplotlib",),
    "pandas": ("pandas",),
    "gymnasium": ("gymnasium", "mujoco"),
    "stable-baselines3": ("stable_baselines3",),
    "h5py": ("h5py",),
    "lerobot": ("lerobot",),
    "setuptools": ("setuptools",),
    "wandb": ("wandb",),
    "onnx": ("onnx",),
    "onnxruntime": ("onnxruntime",),
}


def main() -> int:
    requirements_path = Path(sys.argv[1])
    results = []
    for raw_line in requirements_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or line.startswith("-"):
            continue

        match = re.match(r"([A-Za-z0-9][A-Za-z0-9._-]*)", line)
        if match is None:
            results.append((line, "", "FAIL", "could not parse package name"))
            continue

        package = match.group(1).lower().replace("_", "-").replace(".", "-")
        modules = IMPORTS.get(package)
        if modules is None:
            results.append((package, "", "FAIL", "no import mapping is defined"))
            continue

        try:
            for module in modules:
                importlib.import_module(module)
        except (ImportError, OSError, RuntimeError, AttributeError, ValueError) as error:
            results.append((package, ", ".join(modules), "FAIL", f"{type(error).__name__}: {error}"))
        else:
            results.append((package, ", ".join(modules), "PASS", ""))

    print("| Package | Import(s) | Result | Detail |")
    print("| --- | --- | --- | --- |")
    for package, modules, result, detail in results:
        escaped_detail = detail.replace("|", "\\|")
        print(f"| `{package}` | `{modules}` | **{result}** | {escaped_detail} |")

    return 0 if results and all(result == "PASS" for _, _, result, _ in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())