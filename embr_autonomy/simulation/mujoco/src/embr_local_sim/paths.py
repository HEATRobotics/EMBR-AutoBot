"""Locate packaged simulator data. Independent of the process working directory."""

from __future__ import annotations

from pathlib import Path


def package_dir() -> Path:
    return Path(__file__).resolve().parent


def repo_root() -> Path:
    """Directory that contains scenarios/, assets/, and config/.

    These files ship inside the package so a normal install does not depend on
    the checkout path or the directory the command was started from.
    """
    bundled = package_dir() / "bundled"
    if not (bundled / "scenarios").is_dir():
        raise FileNotFoundError(
            "Simulator scenarios are missing from the installed package "
            f"({bundled}). Reinstall from embr_autonomy/simulation/mujoco: "
            "python -m pip install -e '.[test]'"
        )
    return bundled


def scenarios_dir() -> Path:
    return repo_root() / "scenarios"


def assets_dir() -> Path:
    return repo_root() / "assets"


def config_dir() -> Path:
    path = repo_root() / "config"
    if not path.is_dir():
        raise FileNotFoundError(
            f"Simulator config directory is missing: {path}. "
            "Reinstall the simulation package."
        )
    return path
