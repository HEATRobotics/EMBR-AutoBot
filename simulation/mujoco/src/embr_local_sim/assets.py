"""Geometry asset pipeline. Meshes are optional; primitives are the fallback.

Controllers, sensors, and mission logic must not depend on a mesh existing.
A later CAD STEP/OBJ drop can fill `assets/meshes/` and this module will
prefer those files. Missing CAD is a documented blocker, not a license to
invent millimetre dimensions.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import os

import yaml

from embr_local_sim.paths import assets_dir, repo_root

MESH_SUFFIXES = (".stl", ".obj", ".ply")
CAD_ARCHIVE_NAMES = (
    "final FULL CAD.zip",
    "final_FULL_CAD.zip",
    "final-full-cad.zip",
)


@dataclass(frozen=True)
class MeshSlot:
    name: str
    visual: Path
    collision: Path | None
    scale: float
    available: bool
    source: str  # "mesh" | "primitive"


@dataclass
class AssetManifest:
    version: int
    units: str
    provenance: str
    cad_archive: str | None
    cad_archive_status: str
    slots: dict[str, MeshSlot]
    path: Path


def load_manifest(path: Path | None = None) -> AssetManifest:
    path = path or (assets_dir() / "manifest.yaml")
    raw: dict[str, Any] = {}
    if path.exists():
        raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    slots: dict[str, MeshSlot] = {}
    base = path.parent
    for name, spec in (raw.get("slots") or {}).items():
        visual = (base / str(spec.get("visual", f"meshes/{name}.obj"))).resolve()
        coll_raw = spec.get("collision")
        collision = (base / str(coll_raw)).resolve() if coll_raw else None
        available = visual.is_file() and visual.suffix.lower() in MESH_SUFFIXES
        slots[name] = MeshSlot(
            name=name,
            visual=visual,
            collision=collision if collision and collision.is_file() else collision,
            scale=float(spec.get("scale", 1.0)),
            available=available,
            source="mesh" if available else "primitive",
        )
    return AssetManifest(
        version=int(raw.get("version", 1)),
        units=str(raw.get("units", "metre")),
        provenance=str(raw.get("provenance", "provisional_assumption")),
        cad_archive=raw.get("cad_archive"),
        cad_archive_status=str(raw.get("cad_archive_status", "missing")),
        slots=slots,
        path=path,
    )


def find_cad_archive(roots: list[Path] | None = None) -> Path | None:
    """Look only in packaged assets, or at EMBR_CAD_ARCHIVE if the user set it.

    This does not scan the home directory, agent stores, or upload CAD anywhere.
    """
    env = os.environ.get("EMBR_CAD_ARCHIVE")
    if roots is None and env:
        candidate = Path(env)
        if candidate.is_file() and candidate.name in CAD_ARCHIVE_NAMES:
            return candidate
    if roots is None:
        roots = [assets_dir()]
    for root in roots:
        if not root.exists():
            continue
        for name in CAD_ARCHIVE_NAMES:
            hit = root / name
            if hit.is_file():
                return hit
            # shallow walk of a few folders
            for child in _iter_shallow(root):
                if child.name == name and child.is_file():
                    return child
    return None


def _iter_shallow(root: Path, depth: int = 3):
    stack = [(root, 0)]
    while stack:
        current, d = stack.pop()
        if d > depth:
            continue
        try:
            children = list(current.iterdir())
        except OSError:
            continue
        for child in children:
            if child.name.startswith(".") or child.name in {".venv", "__pycache__", "node_modules"}:
                continue
            yield child
            if child.is_dir():
                stack.append((child, d + 1))


def cad_status() -> dict[str, Any]:
    archive = find_cad_archive()
    manifest = load_manifest()
    mesh_files = []
    for folder in (repo_root() / "models", repo_root() / "assets", repo_root() / "src"):
        if not folder.exists():
            continue
        for ext in (".stl", ".obj", ".step", ".stp", ".sldasm", ".sldprt"):
            mesh_files.extend(str(p) for p in folder.rglob(f"*{ext}"))
    return {
        "archive_found": archive is not None,
        "archive_path": str(archive) if archive else None,
        "mesh_files": mesh_files,
        "slots": {
            name: {"available": slot.available, "source": slot.source, "visual": str(slot.visual)}
            for name, slot in manifest.slots.items()
        },
        "blocker": None
        if archive
        else (
            "The SolidWorks zip is not packaged. morphology=cad uses a local "
            "Onshape STEP tessellation (cad_visual.obj) for visuals only. "
            "The STEP is not in this tree. It has no joint entities, masses, "
            "or actuator ratings. Do not upload team CAD to a web converter."
        ),
    }


def mjcf_asset_compiler(manifest: AssetManifest | None = None) -> str:
    """Return <asset> mesh tags for files that actually exist."""
    manifest = manifest or load_manifest()
    lines = ["  <asset>"]
    any_mesh = False
    for slot in manifest.slots.values():
        if not slot.available:
            continue
        any_mesh = True
        lines.append(
            f'    <mesh name="{slot.name}" file="{slot.visual}" scale="'
            f'{slot.scale} {slot.scale} {slot.scale}"/>'
        )
    if not any_mesh:
        return ""
    lines.append("  </asset>")
    return "\n".join(lines) + "\n"


def mesh_geom(slot_name: str, geom_name: str, rgba: str, group: int = 1, collision: bool = True) -> str:
    """Use a mesh geom if the slot is filled; otherwise empty string (caller draws a primitive)."""
    slot = load_manifest().slots.get(slot_name)
    if slot is None or not slot.available:
        return ""
    contype = 2 if collision else 0
    conaffinity = 1 if collision else 0
    return (
        f'      <geom name="{geom_name}" type="mesh" mesh="{slot.name}" '
        f'rgba="{rgba}" group="{group}" contype="{contype}" conaffinity="{conaffinity}"/>'
    )
