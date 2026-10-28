"""Local STEP tessellation summary for the CAD morphology.

Reads an Onshape AP242 assembly that is already on disk. Writes a geometry
record and a visual OBJ. Does not modify the STEP and does not upload it.

The STEP header says metres. Onshape does not export mate degrees of freedom,
so this file records solid extents and placements only.

Re-run from simulation/mujoco:

    EMBR_STEP_FILE="/path/to/final FULL CAD.step" \\
    EMBR_STEP_GLB="/tmp/embr-cad-inspect/assembly.glb" \\
    python -m embr_local_sim.cad_convert
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import trimesh

from embr_local_sim.paths import assets_dir

# Tessellation linear deflection used when the GLB was built. Extents below
# this are the mesher's tolerance, not a caliper reading.
TESS_TOL_M = 0.008
QUANT_M = 0.006

KEEP = (
    "angle aluminum",
    "base plate",
    "chassis",
    "side wall",
    "dttrack",
    "track_idler",
    "sprocket",
    "idler",
    "frame_assm",
    "frame inside",
    "frame_top",
    "gpx52",
    "drive_shaft_100",
    "shaft_coupler",
    "leadscrew",
    "thermoprobe",
    "the rail",
    "rail assembly",
    "nema17",
    "carriage",
    "pinion",
    "big gear",
    "probe",
    "drum",
    "eagle",
    "planetary",
    "capstan",
    "battery_box",
    "bms_plate",
    "bus bar",
    "18650",
)
SKIP = (
    "screw",
    "nut",
    "standoff",
    "retaining",
    "escon",
    "spider",
    "phillips",
    "shoulder_screw",
)


def _keep(name: str) -> bool:
    low = name.lower()
    if any(token in low for token in SKIP):
        return False
    return any(token in low for token in KEEP)


def _world_vertices(scene: trimesh.Scene, node: str) -> np.ndarray:
    transform, geom_name = scene.graph.get(node)
    vertices = np.asarray(scene.geometry[geom_name].vertices, dtype=np.float64)
    if len(vertices) == 0:
        return vertices.reshape(0, 3)
    ones = np.ones((len(vertices), 1))
    return (np.hstack([vertices, ones]) @ np.asarray(transform, dtype=np.float64).T)[:, :3]


def _aabb(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return points.min(axis=0), points.max(axis=0)


def _collect(scene: trimesh.Scene, needle: str) -> list[tuple[str, np.ndarray, np.ndarray]]:
    rows = []
    needle = needle.lower()
    for node in scene.graph.nodes_geometry:
        if needle not in node.lower():
            continue
        vertices = _world_vertices(scene, node)
        if len(vertices) == 0:
            continue
        rows.append((node, vertices.min(axis=0), vertices.max(axis=0)))
    return rows


def _full_solid(rows: list[tuple[str, np.ndarray, np.ndarray]]) -> tuple[np.ndarray, np.ndarray] | None:
    """The node whose own AABB is the largest is the solid; the rest are faces."""
    if not rows:
        return None
    sizes = [mx - mn for _n, mn, mx in rows]
    index = int(np.argmax([float(np.max(size)) for size in sizes]))
    return rows[index][1], rows[index][2]


def build_geometry(scene: trimesh.Scene) -> dict:
    plate_rows = _collect(scene, "Base plate Chassis")
    if not plate_rows:
        raise RuntimeError("Base plate solid was not in the tessellation.")
    # Individual faces can be zero-thickness. The union is the plate envelope.
    plate_min = np.min([mn for _n, mn, mx in plate_rows], axis=0)
    plate_max = np.max([mx for _n, mn, mx in plate_rows], axis=0)
    plate_size = plate_max - plate_min
    origin_x = float((plate_min[0] + plate_max[0]) * 0.5)
    origin_z = float((plate_min[2] + plate_max[2]) * 0.5)

    idler_rows = _collect(scene, "TRACK_IDLER_WHEEL")
    diameters = []
    for _name, mn, mx in idler_rows:
        size = np.sort(mx - mn)
        if size[-1] > 0.1 and size[-2] > 0.1:
            diameters.append(float(size[-1]))
    if not diameters:
        raise RuntimeError("No idler wheel AABB large enough to read a diameter.")
    idler_diameter = float(np.median(diameters))
    radius = 0.5 * idler_diameter

    # Lower road wheels sit under the upper run. Split on the plate mid-height.
    plate_y = float((plate_min[1] + plate_max[1]) * 0.5)
    lower = []
    for _name, mn, mx in idler_rows:
        cen = (mn + mx) * 0.5
        size = np.sort(mx - mn)
        if cen[1] < plate_y + 0.05 and size[-1] > 0.1 and size[-2] > 0.1:
            lower.append(cen)
    if len(lower) < 4:
        raise RuntimeError(f"Expected lower idlers in four quadrants, found {len(lower)}.")
    lower_pts = np.vstack(lower)
    roller_y = float(np.median(lower_pts[:, 1]))

    corners = {
        "rear_left_maxon": [],
        "rear_right_maxon": [],
        "front_left_maxon": [],
        "front_right_maxon": [],
    }
    for cen in lower_pts:
        front = cen[2] >= origin_z
        left = cen[0] <= origin_x
        if front and left:
            key = "front_left_maxon"
        elif front and not left:
            key = "front_right_maxon"
        elif not front and left:
            key = "rear_left_maxon"
        else:
            key = "rear_right_maxon"
        corners[key].append(cen)
    rollers = []
    for name, points in corners.items():
        if not points:
            raise RuntimeError(f"No lower idler in quadrant {name}.")
        stack = np.vstack(points)
        cen = stack.mean(axis=0)
        # Axial extent of the fragments that landed in this quadrant.
        width = float(stack[:, 0].max() - stack[:, 0].min())
        rollers.append(
            {
                "joint": name,
                "cad_xyz_m": [float(v) for v in cen],
                "width_m": width,
                "width_provenance": "span of tessellated idler fragments at this wheel; not a drawing width",
            }
        )

    def body_point(cad: np.ndarray) -> list[float]:
        return [
            float(cad[2] - origin_z),
            float(origin_x - cad[0]),
            float(cad[1] - roller_y),
        ]

    for roller in rollers:
        roller["body_xyz_m"] = body_point(np.array(roller["cad_xyz_m"]))

    left_y = [r["body_xyz_m"][1] for r in rollers if r["joint"].endswith("left_maxon")]
    right_y = [r["body_xyz_m"][1] for r in rollers if r["joint"].endswith("right_maxon")]
    front_x = [r["body_xyz_m"][0] for r in rollers if r["joint"].startswith("front_")]
    rear_x = [r["body_xyz_m"][0] for r in rollers if r["joint"].startswith("rear_")]
    separation = float(np.mean(left_y) - np.mean(right_y))
    wheelbase = float(np.mean(front_x) - np.mean(rear_x))

    leads = _collect(scene, "leadscrew")
    lead = _full_solid(leads)
    rail = _full_solid(_collect(scene, "the rail"))
    probe = _full_solid(_collect(scene, "thermoprobe"))
    drum = _full_solid(_collect(scene, "big drum"))
    angle_long = _full_solid(_collect(scene, "15.75 in Angle"))
    angle_short = _full_solid(_collect(scene, "12.75 in Angle"))

    def solid_record(pair):
        if pair is None:
            return None
        mn, mx = pair
        return {
            "size_m": [float(v) for v in (mx - mn)],
            "cad_min_m": [float(v) for v in mn],
            "cad_max_m": [float(v) for v in mx],
            "body_center_m": body_point((mn + mx) * 0.5),
        }

    return {
        "units": "metre",
        "tessellation_tol_m": TESS_TOL_M,
        "frame": {
            "forward": "CAD +Z, the long axis of the base plate",
            "left": "CAD -X, chosen so the smaller CAD X track is +Y. The STEP does not label left.",
            "up": "CAD +Y, the base plate's thin axis. Lower idlers hang below the plate.",
            "plate_center_cad_m": [origin_x, plate_y, origin_z],
            "roller_center_cad_y_m": roller_y,
        },
        "plate": solid_record((plate_min, plate_max)),
        "idler_diameter_m": idler_diameter,
        "roller_radius_m": radius,
        "roller_radius_provenance": (
            "half the median tessellated AABB diameter of TRACK_IDLER_WHEEL solids "
            f"whose two large axes exceed 0.1 m. Linear mesher tolerance {TESS_TOL_M} m. "
            "Not a caliper measurement. Track-shoe thickness is not added."
        ),
        "wheel_separation_m": separation,
        "wheelbase_m": wheelbase,
        "rollers": rollers,
        "leadscrew": solid_record(lead),
        "rail": solid_record(rail),
        "thermoprobe": solid_record(probe),
        "big_drum": solid_record(drum),
        "angle_long": solid_record(angle_long),
        "angle_short": solid_record(angle_short),
        "mass_kg": {
            "chassis": 4.0,
            "roller": 0.05,
            "provenance": "ASSUMED. The STEP has no mass, density, or inertia.",
        },
        "max_wheel_torque_nm": {
            "value": 0.50,
            "provenance": (
                "ASSUMED. The proxy uses 0.020 N·m, about 1.4 times "
                "(chassis mass * g * radius / 4). The same factor on the provisional "
                "4 kg chassis and the CAD idler radius is 0.50 N·m. A blocked wheel "
                "can still stall. Not an EC-i52 or ESCON rating."
            ),
        },
        "velocity_kp": {
            "value": 0.10,
            "provenance": "ASSUMED. Saturates the provisional torque near a 5 rad/s speed error, as the proxy loop does.",
        },
    }


def _write_obj(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        handle.write("# visual mesh, metres, body frame, quantized. Not collision.\n")
        for x, y, z in vertices:
            handle.write(f"v {x:.5f} {y:.5f} {z:.5f}\n")
        for a, b, c in faces:
            handle.write(f"f {a + 1} {b + 1} {c + 1}\n")


def export_visual(scene: trimesh.Scene, geometry: dict, path: Path) -> dict:
    origin_x, _plate_y, origin_z = geometry["frame"]["plate_center_cad_m"]
    roller_y = geometry["frame"]["roller_center_cad_y_m"]
    index: dict[tuple[int, int, int], int] = {}
    verts: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    kept = 0
    skipped = 0

    def vid(point: np.ndarray) -> int:
        body = (
            point[2] - origin_z,
            origin_x - point[0],
            point[1] - roller_y,
        )
        key = tuple(int(round(v / QUANT_M)) for v in body)
        found = index.get(key)
        if found is None:
            found = len(verts)
            index[key] = found
            verts.append(tuple(component * QUANT_M for component in key))
        return found

    for node in scene.graph.nodes_geometry:
        if not _keep(node):
            skipped += 1
            continue
        transform, geom_name = scene.graph.get(node)
        geom = scene.geometry[geom_name]
        vertices = np.asarray(geom.vertices, dtype=np.float64)
        triangles = np.asarray(geom.faces)
        if len(vertices) == 0 or len(triangles) == 0:
            continue
        ones = np.ones((len(vertices), 1))
        world = (np.hstack([vertices, ones]) @ np.asarray(transform, dtype=np.float64).T)[:, :3]
        ids = [vid(vertex) for vertex in world]
        for tri in triangles:
            a, b, c = ids[int(tri[0])], ids[int(tri[1])], ids[int(tri[2])]
            if a != b and b != c and a != c:
                faces.append((a, b, c))
        kept += 1
    _write_obj(path, np.asarray(verts, dtype=np.float64), np.asarray(faces, dtype=np.int64))
    return {"nodes_kept": kept, "nodes_skipped": skipped, "vertices": len(verts), "faces": len(faces)}


def main() -> None:
    glb_path = Path(os.environ.get("EMBR_STEP_GLB", "/tmp/embr-cad-inspect/assembly.glb"))
    if not glb_path.is_file():
        raise FileNotFoundError(
            f"No local GLB at {glb_path}. Tessellate the STEP with cascadio first. "
            "Do not upload the STEP to a web converter."
        )
    scene = trimesh.load(glb_path, force="scene")
    geometry = build_geometry(scene)
    mesh_path = assets_dir() / "meshes" / "cad_visual.obj"
    geometry["visual_mesh"] = export_visual(scene, geometry, mesh_path)
    geometry["visual_mesh"]["path"] = "meshes/cad_visual.obj"
    geometry["visual_mesh"]["quantize_m"] = QUANT_M
    out = assets_dir() / "cad_geometry.json"
    out.write_text(json.dumps(geometry, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {out}")
    print(f"wrote {mesh_path} verts={geometry['visual_mesh']['vertices']} faces={geometry['visual_mesh']['faces']}")
    print(
        "separation_m",
        round(geometry["wheel_separation_m"], 4),
        "wheelbase_m",
        round(geometry["wheelbase_m"], 4),
        "radius_m",
        round(geometry["roller_radius_m"], 4),
    )


if __name__ == "__main__":
    main()
