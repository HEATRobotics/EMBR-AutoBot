"""Table, candle, hand-held camera, and the parked CAD body."""

from __future__ import annotations

import os
from pathlib import Path

import mujoco
import numpy as np

from thermal_bench.frame import AMBIENT_C, HeatBlob

# Candle flame, on the table. The hand camera looks along +x toward it.
CANDLE = HeatBlob(x=0.40, y=0.0, z=0.46, peak_c=80.0, sigma_m=0.045)
TABLE_Z = 0.38
# Soldering-iron tip. peak_c is degrees above the room, so the tip is about 400 °C.
# A Lepton 3.1R in high gain clips near 140 °C. The viewer shows both numbers.
IRON = HeatBlob(x=0.40, y=0.0, z=0.50, peak_c=380.0, sigma_m=0.012)

# Body-frame lidar site from the current CAD model. Not moved.
LIDAR_SITE_XYZ = (0.33306, 0.0, 0.00732)


def cad_mesh() -> Path | None:
    """Visual mesh only. Collision stays the box in the MuJoCo file.

    `EMBR_CAD_MESH` wins. Otherwise look next to this checkout for the mesh
    the rover sim already ships. Missing mesh means the viewer draws a box.
    """
    here = Path(__file__).resolve()
    candidates = []
    env = os.environ.get("EMBR_CAD_MESH")
    if env:
        candidates.append(Path(env))
    repo = here.parents[3]
    candidates.append(
        repo / "embr_autonomy/simulation/mujoco/src/embr_local_sim/bundled/assets/meshes/cad_visual.obj"
    )
    candidates.append(
        repo / "simulation/mujoco/src/embr_local_sim/bundled/assets/meshes/cad_visual.obj"
    )
    candidates.append(
        here.parents[2] / "src/embr_local_sim/bundled/assets/meshes/cad_visual.obj"
    )
    for path in candidates:
        if path.is_file():
            return path
    return None


def table_xml() -> str:
    # The CAD mesh is drawn in Viser. Keeping it out of this MuJoCo file
    # avoids a long mesh compile before the window opens.
    rover = f"""
    <body name="rover" pos="-0.55 0.85 0.093">
      <geom name="rover_standin" type="box" size="0.35 0.25 0.04" rgba="0.45 0.48 0.52 1"/>
      <site name="lidar" pos="{LIDAR_SITE_XYZ[0]} {LIDAR_SITE_XYZ[1]} {LIDAR_SITE_XYZ[2]}"
            size="0.012" rgba="0.2 0.6 1 1"/>
    </body>
    """
    return f"""<?xml version="1.0"?>
<mujoco model="thermal_table">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.7 0.7 0.7"/>
  </visual>
  <worldbody>
    <light pos="0.2 -0.4 1.6" dir="0 0.2 -1"/>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.86 0.84 0.78 1"/>
    <body name="table" pos="0.15 0 {TABLE_Z}">
      <geom type="box" size="0.55 0.35 0.02" rgba="0.55 0.40 0.28 1"/>
    </body>
    <body name="candle" pos="{CANDLE.x} {CANDLE.y} {TABLE_Z + 0.02}">
      <geom type="cylinder" size="0.012 0.035" rgba="0.95 0.93 0.88 1"/>
      <geom name="flame" type="sphere" size="0.018" pos="0 0 0.055" rgba="1.0 0.45 0.12 0.85"/>
    </body>
    <body name="iron" pos="{IRON.x} {IRON.y} {TABLE_Z}">
      <geom name="iron_stand" type="cylinder" size="0.012 0.05" pos="0 0 0.05" rgba="0.25 0.25 0.28 1"/>
      <geom name="iron_tip" type="sphere" size="0.015" pos="0 0 0.12" rgba="0.85 0.45 0.15 1"/>
    </body>
    <body name="phone" mocap="true" pos="0.0 0 0.50">
      <geom type="box" size="0.035 0.018 0.006" rgba="0.15 0.15 0.18 1"/>
      <site name="lepton" pos="0.04 0 0" size="0.006" rgba="0.9 0.2 0.15 1"/>
    </body>
    {rover}
  </worldbody>
</mujoco>
"""


def load_model() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_string(table_xml())
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def _place(data: mujoco.MjData, target: HeatBlob, distance_m: float) -> np.ndarray:
    pos = np.array([target.x - distance_m, target.y, target.z], dtype=float)
    data.mocap_pos[0] = pos
    data.mocap_quat[0] = np.array([1.0, 0.0, 0.0, 0.0])
    return pos


def place_hand(data: mujoco.MjData, distance_m: float) -> np.ndarray:
    """Put the camera `distance_m` before the candle, looking +x. Returns xyz."""
    return _place(data, CANDLE, distance_m)


def place_phone(data: mujoco.MjData, distance_m: float) -> np.ndarray:
    """Put the phone `distance_m` before the iron tip, looking +x."""
    return _place(data, IRON, distance_m)


def candle_sources() -> list[HeatBlob]:
    return [CANDLE]


def iron_sources() -> list[HeatBlob]:
    return [IRON]


AMBIENT = AMBIENT_C
