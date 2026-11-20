"""Viser lab: EMBR body, phone rail, soldering iron, swappable distance cloud."""

from __future__ import annotations

import time
from pathlib import Path

import mujoco
import numpy as np
import trimesh
import viser

from thermal_bench.capture import read_uvc
from thermal_bench.ingest import load_thermal
from thermal_bench.frame import (
    HFOV_3_1R_DEG,
    HFOV_IMAGING_DEG,
    colorize,
    hot_pixel_count,
    render_frame,
    sample_temperature,
    to_tlinear_counts,
)
from thermal_bench.motion import PhoneRail
from thermal_bench.ranges import PHONE, UNITREE_L2, scan
from thermal_bench.scene import (
    AMBIENT,
    CANDLE,
    IRON,
    LIDAR_SITE_XYZ,
    TABLE_Z,
    cad_mesh,
    candle_sources,
    iron_sources,
    load_model,
    place_hand,
    place_phone,
)

_HIGH_GAIN_MAX_C = 140.0
_ROVER = np.array([-0.55, 0.85, 0.093])


def main() -> None:
    model, data = load_model()
    server = viser.ViserServer(host="127.0.0.1", port=43150, label="EMBR lab rig")
    try:
        server.gui.configure_theme(show_logo=False, show_share_button=False)
        server.gui.main_panel.dock_left()
        server.gui.main_panel.set_width(380)
    except Exception:
        pass
    server.initial_camera.position = (0.05, -1.35, 0.95)
    server.initial_camera.look_at = (0.15, 0.05, 0.42)
    server.scene.add_grid(
        "/grid",
        width=3.0,
        height=3.0,
        plane="xy",
        cell_color=(90, 90, 90),
        section_color=(50, 50, 50),
    )
    server.scene.add_box(
        "/table",
        dimensions=(1.10, 0.70, 0.04),
        position=(0.15, 0.0, TABLE_Z),
        color=(140, 102, 70),
    )
    server.scene.add_box(
        "/candle/wax",
        dimensions=(0.024, 0.024, 0.07),
        position=(CANDLE.x, CANDLE.y, TABLE_Z + 0.055),
        color=(242, 236, 224),
    )
    server.scene.add_icosphere(
        "/candle/flame",
        radius=0.022,
        position=(CANDLE.x, CANDLE.y, CANDLE.z),
        color=(255, 150, 40),
    )
    server.scene.add_box(
        "/iron/stand",
        dimensions=(0.024, 0.024, 0.10),
        position=(IRON.x, IRON.y, TABLE_Z + 0.05),
        color=(70, 70, 74),
    )
    tip = server.scene.add_icosphere(
        "/iron/tip",
        radius=0.018,
        position=(IRON.x, IRON.y, IRON.z),
        color=(230, 90, 30),
    )
    phone = server.scene.add_box(
        "/phone",
        dimensions=(0.07, 0.036, 0.012),
        position=(IRON.x - 0.45, IRON.y, IRON.z),
        color=(28, 28, 32),
    )
    lens_dot = server.scene.add_icosphere(
        "/phone/lepton",
        radius=0.01,
        position=(IRON.x - 0.45 + 0.04, IRON.y, IRON.z),
        color=(220, 40, 40),
    )
    _add_cad(server)
    blank = np.zeros((120, 160, 3), dtype=np.uint8)
    cloud_handle = {"h": None}
    gui_image = server.gui.add_image(np.repeat(np.repeat(blank, 3, axis=0), 3, axis=1))
    readout = server.gui.add_markdown("Starting…")
    distance = server.gui.add_slider(
        "Phone distance (m)",
        min=0.25,
        max=1.20,
        step=0.02,
        initial_value=0.45,
    )
    target = server.gui.add_dropdown(
        "Heat target",
        options=("candle", "soldering iron"),
        initial_value="candle",
    )
    sensor = server.gui.add_dropdown(
        "Distance cloud",
        options=(PHONE, UNITREE_L2),
        initial_value=PHONE,
    )
    lens = server.gui.add_dropdown(
        "Lens",
        options=("95° Lepton 3.1R", "57° Lepton 3 imaging"),
        initial_value="95° Lepton 3.1R",
    )
    rail = PhoneRail()
    inbox = Path("inbox")

    print("open http://127.0.0.1:43150", flush=True)
    while True:
        dist = float(distance.value)
        kind = str(sensor.value)
        aimed = str(target.value)
        hfov = HFOV_3_1R_DEG if "3.1R" in str(lens.value) else HFOV_IMAGING_DEG
        pos = place_hand(data, dist) if aimed == "candle" else place_phone(data, dist)
        mujoco.mj_forward(model, data)
        phone.position = tuple(float(v) for v in pos)
        lens_dot.position = (float(pos[0] + 0.04), float(pos[1]), float(pos[2]))
        motion = rail.update(dist)
        sources = candle_sources() if aimed == "candle" else iron_sources()
        frame, thermal_src = _thermal(tuple(float(v) for v in pos), hfov, inbox, sources)
        picture = colorize(frame)
        big = np.repeat(np.repeat(picture, 3, axis=0), 3, axis=1)
        gui_image.image = big
        cloud = scan(kind, model, data, site="lepton" if kind == PHONE else "lidar", inbox=inbox)
        _sync_cloud(server, cloud_handle, cloud, candle_sources() + iron_sources())
        center = float(frame[frame.shape[0] // 2, frame.shape[1] // 2])
        clipped = min(center, _HIGH_GAIN_MAX_C)
        counts = int(to_tlinear_counts(np.array([clipped]))[0])
        hot = hot_pixel_count(frame, AMBIENT + 8.0)
        sat = hot_pixel_count(frame, _HIGH_GAIN_MAX_C)
        tip.color = (255, 40, 20) if center > _HIGH_GAIN_MAX_C else (230, 90, 30)
        moved = (
            "holding still"
            if motion.direction == "still"
            else f"moved {motion.direction} by {abs(motion.delta_m) * 100:.0f} cm"
        )
        clip_note = (
            f"High gain would clip this at {_HIGH_GAIN_MAX_C:.0f} °C ({sat} pixels at or above that)."
            if center > _HIGH_GAIN_MAX_C
            else "Center is inside the high-gain window."
        )
        readout.content = (
            f"Screen is the 160×120 frame. Cloud points use the same °C. "
            f"Phone **{dist:.2f} m** from the {aimed}, {moved}. "
            f"Straight-ahead hit **{cloud.forward_m:.2f} m**, nearest cloud point "
            f"**{cloud.nearest_m:.2f} m** ({cloud.xyz.shape[0]} points). "
            f"Center **{center:.0f} °C assumed**, shown as **{clipped:.0f} °C**, "
            f"count **{counts}**. Hot pixels **{hot}**. Source **{thermal_src}**. "
            f"{clip_note}"
        )
        time.sleep(0.05)


def _thermal(pos, hfov: float, inbox: Path, sources) -> tuple[np.ndarray, str]:
    path = inbox / "thermal.npy"
    if path.is_file():
        try:
            return load_thermal(path), "inbox/thermal.npy"
        except ValueError as exc:
            return render_frame(pos, hfov_deg=hfov, sources=sources, ambient_c=AMBIENT), f"inbox refused: {exc}"
    live = read_uvc()
    note = "sim"
    if live is not None:
        note = "board plugged in, picture still the sim until Y16 is decoded"
    return render_frame(pos, hfov_deg=hfov, sources=sources, ambient_c=AMBIENT), note


def _sync_cloud(server: viser.ViserServer, handle: dict, cloud, sources) -> None:
    if cloud.xyz.shape[0] == 0:
        return
    temps = sample_temperature(cloud.xyz, sources, AMBIENT)
    colors = colorize(temps.reshape(-1, 1))[:, 0, :]
    if handle["h"] is None:
        handle["h"] = server.scene.add_point_cloud(
            "/sensors/cloud",
            points=cloud.xyz.astype(np.float32),
            colors=colors,
            point_size=0.012,
        )
    else:
        handle["h"].points = cloud.xyz.astype(np.float32)
        handle["h"].colors = colors


def _add_cad(server: viser.ViserServer) -> None:
    path = cad_mesh()
    lx, ly, lz = LIDAR_SITE_XYZ
    site = (_ROVER[0] + lx, _ROVER[1] + ly, _ROVER[2] + lz)
    if path is None:
        server.scene.add_box(
            "/rover",
            dimensions=(0.70, 0.50, 0.08),
            position=tuple(float(v) for v in _ROVER),
            color=(120, 124, 128),
        )
    else:
        loaded = trimesh.load(path, force="mesh")
        server.scene.add_mesh_simple(
            "/rover/cad",
            vertices=np.asarray(loaded.vertices, dtype=np.float32),
            faces=np.asarray(loaded.faces, dtype=np.uint32),
            position=tuple(float(v) for v in _ROVER),
            color=(176, 182, 186),
            flat_shading=True,
            side="double",
        )
    server.scene.add_icosphere("/rover/lidar", radius=0.02, position=site, color=(40, 140, 255))
