"""Browser drawing for the URDF body, and the arm panel.

Each MuJoCo body gets a viser frame. Its primitive shapes are children of
that frame at their fixed local pose, so moving the frame moves the shapes.
The viewer already moves body frames by name every tick.
"""

from __future__ import annotations

import math
from typing import Any, Callable

import mujoco

from embr_local_sim.urdf_model import visual_geoms


def add_urdf_geometry(server: Any, model: mujoco.MjModel) -> tuple[Any, dict[str, Any]]:
    """Returns the root frame and the other body frames, keyed by body name."""
    frames: dict[str, Any] = {}
    for geom in visual_geoms(model):
        body = geom["body"]
        if body not in frames:
            frames[body] = server.scene.add_frame(f"/urdf/{body}", show_axes=False)
        path = f"/urdf/{body}/{geom['name']}"
        pose = {"position": tuple(geom["pos"]), "wxyz": tuple(geom["quat"]), "color": geom["rgb"]}
        size = geom["size"]
        if geom["type"] == mujoco.mjtGeom.mjGEOM_BOX:
            server.scene.add_box(path, dimensions=tuple(2.0 * size), **pose)
        elif geom["type"] == mujoco.mjtGeom.mjGEOM_CYLINDER:
            server.scene.add_cylinder(path, radius=float(size[0]), height=float(2.0 * size[1]), **pose)
        elif geom["type"] == mujoco.mjtGeom.mjGEOM_SPHERE:
            server.scene.add_icosphere(path, radius=float(size[0]), **pose)
    root = frames.pop("root", None) or server.scene.add_frame("/urdf/root", show_axes=False)
    return root, frames


class ArmPanel:
    """Height, pitch, roll sliders in [-1, 1] and the live ArmStatus."""

    def __init__(self, server: Any, get_sim: Callable[[], Any]) -> None:
        self._get_sim = get_sim
        with server.gui.add_folder("Arm", expand_by_default=True):
            self.height = server.gui.add_slider(
                "Height", min=-1.0, max=1.0, step=0.01, initial_value=0.0, hint="+1 all arms down, -1 all arms up"
            )
            self.pitch = server.gui.add_slider("Pitch", min=-1.0, max=1.0, step=0.01, initial_value=0.0, hint="+ nose down")
            self.roll = server.gui.add_slider("Roll", min=-1.0, max=1.0, step=0.01, initial_value=0.0, hint="+ left side up")
            self.auto = server.gui.add_checkbox(
                "Auto posture in missions",
                initial_value=True,
                hint="Levels on slopes, lifts for clearance, lifts higher when stuck. Off: the sliders hold.",
            )
            self.status = server.gui.add_markdown("**ArmStatus** IDLE")
        for slider in (self.height, self.pitch, self.roll):
            slider.on_update(lambda _event: self._send())
        self.auto.on_update(lambda _event: self._get_sim().set_auto_posture(bool(self.auto.value)))

    def nudge(self, height: float, pitch: float, roll: float) -> None:
        """Keyboard arm control. Manual input takes the arms back from auto posture."""
        if self.auto.value:
            self.auto.value = False
        for slider, delta in ((self.height, height), (self.pitch, pitch), (self.roll, roll)):
            if delta:
                slider.value = round(max(-1.0, min(1.0, float(slider.value) + delta)), 3)

    def _send(self) -> None:
        self._get_sim().set_arm(float(self.height.value), float(self.pitch.value), float(self.roll.value))

    def sync(self) -> None:
        sim = self._get_sim()
        status = sim.arm_status() if hasattr(sim, "arm_status") else None
        if status is None:
            self.status.content = "**ArmStatus** — this body has no arms"
            return
        deg = {k: math.degrees(v) for k, v in sim.arm.joint_positions(sim.data).items()}
        auto = getattr(sim, "auto_posture", None)
        mode = f"auto `{auto.mode}`" if auto is not None and auto.enabled else "manual"
        cmd = sim.arm.normalised
        self.status.content = (
            f"**ArmStatus** `{status.name}` — {status.message}  \n"
            f"{mode} · h `{cmd['height']:+.2f}` p `{cmd['pitch']:+.2f}` r `{cmd['roll']:+.2f}`  \n"
            f"FL `{deg['arm_front_left']:+.0f}°` · FR `{deg['arm_front_right']:+.0f}°` · "
            f"RL `{deg['arm_rear_left']:+.0f}°` · RR `{deg['arm_rear_right']:+.0f}°` (+ is up)"
        )
