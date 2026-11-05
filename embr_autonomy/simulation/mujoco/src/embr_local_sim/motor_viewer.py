"""Browser display of the standalone motor's MuJoCo geometry and replay state."""

from __future__ import annotations

import math
import queue
import time
from typing import TYPE_CHECKING, Any

import mujoco
import numpy as np

from embr_local_sim.motor import MotorSample, MotorTwin
from embr_local_sim.motor_demo import DEMO_DURATION_S, DEMO_HZ, demo_sample

if TYPE_CHECKING:
    from embr_local_sim.drivetrain import DrivetrainTwin


def add_motor_geometry(server: Any, twin: MotorTwin | DrivetrainTwin) -> list[Any]:
    """Create browser primitives from compiled MuJoCo geom sizes and colours."""
    handles = []
    for geom_id in range(twin.model.ngeom):
        geom = twin.model.geom(geom_id)
        name = f"/motor/{geom.name}"
        color = tuple(int(round(float(v) * 255)) for v in geom.rgba[:3])
        size = geom.size
        if geom.type[0] == mujoco.mjtGeom.mjGEOM_BOX:
            handle = server.scene.add_box(name, dimensions=tuple(2 * size), color=color)
        elif geom.type[0] == mujoco.mjtGeom.mjGEOM_CYLINDER:
            handle = server.scene.add_cylinder(name, radius=float(size[0]), height=float(2 * size[1]), color=color)
        else:
            raise ValueError(f"Unsupported motor geometry: {geom.name}")
        handles.append(handle)
    sync_motor_geometry(twin, handles)
    return handles


def sync_motor_geometry(twin: MotorTwin | DrivetrainTwin, handles: list[Any]) -> None:
    """Copy MuJoCo's world geom transforms; the browser computes no shaft motion."""
    quat = np.zeros(4)
    for geom_id, handle in enumerate(handles):
        mujoco.mju_mat2Quat(quat, twin.data.geom_xmat[geom_id])
        handle.position = tuple(twin.data.geom_xpos[geom_id])
        handle.wxyz = tuple(quat)


def run_motor_viewer(twin: MotorTwin, *, host: str = "127.0.0.1", port: int = 43147) -> None:
    """Run a 50 Hz source replay with a 25 Hz browser target until Ctrl-C.

    Callbacks enqueue controls; only this loop writes the twin. Slow machines
    stretch playback rather than skipping observations to catch up with time.
    """
    import viser

    from embr_local_sim.viewer import _require_free_port

    if not 1 <= port <= 65535:
        raise ValueError("--port must be between 1 and 65535")
    _require_free_port(host, port)
    server = viser.ViserServer(host=host, port=port, label="EMBR Motor Twin")
    try:
        server.gui.configure_theme(control_width="medium", show_logo=False, show_share_button=False)
        server.scene.set_up_direction("+z")
        radius = twin.params.housing_radius_m
        height = radius + .014
        span = max(radius * 2, twin.params.housing_length_m + twin.params.shaft_length_m)
        server.initial_camera.position = (span * 1.8, -span * 3, height + span * 1.4)
        server.initial_camera.look_at = (0, -twin.params.housing_length_m * .25, height)
        server.initial_camera.near = .001
        server.scene.add_grid("/grid", width=span * 4, height=span * 4, plane="xy",
                              cell_size=span / 4, section_size=span)
        handles = add_motor_geometry(server, twin)
        server.gui.add_markdown(
            "## Maxon EC-i52\n**420 W · part 667065**\n\n"
            "**Synthetic feedback · one motor · no hardware connected**\n\n"
            "Orange pointer: rotating shaft. Blue mark: fixed zero reference. "
            "Positive rotation is counterclockwise when looking into the exposed shaft end."
        )
        telemetry = server.gui.add_markdown("")
        with server.gui.add_folder("Replay"):
            replay = server.gui.add_button("Run / replay 12-second demo", color="green")
            pause = server.gui.add_button("Pause / resume", disabled=True)
            reset = server.gui.add_button("Reset to zero")
            server.gui.add_markdown(
                "**0–2 s** +30 rpm · **2–4 s** +60 rpm · **4–6 s** stopped\n\n"
                "**6–8 s** −30 rpm · **8–10 s** −60 rpm · **10–12 s** stopped\n\n"
                "Pause freezes replay time. Reset clears the replay and returns the shaft to zero."
            )
        server.gui.add_markdown(
            "The housing stays fixed. Dimensions other than the 52 mm diameter are simplified. "
            "This replays supplied angle/speed into MuJoCo; it does not predict motor torque, "
            "acceleration or ESCON behaviour. Speeds are deliberately low for visibility."
        )
        commands: queue.SimpleQueue[str] = queue.SimpleQueue()
        replay.on_click(lambda _event: commands.put("replay"))
        pause.on_click(lambda _event: commands.put("pause"))
        reset.on_click(lambda _event: commands.put("reset"))
        twin.reset()
        twin.apply(MotorSample(0.0, 0.0, 0.0))
        tick = 0
        running = paused = False
        stage = "Ready"
        deadline = time.monotonic()
        print(f"Motor twin: http://{host}:{port} — synthetic data; Ctrl-C to exit.", flush=True)
        while True:
            changed = False
            while not commands.empty():
                command = commands.get()
                if command == "replay":
                    twin.reset()
                    stage, sample = demo_sample(0.0)
                    twin.apply(sample)
                    tick, running, paused = 0, True, False
                elif command == "reset":
                    twin.reset()
                    twin.apply(MotorSample(0.0, 0.0, 0.0))
                    tick, running, paused, stage = 0, False, False, "Ready"
                elif command == "pause" and (running or paused):
                    running, paused = not running, not paused
                changed = True
            # Wait one interval after replay/resume so the first observation is visible.
            if running and not changed:
                tick += 1
                stage, sample = demo_sample(tick / DEMO_HZ)
                twin.apply(sample)
                if tick == DEMO_DURATION_S * DEMO_HZ:
                    running = False
            if changed or tick % 2 == 0:
                state = twin.state
                status = "Paused (time frozen)" if paused else stage
                with server.atomic():
                    sync_motor_geometry(twin, handles)
                    telemetry.content = (
                        f"### {status}\n"
                        f"**Replay time:** {state.timestamp_s:.2f} / {DEMO_DURATION_S} s\n\n"
                        f"**Shaft angle:** {math.degrees(state.position_rad):.1f}° "
                        f"({state.position_rad / math.tau:.2f} turns)\n\n"
                        f"**Shaft speed:** {state.velocity_rad_s * 60 / math.tau:+.1f} rpm "
                        f"({state.velocity_rad_s:+.3f} rad/s)\n\n"
                        "Position / speed: supplied by the synthetic source."
                    )
                    pause.disabled = not (running or paused)
            deadline = max(deadline + 1 / DEMO_HZ, time.monotonic())
            time.sleep(max(0.0, deadline - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
