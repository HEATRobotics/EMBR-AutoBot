"""Viser browser viewer. Displays physics state; never invents a trajectory."""

from __future__ import annotations

import math
import select
import sys
import threading
import time
from typing import Any

import numpy as np

from embr_local_sim.model import RobotParams, WorldSpec
from embr_local_sim.physics import Simulator, UiRequest
from embr_local_sim.tele_cmd import TeleCmd
from embr_local_sim.types import GroundTruthState

# 90 deg about X: MuJoCo wheel cylinders are Y-axis, Viser cylinders are Z-axis.
_RX90 = np.array([math.cos(math.pi / 4.0), math.sin(math.pi / 4.0), 0.0, 0.0])


def _qmul(a: np.ndarray, b: np.ndarray) -> tuple[float, float, float, float]:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        float(w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2),
        float(w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2),
        float(w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2),
        float(w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2),
    )


def _try_stdin_keys(ctx: dict[str, Any], stop: threading.Event, hold: dict[str, float]) -> None:
    if not sys.stdin.isatty():
        return
    try:
        import termios
        import tty
    except ImportError:
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not stop.is_set():
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not ready:
                continue
            ch = sys.stdin.read(1)
            sim: Simulator = ctx["sim"]
            if ch in ("q", "Q", "\x03"):
                sim.enqueue(UiRequest(kind="stop"))
                stop.set()
                break
            _apply_key(ch, sim, hold)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _apply_key(ch: str, sim: Simulator, hold: dict[str, float]) -> None:
    vx = hold.setdefault("vx", 0.8)
    wz = hold.setdefault("wz", 0.8)
    if ch in ("w", "W"):
        sim.ingest_tele_cmd(TeleCmd(velocity=vx, turn=0.0, timestamp_wall=time.time(), source="keyboard"))
    elif ch in ("s", "S"):
        sim.ingest_tele_cmd(TeleCmd(velocity=-vx, turn=0.0, timestamp_wall=time.time(), source="keyboard"))
    elif ch in ("a", "A"):
        sim.ingest_tele_cmd(TeleCmd(velocity=0.0, turn=-wz, timestamp_wall=time.time(), source="keyboard"))
    elif ch in ("d", "D"):
        sim.ingest_tele_cmd(TeleCmd(velocity=0.0, turn=wz, timestamp_wall=time.time(), source="keyboard"))
    elif ch in (" ", "x", "X"):
        sim.ingest_tele_cmd(TeleCmd(0.0, 0.0, time.time(), source="keyboard-stop"))
    elif ch in ("m", "M"):
        sim.start_mission()
    elif ch in ("c", "C"):
        sim.cancel_mission("operator_cancel")
    elif ch in ("r", "R"):
        sim.request_reset()
    elif ch in ("p", "P"):
        gt, _ = sim.snapshot()
        sim.enqueue(UiRequest(kind="resume" if gt.paused else "pause"))
    elif ch == ".":
        sim.enqueue(UiRequest(kind="step"))


def _track_camera(
    client: Any,
    mode: str,
    root_pos: tuple[float, float, float],
    state: dict[int, dict[str, Any]],
    *,
    span: float,
    overview: tuple[tuple[float, float, float], tuple[float, float, float]],
) -> None:
    """Place the camera once when a preset is chosen, then leave it to the mouse.

    Click-drag orbits. The scroll wheel moves the camera forward and back.
    Later frames do not write the camera, so driving and a click do not yank it.
    Overview uses the scene camera, so a yard does not snap to the small proxy view.
    """
    key = id(client)
    slot = state.setdefault(key, {"mode": None})
    if slot["mode"] == mode:
        return
    slot["mode"] = mode
    x, y, z = (float(v) for v in root_pos)
    if mode == "overview":
        client.camera.position = overview[0]
        client.camera.look_at = overview[1]
    elif span > 2.0:
        if mode == "close":
            client.camera.position = (x - 1.5, y - 3.2, z + 1.6)
            client.camera.look_at = (x + 1.2, y, z + 0.2)
        else:
            client.camera.position = (x - 2.2, y - 7.0, z + 3.6)
            client.camera.look_at = (x + 2.5, y, z + 0.3)
    elif mode == "close":
        client.camera.position = (x + 0.18, y - 0.22, z + 0.14)
        client.camera.look_at = (x, y, z + 0.02)
    else:
        client.camera.position = (x + 0.42, y - 0.48, z + 0.26)
        client.camera.look_at = (x, y, z)


def _install_browser_wasd(server: Any) -> None:
    """Hold-to-drive from the browser keyboard.

    Viser has no key callback, and its camera already uses W A S D. This script
    stops those keys before they reach the camera, then writes the held chord
    into the Held keys field through that input's React onChange.
    """
    from viser import _messages

    source = r"""
if (!window.__embrWasdArc) {
  window.__embrWasdArc = true;
  const held = new Set();
  const driveKeys = {w: 1, a: 1, s: 1, d: 1};
  const cameraCodes = {
    KeyW: 1, KeyA: 1, KeyS: 1, KeyD: 1, KeyQ: 1, KeyE: 1,
    ArrowUp: 1, ArrowDown: 1, ArrowLeft: 1, ArrowRight: 1
  };
  function keysInput() {
    for (const label of document.querySelectorAll("label")) {
      if ((label.textContent || "").trim() !== "Held keys") continue;
      let node = label;
      for (let depth = 0; depth < 6 && node; depth += 1) {
        const input = node.querySelector("input, textarea");
        if (input) return input;
        node = node.parentElement;
      }
    }
    return null;
  }
  function publish() {
    const input = keysInput();
    if (!input) return;
    const next = Array.from(held).sort().join("");
    const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value").set;
    setter.call(input, next);
    let node = input;
    for (let depth = 0; depth < 6 && node; depth += 1) {
      const propsKey = Object.keys(node).find((key) => key.startsWith("__reactProps"));
      const onChange = propsKey && node[propsKey] && node[propsKey].onChange;
      if (typeof onChange === "function") {
        onChange({target: input, currentTarget: input});
        return;
      }
      node = node.parentElement;
    }
    input.dispatchEvent(new Event("input", {bubbles: true}));
  }
  function typingTarget(event) {
    const tag = event.target && event.target.tagName;
    if (tag !== "INPUT" && tag !== "TEXTAREA" && tag !== "SELECT") return false;
    return event.target !== keysInput();
  }
  function findButton(prefix) {
    for (const button of document.querySelectorAll("button")) {
      const text = (button.innerText || "").replace(/\s+/g, " ").trim();
      if (text.startsWith(prefix)) return button;
    }
    return null;
  }
  window.addEventListener("keydown", (event) => {
    const typing = typingTarget(event);
    const key = (event.key || "").toLowerCase();
    if (!typing && (driveKeys[key] || cameraCodes[event.code] || event.code === "Space")) {
      event.preventDefault();
      event.stopPropagation();
    }
    if (event.metaKey || event.ctrlKey || event.altKey || typing) return;
    if (driveKeys[key]) {
      if (!event.repeat) held.add(key);
      publish();
      return;
    }
    if (event.code === "Space") {
      held.clear();
      publish();
      const stopButton = findButton("Space");
      if (stopButton) stopButton.click();
    }
  }, true);
  window.addEventListener("keyup", (event) => {
    const key = (event.key || "").toLowerCase();
    if (driveKeys[key]) {
      event.preventDefault();
      event.stopPropagation();
    }
    if (!held.has(key)) return;
    held.delete(key);
    publish();
  }, true);
  window.addEventListener("blur", () => {
    if (held.size === 0) return;
    held.clear();
    publish();
  });
  setInterval(() => {
    if (held.size > 0) publish();
  }, 50);
}
"""
    server.gui._websock_interface.queue_message(_messages.RunJavascriptMessage(source=source))


def _require_free_port(host: str, port: int) -> None:
    """Fail before Viser starts if another process already has the port."""
    import socket

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind((host, port))
    except OSError as exc:
        reason = exc.strerror or str(exc)
        raise RuntimeError(
            f"Port {port} on {host} is already in use ({reason}). "
            "Stop the other process or choose a free port with --port."
        ) from exc
    finally:
        sock.close()


def run_viewer(
    *,
    host: str = "127.0.0.1",
    port: int = 43147,
    world: WorldSpec | None = None,
    params: RobotParams | None = None,
    start_mission: bool = False,
    mode: str | None = None,
    seed: int = 2,
) -> None:
    import viser

    from embr_local_sim.worlds import prepare_world

    params = params or RobotParams()
    if mode:
        world = prepare_world(mode, seed, params.morphology)
    world = world or WorldSpec(kind="flat")
    sim = Simulator(params=params, world=world, seed=seed)
    sim.reset()
    sim.start_background()
    if world.kind in ("rough", "yard"):
        from embr_local_sim.mission import mission_for_robot

        sim.start_mission(mission_for_robot(sim.params, world, 4))
    elif start_mission:
        sim.start_mission()

    _require_free_port(host, port)
    server = viser.ViserServer(host=host, port=port, label="EMBR MuJoCo")
    try:
        server.gui.configure_theme(
            control_width="small",
            show_logo=False,
            show_share_button=False,
        )
        server.gui.main_panel.dock_left()
        server.gui.main_panel.set_width(300)
    except Exception:
        pass
    from embr_local_sim.body import resolve_body

    body = resolve_body(sim.params.morphology)
    server.scene.set_up_direction("+z")
    server.initial_camera.position = body.camera_position
    server.initial_camera.look_at = body.camera_look_at
    if world.kind == "rough" and params.morphology == "cad":
        server.initial_camera.position = (0.4, -5.5, 2.6)
        server.initial_camera.look_at = (2.4, 0.0, 0.12)
    if world.kind == "yard" and params.morphology == "cad":
        server.initial_camera.position = (0.4, -5.4, 2.6)
        server.initial_camera.look_at = (2.8, 0.0, 0.35)

    grid = body.grid_m
    server.scene.add_grid(
        "/grid",
        width=grid,
        height=grid,
        plane="xy",
        cell_color=(80, 80, 80),
        section_color=(40, 40, 40),
    )

    ctx: dict[str, Any] = {
        "sim": sim,
        "params": params,
        "world": world,
        "chassis": None,
        "wheels": {},
        "extra": {},
        "world_nodes": [],
        "odom_trace": [],
    }
    ctx["world_nodes"] = _add_static_world(server, world, params)
    chassis_handle, wheel_handles, extra_handles = _add_robot_geometry(server, params, sim)
    ctx["chassis"] = chassis_handle
    ctx["wheels"] = wheel_handles
    ctx["extra"] = extra_handles

    fidelity = _fidelity_line(params)
    server.gui.add_markdown(f"**{fidelity}**")
    telemetry = server.gui.add_markdown("Connecting…")
    with server.gui.add_folder("Drive"):
        server.gui.add_markdown(
            "**W** forward&nbsp;&nbsp;**S** back\n\n"
            "**A** turn left&nbsp;&nbsp;**D** turn right\n\n"
            "A and D alone turn in place. W or S with A or D turns while moving. "
            "Backing up, A still bends left and D still bends right.\n\n"
            "The blue trail follows the robot."
        )
        speed = server.gui.add_slider("|velocity|", min=0.1, max=1.0, step=0.05, initial_value=0.7)
        turn = server.gui.add_slider("|turn|", min=0.1, max=1.0, step=0.05, initial_value=0.6)
        btn_f = server.gui.add_button("W  forward", color="green")
        btn_l = server.gui.add_button("A  turn left", color="blue")
        btn_b = server.gui.add_button("S  back", color="green")
        btn_r = server.gui.add_button("D  turn right", color="blue")
        btn_stop = server.gui.add_button("Space  stop / brake", color="red")
        key_box = server.gui.add_text(
            "Held keys",
            "",
            hint="Only the letters w a s d. dw means forward and right.",
        )
    initial_mode = "basic"
    if world.kind in ("forest",):
        initial_mode = "forest"
    if world.kind in ("grove",):
        initial_mode = "grove"
    if world.kind in ("course",):
        initial_mode = "course"
    if world.kind in ("engineering",):
        initial_mode = "engineering"
    if world.kind in ("wildfire",) or start_mission:
        initial_mode = "wildfire-mission"
    if world.kind == "ramp":
        initial_mode = "ramp"
    if world.kind == "wall":
        initial_mode = "wall"
    if world.kind == "split_friction":
        initial_mode = "split-friction"
    if world.kind == "bumps":
        initial_mode = "stones"
    if world.kind == "rough":
        initial_mode = "rough"
    if world.kind == "yard":
        initial_mode = "yard"
    scene_options = (
        "basic",
        "course",
        "grove",
        "forest",
        "engineering",
        "wildfire-mission",
        "ramp",
        "wall",
        "split-friction",
        "stones",
        "rough",
        "yard",
    )
    if mode:
        key = mode.replace("_", "-")
        if key in scene_options or key == "wildfire":
            initial_mode = "wildfire-mission" if key == "wildfire" else key
    if initial_mode not in scene_options:
        initial_mode = "basic"
    with server.gui.add_folder("Scene"):
        mode_dd = server.gui.add_dropdown(
            "Mode",
            options=scene_options,
            initial_value=initial_mode,
            hint="Rebuilds the MuJoCo world. That reset is labeled.",
        )
        seed_slider = server.gui.add_slider("Seed", min=0, max=32, step=1, initial_value=float(seed))
        cam_dd = server.gui.add_dropdown(
            "Camera",
            options=("overview", "follow", "close"),
            initial_value="overview",
            hint="Applies once. Then click-drag to look, or scroll to move forward and back. W A S D drive the robot.",
        )
        show_lidar = server.gui.add_checkbox("LiDAR overlay", initial_value=True)
        show_contacts = server.gui.add_checkbox(
            "Balance overlay",
            initial_value=False,
            hint="Red sphere: subtree CoM. Blue: gravity direction. Orange: contact normal, 0.002 m/N. Not a stability proof.",
        )
        show_gt = server.gui.add_checkbox(
            "Observer-only GT pose / heat coords",
            initial_value=False,
            hint="Not available to autonomy unless oracle is on.",
        )
    from embr_local_sim.mission import instruction_markdown

    with server.gui.add_folder("Mission / autonomy", expand_by_default=True):
        instructions = server.gui.add_markdown(instruction_markdown(world))
        ctx["instructions"] = instructions
        btn_mission = server.gui.add_button("L4 follow instructions", color="orange")
        btn_goto = server.gui.add_button("L2/L3 goto odom target")
        btn_search = server.gui.add_button("L5 bounded search")
        btn_cancel = server.gui.add_button("Operator STOP / FAULT", color="red")
        goto_span = body.view_span_m
        large = goto_span > 2.0
        goto_x = server.gui.add_slider(
            "Goto odom x", min=body.view_min_m, max=goto_span, step=0.05,
            initial_value=3.0 if large else 0.70,
        )
        goto_y = server.gui.add_slider(
            "Goto odom y", min=-goto_span, max=goto_span, step=0.05, initial_value=0.00,
        )
        btn_probe = server.gui.add_button("Probe deploy")
        btn_stow = server.gui.add_button("Probe stow")
        oracle = server.gui.add_checkbox("ORACLE / debug (uses labeled GT map)", initial_value=False)
    from embr_local_sim.inspection import InspectionConfig, elapsed_sim, world_point_to_estimated
    from embr_local_sim.inspection_run import _world as inspection_world
    from embr_local_sim.inspection_run import scenario_catalog

    case_names = tuple(scenario_catalog().keys())
    with server.gui.add_folder("Inspection tests", expand_by_default=False):
        server.gui.add_markdown(
            "These five cases are a flat course with no trees or fire. "
            "Reset scenario replaces the yard. The mission buttons above "
            "keep the scene that is already on screen."
        )
        case_dd = server.gui.add_dropdown("Scenario", options=case_names, initial_value="clear")
        from embr_local_sim.body import KNOWN_BODIES

        model_options = KNOWN_BODIES
        model_dd = server.gui.add_dropdown(
            "Model",
            options=model_options,
            initial_value=params.morphology if params.morphology in model_options else "tracks",
            hint="cad is the Onshape STEP. The others are the older proxies.",
        )
        target_span = body.view_span_m
        insp_tx = server.gui.add_slider(
            "Target x (world m)", min=body.view_min_m, max=target_span, step=0.05,
            initial_value=2.5 if target_span > 2.0 else 0.55,
        )
        insp_ty = server.gui.add_slider(
            "Target y (world m)", min=-target_span, max=target_span, step=0.05,
            initial_value=0.5 if target_span > 2.0 else 0.12,
        )
        btn_case_reset = server.gui.add_button("Reset scenario")
        btn_insp_start = server.gui.add_button("Load flat case and start")
        btn_insp_cancel = server.gui.add_button("Cancel / Stop", color="red")
        btn_takeover = server.gui.add_button("Manual takeover")
        btn_insp_pause = server.gui.add_button("Pause / resume")
        show_route = server.gui.add_checkbox(
            "Show bearing",
            initial_value=False,
            hint="Inspection cases only. A straight line to the case target, not the mission path.",
        )
        show_eval = server.gui.add_checkbox("Evaluation only", initial_value=False, hint="Ground-truth distances. The controller does not use these.")
        insp_status = server.gui.add_markdown("Idle.")
    start_marker = server.scene.add_icosphere(
        "/inspection/start", radius=0.028, position=(0.0, 0.0, 0.03), color=(40, 90, 220)
    )
    target_marker = server.scene.add_icosphere(
        "/inspection/target", radius=0.028, position=(0.55, 0.12, 0.03), color=(220, 140, 40)
    )
    start_marker.visible = False
    target_marker.visible = False
    route_handle: dict[str, Any] = {"h": None}
    ctx["insp_case"] = None
    with server.gui.add_folder("Sim"):
        btn_reset = server.gui.add_button("R  reset (teleport)")
        btn_pause = server.gui.add_button("P  pause / resume")
        btn_step = server.gui.add_button(".  single control tick")
    art_btns = {}
    art_sliders = {}
    if params.morphology in ("articulated", "tracks"):
        with server.gui.add_folder("Modules", expand_by_default=False):
            for corner in ("fl", "fr", "rl", "rr"):
                art_sliders[corner] = server.gui.add_slider(
                    corner,
                    min=-0.55,
                    max=0.55,
                    step=0.05,
                    initial_value=0.0,
                )
            art_btns["flat"] = server.gui.add_button("Flat")
            art_btns["climb"] = server.gui.add_button("Nose down")
            art_btns["fault"] = server.gui.add_button("Supervisor FAULT", color="red")
            art_btns["assisted"] = server.gui.add_button("Supervisor assisted")
    with server.gui.add_folder("Limits and labels", expand_by_default=False):
        server.gui.add_markdown(
            "TeleCmd `velocity` and `turn` are normalized from -1 to 1, not m/s. "
            "Positive turn is right. W is forward, A turns left, S is back, D turns right. "
            "A and D alone turn in place. W or S with A or D turns while moving. "
            "These keys do not move the camera. Click-drag looks around, and "
            "the scroll wheel moves the camera forward and back. Terminal keys still send one "
            "command per keypress. Releasing a key or button stops new commands; "
            "the 0.40 s watchdog then zeros the speed target. Pause freezes physics "
            "and drops drive commands. Resume starts from a zero speed target. "
            "Disconnect zeros the command.\n\n"
            "The probe hinge is an unconfirmed placeholder. CAD includes "
            "`leadscrew.SLDPRT` and `thermoprobe.SLDPRT`, which were not imported. "
            "Thermal peaks are assumed °C, not fire simulation. Ratings are UNKNOWN. "
            "Ground-truth pose and contact forces are debug overlays. The mission "
            "does not read them unless ORACLE is on.\n\n"
            "A support polygon is not drawn. Contacts, slip, and actuator limits "
            "decide stability. CoM inside a polygon would not be a guarantee here."
        )

    hold = {"vx": speed.value, "wz": turn.value}

    def _sim() -> Simulator:
        return ctx["sim"]

    def _tele(velocity: float, turn_cmd: float, source: str) -> None:
        _sim().ingest_tele_cmd(
            TeleCmd(velocity=velocity, turn=turn_cmd, timestamp_wall=time.time(), source=source)
        )

    def _rebuild(
        new_mode: str | None = None,
        *,
        world: WorldSpec | None = None,
        params: RobotParams | None = None,
        teleport: dict | None = None,
        seed: int | None = None,
    ) -> None:
        old: Simulator = ctx["sim"]
        old.enqueue(UiRequest(kind="stop"))
        old.stop_background()
        for node in ctx["world_nodes"]:
            try:
                node.remove()
            except Exception:
                node.visible = False
        for handle in (ctx["chassis"], *ctx["wheels"].values(), *ctx["extra"].values()):
            if handle is None:
                continue
            try:
                handle.remove()
            except Exception:
                handle.visible = False
        _lidar_handle["h"] = None
        _probe_handle["h"] = None
        _contact_handle["h"] = None
        use_seed = int(seed_slider.value) if seed is None else int(seed)
        params_new = params or ctx["params"]
        if world is None:
            world_new = prepare_world(new_mode or "basic", use_seed, params_new.morphology)
        else:
            world_new = world
            from embr_local_sim.body import resolve_body as _resolve_body

            scale = _resolve_body(params_new.morphology).layout_scale
            if scale != 1.0 and not (world_new.extras or {}).get("layout_scale"):
                from embr_local_sim.worlds import scale_world

                world_new = scale_world(world_new, scale)
        sim_new = Simulator(params=params_new, world=world_new, seed=use_seed)
        sim_new.reset(teleport)
        sim_new.start_background()
        ctx["sim"] = sim_new
        ctx["params"] = params_new
        ctx["world"] = world_new
        ctx["odom_trace"] = []
        _clear_trace_visual()
        ctx["world_nodes"] = _add_static_world(server, world_new, ctx["params"])
        ch, wh, eh = _add_robot_geometry(server, ctx["params"], sim_new)
        ctx["chassis"], ctx["wheels"], ctx["extra"] = ch, wh, eh
        panel = ctx.get("instructions")
        if panel is not None:
            from embr_local_sim.mission import instruction_markdown

            panel.content = instruction_markdown(world_new)

    # Mouse holds and browser key repeats both stamp these. One combined
    # command is sent from the render loop so W+A drives and turns together.
    pressed = {"w": 0.0, "a": 0.0, "s": 0.0, "d": 0.0}

    def _mark(axis: str) -> None:
        pressed[axis] = time.time()

    def _drive_from_keys() -> None:
        now = time.time()
        fresh = 0.18
        raw = str(key_box.value).lower().strip()
        held = set(raw) if raw and set(raw) <= set("wasd") else set()
        w = "w" in held or now - pressed["w"] < fresh
        s = "s" in held or now - pressed["s"] < fresh
        a = "a" in held or now - pressed["a"] < fresh
        d = "d" in held or now - pressed["d"] < fresh
        if not (w or s or a or d):
            return
        # W forward, S back, A turn left, D turn right. Positive turn is right.
        vx = (float(speed.value) if w else 0.0) - (float(speed.value) if s else 0.0)
        turn_cmd = (float(turn.value) if d else 0.0) - (float(turn.value) if a else 0.0)
        _tele(vx, turn_cmd, "gui-hold")

    for _axis, _button in (("w", btn_f), ("s", btn_b), ("a", btn_l), ("d", btn_r)):
        def _bind(axis: str, button: Any) -> None:
            @button.on_hold(callback_hz=20.0)
            def _hold(_) -> None:
                _mark(axis)

        _bind(_axis, _button)

    @btn_stop.on_click
    def _stop(_) -> None:
        pressed["w"] = pressed["a"] = pressed["s"] = pressed["d"] = 0.0
        _tele(0.0, 0.0, "gui-stop")

    _install_browser_wasd(server)

    def _mission_cfg(level: int):
        from embr_local_sim.mission import mission_for_robot

        sim_now = _sim()
        cfg = mission_for_robot(sim_now.params, sim_now.world, level)
        cfg.oracle = bool(oracle.value)
        return cfg

    @btn_mission.on_click
    def _mission(_) -> None:
        from embr_local_sim.mission import scene_to_open_for_follow

        target = scene_to_open_for_follow(str(ctx["world"].kind))
        if target is not None:
            _rebuild(target)
            try:
                mode_dd.value = target
            except Exception:
                pass
        ctx["rerun_at"] = None
        _sim().start_mission(_mission_cfg(4))

    @btn_goto.on_click
    def _goto(_) -> None:
        cfg = _mission_cfg(3)
        cfg.goto_x = float(goto_x.value)
        cfg.goto_y = float(goto_y.value)
        _sim().start_mission(cfg)

    @btn_search.on_click
    def _search(_) -> None:
        _sim().start_mission(_mission_cfg(5))

    @btn_cancel.on_click
    def _cancel(_) -> None:
        _sim().cancel_mission("operator_stop")

    def _load_case(start: bool) -> None:
        spec = dict(scenario_catalog()[str(case_dd.value)])
        morph = str(model_dd.value)
        params_new = RobotParams(morphology=morph)  # type: ignore[arg-type]
        world_new = inspection_world(spec)
        _rebuild(world=world_new, params=params_new, teleport=dict(spec["spawn"]), seed=int(spec["seed"]))
        sim_now = _sim()
        sim_now.sensors.lidar_dropout_after = spec["lidar_dropout_after"]
        sim_now.probe.config.retract_fault = bool(spec["retract_fault"])
        ctx["insp_case"] = str(case_dd.value)
        ctx["insp_spec"] = spec
        target_scale = float((world_new.extras or {}).get("layout_scale", 1.0))
        insp_tx.value = float(spec["target_world"][0]) * target_scale
        insp_ty.value = float(spec["target_world"][1]) * target_scale
        if start:
            _begin_inspection()

    def _begin_inspection() -> None:
        sim_now = _sim()
        spec = ctx.get("insp_spec") or scenario_catalog()[str(case_dd.value)]
        sim_now.sensors.lidar_dropout_after = spec["lidar_dropout_after"]
        sim_now.probe.config.retract_fault = bool(spec["retract_fault"])
        sx, sy, syaw = sim_now.spawn_pose
        gx, gy = world_point_to_estimated(sx, sy, syaw, float(insp_tx.value), float(insp_ty.value))
        sim_now.start_inspection(InspectionConfig(goal_x=gx, goal_y=gy))
        ctx["insp_saved"] = False

    @btn_case_reset.on_click
    def _case_reset(_) -> None:
        _load_case(False)

    @btn_insp_start.on_click
    def _insp_start(_) -> None:
        if ctx.get("insp_case") != str(case_dd.value) or ctx["params"].morphology != str(model_dd.value):
            _load_case(True)
        else:
            _begin_inspection()

    @btn_insp_cancel.on_click
    def _insp_cancel(_) -> None:
        _sim().cancel_mission("operator_stop")

    @btn_takeover.on_click
    def _takeover(_) -> None:
        pressed["w"] = pressed["a"] = pressed["s"] = pressed["d"] = 0.0
        _tele(0.0, 0.0, "gui-stop")

    @btn_insp_pause.on_click
    def _insp_pause(_) -> None:
        gt, _obs = _sim().snapshot()
        _sim().enqueue(UiRequest(kind="resume" if gt.paused else "pause"))

    @btn_probe.on_click
    def _probe(_) -> None:
        _sim().enqueue(UiRequest(kind="probe_deploy"))

    @btn_stow.on_click
    def _stow(_) -> None:
        _sim().enqueue(UiRequest(kind="probe_stow"))

    @btn_reset.on_click
    def _reset(_) -> None:
        _sim().request_reset()
        ctx["odom_trace"] = []
        _clear_trace_visual()

    @btn_pause.on_click
    def _pause(_) -> None:
        gt, _obs = _sim().snapshot()
        _sim().enqueue(UiRequest(kind="resume" if gt.paused else "pause"))

    @btn_step.on_click
    def _step(_) -> None:
        if not _sim().snapshot()[0].paused:
            _sim().enqueue(UiRequest(kind="pause"))
        _sim().enqueue(UiRequest(kind="step"))

    @mode_dd.on_update
    def _mode_change(_) -> None:
        wanted = str(mode_dd.value)
        current = str(ctx["world"].kind)
        alias = {
            "wildfire-mission": "wildfire",
            "wildfire": "wildfire",
            "basic": "flat",
            "split-friction": "split_friction",
            "stones": "bumps",
            "rough": "rough",
            "yard": "yard",
        }
        if alias.get(wanted, wanted) == current:
            return
        _rebuild(wanted)

    if art_btns:
        @art_btns["flat"].on_click
        def _flat(_) -> None:
            _sim().set_posture_preset("flat")

        @art_btns["climb"].on_click
        def _climb(_) -> None:
            _sim().set_posture_preset("climb")

        @art_btns["fault"].on_click
        def _fault(_) -> None:
            _sim().set_supervisor_mode("fault")

        @art_btns["assisted"].on_click
        def _assisted(_) -> None:
            _sim().set_supervisor_mode("assisted")

        def _push_modules(_) -> None:
            _sim().set_art_targets({f"art_{name}": float(slider.value) for name, slider in art_sliders.items()})

        for _slider in art_sliders.values():
            _slider.on_update(_push_modules)

    clients_lock = threading.Lock()
    n_clients = {"n": 0}

    @server.on_client_connect
    def _connect(client) -> None:  # type: ignore[no-untyped-def]
        with clients_lock:
            n_clients["n"] += 1

    @server.on_client_disconnect
    def _disconnect(client) -> None:  # type: ignore[no-untyped-def]
        with clients_lock:
            n_clients["n"] = max(0, n_clients["n"] - 1)
            if n_clients["n"] == 0:
                _sim().ingest_tele_cmd(TeleCmd(0.0, 0.0, time.time(), source="viewer-disconnect"))

    stop = threading.Event()
    stdin_thread = threading.Thread(
        target=_try_stdin_keys, args=(ctx, stop, hold), name="embr-stdin", daemon=True
    )
    stdin_thread.start()

    browse_host = "127.0.0.1" if host in ("0.0.0.0", "::") else host
    morph = {
        "tracks": "distributed-contact track approximation (not a belt, not CAD)",
        "articulated": "articulated roller-pod approximation (not tracks, not CAD)",
        "cad": "CAD visual from the Onshape STEP; contact is four idler cylinders, mass and torque assumed",
    }.get(params.morphology, "four-wheel skid-steer proxy (not the full EMBR)")
    print(f"EMBR MuJoCo viewer")
    print(f"  scene: {world.kind}   seed: {seed}   morphology: {morph}")
    print(f"  physics dt: {params.physics_dt} s   control: {params.control_hz} Hz")
    print(f"  listening on {host}:{port}")
    print(f"  open http://{browse_host}:{port}")
    print("  This Python process must keep running. Viser is not a static website.")
    print("  Stop with Ctrl-C. If this terminal is interactive, q also quits.")
    print("  Browser W/A/S/D drive the robot. Click-drag looks around. Scroll moves the camera forward and back.")
    cam_follow: dict[int, dict[str, Any]] = {}
    try:
        last_render = 0.0
        while not stop.is_set():
            now = time.perf_counter()
            if now - last_render < (1.0 / 25.0):
                time.sleep(0.005)
                continue
            last_render = now
            try:
                _drive_from_keys()
                sim_now = _sim()
                gt, obs = sim_now.snapshot()
                try:
                    _sync_bodies(sim_now, ctx["chassis"], ctx["wheels"], ctx["extra"], gt)
                except RuntimeError:
                    # A refresh can drop the mesh handle. Draw the body again.
                    ch, wh, eh = _add_robot_geometry(server, sim_now.params, sim_now)
                    ctx["chassis"], ctx["wheels"], ctx["extra"] = ch, wh, eh
                if show_lidar.value:
                    _sync_lidar(server, obs)
                _sync_probe(server, sim_now)
                _sync_flame_pulse(ctx["world_nodes"], gt.sim_time)
                if show_contacts.value:
                    _sync_contacts(server, gt)
                    _sync_balance(server, sim_now)
                _append_robot_trace(ctx, gt.root_pos)
                _sync_trace(server, ctx["odom_trace"])
                cam = str(cam_dd.value)
                for client in server.get_clients().values():
                    _track_camera(
                        client,
                        cam,
                        gt.root_pos,
                        cam_follow,
                        span=body.view_span_m,
                        overview=(
                            tuple(server.initial_camera.position),
                            tuple(server.initial_camera.look_at),
                        ),
                    )
                telemetry.content = _telemetry_markdown(
                    gt, obs, n_clients["n"], sim_now, show_gt=bool(show_gt.value)
                )
                route = (sim_now.world.extras or {}).get("route")
                if route and sim_now.mission.state == "COMPLETE":
                    when = ctx.get("rerun_at")
                    if when is None:
                        ctx["rerun_at"] = time.time() + 2.0
                    elif time.time() >= when:
                        ctx["rerun_at"] = None
                        ctx["odom_trace"] = []
                        _clear_trace_visual()
                        sim_now.request_reset()
                        sim_now.start_mission(_mission_cfg(4))
                else:
                    ctx["rerun_at"] = None
                inspecting = ctx.get("insp_case") is not None
                start_marker.visible = inspecting
                target_marker.visible = inspecting
                _sync_inspection(
                    sim_now,
                    gt,
                    insp_status,
                    start_marker,
                    target_marker,
                    route_handle,
                    server,
                    show_route=bool(show_route.value) and inspecting,
                    show_eval=bool(show_eval.value),
                    target_xy=(float(insp_tx.value), float(insp_ty.value)),
                )
            except Exception:
                import traceback

                traceback.print_exc()
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        _sim().enqueue(UiRequest(kind="stop"))
        _sim().stop_background()
        server.stop()


def _add_static_world(server: Any, world: WorldSpec, params: RobotParams) -> list[Any]:
    handles: list[Any] = []
    extras = world.extras or {}
    if world.kind in ("wall", "engineering"):
        handles.append(
            server.scene.add_box(
                "/world/wall",
                dimensions=(world.wall_thickness, 2.0 * world.wall_half_y, world.wall_height),
                position=(world.wall_x, 0.0, 0.5 * world.wall_height),
                color=(190, 90, 50),
            )
        )
    if world.kind in ("ramp", "engineering") or extras.get("include_ramp"):
        angle = math.radians(world.ramp_angle_deg)
        half = 0.5 * world.ramp_length
        cx = world.ramp_start_x + half * math.cos(angle)
        cz = half * math.sin(angle)
        wxyz = (math.cos(-angle / 2.0), 0.0, math.sin(-angle / 2.0), 0.0)
        handles.append(
            server.scene.add_box(
                "/world/ramp",
                dimensions=(world.ramp_length, world.ramp_width, 0.005),
                position=(cx, 0.0, cz),
                wxyz=wxyz,
                color=(110, 120, 105),
            )
        )
    from embr_local_sim.model import lane_ramp_geoms

    for ramp in lane_ramp_geoms(extras):
        px, py, pz = ramp["pos"]
        sx, sy, sz = ramp["size"]
        ey = float(ramp["euler"][1])
        on_deck = "deck" in str(ramp["name"])
        handles.append(
            server.scene.add_box(
                f"/world/{ramp['name']}",
                dimensions=(2.0 * sx, 2.0 * sy, 2.0 * sz),
                position=(px, py, pz),
                wxyz=(math.cos(ey / 2.0), 0.0, math.sin(ey / 2.0), 0.0),
                color=(210, 140, 60) if on_deck else (170, 110, 55),
            )
        )
    if world.kind == "bumps":
        h = world.bump_height_scale * params.wheel_radius
        for i, x in enumerate((0.22, 0.34, 0.48)):
            handles.append(
                server.scene.add_box(
                    f"/world/bump_{i}",
                    dimensions=(0.04, 0.24, h),
                    position=(x, 0.0, 0.5 * h),
                    color=(100, 90, 70),
                )
            )
    stones = list(world.stones or [])
    logs = list(world.logs or [])
    heat = list(world.heat or [])
    if world.kind == "forest" and not stones and not logs:
        from embr_local_sim.model import _default_forest

        stones, logs, heat = _default_forest()
    for i, stone in enumerate(stones):
        sx, sy, sz = (float(v) * 2.0 for v in stone.get("size", (0.04, 0.04, 0.03)))
        handles.append(
            server.scene.add_box(
                f"/world/stone_{i}",
                dimensions=(sx, sy, sz),
                position=(
                    float(stone.get("x", 0.3)),
                    float(stone.get("y", 0.0)),
                    float(stone.get("size", (0, 0, 0.03))[2]),
                ),
                color=(110, 100, 85),
            )
        )
    for i, log in enumerate(logs):
        radius = float(log.get("radius", 0.018))
        handles.append(
            server.scene.add_icosphere(
                f"/world/log_{i}",
                radius=radius,
                position=(float(log.get("x", 0.5)), float(log.get("y", 0.0)), radius),
                color=(100, 70, 40),
            )
        )
    for i, src in enumerate(heat):
        kind = str(src.get("kind", "smolder"))
        target = src.get("role") == "inspection_target"
        color = (255, 90, 20) if kind != "flame" else (255, 160, 40)
        handles.append(
            server.scene.add_icosphere(
                f"/world/heat_{i}",
                radius=0.18 if target else 0.04,
                position=(float(src.get("x", 0.8)), float(src.get("y", 0.0)), float(src.get("z", 0.03))),
                color=color,
                opacity=0.9 if target else 0.45,
            )
        )
    for i, shrub in enumerate(extras.get("shrubs") or []):
        handles.append(
            server.scene.add_icosphere(
                f"/world/shrub_{i}",
                radius=float(shrub.get("radius", 0.05)),
                position=(float(shrub["x"]), float(shrub["y"]), float(shrub.get("z", 0.07))),
                color=(70, 140, 60),
                opacity=0.92,
            )
        )
    for i, tree in enumerate(extras.get("trees") or []):
        h = float(tree.get("height", 0.4))
        handles.append(
            server.scene.add_cylinder(
                f"/world/tree_{i}",
                radius=float(tree.get("radius", 0.04)),
                height=h,
                position=(float(tree["x"]), float(tree["y"]), 0.5 * h),
                color=(82, 55, 30),
            )
        )
    for i, cap in enumerate(extras.get("canopy") or []):
        handles.append(
            server.scene.add_icosphere(
                f"/world/canopy_{i}",
                radius=float(cap.get("radius", 0.12)),
                position=(float(cap["x"]), float(cap["y"]), float(cap.get("z", 0.4))),
                color=(28, 110, 42),
                opacity=0.38,
            )
        )
    for i, mound in enumerate(extras.get("mounds") or []):
        sx, sy, sz = (float(v) * 2.0 for v in mound.get("size", (0.06, 0.05, 0.01)))
        handles.append(
            server.scene.add_box(
                f"/world/mound_{i}",
                dimensions=(sx, sy, sz),
                position=(float(mound["x"]), float(mound["y"]), float(mound["size"][2])),
                color=(70, 68, 40),
            )
        )
    for i, flame in enumerate(extras.get("flames") or []):
        h = float(flame.get("height", 0.14))
        node = server.scene.add_icosphere(
            f"/world/flame_{i}",
            radius=float(flame.get("radius", 0.04)),
            position=(float(flame["x"]), float(flame["y"]), 0.45 * h),
            color=(255, 110, 20),
            opacity=0.7,
        )
        node._embr_pulse = True  # type: ignore[attr-defined]
        handles.append(node)
        excl = float(flame.get("exclusion_r", 0.16))
        handles.append(
            server.scene.add_icosphere(
                f"/world/flame_excl_{i}",
                radius=excl,
                position=(float(flame["x"]), float(flame["y"]), 0.01),
                color=(220, 40, 20),
                opacity=0.12,
            )
        )
    for i, puff in enumerate(extras.get("smoke") or []):
        handles.append(
            server.scene.add_icosphere(
                f"/world/smoke_{i}",
                radius=float(puff.get("radius", 0.08)),
                position=(float(puff["x"]), float(puff["y"]), float(puff.get("z", 0.2))),
                color=(150, 150, 150),
                opacity=float(puff.get("opacity", 0.22)),
            )
        )
    for i, step in enumerate(extras.get("route") or []):
        action = str(step.get("action", "drive"))
        color = {"sample": (255, 150, 40), "return": (40, 120, 220)}.get(action, (230, 230, 210))
        handles.append(
            server.scene.add_icosphere(
                f"/world/route_{i}",
                radius=0.08,
                position=(float(step["x"]), float(step["y"]), 0.08),
                color=color,
                opacity=0.85,
            )
        )
    if extras.get("return_xy"):
        rx, ry = extras["return_xy"]
        handles.append(
            server.scene.add_icosphere(
                "/world/return_pad",
                radius=0.06,
                position=(float(rx), float(ry), 0.005),
                color=(40, 120, 220),
                opacity=0.35,
            )
        )
    step = extras.get("step") or {}
    if step:
        sx, sy, sz = (float(v) * 2.0 for v in step.get("size", (0.08, 0.10, 0.01)))
        handles.append(
            server.scene.add_box(
                "/world/step",
                dimensions=(sx, sy, sz),
                position=(float(step["x"]), float(step["y"]), float(step["size"][2])),
                color=(120, 118, 100),
            )
        )
    return handles


def _fidelity_line(params: RobotParams) -> str:
    from embr_local_sim.body import resolve_body

    return resolve_body(params.morphology).fidelity


def _add_robot_geometry(server: Any, params: RobotParams, sim: Simulator):
    if params.morphology == "cad":
        import trimesh

        from embr_local_sim.cad_model import cad_mesh_path, load_cad_geometry

        loaded = trimesh.load(cad_mesh_path(), force="mesh")
        chassis_handle = server.scene.add_mesh_simple(
            "/root/cad_visual",
            vertices=np.asarray(loaded.vertices, dtype=np.float32),
            faces=np.asarray(loaded.faces, dtype=np.uint32),
            color=(176, 182, 186),
            flat_shading=True,
            side="double",
        )
        wheel_handles = {}
        _visual_lift_m["m"] = 0.019
        _clear_spin_handles()
        for spec in load_cad_geometry()["rollers"]:
            body = spec["joint"].replace("_maxon", "")
            wheel_handles[body] = server.scene.add_cylinder(
                f"/root/{body}",
                radius=params.wheel_radius,
                height=params.wheel_width,
                color=(30, 30, 30),
            )
            _spin_handles[body] = server.scene.add_icosphere(
                f"/root/{body}_spin",
                radius=0.02,
                color=(255, 140, 30),
            )
        return chassis_handle, wheel_handles, {}
    _visual_lift_m["m"] = 0.0
    _clear_spin_handles()
    chassis_color = (120, 160, 130) if params.morphology != "wheels" else (158, 207, 237)
    chassis_handle = server.scene.add_box(
        "/root/chassis",
        dimensions=params.chassis_size,
        color=chassis_color,
    )
    wheel_handles: dict[str, Any] = {}
    extra_handles: dict[str, Any] = {}
    pod_spec = None
    if params.morphology == "tracks":
        from embr_local_sim.track_approx import N_ROLLERS, POD_CORNERS, POD_HEIGHT_M, POD_LENGTH_M, POD_WIDTH_M

        pod_spec = (POD_CORNERS, N_ROLLERS, POD_LENGTH_M, POD_WIDTH_M, POD_HEIGHT_M, (40, 42, 40))
    elif params.morphology == "articulated":
        from embr_local_sim.articulated import POD_CORNERS, POD_HEIGHT_M, POD_LENGTH_M, POD_WIDTH_M

        pod_spec = (POD_CORNERS, 3, POD_LENGTH_M, POD_WIDTH_M, POD_HEIGHT_M, (55, 70, 60))
    if pod_spec is not None:
        corners, n_roll, length, width, height, color = pod_spec
        for corner, _primary, _sx, _sy in corners:
            extra_handles[f"pod_{corner}"] = server.scene.add_box(
                f"/root/pod_{corner}",
                dimensions=(length, width, height),
                color=color,
            )
            for i in range(n_roll):
                name = f"roller_{corner}_{i}"
                col = (190, 80, 20) if corner in ("fl", "fr") else (140, 30, 30)
                wheel_handles[name] = server.scene.add_cylinder(
                    f"/root/{name}",
                    radius=params.wheel_radius,
                    height=params.wheel_width,
                    color=col,
                )
    else:
        wheel_handles = {
            "wheel_fl": server.scene.add_cylinder("/root/wheel_fl", radius=params.wheel_radius, height=params.wheel_width, color=(255, 140, 20)),
            "wheel_fr": server.scene.add_cylinder("/root/wheel_fr", radius=params.wheel_radius, height=params.wheel_width, color=(255, 140, 20)),
            "wheel_rl": server.scene.add_cylinder("/root/wheel_rl", radius=params.wheel_radius, height=params.wheel_width, color=(220, 40, 40)),
            "wheel_rr": server.scene.add_cylinder("/root/wheel_rr", radius=params.wheel_radius, height=params.wheel_width, color=(220, 40, 40)),
        }
    _ = sim
    return chassis_handle, wheel_handles, extra_handles


def _quat_rotate(quat: np.ndarray, vector: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(v) for v in quat)
    qv = np.array([x, y, z], dtype=np.float64)
    t = 2.0 * np.cross(qv, vector)
    return vector + w * t + np.cross(qv, t)


def _clear_spin_handles() -> None:
    for handle in _spin_handles.values():
        try:
            handle.remove()
        except Exception:
            try:
                handle.visible = False
            except Exception:
                pass
    _spin_handles.clear()


def _sync_bodies(sim: Simulator, chassis_handle, wheel_handles, extra_handles, gt: GroundTruthState) -> None:
    if chassis_handle is None:
        return
    root_q = np.array(gt.root_quat_wxyz, dtype=np.float64)
    lift = _quat_rotate(root_q, np.array([0.0, 0.0, _visual_lift_m["m"]], dtype=np.float64))
    chassis_handle.position = tuple(float(v) for v in (np.array(gt.root_pos, dtype=np.float64) + lift))
    chassis_handle.wxyz = gt.root_quat_wxyz
    for body, handle in wheel_handles.items():
        bid = int(sim.model.body(body).id)
        pos = np.array(sim.data.xpos[bid], dtype=np.float64)
        quat = np.array(sim.data.xquat[bid], dtype=np.float64)
        handle.position = tuple(float(v) for v in pos)
        handle.wxyz = _qmul(quat, _RX90)
        mark = _spin_handles.get(body)
        if mark is None:
            continue
        side = 0.12 if body.endswith("left") else -0.12
        # Body +Y is the axle. A point on +X orbits as the hinge turns.
        mark.position = tuple(float(v) for v in (pos + _quat_rotate(quat, np.array([0.09, side, 0.0]))))
    for body, handle in extra_handles.items():
        bid = int(sim.model.body(body).id)
        handle.position = tuple(float(v) for v in sim.data.xpos[bid])
        handle.wxyz = tuple(float(v) for v in sim.data.xquat[bid])


_lidar_handle = {"h": None}
_probe_handle = {"h": None}
_contact_handle = {"h": None}
_trace_handle = {"h": None}
# CAD mesh hangs below the roller contact. This lifts the drawing onto the wheels.
_visual_lift_m = {"m": 0.0}
_spin_handles: dict[str, Any] = {}


def _sync_lidar(server: Any, obs) -> None:
    if not obs.lidar_hit_x:
        return
    pts = np.stack(
        [np.array(obs.lidar_hit_x), np.array(obs.lidar_hit_y), np.array(obs.lidar_hit_z)],
        axis=1,
    )
    colors = np.tile(np.array([[80, 220, 255]], dtype=np.uint8), (len(pts), 1))
    if _lidar_handle["h"] is None:
        _lidar_handle["h"] = server.scene.add_point_cloud(
            "/sensors/lidar",
            points=pts,
            colors=colors,
            point_size=0.012,
        )
    else:
        _lidar_handle["h"].points = pts
        _lidar_handle["h"].colors = colors


def _sync_probe(server: Any, sim: Simulator) -> None:
    try:
        sid = int(sim.model.site("probe").id)
    except Exception:
        return
    pos = tuple(float(v) for v in sim.data.site_xpos[sid])
    if _probe_handle["h"] is None:
        _probe_handle["h"] = server.scene.add_icosphere(
            "/sensors/probe",
            radius=0.008,
            position=pos,
            color=(40, 220, 90),
        )
    else:
        _probe_handle["h"].position = pos


def _sync_heat_markers(server: Any, sim: Simulator) -> None:
    return


def _sync_flame_pulse(nodes: list[Any], sim_time: float) -> None:
    phase = 0.55 + 0.25 * math.sin(sim_time * 7.0)
    for node in nodes:
        if getattr(node, "_embr_pulse", False):
            try:
                node.opacity = float(phase)
            except Exception:
                pass


_balance_handle: dict[str, Any] = {"com": None, "forces": None}


def _sync_balance(server: Any, sim: Simulator) -> None:
    """Draw CoM, gravity direction, and contact normals from the MuJoCo state."""
    import mujoco

    root = int(sim.model.body("root").id)
    # Physics steps the same MjData on another thread. Copy under its lock.
    with sim._state_lock:
        com = np.array(sim.data.subtree_com[root], dtype=np.float64)
        n = min(int(sim.data.ncon), 16)
        samples: list[tuple[np.ndarray, np.ndarray]] = []
        for i in range(n):
            raw = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(sim.model, sim.data, i, raw)
            contact = sim.data.contact[i]
            origin = np.array(contact.pos, dtype=np.float64)
            normal = np.array(contact.frame[0:3], dtype=np.float64) * float(raw[0]) * 0.002
            samples.append((origin, origin + normal))
    if _balance_handle["com"] is None:
        _balance_handle["com"] = server.scene.add_icosphere(
            "/diag/com",
            radius=0.012,
            color=(220, 30, 30),
            position=(float(com[0]), float(com[1]), float(com[2])),
        )
    else:
        _balance_handle["com"].position = (float(com[0]), float(com[1]), float(com[2]))
    segments = [[com, com + np.array([0.0, 0.0, -0.08])]]
    colors = [[70, 90, 255]]
    for origin, tip in samples:
        segments.append([origin, tip])
        colors.append([255, 150, 30])
    points = np.asarray(segments, dtype=np.float64)
    cols = np.asarray(colors, dtype=np.uint8)
    previous = _balance_handle["forces"]
    if previous is not None and getattr(previous, "points", None) is not None:
        try:
            if np.shape(previous.points)[0] != len(points):
                previous.remove()
                previous = None
        except Exception:
            previous = None
    if previous is None:
        _balance_handle["forces"] = server.scene.add_arrows(
            "/diag/forces",
            points=points,
            colors=cols,
            shaft_radius=0.003,
            head_radius=0.007,
            head_length=0.012,
        )
    else:
        previous.points = points
        previous.colors = cols
        _balance_handle["forces"] = previous


def _sync_contacts(server: Any, gt: GroundTruthState) -> None:
    if not gt.contacts:
        return
    pts = np.array([c.pos for c in gt.contacts], dtype=np.float64)
    colors = np.tile(np.array([[255, 220, 40]], dtype=np.uint8), (len(pts), 1))
    if _contact_handle["h"] is None:
        _contact_handle["h"] = server.scene.add_point_cloud(
            "/diag/contacts",
            points=pts,
            colors=colors,
            point_size=0.015,
        )
    else:
        _contact_handle["h"].points = pts
        _contact_handle["h"].colors = colors


def _append_robot_trace(ctx: dict[str, Any], root_pos) -> None:
    """Blue trail is the chassis path, not wheel odometry.

    Track rollers slip in a turn, so encoder odometry yaws several times
    farther than the body and the old trail left the robot.
    """
    point = (float(root_pos[0]), float(root_pos[1]), 0.015)
    trace: list[tuple[float, float, float]] = ctx["odom_trace"]
    if trace:
        last = trace[-1]
        if math.hypot(point[0] - last[0], point[1] - last[1]) < 0.02:
            return
    trace.append(point)
    if len(trace) > 400:
        del trace[:-400]


def _clear_trace_visual() -> None:
    handle = _trace_handle["h"]
    _trace_handle["h"] = None
    if handle is None:
        return
    try:
        handle.remove()
    except Exception:
        try:
            handle.visible = False
        except Exception:
            pass


def _sync_trace(server: Any, trace: list[tuple[float, float, float]]) -> None:
    if len(trace) < 2:
        return
    pts = np.array(trace, dtype=np.float64)
    colors = np.tile(np.array([[80, 180, 255]], dtype=np.uint8), (len(pts), 1))
    if _trace_handle["h"] is None:
        _trace_handle["h"] = server.scene.add_point_cloud(
            "/diag/odom_trace",
            points=pts,
            colors=colors,
            point_size=0.008,
        )
    else:
        _trace_handle["h"].points = pts
        _trace_handle["h"].colors = colors


def _sync_inspection(
    sim: Simulator,
    gt: GroundTruthState,
    status: Any,
    start_marker: Any,
    target_marker: Any,
    route_handle: dict[str, Any],
    server: Any,
    *,
    show_route: bool,
    show_eval: bool,
    target_xy: tuple[float, float],
) -> None:
    import json
    from pathlib import Path

    from embr_local_sim.inspection import elapsed_sim

    insp = sim.inspection
    elapsed = elapsed_sim(insp, gt.sim_time)
    elapsed_txt = "—" if elapsed is None else f"{elapsed:.1f} s"
    dist_t = "—" if insp.dist_to_target_m is None else f"{insp.dist_to_target_m:.2f} m"
    dist_s = "—" if insp.dist_to_start_m is None else f"{insp.dist_to_start_m:.2f} m"
    lines = [
        f"**{insp.state}** — {insp.help_text()}",
        f"Action: `{insp.reason}`",
        f"Estimated distance to target `{dist_t}`, to start `{dist_s}`",
        f"Ranges: {insp.sensor_note}. Probe `{sim.probe.state}`.",
        f"Mission time `{elapsed_txt}` (simulation clock).",
    ]
    if insp.measurement_c is not None:
        lines.append(
            f"Probe reading `{insp.measurement_c:.0f} °C`, labeled `{insp.measurement_class}` (provisional)."
        )
    if show_eval:
        err_t = math.hypot(gt.root_pos[0] - target_xy[0], gt.root_pos[1] - target_xy[1])
        sx, sy, _syaw = sim.spawn_pose
        err_s = math.hypot(gt.root_pos[0] - sx, gt.root_pos[1] - sy)
        lines.append(
            f"Evaluation only: ground-truth distance to the world target `{err_t:.2f} m`, "
            f"to the spawn point `{err_s:.2f} m`."
        )
    status.content = "\n\n".join(lines)
    try:
        sx, sy, _syaw = sim.spawn_pose
        start_marker.position = (sx, sy, 0.03)
        target_marker.position = (target_xy[0], target_xy[1], 0.03)
    except Exception:
        pass
    try:
        if show_route:
            positions = (
                (float(gt.root_pos[0]), float(gt.root_pos[1]), 0.04),
                (float(target_xy[0]), float(target_xy[1]), 0.04),
            )
            if route_handle["h"] is None:
                route_handle["h"] = server.scene.add_spline_catmull_rom(
                    "/inspection/bearing",
                    positions=positions,
                    color=(230, 160, 40),
                    line_width=2.0,
                )
            else:
                route_handle["h"].positions = positions
                route_handle["h"].visible = True
        elif route_handle["h"] is not None:
            route_handle["h"].visible = False
    except Exception:
        route_handle["h"] = False
    if insp.state in ("COMPLETE", "BLOCKED", "FAULT", "CANCELLED"):
        token = id(insp)
        if getattr(_sync_inspection, "saved", None) != token:
            _sync_inspection.saved = token  # type: ignore[attr-defined]
            out = Path("embr-sim-runs") / "inspection" / "manual-last.json"
            try:
                out.parent.mkdir(parents=True, exist_ok=True)
                payload = {
                    "controller_state": insp.state,
                    "controller_reason": insp.reason,
                    "help": insp.help_text(),
                    "estimated_dist_to_target_m": insp.dist_to_target_m,
                    "estimated_dist_to_start_m": insp.dist_to_start_m,
                    "measurement_c": insp.measurement_c,
                    "measurement_class": insp.measurement_class,
                    "transitions": list(insp.transitions),
                    "evaluation_gt": {
                        "x": gt.root_pos[0],
                        "y": gt.root_pos[1],
                        "yaw": gt.yaw,
                    },
                    "trace_samples": len(sim.inspection_trace),
                }
                out.write_text(json.dumps(payload, indent=2))
            except OSError:
                pass


def _telemetry_markdown(
    gt: GroundTruthState, obs, n_clients: int, sim: Simulator | None = None, show_gt: bool = False
) -> str:
    contacts = ", ".join(sorted({*sum(([c.geom1, c.geom2] for c in gt.contacts), [])})) or "none"
    wd = "TRIPPED" if gt.watchdog_active else "ok"
    lidar_min = min(obs.lidar_ranges) if obs.lidar_ranges else float("nan")
    thermal_max = max(obs.thermal_c) if obs.thermal_c else obs.ambient_c
    lumps = ", ".join(f"{k}={v:.1f}°C/{obs.lump_rating_c.get(k, 'UNKNOWN')}" for k, v in obs.lump_c.items())
    mission = sim.mission.snapshot() if sim is not None else {"state": "?", "reason": ""}
    spots = mission.get("hotspots") or []
    art_txt = "n/a"
    if gt.art_angles:
        art_txt = " ".join(f"{n}={v:.2f}" for n, v in gt.art_angles.items())
    spot_txt = "; ".join(
        f"odom=({h['estimated_x']:.2f},{h['estimated_y']:.2f}) {h['measured_c']:.0f}°C {h['provenance']}"
        for h in spots
    ) or "none"
    core = (
        f"**{mission.get('state')}** — {mission.get('reason') or 'idle'} · "
        f"t `{gt.sim_time:.2f}` s · roll `{gt.roll:.2f}` pitch `{gt.pitch:.2f}` rad · "
        f"vx `{gt.actual_vx_body:.2f}` m/s · cmd `{gt.target_vx:.2f}` turn `{gt.target_wz:.2f}` · "
        f"{'PAUSED' if gt.paused else wd} · contacts {gt.ncon}"
    )
    if not show_gt:
        return core
    return f"""{core}

| | |
|---|---|
| watchdog | `{wd}` age `{gt.watchdog_age_wall:.2f}` s |
| TeleCmd | v=`{sim._last_tele.velocity if sim else 0:.2f}` turn=`{sim._last_tele.turn if sim else 0:.2f}` |
| GT pose (debug) | `{gt.root_pos[0]:.3f}` `{gt.root_pos[1]:.3f}` yaw `{gt.yaw:.3f}` |
| odom | `{obs.wheel_odom_x:.3f}` `{obs.wheel_odom_y:.3f}` |
| lidar min | `{lidar_min:.3f}` m |
| thermal | `{thermal_max:.1f}` °C / probe `{obs.probe_c:.1f}` |
| lumps | {lumps or "n/a"} |
| modules | {art_txt} |
| mission | `{mission.get("state")}` oracle `{mission.get("oracle", False)}` |
| probe | `{obs.probe_state}` placeholder hinge `{obs.probe_angle:.2f}` rad |
| hotspots | {spot_txt} |
| contact geoms | {contacts} |
| source | `{gt.command_source}` clients `{n_clients}` |
"""
