"""Synthetic replay, UDP reception and evidence for the four-motor twin."""

from __future__ import annotations

import csv
import json
import math
import queue
import socket
import time
from contextlib import ExitStack
from dataclasses import asdict, dataclass
from pathlib import Path

from embr_local_sim.drivetrain import DrivetrainTwin, Feedback, SLOT_NAMES
from embr_local_sim.motor_demo import DEMO_DURATION_S, DEMO_HZ, demo_sample

DEMO_TICKS = DEMO_DURATION_S * DEMO_HZ
SYNTHETIC_COUNTS_PER_REV = 4096


def synthetic_feedback(twin: DrivetrainTwin, tick: int, session: str):
    """Four independent profiles through the same signed encoder/RPM input as live."""
    _, sample = demo_sample(tick / DEMO_HZ)
    for channel, factor in zip(twin.config.channels, (4, -6, 8, -12)):
        counts = round(sample.position_rad * factor * SYNTHETIC_COUNTS_PER_REV / math.tau)
        rpm = round(sample.velocity_rad_s * factor * 60 / math.tau)
        yield Feedback(session, tick, channel.node_id, sample.timestamp_s, 0.0, 1, counts,
                       counts, SYNTHETIC_COUNTS_PER_REV, rpm)


@dataclass
class MotorEvidence:
    samples: int = 0
    consecutive: int = 0
    longest_consecutive: int = 0
    last: Feedback | None = None
    interval_s: float = 0.0
    intervals: int = 0
    max_sdo_poll_ms: float = 0.0
    max_loopback_apply_ms: float | None = None
    errors: int = 0

    def record(self, packet: Feedback, loopback_apply_ms: float | None):
        if packet.error:
            self.errors += 1
            self.consecutive = 0
        else:
            self.samples += 1
            previous = self.last
            if (self.consecutive and previous.session == packet.session
                    and previous.reference == packet.reference and previous.seq + 1 == packet.seq):
                self.consecutive += 1
                self.interval_s += packet.sample_s - previous.sample_s
                self.intervals += 1
            else:
                self.consecutive = 1
            self.longest_consecutive = max(self.longest_consecutive, self.consecutive)
            self.max_sdo_poll_ms = max(self.max_sdo_poll_ms, 1000 * packet.poll_s)
            if loopback_apply_ms is not None:
                self.max_loopback_apply_ms = max(self.max_loopback_apply_ms or 0, loopback_apply_ms)
        self.last = packet


class Evidence:
    """Stream accepted readings and applied joint values to a paired CSV."""

    def __init__(self, twin, stack, out, live):
        self.twin = twin
        self.source = "escon2-sdo" if live else "synthetic"
        self.motors = {node: MotorEvidence() for node in twin.channels}
        self.ignored = 0
        self.out = Path(out) if out else None
        self.writer = None
        if self.out is not None:
            self.out.mkdir(parents=True, exist_ok=True)
            file = stack.enter_context((self.out / "drivetrain_samples.csv").open(
                "w", newline="", encoding="utf-8", buffering=1))
            self.writer = csv.DictWriter(file, fieldnames=[
                *Feedback.__dataclass_fields__, "joint", "received_s", "loopback_apply_ms",
                "qpos_rad", "qvel_rad_s",
            ])
            self.writer.writeheader()

    def accept(self, packet, now, *, loopback=False):
        if not self.twin.accept(packet, now):
            self.ignored += 1
            return
        lag_ms = None
        if loopback and not packet.error and packet.sample_s <= now:
            lag_ms = 1000 * (time.monotonic() - packet.sample_s)
        self.motors[packet.node_id].record(packet, lag_ms)
        if self.writer is not None:
            channel = self.twin.channels[packet.node_id]
            joint = self.twin.data.joint(channel.joint)
            self.writer.writerow({
                **asdict(packet), "joint": channel.joint, "received_s": now,
                "loopback_apply_ms": lag_ms, "qpos_rad": float(joint.qpos[0]),
                "qvel_rad_s": float(joint.qvel[0]),
            })

    def summary(self):
        motors = {}
        for node, evidence in self.motors.items():
            channel = self.twin.channels[node]
            joint = self.twin.data.joint(channel.joint)
            motors[channel.joint] = {
                "node_id": node, "direction": channel.direction, "samples": evidence.samples,
                "longest_consecutive": evidence.longest_consecutive, "errors": evidence.errors,
                "status": self.twin.states[node].status,
                "mean_update_hz": evidence.intervals / evidence.interval_s if evidence.interval_s else None,
                "max_sdo_poll_ms": (evidence.max_sdo_poll_ms
                                    if evidence.samples and self.source == "escon2-sdo" else None),
                "max_loopback_apply_ms": evidence.max_loopback_apply_ms,
                "final_qpos_rad": float(joint.qpos[0]), "final_qvel_rad_s": float(joint.qvel[0]),
            }
        summary = {
            "source": self.source, "gear_ratio": self.twin.config.gear_ratio,
            "motors": motors, "ignored_datagrams": self.ignored,
            "four_motor_sample_window": all(motor.longest_consecutive >= 10
                                            for motor in self.motors.values()),
        }
        if self.out is not None:
            (self.out / "drivetrain_summary.json").write_text(
                json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        return summary


def receive(receiver, evidence):
    """Drain a bounded batch without blocking expiry or the browser."""
    for _ in range(128):
        try:
            data, peer = receiver.recvfrom(65535)
        except BlockingIOError:
            break
        now = time.monotonic()
        try:
            packet = Feedback.decode(data)
        except ValueError:
            evidence.ignored += 1
            continue
        evidence.accept(packet, now, loopback=peer[0].startswith("127."))
    evidence.twin.expire(time.monotonic())


def run_drivetrain(twin, *, live, headless, out, duration, feedback_bind, feedback_port, host, port):
    with ExitStack() as stack:
        receiver = None
        if live:
            receiver = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            receiver.bind((feedback_bind, feedback_port))
            receiver.setblocking(False)
        evidence = Evidence(twin, stack, out, live)
        if headless and live:
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                receive(receiver, evidence)
                time.sleep(.01)
        elif headless:
            for tick in range(DEMO_TICKS + 1):
                for packet in synthetic_feedback(twin, tick, "0" * 32):
                    evidence.accept(packet, tick / DEMO_HZ)
        else:
            run_viewer(evidence, receiver, host, port)
        return evidence.summary()


def run_viewer(evidence, receiver, host, port):
    import viser

    from embr_local_sim.motor_viewer import add_motor_geometry, sync_motor_geometry
    from embr_local_sim.viewer import _require_free_port

    _require_free_port(host, port)
    twin = evidence.twin
    server = viser.ViserServer(host=host, port=port, label="EMBR Four-Motor Twin")
    try:
        server.gui.configure_theme(control_width="medium", show_logo=False, show_share_button=False)
        server.scene.set_up_direction("+z")
        server.scene.add_grid("/grid", width=.6, height=.6, plane="xy", cell_size=.05, section_size=.1)
        server.initial_camera.position = (.33, -.42, .31)
        server.initial_camera.look_at = (0, 0, .035)
        server.initial_camera.near = .001
        handles = add_motor_geometry(server, twin)
        mode = "Live ESCON2 feedback" if receiver else "Synthetic replay"
        angle_label = "the motor shaft" if twin.config.gear_ratio == 1 else "the gearbox output"
        server.gui.add_markdown(
            f"## {mode}\n\n**Gearbox:** {twin.config.gear_ratio:g}:1 · "
            f"Angles and speeds are shown at {angle_label}. "
            "The white stripe marks the front of the chassis."
        )
        telemetry = [server.gui.add_markdown("") for _ in twin.config.channels]
        commands: queue.SimpleQueue[None] = queue.SimpleQueue()
        if receiver is None:
            replay = server.gui.add_button("Run / replay 12-second demo")
            replay.on_click(lambda _event: commands.put(None))
        tick = DEMO_TICKS + 1
        session = 0
        next_render = 0.0
        print(f"Four-motor twin: http://{host}:{port} — {mode}; Ctrl-C to exit.", flush=True)
        while True:
            now = time.monotonic()
            if receiver:
                receive(receiver, evidence)
            elif not commands.empty():
                while not commands.empty():
                    commands.get()
                session += 1
                twin.reset()
                tick = 0
            if receiver is None and tick <= DEMO_TICKS:
                for packet in synthetic_feedback(twin, tick, f"{session:032x}"):
                    evidence.accept(packet, now)
                tick += 1
            if now >= next_render:
                with server.atomic():
                    sync_motor_geometry(twin, handles)
                    for label, channel, panel in zip(SLOT_NAMES, twin.config.channels, telemetry):
                        state = twin.states[channel.node_id]
                        joint = twin.data.joint(channel.joint)
                        speed = (f"{joint.qvel[0] * 60 / math.tau:+.1f} rpm"
                                 if state.status == "Live" else "—")
                        detail = f" · {state.rate_hz:.1f} Hz" if state.rate_hz else ""
                        if state.last and state.last.reference is not None:
                            detail += f" · ref {state.last.reference}"
                        if state.last and state.last.error:
                            detail += "\n\n`" + state.last.error.replace("`", "'") + "`"
                        status = state.status
                        if receiver is None and status == "Live":
                            status = "Replaying" if tick <= DEMO_TICKS else "Finished"
                        panel.content = (f"**{label} · node {channel.node_id} · {status}**{detail}\n\n"
                                         f"{math.degrees(joint.qpos[0]):+.1f}° · {speed}")
                next_render = now + .04
            time.sleep(max(0.0, now + 1 / DEMO_HZ - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
