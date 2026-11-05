"""Fixed-chassis MuJoCo drivetrain twin driven by measured encoder feedback."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import yaml

from embr_local_sim.model import RobotParams
from embr_local_sim.motor import MotorSample
from embr_local_sim.paths import config_dir

# Names and wheel geometry come from embr_simple. The CAN slots follow dt_can.
JOINT_NAMES = ("front_left_maxon", "rear_left_maxon", "front_right_maxon", "rear_right_maxon")
SLOT_NAMES = ("Front left (orange)", "Rear left (cyan)", "Front right (green)", "Rear right (pink)")


@dataclass(frozen=True)
class MotorChannel:
    joint: str
    node_id: int
    direction: int


@dataclass(frozen=True)
class DrivetrainConfig:
    channels: tuple[MotorChannel, ...]
    gear_ratio: float
    stale_after_s: float


def load_drivetrain_config(path: str | Path | None = None) -> DrivetrainConfig:
    file = Path(path) if path is not None else config_dir() / "maxon_drivetrain.yaml"
    try:
        raw = yaml.safe_load(file.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(f"Invalid drivetrain YAML in {file}: {exc}") from exc
    if not isinstance(raw, dict) or set(raw) != {"node_ids", "directions", "gear_ratio", "stale_after_s"}:
        raise ValueError("Expected node_ids, directions, gear_ratio and stale_after_s in the drivetrain config")
    ids, directions = raw["node_ids"], raw["directions"]
    if (not isinstance(ids, list) or len(ids) != 4
            or any(type(value) is not int or not 1 <= value <= 127 for value in ids)
            or len(set(ids)) != 4):
        raise ValueError("node_ids must contain four unique CANopen IDs in [1, 127]")
    if (not isinstance(directions, list) or len(directions) != 4
            or any(type(value) is not int or value not in (-1, 1) for value in directions)):
        raise ValueError("directions must contain four values of +1 or -1")
    ratio = number(raw["gear_ratio"], "gear_ratio")
    stale = number(raw["stale_after_s"], "stale_after_s")
    if ratio < 1 or stale <= 0:
        raise ValueError("gear_ratio must be at least 1 and stale_after_s must be positive")
    return DrivetrainConfig(tuple(MotorChannel(*args) for args in zip(JOINT_NAMES, ids, directions)),
                            ratio, stale)


def number(value, field):
    if type(value) not in (int, float) or not -1e15 <= value <= 1e15:
        raise ValueError(f"{field} must be a finite number in the supported range")
    return float(value)


def integer(value, field, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{field} must be an integer in [{minimum}, {maximum}]")
    return value


@dataclass(frozen=True)
class Feedback:
    session: str
    seq: int
    node_id: int
    sample_s: float
    poll_s: float
    reference: int | None = None
    counts: int | None = None
    relative_counts: int | None = None
    counts_per_rev: int | None = None
    rpm: int | None = None
    error: str = ""

    @classmethod
    def decode(cls, data: bytes) -> Feedback:
        """Decode a version 1 ESCON2 datagram without assuming a common clock."""
        if len(data) > 4096:
            raise ValueError("Oversized motor feedback datagram")
        try:
            raw = json.loads(data)
        except (UnicodeError, RecursionError) as exc:
            raise ValueError("Invalid motor feedback JSON") from exc
        if not isinstance(raw, dict) or type(raw.get("version")) is not int or raw["version"] != 1:
            raise ValueError("Unsupported motor feedback version")
        session = raw.get("session")
        if not isinstance(session, str) or len(session) != 32 or any(c not in "0123456789abcdef" for c in session):
            raise ValueError("Invalid motor feedback session")
        seq = integer(raw.get("seq"), "seq", 0, (1 << 63) - 1)
        node = integer(raw.get("node_id"), "node_id", 1, 127)
        sample_s = number(raw.get("sample_s"), "sample_s")
        poll_s = number(raw.get("poll_s"), "poll_s")
        if not 0 <= poll_s <= sample_s:
            raise ValueError("Invalid feedback timing")
        if "error" in raw:
            error = raw["error"]
            if not isinstance(error, str) or not error or len(error) > 256:
                raise ValueError("Invalid feedback error")
            return cls(session, seq, node, sample_s, poll_s, error=error)
        reference = integer(raw.get("reference"), "reference", 1, (1 << 63) - 1)
        counts = integer(raw.get("counts"), "counts", -(1 << 31), (1 << 31) - 1)
        relative = integer(raw.get("relative_counts"), "relative_counts", -(1 << 63), (1 << 63) - 1)
        scale = integer(raw.get("counts_per_rev"), "counts_per_rev", 64, 10_000_000)
        if scale % 4:
            raise ValueError("Expected quadrature counts per revolution")
        rpm = integer(raw.get("rpm"), "rpm", -(1 << 31), (1 << 31) - 1)
        return cls(session, seq, node, sample_s, poll_s, reference, counts, relative, scale, rpm)

    def shaft_sample(self) -> MotorSample:
        if self.error:
            raise ValueError(self.error)
        return MotorSample(self.sample_s, self.relative_counts * math.tau / self.counts_per_rev,
                           self.rpm * math.tau / 60)


def drivetrain_mjcf() -> str:
    """Use the URDF-derived EMBR wheel layout with visible, independent output joints."""
    params = RobotParams()
    root = ET.Element("mujoco", model="embr_simple_drivetrain_feedback")
    ET.SubElement(root, "compiler", angle="radian")
    ET.SubElement(root, "option", gravity="0 0 0")
    world = ET.SubElement(root, "worldbody")
    chassis = ET.SubElement(world, "body", name="chassis", pos="0 0 .035")

    def geom(body, name, kind, size, pos, rgba, **attrs):
        ET.SubElement(body, "geom", name=name, type=kind, size=size, pos=pos, rgba=rgba,
                      contype="0", conaffinity="0", **attrs)

    cylinder = "0.7071067811865476 0.7071067811865476 0 0"
    geom(chassis, "chassis_plate", "box", ".108 .034 .009", "0 0 .017", ".30 .35 .43 1")
    geom(chassis, "front_marker", "box", ".006 .022 .002", ".095 0 .027", ".9 .9 .9 1")
    colors = (".98 .51 .12 1", ".12 .75 .95 1", ".42 .90 .32 1", ".91 .29 .67 1")
    for i, (name, color) in enumerate(zip(JOINT_NAMES, colors)):
        side = 1 if i < 2 else -1
        x = params.wheelbase_half() * (1 if i % 2 == 0 else -1)
        y = params.track_half() * side
        motor = ET.SubElement(chassis, "body", name=f"{name}_housing", pos=f"{x} {y} 0")
        geom(motor, f"{name}_case", "cylinder", ".018 .014", f"0 {-.021 * side} 0",
             ".46 .51 .59 1", quat=cylinder)
        wheel = ET.SubElement(motor, "body", name=f"{name}_output")
        ET.SubElement(wheel, "joint", name=name, type="hinge", axis="0 1 0", limited="false")
        half_width = params.wheel_width / 2
        geom(wheel, f"{name}_wheel", "cylinder", f"{params.wheel_radius} {half_width}", "0 0 0",
             ".13 .15 .18 1", quat=cylinder, mass=".025")
        geom(wheel, f"{name}_pointer", "box", ".010 .001 .002",
             f".012 {(half_width + .001) * side} 0", color, density="0")
        geom(motor, f"{name}_zero", "box", ".003 .002 .002",
             f".034 {(half_width + .001) * side} 0", color)
    return ET.tostring(root, encoding="unicode")


@dataclass
class ChannelState:
    last: Feedback | None = None
    received_s: float = 0.0
    status: str = "Waiting"
    rate_hz: float | None = None


class DrivetrainTwin:
    def __init__(self, config: DrivetrainConfig):
        self.config = config
        self.channels = {channel.node_id: channel for channel in config.channels}
        self.model = mujoco.MjModel.from_xml_string(drivetrain_mjcf())
        self.data = mujoco.MjData(self.model)
        self.reset()

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.states = {node: ChannelState() for node in self.channels}
        self.session: str | None = None
        self.retired_sessions: set[str] = set()
        self.last_received_s = 0.0
        mujoco.mj_forward(self.model, self.data)

    def accept(self, feedback: Feedback, now: float) -> bool:
        """Apply one fresh channel observation. Return False for ignored packets."""
        if feedback.node_id not in self.channels:
            return False
        if feedback.session != self.session:
            if feedback.session in self.retired_sessions:
                return False
            if self.session is not None and now - self.last_received_s < self.config.stale_after_s:
                return False
            self.expire(now)
            if self.session is not None:
                self.retired_sessions.add(self.session)
            self.session = feedback.session
            for state in self.states.values():
                state.last = None
        channel = self.channels[feedback.node_id]
        state = self.states[feedback.node_id]
        previous = state.last
        if previous and (feedback.seq <= previous.seq or feedback.sample_s <= previous.sample_s):
            return False
        joint = self.data.joint(channel.joint)
        if feedback.error:
            joint.qvel[0] = 0
            state.status = "Unavailable"
            state.rate_hz = None
        else:
            sample = feedback.shaft_sample()
            scale = channel.direction / self.config.gear_ratio
            joint.qpos[0] = sample.position_rad * scale
            joint.qvel[0] = sample.velocity_rad_s * scale
            state.rate_hz = None
            if (state.status == "Live" and previous and feedback.seq == previous.seq + 1
                    and previous.reference == feedback.reference):
                state.rate_hz = 1 / (feedback.sample_s - previous.sample_s)
            state.status = "Live"
        state.last, state.received_s = feedback, now
        self.last_received_s = now
        mujoco.mj_forward(self.model, self.data)
        return True

    def expire(self, now: float) -> None:
        """Keep the last angle and clear displayed velocity when a channel falls silent."""
        changed = False
        for node, state in self.states.items():
            if state.status == "Live" and now - state.received_s >= self.config.stale_after_s:
                state.status, state.rate_hz = "Stale", None
                self.data.joint(self.channels[node].joint).qvel[0] = 0
                changed = True
        if changed:
            mujoco.mj_forward(self.model, self.data)
