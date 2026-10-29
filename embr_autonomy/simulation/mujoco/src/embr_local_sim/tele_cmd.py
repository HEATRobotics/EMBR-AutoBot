"""Normalized TeleCmd contract and the drivetrain-helper conversion.

Verified against EMBR-AutoBot `1054c0efe76ac8ff82505f274aa4a44885a74a81`:
`embr_interfaces/msg/TeleCmd.msg` and `embr/node_maxon_helper.py`.

`velocity` and `turn` are normalized stick levels in [-1, 1], not m/s.
Positive `turn` is **right**. ROS yaw / this sandbox `wz` is positive left,
so the helper negates turn when producing Twist.angular.z.

Timeout layers (do not collapse these into one number):

| Layer | Default | Purpose |
|---|---|---|
| iBUS frame timeout | 0.25 s | Receiver lost a radio frame (not implemented here). |
| Helper command timeout | 0.50 s | ROS helper zeros latched TeleCmd. |
| Simulator actuator watchdog | 0.40 s | Physics zeros wheel-speed targets. |
| GUI hold-to-drive | release | Human deadman; terminal TeleCmd publishers keep repeating. |

Helper `max_linear_speed` / `max_angular_speed` source-code defaults are 1.0.
This proxy converts with `RobotParams.max_vx` / `max_wz` so the tiny chassis
is not commanded at the helper's mock 1 m/s. Those helper numbers are **not**
measured rover ratings.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from embr_local_sim.model import RobotParams
from embr_local_sim.types import DriveCommand

# Pinned AutoBot helper / iBUS defaults (source code, not hardware ratings).
REPO_HELPER_MAX_LINEAR_SPEED = 1.0
REPO_HELPER_MAX_ANGULAR_SPEED = 1.0
REPO_HELPER_COMMAND_TIMEOUT_S = 0.5
REPO_HELPER_PUBLISH_PERIOD_S = 0.05
REPO_IBUS_FRAME_TIMEOUT_S = 0.25

# Hardware Float32MultiArray order from node_maxon_helper.py.
MOTOR_ORDER_FL_RL_FR_RR: tuple[str, ...] = (
    "front_left_maxon",
    "rear_left_maxon",
    "front_right_maxon",
    "rear_right_maxon",
)

OPERATOR_SOURCES = frozenset(
    {
        "gui-hold",
        "gui-stop",
        "keyboard",
        "keyboard-stop",
        "viewer-disconnect",
        "operator",
        "tele-cmd-operator",
    }
)


def clamp_unit(value: float) -> float:
    if not math.isfinite(value):
        raise ValueError(f"non-finite TeleCmd component {value}")
    return max(-1.0, min(1.0, value))


def mix_four_motor_levels(forward: float, turn: float) -> list[float]:
    """Copy of AutoBot `embr_hardware.ibus.mix_four_motor_levels`.

    Order: front-left, rear-left, front-right, rear-right.
    The helper calls this as `mix_four_motor_levels(forward, -turn)`.
    """
    left = forward - turn
    right = forward + turn
    scale = max(1.0, abs(left), abs(right))
    return [left / scale, left / scale, right / scale, right / scale]


@dataclass(frozen=True)
class TeleCmd:
    """Relative `tele_cmd` payload. No timestamp or frame in the .msg; we add wall time locally."""

    velocity: float
    turn: float
    timestamp_wall: float
    source: str


@dataclass(frozen=True)
class TwistCommand:
    vx: float
    wz: float
    timestamp_wall: float
    source: str
    helper_timeout_s: float = REPO_HELPER_COMMAND_TIMEOUT_S


def tele_cmd_to_twist(
    cmd: TeleCmd,
    params: RobotParams,
    *,
    max_linear_speed: float | None = None,
    max_angular_speed: float | None = None,
) -> TwistCommand:
    """Simulation route of `node_maxon_helper._publish_command` (Twist branch)."""
    try:
        forward = clamp_unit(cmd.velocity)
        turn = clamp_unit(cmd.turn)
    except ValueError:
        return TwistCommand(0.0, 0.0, cmd.timestamp_wall, source="invalid")
    lin = params.max_vx if max_linear_speed is None else float(max_linear_speed)
    ang = params.max_wz if max_angular_speed is None else float(max_angular_speed)
    vx = forward * lin
    wz = -turn * ang
    return TwistCommand(vx=vx, wz=wz, timestamp_wall=cmd.timestamp_wall, source=cmd.source)


def twist_to_drive(twist: TwistCommand) -> DriveCommand:
    return DriveCommand(
        vx=twist.vx,
        wz=twist.wz,
        timestamp_wall=twist.timestamp_wall,
        source=twist.source,
    )


def hardware_motor_levels(cmd: TeleCmd) -> list[float]:
    """Hardware route: `mix_four_motor_levels(forward, -turn)` after clamp."""
    try:
        forward = clamp_unit(cmd.velocity)
        turn = clamp_unit(cmd.turn)
    except ValueError:
        return [0.0, 0.0, 0.0, 0.0]
    return mix_four_motor_levels(forward, -turn)


class DrivetrainHelperSim:
    """Latched TeleCmd → Twist with the helper's 0.5 s timeout.

    Does not replace the simulator's 0.40 s actuator watchdog. GUI hold-to-drive
    should send TeleCmd only while held; this latch models the ROS helper timer
    that keeps publishing until *its* timeout.
    """

    def __init__(self, params: RobotParams, timeout_s: float = REPO_HELPER_COMMAND_TIMEOUT_S) -> None:
        self.params = params
        self.timeout_s = timeout_s
        self._last: TeleCmd | None = None
        self._last_wall = 0.0

    def reset(self) -> None:
        self._last = None
        self._last_wall = 0.0

    def ingest(self, cmd: TeleCmd) -> TwistCommand:
        if not math.isfinite(cmd.velocity) or not math.isfinite(cmd.turn):
            zero = TeleCmd(0.0, 0.0, cmd.timestamp_wall, source="invalid")
            self._last = zero
            self._last_wall = cmd.timestamp_wall
            return tele_cmd_to_twist(zero, self.params)
        clamped = TeleCmd(
            velocity=max(-1.0, min(1.0, cmd.velocity)),
            turn=max(-1.0, min(1.0, cmd.turn)),
            timestamp_wall=cmd.timestamp_wall,
            source=cmd.source,
        )
        self._last = clamped
        self._last_wall = cmd.timestamp_wall
        return tele_cmd_to_twist(clamped, self.params)

    def publish(self, wall_now: float) -> TwistCommand:
        if self._last is None or (wall_now - self._last_wall) > self.timeout_s:
            return TwistCommand(0.0, 0.0, wall_now, source="helper-timeout")
        return tele_cmd_to_twist(self._last, self.params)


class CommandGate:
    """Single arbitration owner: operator TeleCmd always beats mission."""

    def __init__(self) -> None:
        self.mode: str = "manual"  # manual | mission
        self.takeover: bool = False
        self.last_operator_wall: float = 0.0

    def reset(self) -> None:
        self.mode = "manual"
        self.takeover = False
        self.last_operator_wall = 0.0

    def start_mission(self) -> None:
        self.mode = "mission"
        self.takeover = False

    def on_tele(self, cmd: TeleCmd) -> str:
        """Return who owns the command after this ingest (`manual` or `mission`)."""
        if cmd.source in OPERATOR_SOURCES:
            if self.mode == "mission":
                self.takeover = True
            self.mode = "manual"
            self.last_operator_wall = cmd.timestamp_wall
            return "manual"
        if self.mode == "mission" and cmd.source.startswith("mission"):
            return "mission"
        if cmd.source.startswith("mission") and self.mode != "mission":
            return "rejected"
        return self.mode
