"""Single owner of approved drive + articulation commands.

Modes: manual, assisted, paused, fault. Zero torque is not used as a hold
for pitch hinges — last finite position targets are kept so the body does
not collapse.
"""

from __future__ import annotations

import math

from embr_local_sim.articulated import ART_JOINTS, ART_RANGE_RAD
from embr_local_sim.observation import ControllerOutput, RobotObservation, SupervisorResult

MODES = ("manual", "assisted", "paused", "fault")


class CommandSupervisor:
    def __init__(self) -> None:
        self.mode = "assisted"
        self.reason = "init"
        self._hold_art = {name: 0.0 for name in ART_JOINTS}
        self._hold_drive = (0.0, 0.0)

    def reset(self) -> None:
        self.mode = "assisted"
        self.reason = "reset"
        self._hold_art = {name: 0.0 for name in ART_JOINTS}
        self._hold_drive = (0.0, 0.0)

    def set_mode(self, mode: str, reason: str = "") -> None:
        if mode in MODES:
            self.mode = mode
            self.reason = reason or mode

    def approve(self, output: ControllerOutput, obs: RobotObservation) -> SupervisorResult:
        art = {name: float(output.art_targets.get(name, self._hold_art.get(name, 0.0))) for name in ART_JOINTS}
        vx, wz = float(output.drive_vx), float(output.drive_wz)

        if not math.isfinite(vx) or not math.isfinite(wz) or not all(math.isfinite(v) for v in art.values()):
            self.mode = "fault"
            self.reason = "non_finite"
            return SupervisorResult(
                mode=self.mode,
                approved_drive_vx=0.0,
                approved_drive_wz=0.0,
                approved_art=dict(self._hold_art),
                reason=self.reason,
                rejected=True,
            )
        if not obs.joints_valid:
            self.mode = "fault"
            self.reason = "joints_invalid"
            return SupervisorResult(
                mode=self.mode,
                approved_drive_vx=0.0,
                approved_drive_wz=0.0,
                approved_art=dict(self._hold_art),
                reason=self.reason,
                rejected=True,
            )

        art = {k: max(-ART_RANGE_RAD, min(ART_RANGE_RAD, v)) for k, v in art.items()}

        if self.mode == "fault":
            return SupervisorResult(
                mode=self.mode,
                approved_drive_vx=0.0,
                approved_drive_wz=0.0,
                approved_art=dict(self._hold_art),
                reason=self.reason,
                rejected=True,
            )
        if self.mode == "paused":
            return SupervisorResult(
                mode=self.mode,
                approved_drive_vx=0.0,
                approved_drive_wz=0.0,
                approved_art=dict(self._hold_art),
                reason="paused",
            )
        # Watchdog zeros drive only. Pitch holds must keep following posture
        # commands or the modules never move after a deadman timeout.
        if not obs.command_fresh:
            self._hold_art = dict(art)
            return SupervisorResult(
                mode=self.mode,
                approved_drive_vx=0.0,
                approved_drive_wz=0.0,
                approved_art=dict(art),
                reason="stale_drive",
            )
        if self.mode == "manual":
            # Operator drive; still hold last/preset art (no competing publishers).
            self._hold_drive = (vx, wz)
            if output.source.startswith("manual_legs") or output.source.startswith("preset"):
                self._hold_art = dict(art)
            return SupervisorResult(
                mode=self.mode,
                approved_drive_vx=vx,
                approved_drive_wz=wz,
                approved_art=dict(self._hold_art),
                reason="manual",
            )

        # assisted
        self._hold_art = dict(art)
        self._hold_drive = (vx, wz)
        return SupervisorResult(
            mode=self.mode,
            approved_drive_vx=vx,
            approved_drive_wz=wz,
            approved_art=dict(art),
            reason=output.source,
        )
