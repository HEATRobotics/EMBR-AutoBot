"""Conservative pitch-posture controller. Not predictive terrain planning.

Uses IMU roll/pitch and measured articulation joints only. A 2D lidar fan is
not a height map; this controller does not claim obstacle preview.
"""

from __future__ import annotations

from embr_local_sim.articulated import ART_JOINTS, ART_RANGE_RAD
from embr_local_sim.observation import ControllerOutput, RobotObservation

PRESETS: dict[str, dict[str, float]] = {
    "flat": {name: 0.0 for name in ART_JOINTS},
    # Positive art (axis +Y) pitches the pod nose toward -Z on this MJCF.
    # "climb" raises the front of the front pods (negative) and plants the rear.
    "climb": {"art_fl": -0.35, "art_fr": -0.35, "art_rl": 0.20, "art_rr": 0.20},
    "cross": {"art_fl": -0.25, "art_fr": 0.25, "art_rl": 0.25, "art_rr": -0.25},
}


class PostureController:
    def __init__(self) -> None:
        self.preset = "flat"
        self.manual_targets: dict[str, float] | None = None
        self.kp_level = 0.40  # rad IMU pitch → extra front bias; ASSUMED

    def reset(self) -> None:
        self.preset = "flat"
        self.manual_targets = None

    def set_preset(self, name: str) -> None:
        if name in PRESETS:
            self.preset = name
            self.manual_targets = None

    def set_manual(self, targets: dict[str, float]) -> None:
        self.manual_targets = {k: float(v) for k, v in targets.items() if k in ART_JOINTS}

    def step(self, obs: RobotObservation, drive_vx: float, drive_wz: float) -> ControllerOutput:
        if self.manual_targets is not None:
            targets = dict(self.manual_targets)
            src = "manual_legs"
        else:
            targets = dict(PRESETS[self.preset])
            src = f"preset:{self.preset}"
            # Tilt-only bias: if nose is down (positive pitch here), lift front pods.
            # This is attitude regulation, not contact planning.
            if obs.imu_valid:
                bias = max(-0.25, min(0.25, -self.kp_level * obs.pitch_rad))
                for name in ("art_fl", "art_fr"):
                    targets[name] = targets[name] + bias
        clipped = {k: max(-ART_RANGE_RAD, min(ART_RANGE_RAD, v)) for k, v in targets.items()}
        return ControllerOutput(
            drive_vx=drive_vx,
            drive_wz=drive_wz,
            art_targets=clipped,
            source=src,
            status="ok",
        )
