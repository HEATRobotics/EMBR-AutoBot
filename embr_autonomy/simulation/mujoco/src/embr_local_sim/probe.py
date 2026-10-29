"""One-DOF hinge placeholder. Not confirmed probe kinematics.

CAD filenames include leadscrew.SLDPRT and thermoprobe.SLDPRT. Those solids
were not imported, so this hinge is not described as the real probe.

States: STOWED → DEPLOYING → SETTLING → SAMPLING → RETRACTING, plus FAULT.
Drive TeleCmd must not command this joint.
"""

from __future__ import annotations

from dataclasses import dataclass

PROBE_JOINT = "probe_hinge"
PROBE_ACTUATOR = "probe_hinge"
STOWED_RAD = 0.0
DEPLOYED_RAD = 0.85  # ASSUMED; not a measured EMBR sampler
ANGLE_TOL = 0.10
PROBE_STATES = ("STOWED", "DEPLOYING", "SETTLING", "SAMPLING", "RETRACTING", "FAULT")


@dataclass
class ProbeConfig:
    deploy_timeout_s: float = 1.5
    settle_s: float = 0.12
    retract_timeout_s: float = 1.5
    stuck: bool = False  # test injection: fault on the next tick
    # Fault injection at the actuator, used by the retraction scenario.
    # Off by default. When set, a stow request reaches RETRACTING and the
    # hinge then faults instead of closing. The mission is not forced to FAULT
    # from outside; the controller sees probe_state == FAULT.
    retract_fault: bool = False


class ProbeMachine:
    def __init__(self, config: ProbeConfig | None = None) -> None:
        self.config = config or ProbeConfig()
        self.state = "STOWED"
        self.reason = "stowed"
        self.command = "stow"  # stow | deploy
        self.state_entered = 0.0
        self.target = STOWED_RAD

    def reset(self) -> None:
        self.state = "STOWED"
        self.reason = "stowed"
        self.command = "stow"
        self.state_entered = 0.0
        self.target = STOWED_RAD

    def request_deploy(self, sim_time: float) -> None:
        if self.state == "FAULT":
            return
        self.command = "deploy"
        if self.state in ("STOWED", "RETRACTING"):
            self.state = "DEPLOYING"
            self.reason = "deploying"
            self.state_entered = sim_time
            self.target = DEPLOYED_RAD

    def request_stow(self, sim_time: float) -> None:
        if self.state == "FAULT":
            return
        self.command = "stow"
        if self.state in ("SAMPLING", "SETTLING", "DEPLOYING"):
            self.state = "RETRACTING"
            self.reason = "retracting"
            self.state_entered = sim_time
            self.target = STOWED_RAD

    def tick(self, *, sim_time: float, angle: float) -> float:
        cfg = self.config
        if cfg.stuck:
            self.state = "FAULT"
            self.reason = "actuator_stuck"
            return float(angle)
        if cfg.retract_fault and self.state == "RETRACTING":
            self.state = "FAULT"
            self.reason = "retract_actuator_fault"
            self.target = float(angle)
            return float(angle)
        if self.state == "DEPLOYING":
            if abs(angle - DEPLOYED_RAD) <= ANGLE_TOL:
                self.state = "SETTLING"
                self.reason = "settling"
                self.state_entered = sim_time
            elif sim_time - self.state_entered > cfg.deploy_timeout_s:
                self.state = "FAULT"
                self.reason = "deploy_timeout"
        elif self.state == "SETTLING":
            if sim_time - self.state_entered >= cfg.settle_s:
                self.state = "SAMPLING"
                self.reason = "sampling"
                self.state_entered = sim_time
        elif self.state == "RETRACTING":
            if abs(angle - STOWED_RAD) <= ANGLE_TOL:
                self.state = "STOWED"
                self.reason = "stowed"
                self.state_entered = sim_time
            elif sim_time - self.state_entered > cfg.retract_timeout_s:
                self.state = "FAULT"
                self.reason = "retract_timeout"
        self.target = DEPLOYED_RAD if self.command == "deploy" and self.state != "FAULT" else STOWED_RAD
        if self.state in ("DEPLOYING", "SETTLING", "SAMPLING"):
            self.target = DEPLOYED_RAD
        if self.state in ("RETRACTING", "STOWED"):
            self.target = STOWED_RAD
        return self.target

    @property
    def extended(self) -> bool:
        return self.state in ("DEPLOYING", "SETTLING", "SAMPLING")

    def snapshot(self) -> dict[str, str | float]:
        return {
            "state": self.state,
            "reason": self.reason,
            "target": self.target,
            "kind": "hinge_boom_local_proposal",
        }
