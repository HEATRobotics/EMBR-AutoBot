"""Single-motor telemetry twin. Input state is imposed, not torque simulated."""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from numbers import Real
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import yaml

from embr_local_sim.paths import config_dir


def _finite(name: str, value: float) -> None:
    """Reject nonnumeric and nonfinite input before changing the MuJoCo state."""
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")


@dataclass(frozen=True)
class MotorParams:
    """SI model parameters; provenance and geometric assumptions are in motor-twin.md."""

    housing_radius_m: float = 0.026
    housing_length_m: float = 0.070
    shaft_radius_m: float = 0.004
    shaft_length_m: float = 0.025
    motor_mass_kg: float = 0.752
    rotor_mass_kg: float = 0.040
    rotor_axial_inertia_kg_m2: float = 1.7e-5
    rotor_transverse_inertia_kg_m2: float = 1.7e-5

    def __post_init__(self) -> None:
        """Require positive finite geometry and a physically admissible inertia tensor."""
        for field in fields(self):
            value = getattr(self, field.name)
            _finite(field.name, value)
            if value <= 0:
                raise ValueError(f"{field.name} must be positive")
        if self.rotor_mass_kg >= self.motor_mass_kg:
            raise ValueError("rotor_mass_kg must be smaller than motor_mass_kg")
        if self.shaft_radius_m >= self.housing_radius_m:
            raise ValueError("shaft_radius_m must be smaller than housing_radius_m")
        if self.rotor_axial_inertia_kg_m2 > 2 * self.rotor_transverse_inertia_kg_m2:
            raise ValueError("rotor inertia must satisfy the triangle inequality")


def load_motor_params(path: str | Path | None = None) -> MotorParams:
    """Load the packaged defaults or a flat YAML mapping of parameter overrides."""
    file = Path(path) if path is not None else config_dir() / "maxon_ec_i52.yaml"
    try:
        raw = yaml.safe_load(file.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(f"Invalid motor YAML in {file}: {exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError("Motor configuration must be a YAML mapping")
    unknown = set(raw) - {field.name for field in fields(MotorParams)}
    if unknown:
        raise ValueError(f"Unknown motor parameters: {sorted(unknown, key=str)}")
    return MotorParams(**raw)


def motor_mjcf(params: MotorParams) -> str:
    """Generate a fixed housing and one continuous shaft hinge about world -Y."""
    p = params
    height = p.housing_radius_m + 0.014
    root = ET.Element("mujoco", model="maxon_ec_i52_667065_telemetry_twin")
    ET.SubElement(root, "compiler", angle="radian")
    ET.SubElement(root, "option", gravity="0 0 0")
    ET.SubElement(root, "visual").append(ET.Element("global", azimuth="-90", elevation="-15"))
    world = ET.SubElement(root, "worldbody")
    # Cylinders are local-Z primitives; this rotation points them along -Y.
    cylinder_quat = "0.7071067811865476 0.7071067811865476 0 0"

    def geom(parent: ET.Element, name: str, kind: str, size: str, pos: str, rgba: str, **kw) -> None:
        ET.SubElement(parent, "geom", name=name, type=kind, size=size, pos=pos,
                      rgba=rgba, contype="0", conaffinity="0", density="0", **kw)

    geom(world, "mount", "box", f"{p.housing_radius_m * 1.3} {p.housing_length_m * .7} .005",
         "0 0 .005", ".12 .15 .18 1")
    housing = ET.SubElement(world, "body", name="housing", pos=f"0 0 {height}")
    geom(housing, "housing_shell", "cylinder", f"{p.housing_radius_m} {p.housing_length_m / 2}",
         "0 0 0", ".43 .48 .53 1", quat=cylinder_quat,
         mass=str(p.motor_mass_kg - p.rotor_mass_kg))
    # A fixed blue reference next to the orange moving pointer makes direction visible.
    pointer_radius = p.housing_radius_m * .70
    face_y = -p.housing_length_m / 2 - p.shaft_length_m - .003
    geom(housing, "fixed_reference", "box", f".002 .001 {p.housing_radius_m * .15}",
         f"{p.housing_radius_m * .90} {face_y} 0", ".15 .65 1 1")
    rotor = ET.SubElement(housing, "body", name="rotor", pos=f"0 {-p.housing_length_m / 2} 0")
    ET.SubElement(rotor, "joint", name="motor_shaft", type="hinge", axis="0 -1 0",
                  limited="false", damping="0")
    ET.SubElement(rotor, "inertial", pos="0 0 0", mass=str(p.rotor_mass_kg),
                  diaginertia=f"{p.rotor_transverse_inertia_kg_m2} {p.rotor_axial_inertia_kg_m2} {p.rotor_transverse_inertia_kg_m2}")
    geom(rotor, "shaft", "cylinder", f"{p.shaft_radius_m} {p.shaft_length_m / 2}",
         f"0 {-p.shaft_length_m / 2} 0", ".82 .85 .89 1", quat=cylinder_quat)
    geom(rotor, "indicator_disk", "cylinder", f"{pointer_radius} .001",
         f"0 {-p.shaft_length_m - .001} 0", ".10 .13 .18 1", quat=cylinder_quat)
    geom(rotor, "shaft_pointer", "box", f"{pointer_radius / 2} .001 .0015",
         f"{pointer_radius / 2} {-p.shaft_length_m - .003} 0", "1 .48 .06 1")
    return ET.tostring(root, encoding="unicode")


@dataclass(frozen=True)
class MotorSample:
    """One shaft observation: monotonic seconds, unwrapped radians and rad/s.

    At least one state field is required. Velocity describes the interval
    starting at this timestamp. Device-specific scaling belongs in the adapter.
    """

    timestamp_s: float
    position_rad: float | None = None
    velocity_rad_s: float | None = None


@dataclass(frozen=True)
class MotorState:
    """Applied state with explicit provenance for absent position/speed fields."""

    timestamp_s: float
    position_rad: float
    velocity_rad_s: float
    position_kind: str
    velocity_kind: str


class MotorTwin:
    """Own a standalone MuJoCo model/data pair; apply feedback on one thread."""

    def __init__(self, params: MotorParams | None = None) -> None:
        """Compile the model with SI parameters and initialize the shaft at zero."""
        self.params = params or MotorParams()
        self.model = mujoco.MjModel.from_xml_string(motor_mjcf(self.params))
        self.data = mujoco.MjData(self.model)
        self.state: MotorState | None = None
        self.reset()

    def reset(self) -> None:
        """Clear the observation history and return the shaft to its visual zero."""
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.state = None

    def apply(self, sample: MotorSample) -> MotorState:
        """Validate a sample, map it to qpos/qvel and refresh MuJoCo transforms.

        Supplied position wins. Speed-only samples integrate the PREVIOUS speed
        over source time (zero-order hold), starting at visual zero if first.
        Missing speed is unknown: qvel is zero, with velocity_kind=unavailable.
        Invalid input leaves the previous state intact.
        """
        _finite("timestamp_s", sample.timestamp_s)
        if sample.timestamp_s < 0:
            raise ValueError("timestamp_s must be nonnegative")
        if sample.position_rad is None and sample.velocity_rad_s is None:
            raise ValueError("At least one of position_rad or velocity_rad_s is required")
        for name in ("position_rad", "velocity_rad_s"):
            value = getattr(sample, name)
            if value is not None:
                _finite(name, value)
        previous = self.state
        if previous is not None and sample.timestamp_s <= previous.timestamp_s:
            raise ValueError("timestamp_s must strictly increase; reset before a new stream")
        position = sample.position_rad
        position_kind = "supplied"
        if position is None:
            position_kind = "estimated"
            position = 0.0
            if previous is not None:
                if previous.velocity_kind == "unavailable":
                    raise ValueError("Speed-only update needs a previous speed or explicit position")
                position = previous.position_rad + previous.velocity_rad_s * (
                    sample.timestamp_s - previous.timestamp_s
                )
            _finite("estimated position_rad", position)
        velocity = sample.velocity_rad_s if sample.velocity_rad_s is not None else 0.0
        state = MotorState(float(sample.timestamp_s), float(position), float(velocity),
                           position_kind, "supplied" if sample.velocity_rad_s is not None else "unavailable")
        self.data.joint("motor_shaft").qpos[0] = state.position_rad
        self.data.joint("motor_shaft").qvel[0] = state.velocity_rad_s
        self.data.time = state.timestamp_s
        mujoco.mj_forward(self.model, self.data)
        self.state = state
        return state
