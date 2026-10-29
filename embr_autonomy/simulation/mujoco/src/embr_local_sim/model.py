"""Provisional robot and world parameters plus MJCF generation.

Mass, inertia, friction, and torque values that are not in the EMBR simple
URDF/controller files are labeled ASSUMED. CAD-exported masses in that URDF
are ~0.2 g total and are not used for contact dynamics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any


def _box_diag_inertia(mass: float, full_x: float, full_y: float, full_z: float) -> tuple[float, float, float]:
    ixx = mass * (full_y**2 + full_z**2) / 12.0
    iyy = mass * (full_x**2 + full_z**2) / 12.0
    izz = mass * (full_x**2 + full_y**2) / 12.0
    return ixx, iyy, izz


def _wheel_diag_inertia(mass: float, radius: float, width: float, rotor_scale: float) -> tuple[float, float, float]:
    """Inertia for a cylinder whose hinge axis is +Y, with lumped rotor inertia."""
    i_axis = 0.5 * mass * radius**2 * rotor_scale
    i_radial = mass * (3.0 * radius**2 + width**2) / 12.0
    i_radial = max(i_radial, 0.51 * i_axis)  # keep a physical triangle inequality
    return i_radial, i_axis, i_radial


@dataclass(frozen=True)
class ParamRecord:
    value: float
    unit: str
    provenance: str  # "repository" | "assumed"
    notes: str
    plausible_range: str | None = None


@dataclass
class RobotParams:
    """Four-wheel skid-steer proxy. Not a tracked/capstan model."""

    name: str = "embr_simple_skidsteer_proxy"
    morphology: str = "wheels"  # wheels | articulated | tracks
    # Repository-derived kinematics (embr_simple_controllers.yaml / URDF comments).
    wheel_radius: float = 0.03
    wheel_separation: float = 0.1175481
    wheelbase: float = 0.1734688
    wheel_width: float = 0.009
    # ASSUMED inertial properties. URDF chassis mass is 1.02e-4 kg (unusable).
    chassis_mass: float = 0.20
    wheel_mass: float = 0.025
    chassis_size: tuple[float, float, float] = (0.15, 0.07, 0.024)  # full xyz
    # Hinge axis is body +Y. Positive hinge velocity produces +X ground travel
    # on this MJCF (verified with a torque sweep). +vx therefore uses +qvel.
    hinge_from_ground_speed: float = 1.0
    max_wheel_torque: float = 0.020  # N·m ASSUMED; stallable against a wall via chassis contact
    velocity_kp: float = 0.004  # N·m / (rad/s) ASSUMED
    max_wheel_accel: float = 25.0  # rad/s^2 ASSUMED
    max_vx: float = 0.35  # m/s ASSUMED interactive cap (repo mock limit was 1.0)
    max_wz: float = 2.5  # rad/s ASSUMED
    wheel_damping: float = 5.0e-4
    wheel_friction: tuple[float, float, float] = (0.40, 0.005, 0.0001)
    floor_friction: tuple[float, float, float] = (0.40, 0.005, 0.0001)
    rotor_inertia_scale: float = 6.0  # ASSUMED lumped motor rotor on wheel axis
    physics_dt: float = 0.001
    control_hz: float = 50.0
    watchdog_wall_s: float = 0.40
    encoder_ticks: int = 1024
    gravity: float = 9.81
    # Labeled 2D fan. Ranges are ASSUMED for this tiny proxy, not a Velodyne spec.
    lidar_n_rays: int = 21
    lidar_fov_deg: float = 120.0
    lidar_max_range: float = 2.0
    lidar_min_range: float = 0.02
    thermal_n_bins: int = 9
    thermal_fov_deg: float = 90.0
    thermal_sample_range: float = 0.55

    def provenance_table(self) -> list[ParamRecord]:
        return [
            ParamRecord(self.wheel_radius, "m", "repository", "embr_simple_controllers.yaml wheel_radius"),
            ParamRecord(
                self.wheel_separation,
                "m",
                "repository",
                "URDF comment / controller wheel_separation for the small model",
            ),
            ParamRecord(self.wheelbase, "m", "repository", "embr_simple.urdf xacro property"),
            ParamRecord(self.wheel_width, "m", "repository", "embr_simple.urdf wheel_width"),
            ParamRecord(
                self.chassis_mass,
                "kg",
                "assumed",
                "CAD mass ~1e-4 kg is not used; small 3D-printed chassis scale",
                "0.08–0.40",
            ),
            ParamRecord(self.wheel_mass, "kg", "assumed", "Four rubber/plastic wheels at this radius", "0.01–0.05"),
            ParamRecord(
                self.max_wheel_torque,
                "N·m",
                "assumed",
                "Sized so four wheels cannot far exceed vehicle weight at this radius; a wall can stall",
                "0.005–0.03",
            ),
            ParamRecord(self.velocity_kp, "N·m/(rad/s)", "assumed", "Finite-gain wheel speed loop at the physics rate", "0.001–0.02"),
            ParamRecord(self.max_wheel_accel, "rad/s^2", "assumed", "Command slew limit", "5–40"),
            ParamRecord(self.watchdog_wall_s, "s", "assumed", "Deadman / lease timeout (wall clock)", "0.2–1.0"),
            ParamRecord(self.physics_dt, "s", "assumed", "Fixed MuJoCo step; not skipped to 'catch up' visually", "5e-4–2e-3"),
            ParamRecord(self.lidar_max_range, "m", "assumed", "2D mj_ray fan max range for this proxy", "0.5–5"),
            ParamRecord(self.lidar_fov_deg, "deg", "assumed", "Horizontal fan, not a 3D lidar", "60–180"),
        ]

    def track_half(self) -> float:
        return 0.5 * self.wheel_separation

    def wheelbase_half(self) -> float:
        return 0.5 * self.wheelbase

    def spawn_height(self) -> float:
        return self.wheel_radius + 0.006

    def chassis_clearance(self) -> float:
        return self.wheel_radius - 0.5 * self.chassis_size[2] - 0.002

    def with_overrides(self, **kwargs: Any) -> "RobotParams":
        return replace(self, **kwargs)


@dataclass
class WorldSpec:
    kind: str = "flat"
    wall_x: float = 0.50
    wall_thickness: float = 0.04
    wall_height: float = 0.18
    wall_half_y: float = 0.45
    ramp_angle_deg: float = 8.0
    ramp_length: float = 0.70
    ramp_width: float = 0.40
    ramp_start_x: float = 0.28
    bump_height_scale: float = 0.35  # relative to wheel radius
    left_friction: float | None = None
    right_friction: float | None = None
    stones: list[dict[str, Any]] = field(default_factory=list)
    logs: list[dict[str, Any]] = field(default_factory=list)
    heat: list[dict[str, Any]] = field(default_factory=list)
    ambient_c: float = 20.0
    extras: dict[str, Any] = field(default_factory=dict)
    seed: int = 0


def _friction_attr(mu: tuple[float, float, float]) -> str:
    return f'{mu[0]} {mu[1]} {mu[2]}'


def build_mjcf(robot: RobotParams, world: WorldSpec | None = None) -> str:
    world = world or WorldSpec()
    from embr_local_sim.worlds import materialize_world

    world = materialize_world(world)
    if robot.morphology == "articulated":
        from embr_local_sim.articulated import build_articulated_mjcf

        return build_articulated_mjcf(robot, world)
    if robot.morphology == "tracks":
        from embr_local_sim.track_approx import build_track_mjcf

        return build_track_mjcf(robot, world)
    if robot.morphology == "cad":
        from embr_local_sim.cad_model import build_cad_mjcf

        return build_cad_mjcf(robot, world)
    hx, hy, hz = (0.5 * s for s in robot.chassis_size)
    chassis_z = 0.002
    ixx, iyy, izz = _box_diag_inertia(robot.chassis_mass, *robot.chassis_size)
    wi = _wheel_diag_inertia(
        robot.wheel_mass, robot.wheel_radius, robot.wheel_width, robot.rotor_inertia_scale
    )
    th = robot.track_half()
    wb = robot.wheelbase_half()
    hw = 0.5 * robot.wheel_width
    spawn_z = robot.spawn_height()

    wheels = [
        ("wheel_fl", "front_left_maxon", wb, th, (1.0, 0.45, 0.05)),
        ("wheel_fr", "front_right_maxon", wb, -th, (1.0, 0.45, 0.05)),
        ("wheel_rl", "rear_left_maxon", -wb, th, (0.85, 0.12, 0.12)),
        ("wheel_rr", "rear_right_maxon", -wb, -th, (0.85, 0.12, 0.12)),
    ]

    wheel_xml = []
    motor_xml = []
    for body, joint, x, y, rgb in wheels:
        wheel_xml.append(
            f"""
      <body name="{body}" pos="{x:.6f} {y:.6f} 0">
        <joint name="{joint}" type="hinge" axis="0 1 0" damping="{robot.wheel_damping}"/>
        <inertial pos="0 0 0" mass="{robot.wheel_mass}" diaginertia="{wi[0]:.8e} {wi[1]:.8e} {wi[2]:.8e}"/>
        <geom name="{body}" type="cylinder" size="{robot.wheel_radius} {hw}"
              euler="1.57079632679 0 0" rgba="{rgb[0]} {rgb[1]} {rgb[2]} 1"
              friction="{_friction_attr(robot.wheel_friction)}"
              condim="3" contype="2" conaffinity="1" group="1"/>
      </body>"""
        )
        motor_xml.append(
            f'    <motor name="{joint}" joint="{joint}" '
            f'gear="1" ctrlrange="{-robot.max_wheel_torque} {robot.max_wheel_torque}"/>'
        )

    world_geoms = _world_geoms(robot, world)
    from embr_local_sim.assets import load_manifest, mjcf_asset_compiler

    asset_xml = mjcf_asset_compiler(load_manifest())

    xml = f"""<?xml version="1.0"?>
<!-- EMBR simple four-wheel skid-steer PROXY. Visual meshes are omitted
     when CAD is absent. Collision uses boxes/cylinders. Not tracks/capstan. -->
<mujoco model="{robot.name}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{robot.physics_dt}" gravity="0 0 {-robot.gravity}"
          integrator="implicitfast" cone="pyramidal"
          iterations="50" solver="Newton"/>
  <default>
    <geom solref="0.004 1" solimp="0.9 0.95 0.001"/>
  </default>
  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.4 0.4 0.4"/>
  </visual>
{asset_xml}  <worldbody>
    <light name="sun" pos="0 0 2" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
{world_geoms}
    <body name="root" pos="0 0 {spawn_z}">
      <freejoint name="root_free"/>
      <inertial pos="0 0 {chassis_z}" mass="{robot.chassis_mass}"
                diaginertia="{ixx:.8e} {iyy:.8e} {izz:.8e}"/>
      <geom name="chassis" type="box" size="{hx} {hy} {hz}" pos="0 0 {chassis_z}"
            rgba="0.62 0.81 0.93 1" contype="2" conaffinity="1" group="1"
            friction="{_friction_attr(robot.floor_friction)}"/>
      <site name="imu" pos="0 0 {chassis_z}" size="0.005"/>
      <site name="lidar" pos="0.02 0 0.018" size="0.004"/>
      <site name="camera" pos="0.04 0 0.016" size="0.003" rgba="0.2 0.2 0.8 1"/>
      <body name="probe_arm" pos="{hx:.4f} 0 {chassis_z}">
        <joint name="probe_hinge" type="hinge" axis="0 1 0" range="0 1.2" damping="0.003"/>
        <geom name="probe_boom" type="capsule" fromto="0 0 0 0.04 0 0" size="0.003"
              rgba="0.15 0.85 0.35 1" group="3" contype="0" conaffinity="0"/>
        <site name="probe" pos="0.04 0 0" size="0.004" rgba="0.1 1.0 0.2 1"/>
      </body>
{''.join(wheel_xml)}
    </body>
  </worldbody>
  <actuator>
{chr(10).join(motor_xml)}
    <position name="probe_hinge" joint="probe_hinge" kp="2.4" dampratio="1" ctrlrange="0 1.2"/>
  </actuator>
</mujoco>
"""
    return xml


def lane_ramp_geoms(extras: dict[str, Any] | None) -> list[dict[str, Any]]:
    """A slope under each track, with the belly left clear.

    A full-width ramp lets the plate high-center. These lanes are only as
    wide as the rollers. `angle_deg` is the slope. The low end sits on the floor.
    """
    spec = (extras or {}).get("track_ramps")
    if not spec:
        return []
    angle = math.radians(float(spec["angle_deg"]))
    length = float(spec["length"])
    start = float(spec.get("x", spec.get("start_x", 0.0)))
    thick = 0.006
    half = 0.5 * length
    out: list[dict[str, Any]] = []
    for i, lane in enumerate(spec.get("lanes") or []):
        y = float(lane["y"])
        width = float(lane["width"])
        out.append(
            {
                "name": f"lane_ramp_{i}",
                "pos": (start + half * math.cos(angle), y, half * math.sin(angle) - thick),
                "size": (half, 0.5 * width, thick),
                "euler": (0.0, -angle, 0.0),
                "rgba": "0.72 0.46 0.18 1",
            }
        )
    return out


def _world_geoms(robot: RobotParams, world: WorldSpec) -> str:
    floor_mu = robot.floor_friction
    chunks: list[str] = []
    if world.kind == "split_friction":
        left = world.left_friction if world.left_friction is not None else 1.1
        right = world.right_friction if world.right_friction is not None else 0.15
        span = float((world.extras or {}).get("layout_scale", 1.0))
        chunks.append(
            f"""    <geom name="floor_left" type="plane" size="{4 * span} {4 * span} 0.05" pos="0 {2.0 * span} 0"
          friction="{left} 0.005 0.0001" contype="1" conaffinity="2" group="0"/>
    <geom name="floor_right" type="plane" size="{4 * span} {4 * span} 0.05" pos="0 {-2.0 * span} 0"
          friction="{right} 0.005 0.0001" contype="1" conaffinity="2" group="0"/>"""
        )
    else:
        floor_rgba = "0.18 0.20 0.18 1"
        if world.kind in ("forest", "wildfire", "grove"):
            floor_rgba = "0.16 0.22 0.14 1"
        elif world.kind in ("engineering", "course"):
            floor_rgba = "0.22 0.22 0.20 1"
        half = float((world.extras or {}).get("floor_half", 5.0))
        chunks.append(
            f"""    <geom name="floor" type="plane" size="{half} {half} 0.05"
          rgba="{floor_rgba}" friction="{_friction_attr(floor_mu)}"
          contype="1" conaffinity="2" group="0"/>"""
        )

    if world.kind in ("wall", "engineering"):
        z = 0.5 * world.wall_height
        chunks.append(
            f"""    <geom name="wall" type="box"
          pos="{world.wall_x} 0 {z}"
          size="{0.5 * world.wall_thickness} {world.wall_half_y} {z}"
          rgba="0.75 0.35 0.2 1" contype="1" conaffinity="2" group="0"/>"""
        )

    if world.kind in ("ramp", "engineering") or bool((world.extras or {}).get("include_ramp")):
        angle = math.radians(world.ramp_angle_deg)
        half_len = 0.5 * world.ramp_length
        thick = 0.0025
        cx = world.ramp_start_x + half_len * math.cos(angle)
        # Sink the slab so the low end is flush with the floor (no curb).
        cz = half_len * math.sin(angle) - thick
        cy = float((world.extras or {}).get("ramp_y", 0.0))
        chunks.append(
            f"""    <geom name="ramp" type="box"
          pos="{cx:.5f} {cy:.5f} {cz:.5f}" euler="0 {-angle:.6f} 0"
          size="{half_len:.5f} {0.5 * world.ramp_width:.5f} {thick}"
          rgba="0.45 0.48 0.42 1" contype="1" conaffinity="2" group="0"/>"""
        )

    for ramp in lane_ramp_geoms(world.extras):
        px, py, pz = ramp["pos"]
        sx, sy, sz = ramp["size"]
        _ex, ey, ez = ramp["euler"]
        rgba = str(ramp.get("rgba", "0.62 0.42 0.22 1"))
        chunks.append(
            f"""    <geom name="{ramp['name']}" type="box"
          pos="{px:.5f} {py:.5f} {pz:.5f}" euler="0 {ey:.6f} {ez:.6f}"
          size="{sx:.5f} {sy:.5f} {sz:.5f}"
          rgba="{rgba}" contype="1" conaffinity="2" group="0"/>"""
        )

    if world.kind == "bumps":
        h = world.bump_height_scale * robot.wheel_radius
        for i, x in enumerate((0.22, 0.34, 0.48)):
            chunks.append(
                f"""    <geom name="bump_{i}" type="box" pos="{x} 0 {0.5 * h}"
          size="0.02 0.12 {0.5 * h}" rgba="0.4 0.35 0.28 1"
          contype="1" conaffinity="2" group="0"/>"""
            )

    stones = list(world.stones)
    logs = list(world.logs)
    heat = list(world.heat)
    if world.kind == "forest" and not stones and not logs:
        stones, logs, heat = _default_forest()
        world.stones = stones
        world.logs = logs
        if not world.heat:
            world.heat = heat

    for i, stone in enumerate(stones):
        x = float(stone.get("x", 0.3))
        y = float(stone.get("y", 0.0))
        sx, sy, sz = (float(v) for v in stone.get("size", (0.04, 0.04, 0.03)))
        movable = bool(stone.get("movable", False))
        mass = float(stone.get("mass", 0.08))
        if movable:
            # size is the MuJoCo half-extent. Inertia uses the full edge.
            ixx, iyy, izz = _box_diag_inertia(mass, 2.0 * sx, 2.0 * sy, 2.0 * sz)
            chunks.append(
                f"""    <body name="stone_{i}" pos="{x} {y} {sz + 0.001}">
      <freejoint name="stone_{i}_free"/>
      <inertial pos="0 0 0" mass="{mass}" diaginertia="{ixx:.8e} {iyy:.8e} {izz:.8e}"/>
      <geom name="stone_{i}" type="box" size="{sx} {sy} {sz}"
            rgba="0.45 0.42 0.38 1" contype="3" conaffinity="3" group="0"
            friction="0.55 0.005 0.0001"/>
    </body>"""
            )
        else:
            chunks.append(
                f"""    <geom name="stone_{i}" type="box" pos="{x} {y} {sz}"
          size="{sx} {sy} {sz}" rgba="0.42 0.38 0.32 1"
          contype="1" conaffinity="2" group="0"/>"""
            )

    for i, log in enumerate(logs):
        x = float(log.get("x", 0.5))
        y = float(log.get("y", 0.0))
        length = float(log.get("length", 0.20))
        radius = float(log.get("radius", 0.018))
        yaw = float(log.get("yaw", 0.0))
        hinged = bool(log.get("hinged", False))
        free = bool(log.get("free", False))
        if free:
            mass = float(log.get("mass", 0.05))
            # contype 3 also matches the floor. A free body with the floor's own
            # bits (1/2) falls through the world. Static obstacles stay on 1/2.
            # Capsule inertia about its long axis is approximate (ASSUMED rigid wood).
            i_long = 0.5 * mass * radius * radius
            i_side = mass * (3.0 * radius * radius + length * length) / 12.0
            chunks.append(
                f"""    <body name="log_{i}" pos="{x} {y} {radius}">
      <freejoint name="log_{i}_free"/>
      <inertial pos="0 0 0" mass="{mass}" diaginertia="{i_side:.8e} {i_side:.8e} {i_long:.8e}"/>
      <geom name="log_{i}" type="capsule"
            fromto="{-0.5 * length * math.cos(yaw):.4f} {-0.5 * length * math.sin(yaw):.4f} 0 {0.5 * length * math.cos(yaw):.4f} {0.5 * length * math.sin(yaw):.4f} 0"
            size="{radius}"
            rgba="0.45 0.32 0.18 1" friction="0.45 0.005 0.0001"
            contype="3" conaffinity="3" group="0"/>
    </body>"""
            )
        elif hinged:
            chunks.append(
                f"""    <body name="log_{i}" pos="{x} {y} {radius}">
      <joint name="log_{i}_hinge" type="hinge" axis="0 0 1" damping="0.002"/>
      <geom name="log_{i}" type="capsule"
            fromto="{-0.5 * length * math.cos(yaw):.4f} {-0.5 * length * math.sin(yaw):.4f} 0 {0.5 * length * math.cos(yaw):.4f} {0.5 * length * math.sin(yaw):.4f} 0"
            size="{radius}"
            rgba="0.40 0.28 0.16 1" contype="1" conaffinity="2" group="0"/>
    </body>"""
            )
        else:
            chunks.append(
                f"""    <geom name="log_{i}" type="capsule"
          fromto="{x - 0.5 * length * math.cos(yaw):.4f} {y - 0.5 * length * math.sin(yaw):.4f} {radius} {x + 0.5 * length * math.cos(yaw):.4f} {y + 0.5 * length * math.sin(yaw):.4f} {radius}"
          size="{radius}"
          rgba="0.40 0.28 0.16 1" contype="1" conaffinity="2" group="0"/>"""
            )

    for i, src in enumerate(heat):
        x = float(src.get("x", 0.8))
        y = float(src.get("y", 0.0))
        z = float(src.get("z", 0.03))
        kind = str(src.get("kind", "smolder"))
        rgba = "1.0 0.25 0.05 0.35" if kind != "flame" else "1.0 0.45 0.05 0.20"
        chunks.append(
            f"""    <geom name="heat_marker_{i}" type="sphere" pos="{x} {y} {z}"
          size="0.035" rgba="{rgba}"
          contype="0" conaffinity="0" group="3"/>"""
        )

    extras = world.extras or {}
    if extras.get("metre_marks"):
        for i, x in enumerate((-1.0, 0.0, 1.0, 2.0)):
            chunks.append(
                f"""    <geom name="mark_x_{i}" type="box" pos="{x:.2f} 0 0.0008"
          size="0.006 1.2 0.0006" rgba="0.85 0.85 0.75 0.55"
          contype="0" conaffinity="0" group="3"/>"""
            )
        for i, y in enumerate((-1.0, 0.0, 1.0)):
            chunks.append(
                f"""    <geom name="mark_y_{i}" type="box" pos="0.5 {y:.2f} 0.0008"
          size="1.4 0.006 0.0006" rgba="0.75 0.78 0.70 0.45"
          contype="0" conaffinity="0" group="3"/>"""
            )

    cross = extras.get("cross_slope") or {}
    if cross:
        angle = math.radians(float(cross.get("deg", 8.0)))
        length = float(cross.get("length", 0.55))
        width = float(cross.get("width", 0.28))
        x = float(cross.get("x", 0.8))
        y = float(cross.get("y", -0.35))
        thick = 0.003
        half = 0.5 * width
        cz = half * math.sin(abs(angle)) - thick
        chunks.append(
            f"""    <geom name="cross_slope" type="box"
          pos="{x:.4f} {y:.4f} {cz:.4f}" euler="{angle:.6f} 0 0"
          size="{0.5 * length:.4f} {half:.4f} {thick}"
          rgba="0.40 0.42 0.36 1" contype="1" conaffinity="2" group="0"/>"""
        )

    step = extras.get("step") or {}
    if step:
        sx, sy, sz = (float(v) for v in step.get("size", (0.08, 0.10, 0.01)))
        x = float(step.get("x", 0.28))
        y = float(step.get("y", -0.28))
        chunks.append(
            f"""    <geom name="step" type="box" pos="{x} {y} {sz}"
          size="{sx} {sy} {sz}" rgba="0.50 0.48 0.42 1"
          contype="1" conaffinity="2" group="0"/>"""
        )

    for i, tree in enumerate(extras.get("trees") or []):
        x = float(tree.get("x", 0.0))
        y = float(tree.get("y", 0.0))
        radius = float(tree.get("radius", 0.04))
        height = float(tree.get("height", 0.4))
        chunks.append(
            f"""    <geom name="tree_{i}" type="cylinder" pos="{x:.4f} {y:.4f} {0.5 * height:.4f}"
          size="{radius:.4f} {0.5 * height:.4f}"
          rgba="0.32 0.22 0.12 1" contype="1" conaffinity="2" group="0"/>"""
        )

    for i, cap in enumerate(extras.get("canopy") or []):
        x = float(cap.get("x", 0.0))
        y = float(cap.get("y", 0.0))
        z = float(cap.get("z", 0.4))
        radius = float(cap.get("radius", 0.12))
        chunks.append(
            f"""    <geom name="canopy_{i}" type="sphere" pos="{x:.4f} {y:.4f} {z:.4f}"
          size="{radius:.4f}" rgba="0.12 0.38 0.16 0.35"
          contype="0" conaffinity="0" group="3"/>"""
        )

    for i, root in enumerate(extras.get("roots") or []):
        x = float(root.get("x", 0.0))
        y = float(root.get("y", 0.0))
        sx, sy, sz = (float(v) for v in root.get("size", (0.04, 0.018, 0.012)))
        chunks.append(
            f"""    <geom name="root_{i}" type="box" pos="{x:.4f} {y:.4f} {sz:.4f}"
          size="{sx:.4f} {sy:.4f} {sz:.4f}" rgba="0.28 0.20 0.12 1"
          contype="1" conaffinity="2" group="0"/>"""
        )

    for i, mound in enumerate(extras.get("mounds") or []):
        x = float(mound.get("x", 0.0))
        y = float(mound.get("y", 0.0))
        sx, sy, sz = (float(v) for v in mound.get("size", (0.06, 0.05, 0.01)))
        chunks.append(
            f"""    <geom name="mound_{i}" type="box" pos="{x:.4f} {y:.4f} {sz:.4f}"
          size="{sx:.4f} {sy:.4f} {sz:.4f}" rgba="0.30 0.28 0.18 1"
          contype="1" conaffinity="2" group="0"/>"""
        )

    for i, shrub in enumerate(extras.get("shrubs") or []):
        x = float(shrub.get("x", 0.0))
        y = float(shrub.get("y", 0.0))
        z = float(shrub.get("z", 0.08))
        radius = float(shrub.get("radius", 0.05))
        chunks.append(
            f"""    <geom name="shrub_{i}" type="sphere" pos="{x:.4f} {y:.4f} {z:.4f}"
          size="{radius:.4f}" rgba="0.20 0.45 0.18 1"
          contype="1" conaffinity="2" group="0"/>"""
        )

    for i, branch in enumerate(extras.get("branches") or []):
        x = float(branch.get("x", 0.5))
        y = float(branch.get("y", 0.0))
        length = float(branch.get("length", 0.14))
        radius = float(branch.get("radius", 0.008))
        yaw = float(branch.get("yaw", 0.0))
        z = float(branch.get("z", radius))
        chunks.append(
            f"""    <geom name="branch_{i}" type="capsule"
          fromto="{x - 0.5 * length * math.cos(yaw):.4f} {y - 0.5 * length * math.sin(yaw):.4f} {z:.4f} {x + 0.5 * length * math.cos(yaw):.4f} {y + 0.5 * length * math.sin(yaw):.4f} {z:.4f}"
          size="{radius}" rgba="0.38 0.26 0.14 1"
          contype="1" conaffinity="2" group="0"/>"""
        )

    for i, flame in enumerate(extras.get("flames") or []):
        x = float(flame.get("x", 0.5))
        y = float(flame.get("y", 0.4))
        h = float(flame.get("height", 0.14))
        r = float(flame.get("radius", 0.03))
        for k, (frac, rgba) in enumerate(((0.25, "1.0 0.35 0.05 0.55"), (0.55, "1.0 0.55 0.08 0.40"), (0.85, "1.0 0.85 0.25 0.28"))):
            chunks.append(
                f"""    <geom name="flame_{i}_{k}" type="sphere" pos="{x:.4f} {y:.4f} {frac * h:.4f}"
          size="{r * (1.0 - 0.22 * k):.4f}" rgba="{rgba}"
          contype="0" conaffinity="0" group="3"/>"""
            )

    for i, puff in enumerate(extras.get("smoke") or []):
        x = float(puff.get("x", 0.5))
        y = float(puff.get("y", 0.4))
        z = float(puff.get("z", 0.2))
        r = float(puff.get("radius", 0.08))
        chunks.append(
            f"""    <geom name="smoke_{i}" type="sphere" pos="{x:.4f} {y:.4f} {z:.4f}"
          size="{r:.4f}" rgba="0.45 0.45 0.45 0.22"
          contype="0" conaffinity="0" group="3"/>"""
        )

    if extras.get("return_xy"):
        rx, ry = extras["return_xy"]
        chunks.append(
            f"""    <geom name="return_pad" type="cylinder" pos="{float(rx):.4f} {float(ry):.4f} 0.001"
          size="0.06 0.001" rgba="0.15 0.45 0.85 0.35"
          contype="0" conaffinity="0" group="3"/>"""
        )
    return "\n".join(chunks)


def _default_forest() -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    """ASSUMED clutter for the tiny proxy. Not a surveyed burn plot."""
    from embr_local_sim.worlds import default_forest_clutter

    return default_forest_clutter()


def write_mjcf(path: Path, robot: RobotParams | None = None, world: WorldSpec | None = None) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(build_mjcf(robot or RobotParams(), world), encoding="utf-8")
    return path
