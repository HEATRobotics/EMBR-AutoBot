"""CAD-grounded morphology. Geometry comes from the local Onshape STEP.

The visual mesh is the tessellated assembly in the body frame. It does not
collide. Contact is the base-plate box plus one cylinder per lower idler.
Those cylinders are not a belt, and the STEP contains no joint entities, so
the four drive hinges are a sim interface at the idler centers.

Mass, torque, and the probe hinge range are ASSUMED. See cad_geometry.json.
"""

from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path

from embr_local_sim.model import (
    RobotParams,
    WorldSpec,
    _box_diag_inertia,
    _friction_attr,
    _wheel_diag_inertia,
    _world_geoms,
)
from embr_local_sim.paths import assets_dir

FIDELITY = (
    "CAD visual from the Onshape STEP. Contact is four idler cylinders and the "
    "base plate, not a belt. Mass and torque are assumed. Hardware validation pending."
)


@lru_cache(maxsize=1)
def load_cad_geometry() -> dict:
    path = assets_dir() / "cad_geometry.json"
    if not path.is_file():
        raise FileNotFoundError(
            f"CAD geometry record is missing: {path}. Run python -m embr_local_sim.cad_convert "
            "against a local tessellation. Do not upload the STEP."
        )
    return json.loads(path.read_text(encoding="utf-8"))


def cad_mesh_path() -> Path:
    path = assets_dir() / "meshes" / "cad_visual.obj"
    if not path.is_file():
        raise FileNotFoundError(f"CAD visual mesh is missing: {path}")
    return path


def cad_body_spec():
    """The numbers other modules need. Geometry stays in this file."""
    from embr_local_sim.body import BodySpec

    plate_forward = float(load_cad_geometry()["plate"]["size_m"][2])
    return BodySpec(
        name="cad",
        fidelity=FIDELITY,
        layout_scale=plate_forward / 0.15,
        lidar_overhang_m=cad_lidar_overhang_m(),
        view_span_m=8.0,
        view_min_m=-2.0,
        camera_position=(6.5, -8.0, 4.2),
        camera_look_at=(1.5, 0.0, 0.2),
        grid_m=24.0,
    )


def cad_lidar_overhang_m() -> float:
    """How far the front idler sticks past the lidar site, in metres.

    The site sits on the plate. The lower idlers are ahead of it. A mission
    that stops at the proxy's 0.14 m range would already be in contact.
    """
    geo = load_cad_geometry()
    radius = float(geo["roller_radius_m"])
    plate_forward = float(geo["plate"]["size_m"][2])
    lidar_x = 0.5 * plate_forward - 0.02
    nose = max(float(roller["body_xyz_m"][0]) for roller in geo["rollers"]) + radius
    return nose - lidar_x


def apply_cad_params(robot: RobotParams) -> RobotParams:
    """Copy CAD extents onto the robot. Inertial numbers stay marked assumed."""
    geo = load_cad_geometry()
    plate = geo["plate"]["size_m"]
    # Plate AABB is CAD (X lateral, Y up, Z forward). Sim chassis box is
    # (forward, left, up) = (CAD Z, CAD X, CAD Y).
    chassis = (float(plate[2]), float(plate[0]), max(float(plate[1]), 0.006))
    return robot.with_overrides(
        name="embr_cad_step",
        morphology="cad",
        wheel_radius=float(geo["roller_radius_m"]),
        wheel_separation=float(geo["wheel_separation_m"]),
        wheelbase=float(geo["wheelbase_m"]),
        wheel_width=float(max(r["width_m"] for r in geo["rollers"])),
        chassis_size=chassis,
        chassis_mass=float(geo["mass_kg"]["chassis"]),
        wheel_mass=float(geo["mass_kg"]["roller"]),
        max_wheel_torque=float(geo["max_wheel_torque_nm"]["value"]),
        velocity_kp=float(geo["velocity_kp"]["value"]),
        # ASSUMED fan. The proxy uses 2 m. Scaled scenes extend several metres.
        # This is not a lidar datasheet.
        lidar_max_range=8.0,
        thermal_sample_range=0.55 * (float(geo["plate"]["size_m"][2]) / 0.15),
    )


def build_cad_mjcf(robot: RobotParams, world: WorldSpec | None = None) -> str:
    world = world or WorldSpec()
    geo = load_cad_geometry()
    radius = float(geo["roller_radius_m"])
    plate = geo["plate"]
    px, py, pz = (float(v) for v in plate["body_center_m"])
    full_x, full_y, full_z = robot.chassis_size
    # The exported plate face can be numerically flat. Keep a 6 mm box so
    # the deck has a volume; 6.3 mm was the union thickness when the faces
    # spanned it, and chassis_size already carries max(thickness, 0.006).
    hx, hy, hz = 0.5 * full_x, 0.5 * full_y, 0.5 * full_z
    # Uniform-box inertia on the idler envelope. The plate is only 6 mm thick;
    # using that as the inertia tensor makes the provisional 4 kg mass unstable.
    # This is not a CAD mass property.
    ixx, iyy, izz = _box_diag_inertia(
        robot.chassis_mass,
        robot.wheelbase + 2.0 * radius,
        robot.wheel_separation + robot.wheel_width,
        2.0 * radius,
    )
    inertia = _wheel_diag_inertia(robot.wheel_mass, radius, robot.wheel_width, robot.rotor_inertia_scale)
    spawn_z = radius + 0.004
    mesh = cad_mesh_path()
    world_geoms = _world_geoms(robot, world)
    torque = robot.max_wheel_torque
    half_w = 0.5 * robot.wheel_width

    from embr_local_sim.suspension import slide_joint_xml

    rollers = []
    motors = []
    for spec in geo["rollers"]:
        name = spec["joint"]
        x, y, z = (float(v) for v in spec["body_xyz_m"])
        body = name.replace("_maxon", "")
        rollers.append(
            f"""
      <body name="{body}" pos="{x:.6f} {y:.6f} {z:.6f}">
        {slide_joint_xml(body)}
        <joint name="{name}" type="hinge" axis="0 1 0" damping="{robot.wheel_damping}"/>
        <inertial pos="0 0 0" mass="{robot.wheel_mass}" diaginertia="{inertia[0]:.8e} {inertia[1]:.8e} {inertia[2]:.8e}"/>
        <geom name="{body}" type="cylinder" size="{radius:.6f} {half_w:.6f}"
              euler="1.57079632679 0 0" rgba="0.15 0.15 0.15 0.35"
              friction="{_friction_attr(robot.wheel_friction)}"
              condim="3" contype="2" conaffinity="1" group="3"/>
      </body>"""
        )
        motors.append(
            f'    <motor name="{name}" joint="{name}" gear="1" ctrlrange="{-torque} {torque}"/>'
        )

    probe = geo.get("thermoprobe") or {}
    probe_at = probe.get("body_center_m") or [0.5 * full_x, 0.0, pz]
    ax, ay, az = (float(v) for v in probe_at)

    xml = f"""<?xml version="1.0"?>
<!-- CAD morphology. Visual mesh is contype 0. Drive cylinders sit on the
     lower idler centers from the STEP tessellation. Mass and torque are ASSUMED.
     The probe hinge is the existing one-DOF interface, not the leadscrew. -->
<mujoco model="{robot.name}">
  <compiler angle="radian" autolimits="true" meshdir="{mesh.parent}"/>
  <option timestep="{robot.physics_dt}" gravity="0 0 {-robot.gravity}"
          integrator="implicitfast" cone="pyramidal"
          iterations="60" solver="Newton"/>
  <default>
    <geom solref="0.008 1" solimp="0.9 0.95 0.001"/>
  </default>
  <asset>
    <mesh name="cad_visual" file="{mesh.name}"/>
  </asset>
  <worldbody>
    <light name="sun" pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
{world_geoms}
    <body name="root" pos="0 0 {spawn_z}">
      <freejoint name="root_free"/>
      <inertial pos="0 0 0" mass="{robot.chassis_mass}" diaginertia="{ixx:.8e} {iyy:.8e} {izz:.8e}"/>
      <geom name="cad_visual" type="mesh" mesh="cad_visual" rgba="0.72 0.74 0.76 1"
            contype="0" conaffinity="0" group="1"/>
      <geom name="chassis" type="box" pos="{px:.5f} {py:.5f} {pz:.5f}"
            size="{hx:.5f} {hy:.5f} {hz:.5f}" rgba="0.2 0.45 0.75 0.15"
            friction="{_friction_attr(robot.floor_friction)}"
            contype="2" conaffinity="1" group="3"/>
      <site name="imu" pos="{px:.5f} {py:.5f} {pz + hz:.5f}" size="0.008"/>
      <site name="lidar" pos="{0.5 * full_x - 0.02:.5f} 0 {pz + hz + 0.04:.5f}" size="0.008"/>
      <body name="probe_arm" pos="{ax:.5f} {ay:.5f} {az:.5f}">
        <joint name="probe_hinge" type="hinge" axis="0 1 0" range="0 1.2" damping="0.02"/>
        <geom name="probe_boom" type="capsule" fromto="0 0 0 0.04 0 0" size="0.004"
              rgba="0.15 0.85 0.35 1" group="3" contype="0" conaffinity="0"/>
        <site name="probe" pos="0.04 0 0" size="0.006" rgba="0.1 1.0 0.2 1"/>
      </body>
{''.join(rollers)}
    </body>
  </worldbody>
  <actuator>
{chr(10).join(motors)}
    <position name="probe_hinge" joint="probe_hinge" kp="2.4" dampratio="1" ctrlrange="0 1.2"/>
  </actuator>
</mujoco>
"""
    return xml
