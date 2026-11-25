"""Primitive-shape URDF mock bot with four corner arms.

The URDF (bundled/urdf/embr_primitive.urdf) is the robot description:
links, primitive shapes, joints, and limits. MuJoCo's own URDF importer reads
it. This module then adds what URDF cannot say: the free joint, sensor sites,
actuators, and the sim's contact groups. The world is
built from the same geoms as every other body and the robot is attached to it.
"""

from __future__ import annotations

from dataclasses import replace
from importlib.resources import files
from pathlib import Path

import mujoco
import numpy as np

from embr_local_sim.arm import ARM_JOINTS
from embr_local_sim.model import RobotParams, WorldSpec, _wheel_diag_inertia, _world_geoms

MORPHOLOGY = "urdf"
DRIVE_JOINTS = ("front_left_maxon", "front_right_maxon", "rear_left_maxon", "rear_right_maxon")
ROLLER_BODIES = tuple(name.replace("_maxon", "") for name in DRIVE_JOINTS)
# Position servo on each arm hinge. A level arm carries about 2.6 N·m of body
# weight; this gain sags it under a degree. ASSUMED, not an ODrive tune.
ARM_KP = 300.0
ARM_FORCE_NM = 30.0
# Scan plane about 0.205 m off the ground with the arms level, over the cylinder tops.
LIDAR_SITE = (0.30, 0.0, 0.116)
IMU_SITE = (0.0, 0.0, 0.04)
PROBE_TIP = (0.04, 0.0, 0.0)
CHASSIS_SIZE = (0.706, 0.508, 0.08)
# Cylinder centres: 0.279 m arm line + 0.055 m outboard.
WHEEL_SEPARATION = 0.668


def urdf_path() -> Path:
    return Path(str(files("embr_local_sim") / "bundled" / "urdf" / "embr_primitive.urdf"))


def apply_urdf_params(robot: RobotParams) -> RobotParams:
    """CAD-scale drive and sensors, with the mock bot's own track and chassis."""
    from embr_local_sim.cad_model import apply_cad_params

    cad = apply_cad_params(robot)
    return cad.with_overrides(
        name="embr_mock",
        morphology=MORPHOLOGY,
        wheel_separation=WHEEL_SEPARATION,
        chassis_size=CHASSIS_SIZE,
        chassis_mass=4.0,
    )


def urdf_body_spec():
    from embr_local_sim.cad_model import cad_body_spec

    return replace(
        cad_body_spec(),
        name=MORPHOLOGY,
        fidelity="Primitive-shape mock bot: PR 88 plate footprint, four corner arms. Mass, torque, and limits assumed.",
    )


def _robot_spec(robot: RobotParams) -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(urdf_path()))
    root = spec.body("root")
    root.add_freejoint(name="root_free")
    radius = robot.wheel_radius
    root.add_site(name="imu", pos=IMU_SITE, size=[0.008, 0, 0])
    root.add_site(name="lidar", pos=LIDAR_SITE, size=[0.008, 0, 0])
    spec.body("probe").add_site(name="probe", pos=PROBE_TIP, size=[0.006, 0, 0], rgba=[0.1, 1.0, 0.2, 1])

    wheel_inertia = _wheel_diag_inertia(robot.wheel_mass, radius, robot.wheel_width, robot.rotor_inertia_scale)
    for body in ROLLER_BODIES:
        b = spec.body(body)
        b.mass = robot.wheel_mass
        b.fullinertia = [*wheel_inertia, 0.0, 0.0, 0.0]
        b.explicitinertial = True
        spec.joint(f"{body}_maxon").damping = [robot.wheel_damping, 0.0, 0.0]
    return spec


def _tag_geoms(spec: mujoco.MjSpec, robot: RobotParams) -> None:
    """Visual shapes: group 1, no contact (the lidar self-check sees them).
    Collision shapes: group 3, the sim's robot contact class."""
    for geom in spec.geoms:
        if geom.contype == 0 and geom.conaffinity == 0:
            geom.group = 1
            continue
        geom.group = 3
        geom.contype = 2
        geom.conaffinity = 1
        geom.condim = 3
        mu = robot.wheel_friction if geom.name in ROLLER_BODIES else robot.floor_friction
        geom.friction = list(mu)


def build_urdf_mjcf(robot: RobotParams, world: WorldSpec | None = None) -> str:
    world = world or WorldSpec()
    spec = _robot_spec(robot)
    spec.body("root").pos = [0.0, 0.0, robot.wheel_radius + 0.004]
    spec.option.timestep = robot.physics_dt
    spec.option.gravity = [0.0, 0.0, -robot.gravity]
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_PYRAMIDAL
    spec.option.iterations = 60
    spec.option.solver = mujoco.mjtSolver.mjSOL_NEWTON
    spec.default.geom.solref = [0.008, 1.0]
    spec.default.geom.solimp = [0.9, 0.95, 0.001, 0.5, 2.0]
    _tag_geoms(spec, robot)

    torque = robot.max_wheel_torque
    for joint in DRIVE_JOINTS:
        spec.add_actuator(
            name=joint,
            target=joint,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            ctrlrange=[-torque, torque],
            gear=[1, 0, 0, 0, 0, 0],
        )
    for joint in ARM_JOINTS:
        lo, hi = spec.joint(joint).range
        act = spec.add_actuator(
            name=joint,
            target=joint,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            ctrlrange=[lo, hi],
            forcerange=[-ARM_FORCE_NM, ARM_FORCE_NM],
        )
        act.set_to_position(kp=ARM_KP, dampratio=1.0)
    act = spec.add_actuator(name="probe_hinge", target="probe_hinge", trntype=mujoco.mjtTrn.mjTRN_JOINT, ctrlrange=[0.0, 1.2])
    act.set_to_position(kp=2.4, dampratio=1.0)
    spec.compile()
    robot_xml = spec.to_xml()
    # The world is the same XML chunk every other body uses. It needs no assets.
    world_chunk = (
        '    <light name="sun" pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>\n' + _world_geoms(robot, world) + "\n"
    )
    marker = "<worldbody>\n"
    if robot_xml.count(marker) != 1:
        raise RuntimeError("Unexpected MJCF export: no single <worldbody> to add the world to.")
    return robot_xml.replace(marker, marker + world_chunk)


def visual_geoms(model: mujoco.MjModel) -> list[dict]:
    """Robot visual primitives, for the browser viewer. Pose is local to the body."""
    root = int(model.body("root").id)
    out = []
    for gid in range(model.ngeom):
        bid = int(model.geom_bodyid[gid])
        if int(model.body_rootid[bid]) != root or int(model.geom_group[gid]) != 1:
            continue
        out.append(
            {
                "body": model.body(bid).name,
                "name": model.geom(gid).name or f"geom_{gid}",
                "type": int(model.geom_type[gid]),
                "size": np.array(model.geom_size[gid], dtype=np.float64),
                "pos": np.array(model.geom_pos[gid], dtype=np.float64),
                "quat": np.array(model.geom_quat[gid], dtype=np.float64),
                "rgb": tuple(int(255 * c) for c in model.geom_rgba[gid][:3]),
            }
        )
    return out
