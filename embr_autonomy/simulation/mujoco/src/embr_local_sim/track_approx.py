"""Distributed-contact track approximation. Not a belt and not imported CAD.

MuJoCo has no sprocket-belt constraint. Each module is a pitch hinge plus
five driven rollers. Tangential force is friction at those roller contacts.
A dark tread geom is visual only (contype 0) and does not push the robot.

Scale matches the existing 0.03 m roller radius. Pod length 0.15 m is assumed.
A CAD filename states a 12 mm × 100 mm idler shaft; that is a part name, not a
measured assembly. Full SolidWorks mates were not imported.

Label: Articulated EMBR approximation — hardware validation pending.
"""

from __future__ import annotations

from embr_local_sim.model import (
    RobotParams,
    WorldSpec,
    _box_diag_inertia,
    _friction_attr,
    _wheel_diag_inertia,
    _world_geoms,
)

POD_CORNERS: tuple[tuple[str, str, int, int], ...] = (
    ("fl", "front_left_maxon", 1, 1),
    ("fr", "front_right_maxon", 1, -1),
    ("rl", "rear_left_maxon", -1, 1),
    ("rr", "rear_right_maxon", -1, -1),
)
ART_JOINTS: tuple[str, ...] = ("art_fl", "art_fr", "art_rl", "art_rr")
# Five rollers. Index 2 keeps the Maxon joint name used by the wheel-speed loop.
ROLLER_OFFSETS_M: tuple[float, ...] = (-0.056, -0.028, 0.0, 0.028, 0.056)
CENTER_ROLLER = 2
N_ROLLERS = len(ROLLER_OFFSETS_M)
ART_RANGE_RAD = 0.55  # ASSUMED. CAD joint limits were not read.
# Straight drive stays at RobotParams.max_wheel_torque (0.020 N·m per corner).
# That cap cannot yaw while both sides still push forward: the sides have to
# push against each other, which stops the robot. Turning adds this much
# extra torque on the outside corner only. ASSUMED, so W+D follows an arc.
TURN_OUTSIDE_EXTRA_NM = 0.060
TURN_CORNER_CAP_NM = 0.080
POD_LENGTH_M = 0.15  # ASSUMED. Same order as the 100 mm idler-shaft filename.
POD_WIDTH_M = 0.024
POD_HEIGHT_M = 0.018
POD_MASS_KG = 0.04  # ASSUMED. Decorative CAD mass was not used.
KIND = "distributed_contact_track_approx"
FIDELITY = "Articulated EMBR approximation — hardware validation pending."


def art_joint_name(corner: str) -> str:
    return f"art_{corner}"


def roller_joint_name(primary: str, index: int) -> str:
    if index == CENTER_ROLLER:
        return primary
    return f"{primary}_r{index}"


def drive_siblings(primary: str) -> tuple[str, ...]:
    return tuple(roller_joint_name(primary, i) for i in range(N_ROLLERS))


def all_drive_joints() -> tuple[str, ...]:
    names: list[str] = []
    for _corner, primary, _sx, _sy in POD_CORNERS:
        names.extend(drive_siblings(primary))
    return tuple(names)


def build_track_mjcf(robot: RobotParams, world: WorldSpec | None = None) -> str:
    world = world or WorldSpec()
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
    pod_i = _box_diag_inertia(POD_MASS_KG, POD_LENGTH_M, POD_WIDTH_M, POD_HEIGHT_M)
    torque = TURN_CORNER_CAP_NM / float(N_ROLLERS)
    art_kp = 1.2  # ASSUMED N·m/rad

    pods_xml: list[str] = []
    motor_xml: list[str] = []
    art_xml: list[str] = []
    for corner, primary, sx, sy in POD_CORNERS:
        px, py = sx * wb, sy * th
        art = art_joint_name(corner)
        rollers = []
        for i, ox in enumerate(ROLLER_OFFSETS_M):
            jname = roller_joint_name(primary, i)
            bname = f"roller_{corner}_{i}"
            rgb = (0.75, 0.32, 0.08) if sx > 0 else (0.55, 0.12, 0.10)
            rollers.append(
                f"""
        <body name="{bname}" pos="{ox:.4f} 0 0">
          <joint name="{jname}" type="hinge" axis="0 1 0" damping="{robot.wheel_damping}"/>
          <inertial pos="0 0 0" mass="{robot.wheel_mass}" diaginertia="{wi[0]:.8e} {wi[1]:.8e} {wi[2]:.8e}"/>
          <geom name="{bname}" type="cylinder" size="{robot.wheel_radius} {hw}"
                euler="1.57079632679 0 0" rgba="{rgb[0]} {rgb[1]} {rgb[2]} 1"
                friction="{_friction_attr(robot.wheel_friction)}"
                condim="3" contype="2" conaffinity="1" group="1"/>
        </body>"""
            )
            motor_xml.append(
                f'    <motor name="{jname}" joint="{jname}" gear="1" '
                f'ctrlrange="{-torque} {torque}"/>'
            )
        pods_xml.append(
            f"""
      <body name="pod_{corner}" pos="{px:.6f} {py:.6f} 0">
        <joint name="{art}" type="hinge" axis="0 1 0"
               range="{-ART_RANGE_RAD} {ART_RANGE_RAD}" damping="0.01" armature="0.0002"/>
        <inertial pos="0 0 0" mass="{POD_MASS_KG}"
                  diaginertia="{pod_i[0]:.8e} {pod_i[1]:.8e} {pod_i[2]:.8e}"/>
        <geom name="pod_{corner}_shell" type="box"
              size="{0.5 * POD_LENGTH_M:.4f} {0.5 * POD_WIDTH_M:.4f} {0.5 * POD_HEIGHT_M:.4f}"
              pos="0 0 {0.012:.4f}"
              rgba="0.18 0.20 0.18 1" group="1" contype="0" conaffinity="0"/>
        <geom name="pod_{corner}_tread_visual" type="box"
              size="{0.5 * POD_LENGTH_M - 0.01:.4f} {0.5 * POD_WIDTH_M:.4f} 0.004"
              pos="0 0 -0.012"
              rgba="0.08 0.08 0.08 1" group="1" contype="0" conaffinity="0"/>
{''.join(rollers)}
      </body>"""
        )
        art_xml.append(
            f'    <position name="{art}" joint="{art}" kp="{art_kp}" dampratio="1" '
            f'ctrlrange="{-ART_RANGE_RAD} {ART_RANGE_RAD}"/>'
        )

    world_geoms = _world_geoms(robot, world)
    from embr_local_sim.assets import load_manifest, mjcf_asset_compiler

    asset_xml = mjcf_asset_compiler(load_manifest())
    xml = f"""<?xml version="1.0"?>
<!-- {FIDELITY}
     Contact: {N_ROLLERS} rollers per module. Tread geoms are visual only.
     Probe hinge is an unconfirmed placeholder. CAD has leadscrew.SLDPRT and
     thermoprobe.SLDPRT; those solids were not imported.
     Capstan parts exist in CAD and are not routed here. -->
<mujoco model="embr_track_roller_approx">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{robot.physics_dt}" gravity="0 0 {-robot.gravity}"
          integrator="implicitfast" cone="pyramidal"
          iterations="60" solver="Newton"/>
  <default>
    <geom solref="0.004 1" solimp="0.9 0.95 0.001"/>
  </default>
  <visual>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.5 0.5 0.5"/>
  </visual>
{asset_xml}  <worldbody>
    <light name="sun" pos="0.4 0.2 2.2" dir="0 0 -1" diffuse="0.85 0.85 0.8"/>
{world_geoms}
    <body name="root" pos="0 0 {spawn_z}">
      <freejoint name="root_free"/>
      <inertial pos="0 0 {chassis_z}" mass="{robot.chassis_mass}"
                diaginertia="{ixx:.8e} {iyy:.8e} {izz:.8e}"/>
      <geom name="chassis" type="box" size="{hx} {hy} {hz}" pos="0 0 {chassis_z + 0.01}"
            rgba="0.45 0.62 0.48 1" contype="2" conaffinity="1" group="1"
            friction="{_friction_attr(robot.floor_friction)}"/>
      <site name="imu" pos="0 0 {chassis_z}" size="0.005"/>
      <site name="lidar" pos="{hx:.4f} 0 0.02" size="0.004"/>
      <site name="com_marker" pos="0 0 {chassis_z}" size="0.006" rgba="1 0.2 0.2 1"/>
      <body name="probe_arm" pos="{hx:.4f} 0 {chassis_z + 0.01}">
        <joint name="probe_hinge" type="hinge" axis="0 1 0" range="0 1.2" damping="0.003"/>
        <geom name="probe_boom" type="capsule" fromto="0 0 0 0.045 0 -0.01" size="0.003"
              rgba="0.75 0.75 0.2 1" group="1" contype="0" conaffinity="0"/>
        <site name="probe" pos="0.045 0 -0.01" size="0.004" rgba="0.9 0.9 0.2 1"/>
      </body>
{''.join(pods_xml)}
    </body>
  </worldbody>
  <actuator>
{chr(10).join(motor_xml)}
{chr(10).join(art_xml)}
    <position name="probe_hinge" joint="probe_hinge" kp="2.4" dampratio="1" ctrlrange="0 1.2"/>
  </actuator>
</mujoco>
"""
    return xml
