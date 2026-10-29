"""Parametric four-pod articulated morphology. Not imported CAD.

The 9 Oct 2026 prompt states four modules that pitch independently, with
propulsion separate from articulation. No STL/OBJ/STEP/URDF of that assembly
is on disk. Numbers below are ASSUMED at the existing small-proxy scale
(wheel_radius 0.03 m). This is a roller/contact approximation of a tread,
not a moving belt, not capstan routing, and not a digital twin.
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

# Front-left, front-right, rear-left, rear-right. TeleCmd mix uses the same
# corner names as the wheeled proxy (FL, FR, RL, RR). Hardware helper order
# remains FL, RL, FR, RR and is not sent to these joints.
POD_CORNERS: tuple[tuple[str, str, int, int], ...] = (
    ("fl", "front_left_maxon", 1, 1),
    ("fr", "front_right_maxon", 1, -1),
    ("rl", "rear_left_maxon", -1, 1),
    ("rr", "rear_right_maxon", -1, -1),
)
ART_JOINTS: tuple[str, ...] = ("art_fl", "art_fr", "art_rl", "art_rr")
# Three rollers per pod along +X. Index 1 (mid) keeps the legacy Maxon name
# so wheel-odom and VelocityController still see four drive channels.
ROLLER_OFFSETS_M: tuple[float, ...] = (-0.045, 0.0, 0.045)
ART_RANGE_RAD = 0.70  # ASSUMED pitch limits; CAD UNKNOWN
POD_LENGTH_M = 0.12  # ASSUMED elongated module
POD_WIDTH_M = 0.018
POD_HEIGHT_M = 0.022
POD_MASS_KG = 0.035
KIND = "four_pod_pitch_hinge_roller_approx"


def art_joint_name(corner: str) -> str:
    return f"art_{corner}"


def roller_joint_name(primary: str, index: int) -> str:
    if index == 1:
        return primary
    suffix = ("rear", "mid", "front")[index]
    return f"{primary}_{suffix}"


def drive_siblings(primary: str) -> tuple[str, ...]:
    return tuple(roller_joint_name(primary, i) for i in range(len(ROLLER_OFFSETS_M)))


def all_drive_joints() -> tuple[str, ...]:
    names: list[str] = []
    for _c, primary, _sx, _sy in POD_CORNERS:
        names.extend(drive_siblings(primary))
    return tuple(names)


def build_articulated_mjcf(robot: RobotParams, world: WorldSpec | None = None) -> str:
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
    torque = robot.max_wheel_torque / float(len(ROLLER_OFFSETS_M))
    art_kp = 1.2  # ASSUMED N·m/rad; finite, not an infinitely stiff motor

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
            rgb = (1.0, 0.45, 0.05) if sx > 0 else (0.85, 0.12, 0.12)
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
                f'    <motor name="{jname}" joint="{jname}" '
                f'gear="1" ctrlrange="{-torque} {torque}"/>'
            )
        pods_xml.append(
            f"""
      <body name="pod_{corner}" pos="{px:.6f} {py:.6f} 0">
        <joint name="{art}" type="hinge" axis="0 1 0"
               range="{-ART_RANGE_RAD} {ART_RANGE_RAD}" damping="0.008" armature="0.0002"/>
        <inertial pos="0 0 0" mass="{POD_MASS_KG}"
                  diaginertia="{pod_i[0]:.8e} {pod_i[1]:.8e} {pod_i[2]:.8e}"/>
        <geom name="pod_{corner}_shell" type="box"
              size="{0.5 * POD_LENGTH_M:.4f} {0.5 * POD_WIDTH_M:.4f} {0.5 * POD_HEIGHT_M:.4f}"
              pos="0 0 {0.5 * POD_HEIGHT_M + 0.004:.4f}"
              rgba="0.22 0.28 0.24 1" group="3" contype="0" conaffinity="0"/>
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
<!-- EMBR four-pod PITCH-HINGE prototype. CAD meshes were not supplied.
     Contact is three rigid rollers per pod (not a belt). Capstan UNKNOWN.
     morphology=articulated. Do not treat as a calibrated twin. -->
<mujoco model="embr_four_pod_roller_approx">
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
            rgba="0.55 0.72 0.62 1" contype="2" conaffinity="1" group="1"
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
