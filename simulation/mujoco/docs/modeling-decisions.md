# EMBR modeling decisions

Simulation implementation verified; physical accuracy unvalidated.

The default launch is unchanged: four-wheel skid-steer proxy, basic scene.
`--morphology tracks` and `--scene course` / `--scene grove` are the research
additions. Nothing here is a hardware measurement.

## What was readable

The local archive is native SolidWorks: 143 files, about 269 MB, flat
directory. The top assembly is `final FULL CAD.SLDASM` (about 47 MB). This
Mac cannot open those binaries. The only STEP files are motor and controller
parts (`MOTOR_BLDC_EAGLE_POWER_90KV.STEP`, `783734_ESCON2_Compact_60-30.stp`,
`EC-i52 420W.stp`). There is no full-robot STEP, STL, or OBJ, and no drawing.

Filenames that guided the *kind* of mechanism, not its dimensions:

| Filename evidence | Classification | Use |
|---|---|---|
| `final dtTrack_for_boston.SLDPRT`, sprocket and idler parts | filename-stated | EMBR has tracks and sprockets. Not imported. |
| `DT_TRACK_IDLER_WHEEL_SHAFT_12MMDIA_100MM` | filename-stated | Order-of-magnitude context for a short pod. Not a measured length. |
| `final leadscrew.SLDPRT`, `final the rail.SLDPRT`, `final thermoprobe.SLDPRT`, `final probe final3.SLDASM` | filename-stated | Probe is a rail and leadscrew assembly in CAD. The sim hinge is not that mechanism. |
| filenames containing pitch or pod | none | The four pitch pods are a sim proposal. CAD does not name them. |
| `final AA_CAPSTAN_STRUCTURAL_SHAFT COUPLE_gala.SLDPRT` | filename-stated | Capstan parts exist. They are not routed in the model. |

Historical photos were not used as dimensions.

## Track contact

MuJoCo documents bodies, joints, geoms, and actuators
([modeling](https://mujoco.readthedocs.io/en/stable/modeling.html),
[computation](https://mujoco.readthedocs.io/en/stable/computation/)).
It does not provide a sprocket-belt constraint.
[mujoco#3220](https://github.com/google-deepmind/mujoco/issues/3220) is a
request, not a feature. Menagerie and dm_control do not ship an EMBR-like
track. Project Chrono’s track shoes are a reference for what a shoe model
contains. Chrono is not linked in, and it is not a second runtime.

| Option | What propulsion is | Decision |
|---|---|---|
| Existing wheel proxy | Friction at four driven cylinders | Kept as the selectable baseline. |
| Distributed rollers | Friction at five driven cylinders per pitch module (`condim` 3). Straight-drive torque per roller is `max_wheel_torque / 5`. While turning, the outside corner may use up to 0.080 N·m total, assumed, so the robot can arc without reversing the inside track. The five hinges are independent. They share that corner torque. They are not locked by a fixed tendon, because a tendon would force identical angles when one roller is on an obstacle and another is on flat ground. | Selected. Tangential force is contact friction opposing slip at the spinning rollers. Rolling friction is not the drive. |
| Explicit belt segments | Many bodies, pin joints, and sprocket engagement. Equality loop joints are slower and less accurate for this. | Rejected for this pass. No supported MuJoCo primitive, and a full second robot was not justified. |
| Static track hull with an internal wheel | The hull does not circulate | Rejected. A visual tread with `contype="0"` is drawn so the shape is visible and cannot be mistaken for the contact model. |

Pod length 0.15 m, width 0.024 m, height 0.018 m, mass 0.04 kg, and pitch
range ±0.55 rad are **assumed**. Roller radius 0.03 m, track separation, and
wheelbase are the existing proxy values (team URDF / simple controller for
the wheel geometry; chassis mass 0.20 kg assumed). Inertia uses those box
and cylinder formulas. It is not computed from CAD materials.

## Obstacles and forest

Course cases are rigid. A stop, a failed crossing, or extra roll is a valid
result. Sizes are in metres and in multiples of the 0.03 m roller radius.
Chassis clearance on the proxy is about 0.016 m.

| Case | Size | Intent |
|---|---|---|
| flat | — | Both morphologies should roll forward. |
| branch | radius 0.010 m (0.33 R), secured | Wheel proxy crests it. The roller pods contact it and stall. A cylinder cannot wrap the branch the way a belt would. |
| log | radius 0.028 m (0.93 R), secured | Both morphologies stop against it. |
| oblique | radius 0.012 m, yaw 0.70 rad | Not aligned with the axle. |
| roll | board top 0.008 m, +y side only | Wheel proxy rolls onto it. Roller pods yaw and stall. |
| slope | 8° ramp and 10° cross-slope | Rigid ground, not soil. |
| loose | radius 0.010 m, mass 0.04 kg assumed | Free rigid body. It rests on the floor and can be pushed. |

Static obstacles use the floor's contact bits because they are welded to the
world, so they only need to touch the robot. A free body with those same bits
falls through the floor. Loose logs and movable stones use contact bits that
match both the floor and the rollers (`contype` 3, `conaffinity` 3).

Grove trunks are collision cylinders about 0.08–0.14 m radius, placed outside
a clearing. Shrubs use the same contact bits as the trunks, so the robot
stops against them. Canopy, flames, and smoke stay visual only. Heat markers
are assumed °C exclusion cues. They are not combustion, spread, smoke
transport, or heat damage.

Straight drive uses the soft wheel-speed loop, then splits that torque across
the five rollers, capped at 0.020 N·m per corner. A or D alone pushes the
two sides in opposite directions so the robot yaws in place instead of
driving forward. With W held, the inside corner keeps that drive and the
outside corner gets up to 0.060 N·m extra forward (0.080 N·m cap, assumed).
With S held, A still bends the path left and D still bends it right, so the
yaw rate is opposite the forward case. W and S alone are unchanged.

## Balance overlay

The optional overlay draws the subtree center of mass, a gravity-direction
segment, and contact-normal arrows scaled at 0.002 m per newton from
`mj_contactForce`. That drawing is an observer diagnostic. The mission does
not read it unless ORACLE is on. It is not a support-polygon guarantee.
Chassis tilt while crossing an obstacle is expected.

## Controller

Manual module sliders and the existing posture presets command the pitch
actuators. Automatic articulation remains the previous conservative
supervisor. No policy from another robot was copied.

TeleCmd `velocity` and `turn` stay normalized on [-1, 1]. Positive turn is
right (`wz = -turn * max_wz` in `tele_cmd.py`). GUI buttons stay hold-to-drive.
Reset remains the only root-pose write.

## Missing data

- Assembly dimensions, joint axes, and joint limits from the current CAD revision.
- Masses and centers of mass with known materials.
- Sprocket ratio, belt pretension, and grouser geometry.
- Motor torque, current, and speed limits measured on this robot.
- Probe travel (leadscrew pitch, carriage stroke).
- Obstacle and surface measurements from a hardware trial.

## Hardware protocol (not run)

Measure the obstacle and the surface. Record the initial pose and the
commands. Capture a side-view video and encoder or IMU logs if they exist.
Repeat the trial. Compare tilt, displacement, time, and outcome. Keep a
separate set of trials for validation; do not use every run to retune the
assumed masses.

## Sources

- https://github.com/HEATRobotics/EMBR-AutoBot
- https://mujoco.readthedocs.io/en/stable/modeling.html
- https://mujoco.readthedocs.io/en/stable/computation/
- https://github.com/google-deepmind/mujoco
- https://github.com/google-deepmind/mujoco_menagerie
- https://github.com/google-deepmind/mujoco/issues/3220
- https://github.com/google-deepmind/dm_control
- https://viser.studio/main/
- https://github.com/projectchrono/chrono
- https://engineering.ok.ubc.ca/2024/05/29/soe-feature-learn-more-about-the-heat-robotics-team-and-their-award-winning-embr-project/

Local pins already in this package: MuJoCo 3.15.0, Viser 1.1.1, NumPy 2.5.4.
No new dependency was added.
