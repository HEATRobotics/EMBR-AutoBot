# EMBR MuJoCo simulation

A laptop simulator for driving a **stand-in robot** before hardware tests.
It is not a calibrated model of the real EMBR.

## What this is

This package runs a small robot in [MuJoCo](https://mujoco.org/). MuJoCo steps
contact physics: wheel torques, collisions, and gravity. It does not talk to
motors or CAN hardware.

[Viser](https://viser.studio/) opens a view in your browser and draws the poses
MuJoCo already computed. The browser is only a window. Closing the tab does not
replace this Python process, and there is no static site to host.

**What you are driving**

| Model | What it is |
| --- | --- |
| `tracks`, opened by `embr-local-sim launch` | Five driven rollers on each pitch module, in the grove. Trunks and shrubs stop the robot. Not a belt, not CAD, and not the full EMBR. |
| `--morphology wheels` | Four-wheel skid-steer **proxy**. Wheel radius 0.03 m comes from the simple ROS controller. |
| `--morphology articulated` | Four pitch-hinge pods with three rollers each. A labeled approximation. Not tracks and not a belt. |
| `--morphology cad` | Visual mesh from the local Onshape STEP. Contact is the base plate plus four idler cylinders, not a belt. Mass and torque are assumed. Scenes are the proxy layouts stretched by plate length / 0.15 m. |

The wheel, articulated, and track models are still MuJoCo primitives. `cad` is the only morphology that loads a mesh, and that mesh does not collide.

ROS 2 is **not** required and **not** connected. The ROS workspace stays in
`embr_phys/ros2_ws`. RViz mock hardware is a separate demo. A ROS adapter file
in this package records the intended topic names; it does not start a node.

## Quick start

Tested here: **macOS, Python 3.12.14**. The package requires **Python 3.12 or
3.13** because NumPy 2.5, which `pip` resolves from the version range, requires
3.12. Python 3.13 was not run. Windows was not run.

Clone the branch and install from the repository root:

```bash
git clone https://github.com/HEATRobotics/EMBR-AutoBot.git
cd EMBR-AutoBot
git checkout autonomy/EMBR-19/mujoco-baseline
cd embr_autonomy/simulation/mujoco
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -e '.[test]'
embr-local-sim launch
```

That command opens the grove and the track rollers: trunks, shrubs, and the
robot with a motor on each roller. Open the URL the terminal prints. The
default is [http://127.0.0.1:43147](http://127.0.0.1:43147). If that port is
taken, the program says so; pick another with `--port 43148`.

`requirements.txt` pins the runtime versions that were run; its header says where.
`requirements-dev.txt` adds the pytest pin. The ranges in `pyproject.toml`
are what pip is allowed to resolve; the pin files are the set that was run.

Leave that terminal running. Stop with **Ctrl-C**. If the terminal is
interactive, **q** also quits.

## Three hotspot rooms

One command opens the rooms in Chrome. Room 1 is a clear floor. Room 2 has two blue boxes on the way to the spot. Room 3 starts in one corner and hides the spot in the other, behind two rows of trunks. Each row has two openings. Nothing drives until you click **Go to the heat source**, which sits above Drive. The status line says `vfh`. The body is the CAD plate. This does not train a model.

```bash
cd embr_autonomy/simulation/mujoco
python3.12 -m venv .venv
.venv/bin/pip install -e '.[test]'
.venv/bin/python -m embr_local_sim rooms
```

Room 1 is [http://127.0.0.1:43147](http://127.0.0.1:43147). Room 2 is [http://127.0.0.1:43148](http://127.0.0.1:43148). Room 3 is [http://127.0.0.1:43149](http://127.0.0.1:43149). Leave the terminal open. Ctrl-C stops all three.

## Autonomy test course

Simulation-only slice of the Autonomy Test Deliverable: no hardware, no ROS. A 22 m × 9 m walled field with a centre divider and two routes to an assumed 80 °C smolder past the divider. The left route is blocked by a 15° hump (red). The right route has a 3° and a 6° hump (orange). The CAD body starts on the centre line and the VFH histogram has to pick the route it can drive.

```bash
.venv/bin/python -m embr_local_sim launch --scene autonomy-course --histogram
.venv/bin/python -m pytest tests/test_autonomy_course.py tests/test_sensor_overlay.py -q
```

Click **Go to the heat source**. The **Sensor views** panel shows:

- **LiDAR beams**: cyan hit something, grey reached max range, orange hit the robot's own body.
- **VFH sectors**: one spoke per sector (long and green is open, short and red is blocked). Blue is the goal heading, white the heading VFH picked. The bar image is the same histogram unrolled, robot's left on the left.
- **Thermal bins**: the 9-bin thermal "camera" as a coloured strip and as points where each bin samples.

**Lidar (CAD body).** A 360°, 60-ray puck on a mast at the front-centre of the top deck, scan plane about 0.205 m off the ground. A second ray against the CAD mesh flags beams the robot's own track tops block (about ±90° and ±140°). Returns that land on the ground plane are dropped, standing in for 3D-lidar ground segmentation. The planner reads the whole ring minus blocked beams, so VFH can pick a heading to the side or behind. It keeps 8° sectors, so 45 sectors around the full circle. The jam check reads only the straight-ahead ray, because an angled ray grazing a wall the robot drives along keeps the same range while the robot moves.

**Ramps.** The CAD body's climb limit with its assumed 0.5 N·m rollers was measured at 7.5° (7° crosses, 8° stalls). Humps under that limit sit in a geom group the flat fan does not see, so they read as floor. Steeper humps are visible and read as walls. This is a stand-in for slope-based traversability from a 3D lidar. Geometry, torque and temperatures are assumed.

## URDF mock bot with arm actuation

`--morphology urdf` loads `src/embr_local_sim/bundled/urdf/embr_primitive.urdf`, a standard URDF built only from boxes and cylinders:

- **Chassis:** a box with the footprint of the PR 88 mock bot plate (0.706 m × 0.508 m × 0.08 m).
- **Four arms:** 0.243 m boxes hinged at the chassis corners, each with its drive cylinder (r 0.089 m) on the far end. Level arms put the cylinders at the PR 88 wheelbase (1.192 m). Each arm swings 90° up or down.
- Drive joints keep the usual names (`front_left_maxon`, …), so the planner, odometry and courses work unchanged.

Arm actuation follows the Autonomy Test Deliverable: `height`, `pitch`, `roll`, each normalised to [-1, 1]. Lowering one cylinder by *d* lifts that corner by *d*, so each corner's lift becomes one arm angle:

```
lift_i  = L * clip(height - front_i * pitch + left_i * roll, -1, 1)
angle_i = -asin(lift_i / L)          # + is the cylinder going up
```

`height` +1 is all arms 90° down (chassis about 0.33 m up), −1 is all arms up (chassis on its belly). Pitch and roll follow ROS REP 103: + pitch is nose down, + roll is left side up. Setpoints ramp at 1 rad/s, because a servo jumping straight to a far angle tips the chassis.

```bash
.venv/bin/python -m embr_local_sim launch --scene basic --morphology urdf
.venv/bin/python -m embr_local_sim launch --scene autonomy-course --histogram --morphology urdf
```

While driving with W A S D, hold **I** / **K** for pitch (nose down / up), **J** / **L** for roll (left / right side down) and **P** / **;** for height (up / down). The keys move the sliders at 0.6 per second, and pressing one turns auto posture off. The **Arm** panel has the three sliders and shows `ArmStatus` (IDLE 0, MOVING 1, REACHED 2, FAULT 3) plus the four arm angles. From Python: `sim.set_arm(height, pitch, roll)` and `sim.arm_status()`.

**Auto posture.** While a mission drives, `auto_posture.py` owns the arms (toggle: **Auto posture in missions**). It reads only deployable signals: IMU roll and pitch, wheel encoder speed, and the mission's own jam reaction.

- **Level:** pitch and roll commands are integrated against the measured tilt (1° deadband).
- **Slope:** past 2.5° of tilt it lifts to height 0.35 for belly clearance at crests, and holds that for 3 s after.
- **Unstick:** when the mission backs up from a jam, or the encoders turn under a quarter of the commanded speed for 1 s, it lifts to 0.8 for 6 s.

On flat ground it stays at height 0, because a taller body raises the flat lidar over the hot rooms' 0.22 m walls. On a 7° hump with an 11 cm crest, the level body high-centres. With auto posture it clears (`tests/test_auto_posture.py`). Sharp steps taller than the belly need a step-climbing sequence, which this does not do yet.

MuJoCo reads the URDF with its own importer. `urdf_model.py` adds what URDF cannot express: the free joint, sensor sites, actuators and contact groups. Mass, gains and limits are assumed.

## Standalone Maxon motor twin

`embr-local-sim motor` opens a single EC-i52 motor with an orange shaft
pointer, fixed housing and synthetic position/speed feedback. Run, pause,
reset or replay the 12-second sequence from the browser. No hardware or ROS
is required. This imposes telemetry state in MuJoCo; it does not predict
motor dynamics. Live ESCON feedback is tracked separately in issue #74.

```bash
embr-local-sim motor
embr-local-sim motor --headless --out embr-sim-runs/motor
```

See [motor setup, input contract and manual checks](docs/motor-twin.md).

## CAD rover

`launch` above is the grove stand-in. The converted assembly is a different
command. It loads `cad_visual.obj` (the Onshape STEP, tessellated) in the grove,
which is the scene this command opens. Trunks and shrubs are in the way.
The mesh is visual. Contact is the base plate and four idler cylinders.
Mass and torque are assumed. **W / A / S / D** drive from that grove. **Space**
stops. Manual drive does not use the planner.

```bash
embr-local-sim cad
```

Open the URL it prints. The default is [http://127.0.0.1:43147](http://127.0.0.1:43147).
**W / A / S / D** drive. **Space** stops. In the Scene panel, **engineering**
and **wildfire-mission** keep this CAD model and change the world. Leave
**Model** on `cad`.

The SolidWorks zip is not in the repo. Do not upload it to convert it again.
The obj and `cad_geometry.json` already ship with this package.
What a person actually ran, and what is still assumed, is in
[`docs/human-claims.md`](docs/human-claims.md).

## Replacing the steer planner

Follow instructions calls `Simulator.steer_planner`. The default is
`VfhPlanner` (a vector-field histogram on the lidar fan). Manual **W/A/S/D**
does not call it. This package does not train a model. A teammate can
drive with the keyboard, then try their own model from a script:

```python
from embr_local_sim.model import RobotParams
from embr_local_sim.physics import Simulator
from embr_local_sim.planners.learned import LearnedSteerPlanner
from embr_local_sim.worlds import prepare_world

class MyModel(LearnedSteerPlanner):
    def act(self, ctx):
        # ctx.view is lidar, thermal, probe, and wheel odom. Not ground truth.
        # Return sticks in [-1, 1]. Positive turn is right.
        return 0.4, 0.0

world = prepare_world("forest", seed=1, morphology="wheels")
sim = Simulator(params=RobotParams(), world=world, seed=1)
sim.reset()
sim.steer_planner = MyModel()
sim.start_mission()
sim.run_for(2.0)
```

Any object with `steer(ctx) -> (TeleCmd, reason)` can be assigned the same
way. `None` from `steer` falls back to the small classical policy. The
mission still decides when to sample, retreat, and stop for a blocked wall.

You can start `embr-local-sim` from any working directory after the package is
installed. Scenes are loaded from the installed package, not from the shell's
current folder.

## Platform guidance

| Platform | Status |
| --- | --- |
| macOS, Python 3.12 | Tested locally for install, headless physics, and viewer startup. |
| Ubuntu / Linux | Expected for teammates using the team ROS machines. The `mujoco simulation` GitHub Actions workflow runs pytest and the headless checks on `ubuntu-latest` and passes. The viewer has not been opened on Ubuntu. |
| Windows | Not tested. Commands below are the usual venv steps, not a claim that the stack was run. |

Create the venv from `embr_autonomy/simulation/mujoco` the same way on each OS, then activate:

```bash
# macOS / Linux bash or zsh
source .venv/bin/activate

# Windows Command Prompt
.venv\Scripts\activate.bat

# Windows PowerShell
.venv\Scripts\Activate.ps1
```

ROS 2 Humble for `embr_phys/` uses the team's Ubuntu 22.04 setup or the Docker
environment in `documentation/docker/development.md`. Do not install ROS inside
this virtualenv to run the MuJoCo viewer.

## Controls and examples

Speed values in the buttons are normalized **TeleCmd** levels from -1 to 1,
not metres per second. Positive turn is **right**. The simulator converts that
to a yaw rate that is positive to the left, then to wheel torques.

| Control | What it actually does |
| --- | --- |
| Browser **W / A / S / D** | **W** forward, **A** turn left, **S** back, **D** turn right. A and D alone turn in place. Hold W or S with A or D to turn while moving. These keys do not move the camera. |
| GUI **W / S / A / D** buttons | **Hold-to-drive.** The same command path as the keyboard. Releasing sends nothing new. |
| Terminal **W / A / S / D** | **One command per keypress**, including OS key repeat. This is the terminal, not the browser. |
| **Space** or the stop button | Zero the command. The wheels brake through torque. The robot is not teleported. |
| **P** or Pause | Freeze physics. Drive commands received while paused are dropped. Resume starts from a zero speed target. Velocity already stored in the frozen state is then braked; it is not a queued burst of old keypresses. |
| **.** | One control tick while paused. |
| **R** or Reset | Labeled teleport back to the spawn pose, then velocities are cleared. Used for initialization and the reset button, not for driving. |
| Browser disconnect | Sends a zero command. |
| No fresh command for 0.40 s | Actuator watchdog zeros the speed target. This timer is wall-clock. It is separate from the 0.50 s ROS helper timeout and the 0.25 s iBUS frame timeout, which are not running here. |
| Camera | A preset applies once. Click-drag orbits. Scroll moves the camera forward and back. |
| LiDAR overlay | Cyan hit points from the 2D ray fan. On by default. |
| Balance overlay | Off by default. Red sphere is the center of mass, blue is gravity, orange marks contact normals. |
| Blue trail | Path of the chassis. It is not wheel odometry. |
| **M** / mission buttons | Optional demonstration. See below. |
| **C** or Operator STOP | Cancels the mission into FAULT. |

**Basic scene:** flat ground and the four-wheel proxy. This is not what `launch` opens.

```bash
embr-local-sim launch --scene basic --morphology wheels --seed 1 --host 127.0.0.1 --port 43147
embr-local-sim launch --config src/embr_local_sim/bundled/config/basic.yaml
```

**Forest example:** procedural logs and stones. Not a surveyed site. With no
`--morphology` flag this still uses the track rollers.

```bash
embr-local-sim launch --scene forest --seed 2
embr-local-sim launch --scene forest --morphology wheels --seed 2
```

**Grove and the track approximation.** `embr-local-sim launch` opens this.
`--morphology tracks` is five driven rollers per pitch module. The dark tread
does not collide. It is not a belt and not imported CAD. The interface label
is “Articulated EMBR approximation — hardware validation pending.” Trunks and
shrubs stop the robot. Canopy, flames, and smoke stay visual. `--scene course`
is a rigid branch, log, oblique branch, one-sided step, ramp, cross-slope, and
loose branch. Modeling notes: `docs/modeling-decisions.md`.

```bash
embr-local-sim launch --scene course --morphology tracks --host 127.0.0.1 --port 43147
embr-local-sim launch --scene grove --morphology tracks --host 127.0.0.1 --port 43147
embr-local-sim launch --morphology articulated
embr-local-sim launch --scene engineering
embr-local-sim headless --steps 80 --scene course --morphology tracks --seed 1
```

**Optional mission demonstration:** the robot searches using wheel odometry,
the 2D ray fan, and the synthetic temperature field. It commands the same
TeleCmd path as the buttons. It does not teleport the chassis. The
**ORACLE / debug** checkbox may use a labeled ground-truth exclusion map.
Leave it off if you want the sensor-only demonstration. This is not
sensor-based field autonomy.

```bash
embr-local-sim launch --scene wildfire --mission
```

**Headless check** (no browser, Viser is not imported):

```bash
embr-local-sim headless --steps 200 --scene basic --seed 1
embr-local-sim headless --steps 80 --scene forest --seed 2
embr-local-sim --help
embr-local-sim launch --help
```

## What is simulated

**Movement.** Normal driving applies bounded wheel torques. The chassis moves
only when those torques produce contact forces. Pose writes happen on reset.

**Contacts.** Floor, walls, ramps, logs, and stones in a scene are rigid
MuJoCo geoms. They are not soil.

**Articulation.** `launch` uses the track approximation above. `--morphology wheels`
has no track modules. `--morphology articulated` adds assumed pitch hinges and
three rollers per pod. The probe is a **one-DOF hinge boom** proposed in this
sandbox. It is not the capstan and not a sensor mast from CAD.

**Sensors, and what they are not**

| Reading | What it is |
| --- | --- |
| Wheel odometry | Integrated from wheel joints. Not ground-truth pose. |
| IMU and encoders | Copied from simulator state. Default noise is zero. Not a hardware spec. |
| 2D lidar | `mj_ray` fan, 21 rays, 120°, 2.0 m. Those numbers are assumed for this proxy. Not a Velodyne. |
| Temperature | Samples of a Gaussian field in the scene. Peaks in the YAML are assumed °C. Not fire simulation and not FDS. Component heat ratings are **UNKNOWN**. |
| Ground-truth pose overlay | Debug checkbox in the viewer. Hidden from the default mission. |

Passing a headless run shows that this proxy stays finite and that a forward
command produces torque and, on flat ground, forward motion. It does not
validate the real robot.

## Configuration

Packaged files:

- `src/embr_local_sim/bundled/config/basic.yaml`
- `src/embr_local_sim/bundled/config/forest.yaml`
- `src/embr_local_sim/bundled/scenarios/*.yaml` for scripted checks
- Parameter definitions and units: `RobotParams` in `src/embr_local_sim/model.py`

| Parameter | Value | Unit | Where it came from |
| --- | --- | --- | --- |
| `wheel_radius` | 0.03 | m | `embr_simple_controllers.yaml` |
| `wheel_separation` | 0.1175481 | m | simple model |
| `wheelbase` | 0.1734688 | m | simple URDF |
| `chassis_mass` | 0.20 | kg | **Assumed.** The URDF CAD mass is about 1e-4 kg and is not used. |
| `max_wheel_torque` | 0.020 | N·m | **Assumed.** Not a motor rating. |
| `max_vx` | 0.35 | m/s | **Assumed** cap so the tiny proxy is not commanded at 1 m/s. |
| `physics_dt` | 0.001 | s | Fixed MuJoCo step. |
| `control_hz` | 50 | Hz | Control tick. |
| `watchdog_wall_s` | 0.40 | s | Wall clock, assumed deadman. |

Measurements printed by the headless command (`sim_time_s`, `dx_m`) use
**simulation time** and metres in the MuJoCo world. The watchdog uses wall
clock on purpose, because it is a loss-of-command timer.

Edit a YAML file, then start the process again. The running viewer does not
reload config. To repeat a forest layout, pass the same `--seed` and `--scene`.
Copy the config file if you want to keep a variant.

## Tests and outputs

From `embr_autonomy/simulation/mujoco`, with the test extra installed:

```bash
python -m pytest
embr-local-sim headless --steps 200 --scene basic --seed 1
embr-local-sim headless --steps 80 --scene forest --seed 2
embr-local-sim headless --steps 80 --scene grove --morphology tracks --seed 1
embr-local-sim headless --all
```

`pytest` prints to the terminal. `headless --all` writes JSON, CSV, and
Markdown under `./embr-sim-runs/` (gitignored). `headless --steps` prints one
summary line and does not need that folder.

A passing test means the proxy's contact drive, reset, pause, and packaged
scenarios behaved as coded. It does **not** mean the result matches the
physical EMBR. Masses, torques, sensor ranges, and temperatures still need a
hardware comparison that this package does not do.

## Troubleshooting

**Port already used.** The program stops with `Port ... is already in use`.
Pick another one: `embr-local-sim launch --port 43148`.

**Missing dependency.** Create the venv and run `python -m pip install -e '.[test]'`
from `embr_autonomy/simulation/mujoco`. `embr-local-sim doctor` lists the Python packages.
A missing ROS install is a note, not a failure of this viewer.

**ROS sourced in the same terminal.** Sourcing ROS sets `PYTHONPATH`, which
the venv also reads. pytest then auto-loads ROS's plugins and fails with
errors such as `No module named 'lark'`. Run without it:
`env -u PYTHONPATH python -m pytest`, or use a terminal where ROS is not sourced.

**Missing config or scenario.** The command prints the path it tried and, for
scenarios, the packaged names. Primitive shapes are used when CAD meshes are
absent; that is expected.

**Browser cannot connect.** Use the `http://127.0.0.1:<port>` line from the
terminal that is still running. The default bind is localhost only. Viser will
not load if that process has exited.

**Slow view.** The physics step stays at 0.001 s. If the machine falls behind,
the loop does not skip physics steps to catch the clock. Close the forest
scene or other browser tabs if drawing is the slow part. A GPU is not required.

**macOS.** Use `python3.12`, not the system `python3`, unless that `python3`
is 3.11 or newer. This checkout was run with Homebrew Python 3.12.

## Development roadmap

These are not done:

1. An articulated EMBR model from real geometry, joints, and inertias.
2. Hardware comparison for mass, torque, and sensor values that are still assumed or unknown.
3. A ROS 2 adapter that publishes the existing `TeleCmd` and sensor topics. Hardware execution stays in `embr_phys/`.
4. An inspection mission checked against sensor logs, with oracle mode kept separate.
5. A learned steer model, trained outside this package and passed in as `Simulator.steer_planner`. The default planner is the histogram in `planners/vfh.py`. This package does not train a policy.
