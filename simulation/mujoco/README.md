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
| Default `wheels` | Four-wheel skid-steer **proxy**. Wheel radius 0.03 m comes from the simple ROS controller, not from a full-size EMBR. |
| Optional `--morphology articulated` | Four pitch-hinge pods with three rollers each. A labeled approximation. Not tracks, not a belt, and not CAD. |

No mesh from the team CAD is included. Bodies are MuJoCo primitives.

ROS 2 is **not** required and **not** connected. The ROS workspace stays in
`embr_phys/ros2_ws`. RViz mock hardware is a separate demo. A ROS adapter file
in this package records the intended topic names; it does not start a node.

## Quick start

Tested here: **macOS, Python 3.12.14**. The package requires **Python 3.12 or
3.13** because NumPy 2.5, which `pip` resolves from the version range, requires
3.12. Python 3.13 was not run. Windows was not run.

From the **repository root**:

```bash
cd simulation/mujoco
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -e '.[test]'
embr-local-sim launch
```

`requirements.txt` pins the runtime versions executed on this Mac.
`requirements-dev.txt` adds the pytest pin. The ranges in `pyproject.toml`
are what pip is allowed to resolve; the pin files are the set that was run.

Open the URL the terminal prints. The default is
[http://127.0.0.1:43147](http://127.0.0.1:43147).

Leave that terminal running. Stop with **Ctrl-C**. If the terminal is
interactive, **q** also quits.

You can start the command from any working directory after the package is
installed. Scenes are loaded from the installed package, not from the shell's
current folder.

## Platform guidance

| Platform | Status |
| --- | --- |
| macOS, Python 3.12 | Tested locally for install, headless physics, and viewer startup. |
| Ubuntu / Linux | Expected for teammates using the team ROS machines. The GitHub Actions file runs the headless checks on Ubuntu. That workflow has **not** been executed on GitHub from this change. |
| Windows | Not tested. Commands below are the usual venv steps, not a claim that the stack was run. |

Create the venv from `simulation/mujoco` the same way on each OS, then activate:

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

**Basic scene** (default): flat ground, wheel proxy.

```bash
embr-local-sim launch --scene basic --seed 1 --host 127.0.0.1 --port 43147
embr-local-sim launch --config src/embr_local_sim/bundled/config/basic.yaml
```

**Forest example:** procedural logs and stones. Same proxy. Not a surveyed site.

```bash
embr-local-sim launch --scene forest --seed 2
```

**Track approximation and obstacle scenes.** Default launch stays the wheel
proxy. `--morphology tracks` is five driven rollers per pitch module. The
dark tread does not collide. It is not a belt and not imported CAD. The
interface label is “Articulated EMBR approximation — hardware validation
pending.” `--scene course` is a rigid branch, log, oblique branch, one-sided
step, ramp, cross-slope, and loose branch. `--scene grove` adds larger trunks,
a clearing, shrubs that stop the robot, and synthetic flame markers.
Canopy, flames, and smoke stay visual. Modeling notes:
`docs/modeling-decisions.md`.

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

**Articulation.** The default model has no track modules. The optional
articulated morphology adds assumed pitch hinges. The probe is a **one-DOF
hinge boom** proposed in this sandbox. It is not the capstan and not a sensor
mast from CAD.

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

From `simulation/mujoco`, with the test extra installed:

```bash
python -m pytest
embr-local-sim headless --steps 200 --scene basic --seed 1
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
from `simulation/mujoco`. `embr-local-sim doctor` lists the Python packages.
A missing ROS install is a note, not a failure of this viewer.

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
5. A forest benchmark, and only later any learning work. This package does not train a policy.
