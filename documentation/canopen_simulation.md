# CANopen Simulation

## Scope

The `canopen-sim` service is the development environment for the Maxon motor
model and future CANopen/ESCON simulation. The current implementation provides:

- A reproducible ROS 2 Humble container
- ROS 2 CANopen and SocketCAN tooling
- A host-mounted ROS workspace
- A Maxon motor description with movable shaft joint
- RViz visualization and joint-state controls
- Empty scaffolds for the simulated ESCON and command adapter

The current RViz launch does not emulate an ESCON controller, transmit CANopen
frames, or simulate motor physics.

## Files

```text
compose.linux.yaml
compose.windows.yaml
docker/
├── CANopen-Spec-Dockerfile
└── canopen-entrypoint.sh
embr_phys/ros2_ws/src/
└── embr_description/
│   ├── launch/view_maxon_motor.launch.py
│   ├── meshes/maxon_motor/meshes/*.stl
│   ├── rviz/maxon_motor.rviz
│   └── urdf/maxon_motor/urdf/maxon_motor.urdf.xacro
embr_sim/ros2_ws/src/
├── embr_canopen_sim/
│   ├── config/motor_nodes.yaml
│   ├── launch/canopen_sim.launch.py
│   └── src/
│       ├── motor_command_adapter.cpp
│       └── simulate_escon.cpp
└── embr_gazebo/
    ├── config/bridge.yaml
    └── worlds/
```

## Quick start: RViz model

From the repository root:

```bash
docker compose -f compose.linux.yaml build canopen-sim &&
docker compose -f compose.linux.yaml up -d --force-recreate canopen-sim
docker compose -f compose.linux.yaml exec canopen-sim bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/embr_phys_ws
colcon build --symlink-install
source /workspace/embr_phys_ws/install/setup.bash
cd /workspace/embr_sim_ws
colcon build --symlink-install
source /workspace/embr_sim_ws/install/setup.bash
ros2 launch embr_description view_maxon_motor.launch.py
```

RViz displays the body, shaft, and electrical tabs. The
`joint_state_publisher_gui` window controls the continuous `shaft` joint.

## Windows setup (Docker Desktop)

`compose.windows.yaml` builds the same `docker/CANopen-Spec-Dockerfile` image
as Linux. The container itself is unchanged (Docker Desktop still runs Linux
containers under the hood); what differs is how the GUI, networking, and
devices reach it, because those are Linux Docker Engine features without a
Docker Desktop equivalent.

### Prerequisites

- Docker Desktop with the WSL2 backend enabled.
- An X server for Windows, e.g. [VcXsrv](https://sourceforge.net/projects/vcxsrv/).

### One-time X server setup

1. Install and launch VcXsrv (`XLaunch`).
2. Choose **Multiple windows**, display number `0`.
3. Choose **Start no client**.
4. On the **Extra settings** page, check **Disable access control**. Without
   this, VcXsrv rejects connections from the container.
5. Leave VcXsrv running for the duration of the session (it can be added to
   Windows startup).
6. Allow VcXsrv through the Windows Defender Firewall for private networks
   when prompted the first time a container connects.

### Quick start: RViz model

From the repository root, in PowerShell or a WSL2 shell:

```bash
docker compose -f compose.windows.yaml build canopen-sim &&
docker compose -f compose.windows.yaml up -d --force-recreate canopen-sim
docker compose -f compose.windows.yaml exec canopen-sim bash
```

Inside the container, the same commands as the Linux quick start apply:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/embr_phys_ws
colcon build --symlink-install
source /workspace/embr_phys_ws/install/setup.bash
cd /workspace/embr_sim_ws
colcon build --symlink-install
source /workspace/embr_sim_ws/install/setup.bash
ros2 launch embr_description view_maxon_motor.launch.py
```

RViz and the joint-state GUI should open as separate windows on the Windows
desktop, rendered through VcXsrv.

### How `compose.windows.yaml` differs from `compose.linux.yaml`

| Linux | Windows | Why |
| --- | --- | --- |
| `/tmp/.X11-unix` bind mount + `XAUTHORITY` | `DISPLAY=host.docker.internal:0.0` + `extra_hosts` | Windows has no host X11 socket to bind-mount; GUI apps connect out to an X server running on the host instead. |
| `network_mode: host`, `ipc: host` | Default bridge network | Docker Desktop does not support Linux host networking; DDS/X11 traffic instead crosses the bridge network and `host.docker.internal`. |
| `devices: /dev/dri` | *(omitted)* + `LIBGL_ALWAYS_SOFTWARE=1`, `GALLIUM_DRIVER=llvmpipe` | There is no `/dev/dri` device node to pass through on Windows. RViz/Gazebo fall back to Mesa software rendering (`llvmpipe`), which is slower but does not require GPU passthrough. |

### Known limitations

- **SocketCAN (`vcan0`) is not available by default.** The `vcan` kernel
  module is not built into the default Microsoft WSL2 kernel, so
  `modprobe vcan` fails inside the container even though `cap_add:
  NET_ADMIN` is set. A custom WSL2 kernel build with CAN drivers is required
  for actual virtual-CAN development on Windows; until then, CANopen/ESCON
  bus work should happen on Linux. This does not block the RViz model
  workflow above, which does not use CAN.
- **Cross-host ROS 2 discovery is not configured.** Without host networking,
  DDS multicast discovery does not reach other containers or the host by
  default. This is not required for the single-container RViz workflow.
- **Rendering is software-only.** Expect RViz/Gazebo to be noticeably slower
  than on native Linux with GPU passthrough.

## Runtime interfaces

The table below describes `compose.linux.yaml`. See
[How `compose.windows.yaml` differs](#how-composewindowsyaml-differs-from-composelinuxyaml)
above for the Windows equivalents.

The service uses:

| Interface | Purpose |
| --- | --- |
| `/workspace/embr_phys_ws` | Physical underlay bind-mounted from `embr_phys/ros2_ws` |
| `/workspace/embr_sim_ws` | Simulation overlay bind-mounted from `embr_sim/ros2_ws` |
| `/tmp/.X11-unix` | X11 socket used by RViz and Gazebo |
| `XAUTHORITY` | Host X11 authorization file |
| `CAN_INTERFACE` | SocketCAN interface name; defaults to `vcan0` |
| `ROS_DOMAIN_ID` | ROS 2 discovery domain; defaults to `0` |
| Host network | ROS 2 discovery and access to host network interfaces |
| `NET_ADMIN` | Allows CAN network-interface configuration where supported |

## Virtual CAN interface

The intended development interface is `vcan0`. Check it inside the container:

```bash
ip -brief link show type can
```

If no interface is present, the host must provide Virtual CAN kernel support.
Depending on the host Docker and kernel configuration, create and activate the
interface on the host or from a process with `NET_ADMIN`:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

Verify traffic with SocketCAN utilities:

```bash
candump vcan0
```

In a second terminal:

```bash
cansend vcan0 123#DEADBEEF
```

These commands test the virtual bus only; they do not start the ROS 2 CANopen
stack.

## Generated directories

Running `colcon build` creates:

- `build/`: CMake and compiler intermediates
- `install/`: installed packages and ROS environment hooks
- `log/`: build logs

They appear in `embr_phys/ros2_ws` and `embr_sim/ros2_ws` because those
directories are bind mounts. This is expected. They are generated artifacts
and should not be committed.

## Rebuild rules

Rebuild the Docker image after changes to:

- `docker/CANopen-Spec-Dockerfile`
- System or ROS package dependencies
- Build arguments or the base image

Only rebuild the ROS workspace after changes to:

- CMake or package manifests
- URDF/Xacro, launch, RViz, or YAML files
- C++ source files

## Current limitations and next integration steps

Before the motor can be controlled through simulated CANopen:

1. Define the motor/drive object dictionary and bus configuration.
2. Implement or configure the simulated ESCON node.
3. Define command and feedback interfaces.
4. Populate `motor_nodes.yaml`.
5. Populate `canopen_sim.launch.py`.
6. Add the C++ targets and dependencies to `embr_canopen_sim/CMakeLists.txt`.
7. Connect joint feedback to the `shaft` joint.
8. Add Gazebo collision geometry and verified mass/inertia values if physical
   simulation is required.

RViz should remain the first model-validation step. It confirms mesh paths,
joint topology, TF publication, and joint movement without adding physics or
CANopen complexity.

## Troubleshooting

### RViz opens but the motor is not visible

- Confirm the fixed frame is `root`.
- Check the `Maxon Motor` display status.
- Use the close-range Orbit view or select the model and press `F`.
- Confirm `/robot_description` and `/tf` are being published.

### `ament_cmake` cannot be found

The new image was probably not built or ROS was not sourced:

```bash
source /opt/ros/humble/setup.bash
```

Rebuild the image and use `&&` before `docker compose up` so an old cached image
is not started after a failed build.

### Package manifest XML error

The XML declaration must begin at the first byte of `package.xml`:

```xml
<?xml version="1.0"?>
```

No spaces or blank lines may precede it.

### Workspace cannot be edited on the host

Confirm the Compose service uses the host UID/GID and repair artifacts left by
older root-running containers:

```bash
sudo chown -R "$(id -u):$(id -g)" embr_phys/ros2_ws embr_sim/ros2_ws
```

### (Windows) RViz fails to connect to the display

- Confirm VcXsrv is running and **Disable access control** was checked when
  it was launched.
- Confirm the Windows Defender Firewall prompt for VcXsrv was allowed on the
  private network.
- From inside the container, `echo $DISPLAY` should print
  `host.docker.internal:0.0`. If it prints something else, unset a stale
  `DISPLAY` value in the host shell before running `docker compose up`.

### (Windows) RViz/Gazebo render as blank or garbled windows

This is usually indirect-GLX rendering being attempted instead of the Mesa
software renderer. Confirm `LIBGL_ALWAYS_SOFTWARE=1` and
`GALLIUM_DRIVER=llvmpipe` are present in `docker compose -f
compose.windows.yaml config` output for `canopen-sim`.

### (Windows) `vcan0` does not appear

Expected with the default WSL2 kernel; see
[Known limitations](#known-limitations) above.
