# EMBR Simulation Pipeline Architecture

## Overview

The repository provides containerized ROS 2 environments for developing and
testing EMBR robot software. Each Compose service isolates a specific workload
while bind mounts keep source code and generated artifacts available on the
host.

The CANopen path currently supports the Maxon motor description in RViz and
contains scaffolding for simulated CANopen motor behavior.

```text
Host repository
├── embr_phys/ros2_ws
│   └── src
│       └── embr_description     Maxon URDF, meshes, RViz, and launch files
└── embr_sim/ros2_ws
    └── src
        ├── embr_canopen_sim     CANopen simulation scaffold
        └── embr_gazebo          Gazebo resources scaffold
                  │
                  │ bind mounts
                  ▼
CANopen container
├── /workspace/embr_phys_ws      ROS underlay
└── /workspace/embr_sim_ws       ROS overlay
```

## Container boundary

The `canopen-sim` service is defined per-OS in `compose.linux.yaml` and
`compose.windows.yaml`, both built from `docker/CANopen-Spec-Dockerfile`. The
two compose files use the same image but differ in how the GUI, networking,
and devices reach the container; see `documentation/canopen_simulation.md`
for details.

The service provides:

- ROS 2 Humble on Ubuntu 22.04
- ROS 2 CANopen packages
- `ros2_control`
- Gazebo integration packages
- RViz, Xacro, and robot/joint state publishers
- SocketCAN utilities
- X11 access for graphical applications

The container uses the host UID and GID, defaulting to `1000:1000`. This keeps
files created in the bind-mounted workspace editable by the local user.
`LOCAL_UID`, `LOCAL_GID`, and `CONTAINER_USER` can be set in `.env` when the
defaults do not match the host.

## ROS workspace

The host directories `embr_phys/ros2_ws` and `embr_sim/ros2_ws` are mounted at
`/workspace/embr_phys_ws` and `/workspace/embr_sim_ws`. The physical workspace
is built and sourced as an underlay before the simulation overlay. Consequently,
`colcon` creates `build`, `install`, and `log` in each host workspace. These
generated artifacts are excluded from the Docker build context.

The image build copies only each workspace's `src` directory. It installs
declared dependencies with `rosdep` and builds both workspaces in underlay then
overlay order. At runtime, the bind mounts replace the image workspaces with
the live host workspaces.

## Package responsibilities

### `embr_description`

Lives in `embr_phys/ros2_ws/src` and owns hardware-neutral geometry and
visualization resources:

- Maxon motor Xacro/URDF
- STL meshes
- RViz configuration
- `view_maxon_motor.launch.py`

The launch file produces `robot_description`, publishes the TF tree, starts the
joint-state GUI, and opens RViz. The continuous shaft joint can be adjusted
from the joint-state GUI.

### `embr_canopen_sim`

Owns the CANopen simulation interface:

- `config/motor_nodes.yaml`
- `launch/canopen_sim.launch.py`
- Future simulated ESCON and command-adapter nodes

The C++ sources and CANopen launch/configuration are currently scaffolds. They
are intentionally not compiled until node implementations are added.

### `embr_gazebo`

Reserved for Gazebo worlds and ROS/Gazebo bridge configuration. The Maxon
model is currently validated in RViz only. Gazebo physics will additionally
require collision geometry, verified inertia values, spawning, and a control
interface.

## Visualization data flow

```text
Maxon Xacro
    │
    ▼
robot_state_publisher ◄── joint_state_publisher_gui
    │                           │
    ├── /robot_description     └── /joint_states
    └── /tf
             │
             ▼
            RViz
```

RViz visualizes state; it does not simulate motor physics. Gazebo and the
CANopen simulation nodes will form separate runtime layers when implemented.
