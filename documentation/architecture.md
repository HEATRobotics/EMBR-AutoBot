# EMBR development architecture

The implemented ROS packages currently live in `embr_phys/ros2_ws/src`.
Ubuntu developers can build this workspace natively. Windows and macOS
developers can build it in the Ubuntu ROS 2 Humble Docker environment.

```text
Host repository
└── embr_phys/ros2_ws/src
    ├── embr_core          Teleoperation and drivetrain nodes
    └── embr_description  Robot model, meshes, RViz files, and launch files
```

## Container environments

Windows and macOS Compose files build the same Linux development image from
`docker/development.Dockerfile`. They differ in how X11 graphics reach the
host: VcXsrv on Windows and XQuartz on macOS. Both use software rendering.
Ubuntu users generally do not need a container and can follow the [native
workflow](workflow.md#native-ubuntu-development).

## Physical workspace

`embr_phys/ros2_ws` contains the robot software and description packages.
`embr_core` provides teleoperation and drivetrain entry points, and
`embr_description` provides the Maxon motor and EMBR models, meshes, RViz
configurations, and launch files.

The `view_maxon_motor.launch.py` launch file starts
`joint_state_publisher_gui`, `robot_state_publisher`, and RViz. The GUI
publishes the shaft joint state; RViz displays the model and TF tree. This
launch does not model motor physics or provide a CANopen implementation.

## Container build and mounts

The Docker image installs system tools from the shared package list,
resolves package manifest dependencies with `rosdep`, and builds the physical
workspace. At runtime the repository is bind-mounted at
`/home/embr/EMBR-AutoBot`, so source changes are available without rebuilding
the image. Workspace build products are generated under
`embr_phys/ros2_ws/build`, `install`, and `log`.
