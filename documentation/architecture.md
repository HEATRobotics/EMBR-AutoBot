# EMBR development architecture

The implemented ROS packages currently live in `embr_phys/ros2_ws/src`.
Ubuntu developers can build this workspace natively. Windows and macOS
developers can build it in the Ubuntu ROS 2 Humble Docker environment.

```text
Host repository
└── embr_phys/ros2_ws/src
    ├── embr_interfaces     Shared TeleCmd message
    ├── embr_teleoperation  Operator command publisher
    ├── embr_drivetrain     Maxon CANopen control and command routing
    ├── embr_capstan        Capstan command routing and hardware documentation
    ├── embr_lidar          LiDAR package
    ├── embr_thermals       Thermal sensing package
    └── embr_description    Models, meshes, controllers, RViz, and launch files
```

## Container environments

Windows and macOS Compose files build the same Linux development image from
`docker/development.Dockerfile`. They differ in how X11 graphics reach the
host: VcXsrv on Windows and XQuartz on macOS. Both use software rendering.
Ubuntu users generally do not need a container and can follow the [native
workflow](workflow.md#native-ubuntu-development).

## Physical workspace

`embr_phys/ros2_ws` contains the robot software and description packages.
`embr_teleoperation` publishes the shared `embr_interfaces/TeleCmd` message.
`embr_drivetrain` routes commands to the Maxon motors or simulation topics,
and `embr_capstan` routes commands to the capstan or its RViz demonstration.
`embr_description` provides the Maxon, simple EMBR, and planetary Eagle models,
meshes, controller configurations, RViz configurations, and launch files.

The simple EMBR and planetary Eagle demonstrations use mock ros2_control
hardware. They visualize commanded motion without simulating terrain or
contact physics. There is no separate Gazebo workspace in this layout.

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
