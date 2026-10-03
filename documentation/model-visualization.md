# Robot model visualization

The physical workspace contains the implemented robot description and RViz
launch files in `embr_phys/ros2_ws/src/embr_description`.

## View the Maxon motor

Build and source the physical workspace, then run:

```bash
ros2 launch embr_description view_maxon_motor.launch.py
```

RViz displays the motor, while `joint_state_publisher_gui` controls the shaft
joint. This displays the robot model; it does not simulate motor physics or
communicate with a motor controller.

On Windows, first start the [Docker development environment](docker/development.md)
and run the launch command inside its container. Ubuntu users can use the
[native Ubuntu workflow](workflow.md#native-ubuntu-development). On macOS,
XQuartz can forward X11 windows, but RViz may not get the OpenGL context it
needs; see the [Mac Docker notes](docker/development.md#macos-setup).
