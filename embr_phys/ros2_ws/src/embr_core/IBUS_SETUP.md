# Raspberry Pi 4B iBUS teleoperation

The `teleoperation` node publishes `tele_cmd` (`embr_interfaces/TeleCmd`) with
normalized `velocity` and `turn` fields in [-1, 1], plus `ibus_channels`
(`std_msgs/Int32MultiArray`) containing all 14 raw receiver channels.
Positive velocity means forward; positive turn means right (keyboard D).
Receiver timeout publishes a zero command.

The `drivetrain` node subscribes to `tele_cmd`. Its startup `simulation` ROS
parameter defaults to false; `--sim` and `-sim` select a true default. An explicit
ROS parameter overrides the CLI default.

- Real mode publishes `motor_velocity_levels` (`std_msgs/Float32MultiArray`),
  normalized to [-1, 1], ordered `[front_left, rear_left, front_right, rear_right]`.
  Forward/right mixing preserves the ratio while limiting the largest magnitude
  to 1. The CANopen handler is currently a placeholder: it must subscribe to this
  topic and convert levels into configured motor speeds and CANopen commands.
- Simulation mode publishes `cmd_vel` (`geometry_msgs/Twist`), which the
  `view_embr_simple.launch.py` controller already consumes. `linear.x` is in m/s
  and `angular.z` in rad/s, with positive yaw turning left. The startup parameters
  `max_linear_speed` and `max_angular_speed` both default to 1.0.

Output repeats every `publish_period` (default 0.05 seconds), and stops when no
`tele_cmd` arrives for `command_timeout` (default 0.5 seconds). Both parameters
must be positive, and the publish period must be smaller than the timeout.
The eventual CANopen handler should also stop on loss of drivetrain messages.

Run real mode alongside receiver teleoperation:

```sh
ros2 run embr_core drivetrain
```

For RViz, run each command in a separate sourced terminal:

```sh
ros2 launch embr_description view_embr_simple.launch.py
ros2 run embr_core drivetrain --sim
ros2 run embr_core teleoperation --sim
```

Equivalent drivetrain mode selectors are `-sim` and
`--ros-args -p simulation:=true`. Topic names can be remapped with ROS arguments.

## Wiring

The receiver is connected to UART3, exposed as `/dev/ttyAMA1`. Connect its iBUS
signal to the RX pin configured for UART3 and connect receiver ground to Pi
ground. The UART3 TX pin is not required because iBUS telemetry is not
transmitted by this node.

Raspberry Pi GPIO is **3.3 V only**. Confirm the receiver's iBUS signal level; use
a level shifter or resistor divider if it outputs 5 V. Power the receiver from a
suitable regulated supply and share ground with the Pi. Do not power motors from
the Pi.

Enable UART3 in the Raspberry Pi boot configuration, ensure no serial console is
using it, then reboot. Confirm that `/dev/ttyAMA1` exists and ensure the ROS user
belongs to the group allowed to access it (commonly `dialout`).

## Run

Build/source the workspace, then run:

```sh
ros2 run embr_core teleoperation --ros-args \
  -p serial_port:=/dev/ttyAMA1 \
  -p forward_channel:=1 \
  -p turn_channel:=0
```

Channel indexes are zero-based. Other parameters include `channel_min`,
`channel_center`, `channel_max`, `deadband`, `invert_forward`, `invert_turn`, and
`frame_timeout`. Valid frames are printed to the node console at most twice per
second by default. Set `frame_display_period` to the desired interval in seconds;
use `0.0` to print every frame. The complete stream can also be inspected with
`ros2 topic echo /ibus_channels`. Keep the wheels clear of the ground while
confirming channel order, direction, and failsafe behavior.

## Terminal simulation mode

To publish commands without an iBUS receiver or serial port, run:

```sh
ros2 run embr_core teleoperation --sim
```

The `-sim` spelling is also accepted. Press `W`/`S` to increment or decrement
forward motion, `A`/`D` to increment or decrement turning, space to stop, and `Q`
to stop and exit. Commands use the same `tele_cmd` topic as hardware mode and repeat while
the terminal is connected so the drivetrain watchdog can detect disconnection. The increment defaults to `0.1`
and can be changed with, for example:

```sh
ros2 run embr_core teleoperation --sim --ros-args -p sim_step:=0.25
```
