# Desktop motor control

`cp_can --sim` is the desktop USB CAN bench mode, on Linux or native Windows
with ROS 2 and this workspace installed. Real mode is an inactive placeholder
for the Raspberry Pi. This mode drives a physical, already configured and
calibrated ODrive 0.5.6 axis; it does not perform calibration.

Install `python-can>=4.3,<5` and `pyserial>=3.5` in the Python environment used
by ROS. Build from `embr_phys/ros2_ws` using
`colcon build --packages-up-to embr_capstan --symlink-install`, then source
`install/setup.bash` (Linux bash), `install/setup.zsh` (Linux zsh), or call
`install\setup.bat` (Windows command prompt) in each terminal.

Start the bridge, selecting your adapter port and configured axis CAN node ID:

```sh
ros2 run embr_capstan cp_can --sim --ros-args -p channel:=/dev/ttyACM0 -p node_id:=0
```

On native Windows replace `/dev/ttyACM0` with e.g. `COM4`. For Linux candleLight
firmware use `-p interface:=socketcan -p channel:=can0` with can0 already
configured at the matching CAN bitrate. See README.md for adapter setup.
There is deliberately no default port. Opening enables torque passthrough
and closed loop on the selected axis.

In a second terminal start the helper **without** its `--sim` flag:

```sh
ros2 run embr_capstan cp_helper
```

The helper's `--sim` flag targets the visual joint simulation instead of CAN.
To move one direction, publish repeatedly from a third terminal:

```sh
ros2 topic pub --rate 10 /tele_cmd embr_interfaces/msg/TeleCmd "{velocity: 0.2, turn: 0.0}"
```

Stop that publisher with Ctrl+C; to reverse, repeat with `velocity: -0.2`.
The helper uses `velocity` for this single motor; `turn` is ignored. The motor's
mounting determines which sign means left. Torque is limited to 0.1 Nm by
default; set `max_torque` on both nodes to change that limit. Torque control
does not prescribe a fixed speed or travel distance.

Both nodes command zero when input is stale (0.5 seconds by default). Bridge
shutdown attempts zero torque and idle before closing USB. A disconnected
adapter or killed process cannot guarantee delivery of a stop command; the
drive's hardware watchdog must be configured separately for that case.

`interface:=virtual` permits automated tests without hardware; python-can's
virtual backend only connects buses inside the same process, so a separate
virtual process will not simulate or move a motor.
