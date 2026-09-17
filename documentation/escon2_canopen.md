# Physical ESCON2 CANopen control

`embr_core` uses [canopen-python](https://github.com/canopen-python/canopen)
SDO transfers for four ESCON2 drives in profile velocity mode (3).
`motor_velocity_levels` contains four normalized values in [-1, 1], ordered
front-left, back-left, front-right, back-right. Default node IDs in that order
are **1, 3, 2, 4**. `can_status` reports acknowledgements and failures, not
measured motor motion. Transfers are sequential, not synchronized PDO updates.

Commission each ESCON2 for the maxon EC-i 52 part **667065** in Motion Studio
before running. Set motor/current limits, Hall feedback, acceleration,
deceleration, quick-stop deceleration and quick-stop option. Verify the exact
ESCON2 variant can supply the required current. The
[motor catalogue](https://www.maxongroup.com/medias/sys_master/root/9406692622366/Cataloge-Page-EN-315.pdf)
lists 24 V, nominal speed 3800 rpm, nominal current 18.3 A, eight pole pairs,
and a mechanical maximum of 6000 rpm for this part. These are motor ratings,
not automatic configuration values or drivetrain operating limits.

Configure velocity units (`0x60A9`) as rpm (`0x00B44700`). The handler checks
this and the drive's maximum motor/profile speeds (`0x6080`, `0x607F`).
Motor settings and ramps are retained from commissioning; no parameters are
saved to flash. The profile mode and controlword sequence follow the
[ESCON2 application notes](https://www.maxongroup.com/medias/sys_master/root/9523021905950/ESCON2-Application-Notes-En.pdf).

Install the declared canopen dependency and build/source the physical ROS
workspace (including `embr_interfaces`). Example, after choosing a suitable
motor-shaft speed limit for the installed mechanism:

```sh
ros2 run embr_core CANopen --ros-args \
  -p interface:=socketcan -p channel:=can0 -p bitrate:=1000000 \
  -p max_speed_rpm:=100.0
```

100 rpm is an illustrative commissioning value. `max_speed_rpm` defaults to
zero and must be explicitly set positive before connection. Configure the
SocketCAN interface externally at the same bitrate. Kvaser defaults to
channel "0". Optional `eds_path` accepts the EDS matching your ESCON2 firmware;
without it, typed standard-object SDO uploads/downloads need no EDS.
`front_left_motor_id`, `back_left_motor_id`, `front_right_motor_id` and
`back_right_motor_id` select unique IDs 1–127. Corresponding `_direction`
parameters accept 1 or -1 to account for mounting orientation.

## Testing only selected motors

Set `enabled_motor_ids` at startup to the wired motor IDs. By default it
includes all four configured IDs. For example, to use only node ID 2:

```sh
ros2 run embr_core CANopen --ros-args \
  -p interface:=socketcan -p channel:=can0 \
  -p max_speed_rpm:=100.0 -p 'enabled_motor_ids:=[2]'
```

Only selected drives are connected, initialized, commanded, monitored and
stopped. Unselected drives need not be wired and receive no commands from this
handler. Exclusion does **not** power off a drive or stop one already running;
stop the previous handler before changing the selection. Restart to apply a
new selection. The list must be nonempty, contain no duplicates, and match
the configured motor IDs. For a different hardware ID, configure the motor
slot too (for example `-p front_right_motor_id:=12 -p 'enabled_motor_ids:=[12]'`).

The command remains four values in **FL, BL, FR, BR** order, even for one
motor. With default IDs, node 2 uses the **third** value. In another terminal,
this sends a 10 rpm target when `max_speed_rpm` is 100:

```sh
ros2 topic pub --rate 10 /motor_velocity_levels std_msgs/msg/Float32MultiArray \
  '{data: [0.0, 0.0, 0.1, 0.0]}'
```

Run only one command publisher during this test. Stopping publication triggers
the command timeout and quick stop; restart the handler after that latched
timeout. To command zero without latching a timeout, keep publishing with all
four values zero. A zero velocity target leaves the drive enabled.

Drives stay disabled at startup until a valid command arrives. Invalid input,
a command timeout (default 0.5 s), disabled/faulted drives or communication
errors trigger a best-effort quick stop on all selected drives and latch failure.
Correct the cause and restart the node to resume; faults are never reset
automatically. Normal shutdown (including Ctrl+C) attempts quick stop, zero target velocity,
and disable voltage before disconnecting, in both CAN communication nodes. Acknowledgement of a
stop does not confirm standstill. If CAN is disconnected, software cannot
ensure stopping: commission an independent drive-side communication watchdog
and physical stop circuit. This handler does not configure a heartbeat consumer
or provide a hard real-time watchdog. SDO delays can extend timeout response;
validate timing and stop behavior on the actual hardware before operation.

Transport contract tests can run without ROS or CAN hardware:

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  embr_phys/ros2_ws/src/embr_core/test/test_canopen_handler.py
```


## Continuous single-motor showcase

```sh
ros2 run embr_core showcase --ros-args -p channel:=can0 \
  -p showcase_speed_rpm:=10.0
```

This starts motion automatically on `showcase_motor` (default ID 1), using
startup as center. It performs relative gearbox-output rotations in degrees:
`180, 90, 180, -90, 180, 180, 90, -90, -90, -90, 180, 90`, returns to
startup center, then repeats until Ctrl+C. It does not subscribe to external
velocity commands. Run it with exclusive control of that drive.

The showcase uses **Sensor 2 digital incremental encoder** feedback, connected
to X5 on the ESCON2 Compact hardware. Configure Sensor 2 as digital incremental
encoder in Motion Studio (`0x3000:01`, bits 15–8 = 1), and set `0x3010:01` to
**1024 pulses/revolution** for this encoder. Quadrature decoding gives
**4096 increments/revolution**, the default `encoder_counts_per_rev` parameter.
The node checks that this parameter equals four times the commissioned pulse
count, and that `0x60A8` specifies increments (`0x00B50000`), before enabling.
It reads settings without changing or saving commissioning parameters.

Position is signed INTEGER32 **`0x60E4:02` (Position actual value sensor 2)**,
not `0x6064`. These objects are documented in the
[ESCON2 Firmware Specification, 2026-02](https://www.maxongroup.com/medias/sys_master/root/9523021185054/ESCON2-Firmware-Specification-En.pdf),
sections 6.2.47.1, 6.2.127 and 6.2.134.2. X5/Sensor 2 wiring is documented in
the [Compact 60/30 hardware reference](https://www.maxongroup.com/medias/sys_master/root/9523021381662/ESCON2-Compact-60-30-Hardware-Reference-En.pdf).
Installed firmware must support these objects; an unavailable position object
fails startup before motion, with no fallback to integrating speed.

The node captures startup encoder counts while disabled and unwraps 32-bit
counter rollover. Position no longer accumulates velocity-integration drift.
This is an incremental reference, not an absolute encoder or physical homing;
a successful SDO read alone cannot prove correct wiring or detect every missed
encoder pulse. Commission feedback direction to agree with commanded shaft
rotation. Measured velocity (`0x606C`) still gates settling. Velocity control,
drive ramps, sampling, and the configured tolerance affect positioning accuracy.

To inspect S2 feedback on node 1 without starting the showcase, stop the ROS
node, keep the drive disabled and NMT pre-operational, run `candump can0` in
another terminal, then request:

```sh
cansend can0 601#40E4600200000000
```

The reply on `581` contains signed little-endian counts after the first four
bytes. A successful reply does not prove that the counter changes with shaft
movement. One motor-shaft revolution corresponds to 4096 counts with this
encoder configuration.

`gear_ratio` defaults to **20.0 motor revolutions per output revolution**.
Encoder scaling stays at 4096 counts per motor revolution. A 180-degree output
move requires 10 motor revolutions. Use the exact gearbox ratio when known;
output accuracy depends on this ratio and gearbox backlash. Set `gear_ratio:=1.0`
for motor-shaft angles.

`showcase_speed_rpm` caps **motor** speed (default 10 rpm, or 0.5 output rpm
at 20:1); movement slows near
its target. `showcase_pause` requires a settled pause between moves (default
1 s), `angle_tolerance` defaults to 3 output degrees, and `move_timeout` defaults to
600 s per movement including settling, allowing the longer return to center
at 20:1 gearing. A 180-degree output move takes at least 60 seconds at the
default motor speed; returning from the sequence’s net +810 degrees takes
about 270 seconds plus settling. A timeout or communication fault stops
motion and requires restart. Commissioned acceleration/deceleration settings
still apply. Ctrl+C sends quick stop, zero velocity, and disable voltage;
disabling removes holding torque and does not confirm physical standstill.
