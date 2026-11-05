# Four-motor Maxon drivetrain twin

[#74](https://github.com/HEATRobotics/EMBR-AutoBot/issues/74) adds a read-only ESCON2 feedback path to one MuJoCo model with four gearbox output joints. It reuses the motor-shaft units from [the single-motor twin](motor-twin.md) and the joint names and wheel layout from `embr_simple`. The chassis stays fixed for bench testing. The simplified geometry uses the existing EMBR wheelbase, track width and wheel size with four colored pointers and a front marker. Chassis motion, thermal tracking and simulated LiDAR are integration work under [#67](https://github.com/HEATRobotics/EMBR-AutoBot/issues/67).

## Run without hardware

From `embr_autonomy/simulation/mujoco`, install the existing simulator package using Python 3.12 or 3.13.

```sh
python -m pip install -e '.[test]'
embr-local-sim drivetrain
embr-local-sim drivetrain --headless --out embr-sim-runs/drivetrain-synthetic
embr-local-sim drivetrain --write-model embr-sim-runs/drivetrain.xml
```

Open the printed browser address (normally `http://127.0.0.1:43148`) and click **Run / replay 12-second demo**. The orange, cyan, green and pink markers are front left, rear left, front right and rear right. They turn at different speeds and in different directions, stop at 4 seconds, reverse at 6 seconds and return to zero at 10 seconds. The chassis and the four reference marks remain fixed. Headless replay applies 601 observations per motor and writes paired CSV and JSON evidence when `--out` is supplied. Its 50 Hz source schedule and 25 Hz browser refresh target are software settings.

## Map the four drives

The default mapping follows the command-node slot order and the existing right-side polarity convention. Confirm each joint's direction and its actual controller ID on the bench.

| Slot | CANopen ID | MuJoCo joint | Feedback sign |
| --- | ---: | --- | ---: |
| Front left | 1 | `front_left_maxon` | +1 |
| Rear left | 3 | `rear_left_maxon` | +1 |
| Front right | 2 | `front_right_maxon` | −1 |
| Rear right | 4 | `rear_right_maxon` | −1 |

To change these values, copy the packaged `src/embr_local_sim/bundled/config/maxon_drivetrain.yaml` and pass it with `--config PATH`. Keep the four slots in table order. The working ratio is **20 motor revolutions per gearbox output revolution**. For each joint, output angle is `direction × relative_counts × 2π / (counts_per_rev × gear_ratio)` and output speed is `direction × measured_motor_rpm × 2π / (60 × gear_ratio)`. Configure the ratio for the actual hardware. Set `gear_ratio: 1` only when deliberately displaying motor-shaft motion.

## Run on the bench

Use the commissioned motors and CAN interface with the team's existing [ESCON2 bring-up procedure](../../../../documentation/escon2_canopen.md). Confirm the encoder settings and ratio, agree on command and stop responsibilities, and check the geometry and joint directions with Thanmay against the bench assembly.

1. Start MuJoCo with `embr-local-sim drivetrain --live --out embr-sim-runs/drivetrain-live`. It listens for feedback on UDP port `43153` and initially displays **Waiting** for each motor. A separate computer can use `--feedback-bind <simulator IPv4>` on the test LAN. Keep the browser listener local.
2. Rebuild and source the physical ROS 2 workspace so `dt_can` includes the feedback publisher. Append the parameters below to the bench team's normal `ros2 run embr_drivetrain dt_can --ros-args ...` command. For the first bench test also pass `-p enabled_motor_ids:="[1]"`. To exercise all four, start with all drives configured and omit that selection. Use only one command node on this SDO channel.

   ```sh
   -p feedback_port:=43153 -p feedback_host:=127.0.0.1
   ```

   Use the simulator IPv4 for `feedback_host` when running on a separate computer. `feedback_port:=0` disables the feature. `feedback_encoder_counts_per_rev` is a four-integer ROS parameter in FL, RL, FR, RR order. It defaults to `[4096, 4096, 4096, 4096]`, matching the showcase's 1024 pulses per revolution with quadrature. The reader checks that the actual configured encoder matches. The available timing settings are `feedback_period` (0.02 s between round-robin polls), `feedback_sdo_timeout` (0.015 s per feedback upload, capped by the existing SDO timeout), and `feedback_reference_gap` (0.5 s before taking a fresh relative zero and between failed-node retry attempts).
3. Use the existing drivetrain controls to move each motor forward and backward at several speeds and stop. Confirm the matching joint moves in the expected direction and the other joints report their own motion. Repeat with multiple motors moving together. Watch the observed update rate and the control node's watchdog under load. The default poll schedule allows up to about 12.5 observations/s per motor with four connected before time spent in CAN transfers, command handling and ROS scheduling. SDO reads are sequential and position/speed are read in separate transfers.
4. With the drives stopped and the hardware owner coordinating any disconnect, interrupt one feedback channel. Expect **Unavailable** after a failed read or **Stale** after 0.5 s without a valid packet. The last angle stays visible and the displayed speed becomes unavailable. Other channels can continue updating. A new successful read after an SDO error or gap establishes a new relative zero and increments that motor's reference number. The existing hardware controller still handles drive faults and may stop multiple drives. Restarting the publisher opens a new session after the previous stream times out.
5. Inspect `drivetrain_samples.csv`. For each motor, compare at least 10 consecutive rows with the same session and reference against the conversion above. Check that sequence numbers are consecutive and that counts, rpm and the applied joint motion agree. For unattended capture, use `--live --headless --duration 20 --out embr-sim-runs/drivetrain-live` instead of the viewer. That command exits nonzero when a 10-sample window is missing for any motor. Record the live demo, bench configuration, measured per-motor rate and disconnect results in #74.

## Feedback and evidence

The feedback timer shares the existing CANopen client with the ROS control and watchdog callbacks so a second SDO client does not compete for the same responses. The timer uploads Sensor 2 position (`0x60E4:02`) and measured motor rpm (`0x606C:00`). On the first sample and after a failed poll or gap, it checks encoder type, pulse count and configured position and velocity units before forwarding either value. Adjacent signed 32-bit counter readings are unwrapped at the source. The first reading in each reference defines relative zero. A silent controller reset with no missed reads may require restarting feedback to establish a new reference.

Each UDP datagram contains one version 1 JSON observation with `session`, `node_id`, per-node `seq`, monotonic `sample_s` at the end of the read, and elapsed `poll_s`. A valid observation also has `reference`, raw signed `counts`, unwrapped `relative_counts`, `counts_per_rev` and measured `rpm`. A failed poll sends `error` instead. Malformed, unknown, duplicate and out-of-order packets do not refresh a joint. There is no speed-only fallback.

The evidence CSV adds receiver time, applied joint angle and speed in SI units, and observation-to-application time for loopback traffic. The JSON summarizes per-motor sample counts, longest uninterrupted runs, average source update rates and maximum polling and loopback application times. Across computers, the two monotonic clocks have different origins. Use a shared-clock measurement or an external recording for end-to-end lag across hosts. The polling and application measurements do not include the browser's rendering time.
