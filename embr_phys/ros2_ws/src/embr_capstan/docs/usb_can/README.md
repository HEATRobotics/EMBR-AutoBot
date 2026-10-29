# RH02 USB-to-CAN helper

`embr_capstan.capstan_hardware.usb_can.UsbCan` is a ROS-independent helper
for the simulator's eventual hardware bridge. It opens an adapter explicitly,
sends standard classic CAN frames, and receives `can.Message` objects. The
default CAN rate is **1 Mbps**. It does not yet subscribe to ROS topics.

On Linux, run directly on the host machine. From `embr_phys/ros2_ws`, install
dependencies and build the package with its ROS dependencies:

```sh
source /opt/ros/humble/setup.bash
python3 -m pip install 'python-can>=4.3,<5' 'pyserial>=3.5'
colcon build --packages-up-to embr_capstan --symlink-install
source install/setup.bash
```

For zsh, use `setup.zsh` in both source commands. No separate Python environment
is required. Windows users run the package inside the Linux container described
below and install these dependencies there.

## Select the adapter firmware

USB-A/B connector shape does not define the software interface. Check the
firmware installed on your RH02 before selecting a backend:

| Firmware/interface | Helper configuration |
| --- | --- |
| SLCAN serial firmware | `UsbCan('/dev/ttyACM0')` (or `/dev/ttyUSB0`) |
| candleLight/gs_usb, Linux CAN network device | `UsbCan('can0', interface='socketcan')` |
| No adapter, in-process simulation | `UsbCan('capstan-sim', interface='virtual')` |

These are supported firmware paths, not a guarantee about your adapter's
factory firmware. Do not flash it merely because no serial port appears.
Inspect `lsusb`, `ip link`, and `/dev/serial/by-id/` first. Prefer the stable
`/dev/serial/by-id/...` path when available. Native Windows SLCAN accepts a
`COM4` channel; the Windows Docker workflow below uses Linux device paths.

## Linux host setup

Connect the RH02 directly to the laptop and run the helper on the host.
Identify the adapter with:

```sh
lsusb
ls -l /dev/serial/by-id/
ip link
```

A serial firmware normally exposes `/dev/ttyACM0` or `/dev/ttyUSB0`;
gs_usb/candleLight normally exposes a CAN interface such as `can0`.

For SLCAN, give your account access to the serial device (usually the `dialout`
group; log out and back in after adding group membership). USB serial baud
defaults to 115200 and is separate from the 1 Mbps CAN bitrate; change
`tty_baudrate` if the adapter firmware requires it. Opening SLCAN configures
the CAN bitrate and opens the adapter's CAN channel.

On Ubuntu, add your user to `dialout` if needed, then log out and back in:

```sh
sudo usermod -aG dialout "$USER"
```

Run your Python caller on the host using `UsbCan('/dev/ttyACM0')`, substituting
the device path you identified. There is no Docker device mapping for this path.

For gs_usb/candleLight, configure SocketCAN on the Linux host first:

```sh
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

The helper does not configure Linux network interfaces; its `bitrate` argument
does not change SocketCAN's host configuration. Substitute your actual CAN
interface name, then use `UsbCan('can0', interface='socketcan')` from your host
Python caller. Repeat interface configuration if unplugging the adapter removes
the CAN interface.

## Windows with the EMBR development container

Use this repository's `compose.windows.yaml` service `development` (image
`embr-bot:development`, container `embr-bot-development`). Follow the repository's
[Windows container setup](../../../../../../documentation/docker/development.md#windows-setup)
for Docker Desktop with WSL2 and VcXsrv. From the repository root:

```sh
docker compose -f compose.windows.yaml build development
docker compose -f compose.windows.yaml up -d development
docker compose -f compose.windows.yaml exec development bash
```

Inside the container, build the helper using the workspace path supplied by
the image. Install Python dependencies here, rather than in Windows Python:

```sh
cd "$EMBR_PHYS_WS"
python3 -m pip install 'python-can>=4.3,<5' 'pyserial>=3.5'
colcon build --packages-up-to embr_capstan --symlink-install
source install/setup.bash
```

The standard Compose service supports simulation with `interface='virtual'`.
It currently has no USB device mapping. Physical RH02 access needs the additional
forwarding and Compose configuration below.

### Forward the physical RH02

Windows COM ports are not directly mapped to `/dev/ttyACM0` by Docker. Use
WSL2 USB forwarding with `usbipd-win`:

1. Install WSL2 and usbipd-win following Microsoft's linked guide below.
2. In an administrator PowerShell, run `usbipd list`, identify the RH02's
   BUSID, then run `usbipd bind --busid BUSID`.
3. With a WSL2 distribution running, run `usbipd attach --wsl --busid BUSID`.
4. Inside WSL, inspect `lsusb`, `/dev/ttyACM*`, and `ip link` to identify
   the exposed backend. Follow the corresponding Linux configuration above.
5. Run the container with a Docker Engine hosted in that WSL distribution,
   so the engine sees the forwarded device and network interface. A Docker
   CLI in WSL can still target Docker Desktop's separate engine; verify the
   daemon location before relying on `--device` or `--network=host`.

For SLCAN, create a local `compose.rh02.yaml` in the repository root:

```yaml
services:
  development:
    devices:
      - "${RH02_DEVICE:-/dev/ttyACM0}:/dev/ttyACM0"
    group_add:
      - "${RH02_GID}"
```

From the WSL distribution hosting the Docker Engine, set the device's group
and recreate the same EMBR service with the override:

```sh
export RH02_DEVICE=/dev/ttyACM0
export RH02_GID="$(stat -c %g "$RH02_DEVICE")"
docker compose -f compose.windows.yaml -f compose.rh02.yaml up -d --build --force-recreate development
docker compose -f compose.windows.yaml -f compose.rh02.yaml exec development bash
```

Substitute the serial path found in WSL for `RH02_DEVICE`. Inside the container,
use `UsbCan('/dev/ttyACM0')`; the override maps the chosen host device to that
fixed container path and grants the container's development user group access.

For SocketCAN, configure `can0` inside WSL using the Linux commands above,
then use this alternative `compose.rh02.yaml` to share that WSL host's network
namespace:

```yaml
services:
  development:
    network_mode: host
```

Recreate and enter `development` using the same two-file Compose commands above
(the serial environment variables are unnecessary for this alternative).
Inside the container, use `UsbCan('can0', interface='socketcan')`.

These hardware overrides require the Docker daemon to see the forwarded device
or CAN interface. Attaching USB to your Ubuntu WSL distribution alone does not
verify visibility in Docker Desktop's daemon. The WSL-hosted Engine path above
uses the project's existing image and Compose service; it changes where Docker
runs. Docker Desktop users should verify device visibility in its backend before
using the override. Docker's separate
[USB/IP guide](https://docs.docker.com/desktop/features/usbip/) documents Windows
Hyper-V support, so it is not a verified recipe for this project's WSL2 backend.
Recreate serial device mapping after disconnecting/reconnecting USB.

USB attachment may need repeating after unplugging or restarting WSL. This
workflow requires the WSL kernel to support the adapter's serial or gs_usb
driver; the helper cannot provide that driver or USB passthrough itself.

## ODESC 3.6 use

The command helpers target ODrive **0.5.6 CANSimple** wire messages. Confirm
that your ODESC firmware implements this protocol. Configure the controller's
CAN bitrate to 1000000 and give each axis its own node ID using the firmware's
configuration tool. `node_id` is the configured CAN axis address, not the USB
port or necessarily the axis index. Connect CAN H, CAN L and the reference
ground as specified by the hardware, with 120-ohm termination at both bus ends.

For your RH02 detected as `1d50:606f` with a `can0` interface, use SocketCAN
on Linux (or inside the Windows container after exposing that interface).
The missing `/dev/serial/by-id/` directory is expected for this adapter mode.

Check the controller firmware in `odrivetool` over its own USB connection:

```python
print(dev0.fw_version_major, dev0.fw_version_minor, dev0.fw_version_revision)
# Expected: 0 5 6
```

Configure CAN in that same session, then reconnect after saving/rebooting:

```python
dev0.can.config.baud_rate = 1000000
dev0.axis0.config.can.node_id = 0
dev0.axis1.config.can.node_id = 1
dev0.save_configuration()
```

Saving configuration on 0.5.6 reboots the board, so `dev0 disappeared` and a
USB protocol error during the save can be expected. Wait for `odrivetool` to
reconnect, then verify the saved values rather than repeating the save:

```python
print(dev0.can.config.baud_rate)       # 1000000
print(dev0.axis0.config.can.node_id)   # 0
print(dev0.axis1.config.can.node_id)   # 1
```

Use standard 11-bit IDs in the controller configuration. In 0.5.6, axis node
IDs range from 0 through 63; assign unique IDs across all axes on the bus.
There is no 0.6.12-style broadcast/unaddressed node or autobaud setup here.
Firmware version must be checked over USB; CAN command 0x00 is reserved for
CANopen NMT in 0.5.6 and is not a version request. The existing
`capstan_hardware/odrive_config.py` configures motor/encoder settings through
direct controller USB; it does not configure the RH02 or replace CAN setup.

This example only requests feedback:

```python
from embr_capstan.capstan_hardware.usb_can import UsbCan, can_simple_id
import struct
import time

with UsbCan('can0', interface='socketcan') as adapter:
    adapter.request_encoder_estimates(node_id=0)
    deadline = time.monotonic() + 0.5
    while (remaining := deadline - time.monotonic()) > 0:
        frame = adapter.receive(timeout=remaining)
        if (frame is not None and not frame.is_extended_id
                and not frame.is_remote_frame and not frame.is_error_frame
                and frame.arbitration_id == can_simple_id(0, 0x09)
                and len(frame.data) == 8):
            position_turns, velocity_turns_per_second = struct.unpack('<ff', frame.data)
            print(position_turns, velocity_turns_per_second)
            break
    else:
        print('No encoder feedback from axis node 0 within 0.5 seconds')
```

For an adapter running SLCAN firmware, substitute `UsbCan('/dev/ttyACM0')`.
The loop skips other traffic, including axis heartbeats and feedback from
the second axis, instead of treating the first frame as the requested reply.

`send_frame(id, data)` sends raw frames; `send_command(node, command, data)`
constructs CANSimple IDs. Explicit motor commands are `set_axis_state`,
`set_controller_mode`, `set_input_velocity` (turns/s, torque feedforward Nm),
and `set_input_torque` (Nm). Motor configuration, calibration, watchdog policy
and periodic command scheduling belong to the caller. `close()` releases the
transport; it does not send idle or stop the motor. Transport errors propagate
to the caller, and a successful send is not confirmation of controller action.

## Simulation without USB

```python
from embr_capstan.capstan_hardware.usb_can import UsbCan

with UsbCan('capstan-sim', interface='virtual') as simulator, \
        UsbCan('capstan-sim', interface='virtual') as controller:
    simulator.set_input_torque(0, 0.25)
    frame = controller.receive(timeout=0.1)
    assert frame.arbitration_id == 0x0E
```

Virtual peers must live in the same Python process. This transports messages;
it does not simulate ODESC responses, motor dynamics, or CAN bus timing.

## References

- [RH02 hardware](https://docs.zephyrproject.org/latest/boards/jhoinrch/rh02/doc/index.html)
- [python-can SLCAN](https://python-can.readthedocs.io/en/stable/interfaces/slcan.html)
- [python-can virtual bus](https://python-can.readthedocs.io/en/stable/interfaces/virtual.html)
- [Microsoft WSL USB forwarding](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)
- [ODrive 0.5.6 CANSimple](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.6/docs/can-protocol.rst)
- [ODrive 0.5.6 wire payloads](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.6/docs/figures/can-protocol.csv)
