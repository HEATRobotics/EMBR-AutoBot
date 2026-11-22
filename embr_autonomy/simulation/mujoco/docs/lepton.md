# PureThermal 3 / Lepton stream

The viewer can read a FLIR Lepton on a GroupGets PureThermal 3 that is plugged
into the operator laptop. MuJoCo keeps stepping if that camera is absent.
The image is the laptop's view. It is not a render of the simulated grove, and
the camera is not mounted on the simulated robot.

## What is converted

PureThermal boards expose the Lepton as a USB video device. The capture thread
asks for Y16 (16-bit gray), 160×120.

| Frame | What the code does |
| --- | --- |
| 160×120, pixels in centi-Kelvin | `celsius = (raw - 27315) / 100`. A raw value of 30000 is 26.85 °C. This is the GroupGets `ktoc` conversion used for radiometric Lepton output (Lepton 3.1R and 3.5). |
| 160×121 through 160×124 | The rows after 120 are telemetry. They are dropped and are not treated as temperature. |
| 14-bit raw, maximum at or below 16383 | Stored and shown as a picture. **Celsius is left invalid.** A non-radiometric Lepton 3.1 emits this kind of frame. This package does not invent a °C scale for it. |
| All zeros, or all 65535 | Marked invalid (dropped packet or the flat-field shutter). |
| No device, or OpenCV missing | The capture thread records the error and retries every 2 s. Physics uses the synthetic Gaussian field. |
| Windows 8-bit BGR, shape `(120, 160, 3)` | Not converted. DirectShow's color converter ate the samples. The reader asks for Y16 after the device is open and accepts only a `uint16` frame, 160 wide and 120 or 122 tall. |
| Raw pixel 0 inside a Y16 frame | Left out of min / mean / max. `(0 - 27315) / 100` is −273.15 °C and is not a scene temperature. |

The display picture is a 1st–99th percentile stretch. It is for the browser
panel. Mission code does not read those 8-bit colors.

FLIR states radiometric accuracy of about ±5 °C or ±5% in high gain for a
radiometric module, and about ±10 °C in low gain. No blackbody reading from
this board is in the repository, so that tolerance is not checked here.

Lepton 3.1 (no R) and Lepton 3.1R are different parts. The 3.1R is the
radiometric 160×120 module GroupGets pairs with PureThermal 3 for temperature.
If `launch --lepton` reports `14-bit raw counts`, the module is not sending
T-linear Celsius. Changing the formula will not fix that.

On Windows, the opener asks for T-linear before the Y16 stream starts. It
writes three PureThermal RAD controls, four little-endian bytes each, and
reads them back:

| Control | Command | Value |
| --- | --- | --- |
| 5 | radiometry enable (`0x10`) | 1 |
| 50 | T-linear resolution (`0xC4`) | 1, which is 0.01 K per count |
| 49 | T-linear enable (`0xC0`) | 1 |

The DirectShow capture filter on this host does not expose the radiometry
node, so the same camera index is opened by its device path for that write
and closed before OpenCV takes the camera. The launch log prints the three
read-back values and the Y16 median. A room in this mode is near raw
29000–31000, about 17–37 °C. A failed control means the module did not accept
radiometry. The Y16 stream still starts, and no other Celsius scale is used.

## Threads

```text
embr-lepton          USB read, normalize, store one LeptonFrame under a lock
embr-physics         copies latest() into SensorObservations at the control tick
Viser render loop    draws display_rgb and the min/mean/max line
```

The lepton thread does not call MuJoCo. A frame older than 0.50 s is reported
stale and is not Celsius. The synthetic thermal bins stay in place unless
`--lepton-drive` is set. That flag replaces `thermal_c` with the max °C of
each horizontal slice of the live image. Leave it off for wildfire and yard:
those scenes' heat is not in front of the laptop camera.

`SensorObservations.lepton_celsius` and `MissionView.lepton_celsius` hold the
120×160 float image when conversion succeeded. Planners can read it. The
mission state machine still follows `thermal_c`.

## Run

From `embr_autonomy/simulation/mujoco`, in the same virtualenv as the viewer:

```bash
python -m pip install -e ".[lepton]"
embr-local-sim launch --lepton
embr-local-sim launch --lepton --lepton-index 1
embr-local-sim cad --lepton
```

`--lepton-index` is the UVC camera index, default 0. On Windows the opener
tries DirectShow first. OpenCV is not in `requirements.txt`. That pin file is
the set that was run without a camera attached.

Without `--lepton`, behavior matches the previous viewer. `embr-local-sim doctor`
lists `cv2` as optional.

## What was not tested on hardware

- Opening the team's PureThermal 3, including the T-linear control write.
  The board was unplugged when that write was added, so the three RAD values
  and the Y16 median were not read back on hardware. Selector 5 / 49 / 50 and
  the four-byte payload are covered without a camera. Set `LEPTON_DEVICE` to
  the camera index and run pytest to attempt one live read.
- Agreement with a known surface temperature. There is no blackbody log.
- Which telemetry words this firmware puts in the extra rows. The rows are
  discarded rather than decoded.
- Radiometric accuracy across high-gain and low-gain, or during the shutter
  event beyond the all-zero / all-ones check.

The conversions, the stale flag, a missing-device thread, and the copy into
`Simulator.snapshot` are covered by `tests/test_lepton.py` with synthetic buffers.
