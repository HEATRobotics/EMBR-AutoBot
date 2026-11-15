# Thermal lab rig

A separate folder from the rover sim. Teammates clone branch `autonomy/EMBR-83/thermal-lab` and run everything below from `thermal-bench/`.

A desk check for the Lepton frame and a distance cloud, before either one is mounted on the robot. The MuJoCo scene is a table, a candle, a soldering-iron tip, and a parked EMBR body. The body is not driven. Temperatures in the scene are assumed until you drop a real frame in `inbox/`.

This is the software half of GitHub issue 70 (160×120 thermal twin, a known synthetic frame, documented limits). It does not train a policy, and it does not replace the rover sim's 9-bin thermal fan.

## Run

```bash
python -m venv .venv
.venv/bin/pip install -e ".[dev]"
.venv/bin/python -m pytest tests -q
.venv/bin/python -m thermal_bench
```

Open http://127.0.0.1:43150

On a machine that already has the rover sim's virtualenv, this also works:

```bash
PYTHONPATH=src /path/to/mujoco/.venv/bin/python -m pytest tests -q
PYTHONPATH=src /path/to/mujoco/.venv/bin/python -m thermal_bench
```

The CAD mesh is visual only. Set `EMBR_CAD_MESH` to a `.obj` if it is not beside this checkout. With no mesh, the viewer draws a box and the tests still pass.

## What the window is

The image at the top of the **EMBR lab rig** panel is the 160×120 frame. Dark blue is 20 °C. 140 °C is full brightness. Hotter than that stays at the top of the scale, and the readout prints the unclipped number. The same scale colors the distance cloud in the 3D view, so a hit on the candle is warm and the floor stays dark.

**Heat target** aims the phone at the candle or the iron. **Phone distance** is the tape measurement from the phone body to that target. The readout says closer or farther, and the straight-ahead hit in metres. **Distance cloud** swaps `phone` (forward, 0.15–5 m) for `unitree-l2` (roof dome, blind inside 5 cm, up to 15 m). Reflectance on those points is not Celsius.

## Tuesday, with the phone and the thermal board

Tape the PureThermal to the phone so both face the same way. +x is out of the lens, +z is up. The board is USB. Plug it into the laptop. The phone does not carry the thermal stream.

1. Write the tape distance. Set the slider to that number.
2. Export the phone scan as PLY, OBJ, or an Nx3 `.npy` in metres, +x forward. AirDrop it to the laptop.
3. Convert it, from this folder:

```bash
python -m thermal_bench.ingest cloud scan.ply --as phone
```

A file whose points are in millimetres is scaled to metres. More than 8000 points is thinned. The same command with `--as unitree-l2` is the later swap, once that scan exists.

4. Thermal frame, Y16 with gain off, 120×160 counts:

```bash
python -m thermal_bench.ingest thermal frame_y16.npy
```

Or, with the board plugged in and OpenCV installed (`pip install -e ".[usb]"`):

```bash
python -m thermal_bench.ingest thermal --uvc 0
```

A color picture is refused. On a Lepton 3.1R in TLinear, Celsius = count / 100 − 273.15. The datasheet example is 30000 → 26.85 °C. High gain is about −10 °C to +140 °C. A candle or an iron can sit above that, so the core clips.

5. Restart the viewer, or leave it running. `inbox/phone.npy` replaces the rays. `inbox/thermal.npy` replaces the picture. Delete either file to go back to the simulated candle.

## What is assumed

- The candle is 80 °C above a 20 °C room, a few centimetres wide. The iron tip is about 400 °C and will clip the high-gain window. Neither number was measured.
- The phone cloud here is a forward grid standing in for a scan. A real export replaces it. One view paints °C onto those hits. It is not a temperature volume of the room.
- The rover sim still senses heat as 9 bins. Growing that to this 160×120 frame is a later change, after a saved frame and a written distance exist.
- Masses, torques, and the CAD contact shape are not this folder's job. The mesh does not collide.
