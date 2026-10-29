# Human claims

What a person on this branch is willing to say. Numbers below are from this
checkout on one Mac. They are not a hardware rating and not a result from
another machine.

## Who ran what

Thanmay Gopal, on macOS with Homebrew Python 3.12, in `simulation/mujoco`:

- Installed the package with `python -m pip install -e '.[test]'`.
- Drove the CAD rover in the browser with W/A/S/D. Forward and back both move
  the robot. That drive path is the keyboard, not the follow-instructions planner.
- Ran `python -m pytest` in `simulation/mujoco`. The suite passed on this Mac
  before this branch was pushed. Ubuntu and Windows were not run here.
- Ran the packaged mission scenarios headless, including a hotspot behind a
  trunk and a wall that fills the lidar fan. Those checks are in `tests/`.
  They use the wheel proxy unless the test says `morphology="cad"`.

The author did not compare these runs to the physical EMBR. Mass, torque,
lidar range, and temperatures in the CAD model are assumed. The visual mesh
does not collide. Contact is the base plate and four idler cylinders.

## What follow-instructions is

Follow instructions uses `VfhPlanner` in `src/embr_local_sim/planners/vfh.py`.
That is a vector-field histogram on the current lidar fan, aimed at the
warmest thermal bin. It is not a trained model. This package does not train
a policy and does not ship weights.

A flame-hot bin is excluded from the goal. A wall that fills the fan backs
up and then ends in FAULT. The yard scene follows its ramp waypoints instead
of this planner.

## What a teammate can change

Manual driving stays on TeleCmd. To try another steer model, set
`Simulator.steer_planner` before `start_mission`. The object needs one
method, `steer(ctx)`, and must return a TeleCmd. `LearnedSteerPlanner` in
`planners/learned.py` is the empty slot: implement `act` and pass an
instance. Training code stays outside this package. The mission still owns
the state names, the probe, and the blocked-wall stop.
