# Inspection tests

This is a local synthetic mission on the approximate robot. It is not
hardware-validated autonomy and it is not a wildfire model. The controller
uses wheel travel, a synthetic gyro for heading, the forward range fan, and
the hinge probe. It does not read the hidden obstacle list or the true
hotspot field.

Tracks are the model that can turn around and come back. The wheel proxy
usually cannot hold a turn long enough to face the return path. That is a
failed return, not a success.

Reports from the automated runs are written to `embr-sim-runs/inspection/`.
A browser run writes `embr-sim-runs/inspection/manual-last.json` when the
mission stops. Run the commands below from `embr_autonomy/simulation/mujoco`.

No new packages. The existing install is enough.

```bash
cd embr_autonomy/simulation/mujoco
.venv/bin/embr-local-sim launch --host 127.0.0.1 --port 43170
```

Open the URL the terminal prints. If that port is taken, pick another free port.

Automated check:

```bash
cd embr_autonomy/simulation/mujoco
.venv/bin/pytest tests/test_inspection.py -q
```

| Step | What to do | What you should see | A failure looks like |
| --- | --- | --- | --- |
| 1 | Open the page the terminal prints. In **Inspection tests**, choose **clear**, model **tracks**. Press **Reset scenario**. | Flat ground, a blue start dot and an orange target dot. Status says Idle. | The old grove stays, or the dots never appear. |
| 2 | Press **Start mission**. | State walks through navigate, stop, deploy, sample, retract, return, then **COMPLETE**. The reading is about 20 °C and labeled **cold**. That still counts. | It drives off and ends in **FAULT timeout**, or the status never changes. |
| 3 | Choose **detour**, **Reset scenario**, **Start mission**. | It steers around the post, samples, and returns. The evaluator report has no stone contact. | It sits against the post, or **COMPLETE** while touching it. |
| 4 | Choose **blocked**, then **dropout**, then **retract**. Reset and start each one. | **blocked** stops with `no_observed_passage` and does not say the target is unreachable. **dropout** stops with `sensor_invalid` or `stale_lidar`. **retract** samples, then stops with `probe_retract_failed` and does not drive away. | Any of those ends **COMPLETE**, or the robot keeps driving after the fault. |
| 5 | During a clear run, press **Manual takeover** or **Cancel / Stop**. Then press **Pause / resume** on a new run. | Takeover or cancel shows **CANCELLED** and stays there until you press Start again. Pause freezes the mission time. | The mission keeps driving after Stop, or the mission clock keeps climbing while paused. |

**Start mission** uses the target sliders. Those world metres are converted once, from the spawn pose, into the odometry frame. After that the controller only follows that goal with its own estimate.

The orange line is a bearing, not a route. **Evaluation only** shows ground-truth distances. Those numbers are separate from the estimated distances in the status line.

Steering uses the current scan. A robot-width gap that is long enough to pass wins over a short look straight at the goal, and the robot turns to face that gap before it drives. After it clears the post it drives straight for a short distance, then turns back. A closed corridor still stops as blocked. Moving the posts does not require a new motion script.
