# EMBR Autonomy Layer

Work in progress. This directory holds autonomy, simulation, and RL work for
EMBR. Run `ros2_ws/` and `rl/` inside the
[autonomy container](../documentation/docker/autonomy.md) (ROS 2 Jazzy).

| Location | Purpose | Run with |
| --- | --- | --- |
| `ros2_ws/` | ROS 2 Jazzy packages (navigation, Nav2 config, autonomy nodes) | `python3`, `ros2`, `colcon` |
| `rl/` | MuJoCo simulation and RL training code | `rlpy` |
| [`simulation/mujoco/`](simulation/mujoco/README.md) | Native MuJoCo proxy: install, drive, and headless check. Runs in its own venv, not in the container. | `embr-local-sim` |

Keep the two sides separate: `rlpy` and ROS use different NumPy versions and
can't share a process. How RL connects to ROS is still to be decided.

## Build the ROS workspace

From the repository root, inside the autonomy container:

```bash
cd embr_autonomy/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

For a clean build, delete `build/`, `install/`, and `log/` first.
