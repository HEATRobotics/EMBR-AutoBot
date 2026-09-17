# EMBR Physical Layer

This directory contains ROS 2 packages that describe or operate the physical
robot without depending on a simulator.

Build `ros2_ws` as the base workspace for both real-robot and simulation
environments. Simulation-specific packages in `../embr_sim/ros2_ws` consume it
as an underlay.

## Keyboard movement in RViz (ROS 2 Humble)

The simple model uses mock ros2_control hardware and command-based odometry for
an RViz movement proof of concept. No Gazebo or physical motors are required.

Build from the workspace root (`embr_phys/ros2_ws`), not from
`src/embr_description`. From the repository root:

```zsh
source /opt/ros/humble/setup.zsh
cd embr_phys/ros2_ws
colcon build --packages-select embr_description --symlink-install
source install/setup.zsh
ros2 launch embr_description view_embr_simple.launch.py
```

In a second terminal:

```zsh
source /opt/ros/humble/setup.zsh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.1 -p turn:=0.5 -p repeat_rate:=10.0
```

Keep the keyboard terminal focused: `i` drives forward, `,` reverses, `j`/`l`
turn, and `k` stops. Commands repeat until changed; press `k` to stop.
RViz uses `odom` for its fixed frame, camera target, and grid reference, so
the robot moves across a stationary grid. White stripes on the wheels make
rotation visible.
The controller subscribes to `/cmd_vel` and stops after its 0.5-second command
timeout if the keyboard publisher exits.

If dependencies are missing, install them with
`rosdep install --from-paths src --ignore-src -r -y` from `embr_phys/ros2_ws`.

### Animation and restarting

The controller updates at 50 Hz, `robot_state_publisher` publishes moving-joint
transforms at up to 50 Hz, and RViz targets 60 FPS. The transform rate was raised
from its 20 Hz default to reduce stepping during turns; the actual display rate
depends on rendering performance. Wheels have fixed joints to their shafts,
so this model does not simulate mechanical lag or wheel slip.

With the symlink build, edits to existing launch, URDF, controller YAML, and RViz
files take effect after stopping and restarting the launch. Rebuild when adding
installed files or changing package dependencies or CMake installation rules.

### Setup script or CMake path errors

Use `embr_phys/ros2_ws/install/setup.zsh`, not an install directory inside
`src/embr_description`. Generated setup scripts should not be edited manually.
If CMake reports an old Docker path such as `/workspace/embr_phys_ws` while
building on the host, regenerate its cache. In a fresh terminal, from the
repository root:

```zsh
source /opt/ros/humble/setup.zsh
cd embr_phys/ros2_ws
colcon build --packages-select embr_description --symlink-install --cmake-clean-cache
source install/setup.zsh
```

Host and container builds use different absolute paths; keep their build/install
artifacts separate when alternating between environments.
