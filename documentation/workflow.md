# Development workflow

## Native Ubuntu development

On Ubuntu 22.04, install ROS 2 Humble using the [official ROS 2 Humble
installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
Then, from the repository root, install the project's system and ROS package
dependencies:

```bash
bash scripts/environment/setup_native_ubuntu.sh
```

Build the physical workspace:

```bash
source /opt/ros/humble/setup.bash
cd embr_phys/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

In each new terminal, source the workspace before running ROS commands:

```bash
source /opt/ros/humble/setup.bash
source embr_phys/ros2_ws/install/setup.bash
```

For RViz model viewing, see [robot model visualization](model-visualization.md).

## Windows and macOS Docker development

Windows and macOS users can use the Docker Desktop container documented in
[Docker development](docker/development.md). Ubuntu users can also use Docker,
but native development is the recommended path on Ubuntu.
