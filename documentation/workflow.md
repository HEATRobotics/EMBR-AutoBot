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

### Updating an existing checkout

From the repository root, pull the latest changes and rerun the same dependency
setup command:

```bash
git pull --ff-only
bash scripts/environment/setup_native_ubuntu.sh
```

The script requires Ubuntu 22.04 and ROS 2 Humble. If `ROS_DISTRO` is set to
another distribution, use a fresh Humble terminal. It refreshes APT and rosdep
metadata, installs newly declared dependencies, and checks that all workspace
dependencies are satisfied. You can rerun it whenever dependencies change.
Run it as your normal user; it requests sudo for system package installation.

Rebuild after updating:

```bash
source /opt/ros/humble/setup.bash
cd embr_phys/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

For zsh terminals, use `setup.zsh` in the sourcing commands above. Continue to
run the dependency setup script with `bash`.

This command installs missing ROS dependencies; it does not upgrade every
already-installed ROS dependency. Host tools explicitly listed in
`ubuntu-packages.txt` may be upgraded by APT. Dependencies removed from the
project remain installed.

### Declaring dependencies

Add shared host development tools to
`scripts/environment/ubuntu-packages.txt`, one APT package per line. Add a ROS
package's build, runtime, or test dependencies to that package's `package.xml`
under `embr_phys/ros2_ws/src`; rosdep resolves them to system packages.
Both native setup and the Docker build use these declarations, so no separate
native dependency list is needed.

## Windows and macOS Docker development

Windows and macOS users can use the Docker Desktop container documented in
[Docker development](docker/development.md). Ubuntu users can also use Docker,
but native development is the recommended path on Ubuntu.
