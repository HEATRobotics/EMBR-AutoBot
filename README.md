<p align="center">
  <a href="https://github.com/HEATRobotics">
    <img src="documentation/assets/heat-robotics-logo.webp" alt="HEAT Robotics logo" width="160" />
  </a>
</p>

# EMBR AutoBot

This repository is dedicated to the development of EMBR and it's autonomy. Our goals this year is to get
EMBR up and running through remote control, and eventually reach a basic level of autonomy. We plan to achieve 
this through simulations and real world testing. By the end of the year we expect to be able to not only remote control
EMBR, but for EMBR to be able to navigate a simple space, find hotspots, and log them, on its own. 

# Welcome

Welcome to EMBR, HEAT Robotics’ robot software workspace. Whether you are joining
the team, returning for another term, or exploring the project, this guide will
help you understand what we are building and make your first contribution.

[About](#about) · [EMBR Goals](#embr-goals) · [Getting Started](#getting-started) · [Contributing](#contributing) · [Documentation Standards](#documentation-standards) · [Maintainers](#maintainers)

## Maintainers

Maintained by the [HEAT Robotics team](https://github.com/HEATRobotics).
For repository access, project direction, and a first task, connect with the team.
For reproducible bugs or documentation questions, open a
[GitHub issue](https://github.com/HEATRobotics/EMBR-AutoBot/issues).

## About

HEAT Robotics is a student engineering team at UBC Okanagan developing robotics
for wildfire detection and response. EMBR stands for **Ember Mitigation Bot
Responder** and supports the team’s work on finding residual hotspots after a
wildfire. Learn more in [UBC’s introduction to EMBR](https://engineering.ok.ubc.ca/2024/05/29/soe-feature-learn-more-about-the-heat-robotics-team-and-their-award-winning-embr-project/).

This repository brings together robot descriptions, control software, and
simulation environments built around **ROS 2 Humble**, **RViz**, and **Gazebo
Fortress**. Physical robot packages form the base workspace;
simulation packages build on top of them.

**Website:** [HEAT Robotics’ UBC team profile](https://experience.apsc.ubc.ca/okanagan/student-groups/design-teams-clubs-and-associations).

**Follow the team:** [GitHub](https://github.com/HEATRobotics) ·
[LinkedIn](https://www.linkedin.com/company/heat-robotics/) ·
[Instagram](https://www.instagram.com/heat.robotics?stkn=d2o2YjZ2cnAwc3Zp)

## EMBR Goals

- **Support wildfire response:** develop a robot platform that can help locate
  residual hotspots and support responders.
- **Build dependable motion and control:** connect robot models, ROS 2
  interfaces, and CANopen motor integration.
- **Validate through simulation:** develop repeatable visualization and
  simulation workflows before integrating changes with hardware.
- **Make the project sustainable:** keep setup instructions, design decisions,
  and validation results accessible to the next group of contributors.

**Current starting point:** the simple robot can be driven in RViz with mock
hardware and command-based odometry. CANopen simulation contains scaffolding;
full motor and terrain physics remain development work.

### Road to the End of the 2026–2027 Academic Year

![Academic-year timeline progress](documentation/assets/academic-year-progress.svg?v=1f1662b99c823ab9)

The bar tracks **elapsed calendar time**, not completed engineering milestones.
The window is **September 1, 2026 – August 31, 2027**, using Vancouver dates.
The bar reaches 100% on August 31, 2027.

<details>
<summary>How the daily update works</summary>

The [timeline workflow](.github/workflows/readme-progress.yml) regenerates the
image daily and updates its versioned README link to refresh image caches.
It commits these files only when they change. Scheduled updates begin after
the workflow reaches the default branch, with GitHub Actions enabled and bot
commits permitted by repository rules. GitHub may delay scheduled runs or disable
them after inactivity; see [GitHub’s schedule documentation](https://docs.github.com/en/actions/reference/workflows-and-actions/events-that-trigger-workflows#schedule).

To change the window, edit `START` and `END` in
[`scripts/documentation/update_progress.py`](scripts/documentation/update_progress.py),
update the dates above, and run `python3 scripts/documentation/update_progress.py`.
The workflow can also be run manually from GitHub Actions.

</details>

## Getting Started

You do not need to know every part of the stack to contribute. Start with a
working visualization, then choose a small task with a maintainer.

1. **Connect with the team.** Confirm repository access and the area you will
   work on: robot descriptions, controls, simulation, or documentation.
2. **Get the source.** Clone the repository and enter its root directory:

   ```bash
   git clone https://github.com/HEATRobotics/EMBR-AutoBot.git
   cd EMBR-AutoBot
   ```

3. **Choose an environment.** Use an existing ROS 2 Humble installation for the
   host RViz walkthrough, or the Linux Docker environment below.
4. **Reach a first milestone.** Follow the
   [keyboard movement walkthrough](embr_phys/README.md#keyboard-movement-in-rviz-ros-2-humble)
   to view and drive the simple robot in RViz.

### Linux Docker Environment

Use a Linux machine with Docker Engine, the Docker Compose plugin, and an X11
display (or configured XWayland) for RViz. The current Compose configuration
expects `/dev/dri`, `/tmp/.X11-unix`, a valid `DISPLAY`, and an `XAUTHORITY` file.
The Windows Compose file is currently a placeholder.

From the repository root, copy `.env.example` to `.env` if you do not already
have one. Set `LOCAL_UID` and `LOCAL_GID` to the values from `id -u` and `id -g`.
Ensure `XAUTHORITY` points to your session’s X11 authorization file.

```bash
# Select the Linux configuration for this terminal session.
export COMPOSE_FILE=compose.linux.yaml

docker compose build canopen-sim
# Continue only after the build succeeds.
docker compose up -d canopen-sim
docker compose exec canopen-sim bash
```

Inside the container, build and source the physical workspace first:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/embr_phys_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch embr_description view_embr_simple.launch.py
```

Continue with the [development workflow](documentation/workflow.md) for keyboard
control, building the simulation overlay, and the edit/build/relaunch cycle.
Set `COMPOSE_FILE=compose.linux.yaml` in each new host terminal when following
that guide. Host and container builds use different paths; keep their generated
build artifacts separate.

## Find Your Way Around

| Location | Purpose |
| --- | --- |
| [`embr_phys/`](embr_phys/README.md) | Physical workspace: robot descriptions, core packages, and interfaces |
| [`embr_sim/`](embr_sim/README.md) | Simulation workspace: Gazebo and CANopen packages |
| [`documentation/workflow.md`](documentation/workflow.md) | Everyday build, visualization, and development steps |
| [`documentation/architecture.md`](documentation/architecture.md) | Workspace boundaries and simulation architecture |
| [`docker/`](docker/) | Container images and entrypoint configuration |
| [`scripts/`](scripts/) | Environment checks, simulation helpers, and documentation automation |
| [`compose.linux.yaml`](compose.linux.yaml) | Linux development service configuration |

## Contributing

Discuss larger changes in an issue before implementation so the team can agree
on scope and interfaces. For each contribution:

1. Name your branch using the format `subsystem/SWE-ID/short-description`.
   Supported subsystems are `drivetrain`, `capstan`, `teleoperation`, `thermals`,
   `lidar`, and `autonomy`.
2. Keep the change focused and update relevant documentation alongside it.
3. Build the affected ROS packages and run relevant tests or launch checks.
   Include the commands and results in your pull request; screenshots help for
   RViz or model changes.
4. Open a pull request describing the problem, the resulting behavior, and any
   remaining limitations. Each pull request must receive two reviews and pass
   all enabled status checks.

Keep generated `build/`, `install/`, and `log/` directories, local `.env` files,
and credentials out of commits.

## Documentation Standards

Record initial documentation in the Notion ticket associated with your SWE-ID.
Before opening a pull request, organize and move that documentation into the
team’s documentation database in Notion.

Keep documentation in the repository focused on:

- Setup and build instructions.
- Architecture and technical decisions.
- Instructions that change with the code.

Keep code readable and as simple as practical. Use comments to explain intent
rather than describe every line. Document each function’s purpose, expected
inputs, and return values.

## Need a Hand?

For setup problems, include your operating system, whether you are running on
the host or in Docker, the command that failed, and the relevant error output in
an issue. Remove credentials and personal information from logs before sharing.

If you are new to ROS or Git, let the team know. A clear bug report, a corrected
setup step, or a small documentation improvement is a useful first contribution.
