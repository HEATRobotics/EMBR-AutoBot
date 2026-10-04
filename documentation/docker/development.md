# Docker development

Docker Desktop runs the project's Ubuntu ROS 2 Humble environment for Windows
and macOS developers. On Ubuntu, use the [native Ubuntu
workflow](../workflow.md#native-ubuntu-development) unless you specifically
need the container.

## macOS setup

Install [Docker Desktop for Mac](https://docs.docker.com/desktop/setup/install/mac-install/)
and [XQuartz](https://www.xquartz.org/). In XQuartz, open **Settings /
Preferences > Security**, enable **Allow connections from network clients**,
then restart XQuartz. This enables X11 connections from the container.

Open an XQuartz terminal and allow X clients to connect:

```bash
xhost +
```

This disables X server access checks. Use it on a trusted network, and run
`xhost -` when finished. Start the container from the repository root:

```bash
docker compose -f compose.macos.yaml build development
docker compose -f compose.macos.yaml up -d development
docker compose -f compose.macos.yaml exec development bash
```

The Compose file directs X11 applications to `host.docker.internal:0.0` and
uses software rendering. Simple X11 applications can use this connection, but
RViz may fail to start: it needs an OpenGL context that XQuartz does not
reliably provide to Docker clients, particularly on Apple Silicon. The
`LIBGL_ALWAYS_SOFTWARE` setting does not guarantee RViz compatibility. For
reliable RViz model viewing, use native Ubuntu or the Windows container setup.
Docker Desktop runs Linux containers inside a Linux VM, so they do not have
direct access to Mac hardware or a Linux host's SocketCAN devices.

## Windows setup

Install Docker Desktop with the WSL2 backend and VcXsrv. Start XLaunch and:

1. Choose **Multiple windows** and set **Display number** to `0`.
2. Choose **Start no client**.
3. Enable **Disable access control**, finish, and allow VcXsrv through the
   Windows Firewall if prompted.

Keep VcXsrv running while using graphical applications. Disabling access
control allows network clients to connect to the X server, so use this setup
on a trusted network. From the repository root, run:

```bash
docker compose -f compose.windows.yaml build development
docker compose -f compose.windows.yaml up -d development
docker compose -f compose.windows.yaml exec development bash
```

## Build and run the model

The container builds the physical workspace on its first start and automatically
sources ROS 2 Humble and the physical workspace in its shells. Launch the model
inside the container:

```bash
ros2 launch embr_description view_embr_simple.launch.py
```

See [robot model visualization](../model-visualization.md) for what this
launch displays. Shut down the container when finished:

```bash
docker compose -f compose.macos.yaml down
```

Use `compose.windows.yaml` instead of `compose.macos.yaml` on Windows.
