# Docker autonomy environment

The `autonomy` service provides a separate ROS 2 Jazzy / Ubuntu 24.04
container for reinforcement-learning and simulation-rendering workflows. It
includes CPU-only PyTorch and torchvision, MuJoCo, Gymnasium, Stable-Baselines3,
LeRobot, H5py, Weights & Biases, ONNX and ONNX Runtime. It also includes
headless OpenCV, MoviePy, Matplotlib, pandas, FFmpeg and OpenGL/EGL/OSMesa
libraries for headless rendering, video, and data workflows. The ROS 2 Jazzy
Nav2 packages are installed in this image as well.

The autonomy image is separate from the regular development image. It is
profile-gated, so normal Docker development commands continue to use only the
existing `development` service and do not install or start these additional
dependencies.

## Prerequisites

- Docker Desktop.
- For rendered windows, an X server: XQuartz on macOS or VcXsrv on Windows.
  Follow the setup instructions in [Docker development](development.md).

## Build and start

Run commands from the repository root. The `autonomy` profile must be enabled
to build or start the service.

On Windows:

```bash
docker compose --profile autonomy -f compose.windows.yaml build autonomy
docker compose --profile autonomy -f compose.windows.yaml up -d autonomy
docker compose --profile autonomy -f compose.windows.yaml exec autonomy bash
```

On macOS, use `compose.macos.yaml` instead:

```bash
docker compose --profile autonomy -f compose.macos.yaml build autonomy
docker compose --profile autonomy -f compose.macos.yaml up -d autonomy
docker compose --profile autonomy -f compose.macos.yaml exec autonomy bash
```

Both Compose files mount the repository at
`/home/embr/EMBR-AutoBot` and configure software rendering. Keep scripts,
datasets, and generated results in the mounted repository if they need to
persist on the host. The image sets `MUJOCO_GL=osmesa` and
`PYOPENGL_PLATFORM=osmesa` for software rendering without a display. The
PyTorch build is CPU-only; GPU acceleration is not configured by this image.
Stable-Baselines3 provides PPO and SAC; use `SubprocVecEnv` for parallel PPO
workers and a `DummyVecEnv` where required by a single-environment SAC
workflow. Evaluation callbacks and W&B logging still need to be configured in
your autonomy application.

Nav2 is installed, but it needs robot-specific configuration (maps, frames,
sensor topics, and parameters) before it can navigate EMBR. Point-LIO and
FAST-LIO are not included: ROS 2 ports depend on the specific LiDAR hardware
and driver, and the project has not selected a compatible ROS 2 Jazzy port.
Choose the robot's LiDAR model and corresponding port before adding either
odometry implementation to this image.

To leave the regular development container alone, stop only the autonomy
service:

```bash
docker compose --profile autonomy -f compose.windows.yaml stop autonomy
```

Use the macOS Compose filename on macOS. You can also use a temporary autonomy
container without starting a background service:

```bash
docker compose --profile autonomy -f compose.windows.yaml run --rm autonomy
```

## Python dependencies

The image installs packages listed in
[`docker/requirements-rl.txt`](../../docker/requirements-rl.txt) into an
isolated Python virtual environment. To add or pin autonomy dependencies,
update that file and rebuild the `autonomy` image. These packages are not added
to the regular `development` image. ROS dependencies are installed separately
in `docker/autonomy.Dockerfile`.

Run RL code with rlpy (for example `rlpy train.py, rlpy -m pip list`). `python3` and `ros2` use the system Python with ROS. The two can't share a process because of the numpy 1/2 split. Connecting RL to ROS will be decided in a follow-up.
