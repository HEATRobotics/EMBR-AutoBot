"""ROS 2 adapter contract. This is not a ROS node and does not import rclpy at module load.

ROS runtime on this host: **unverified** (no Humble/Jazzy, no rclpy).

Intended mapping once a real adapter exists
------------------------------------------
- relative `tele_cmd` — `embr_interfaces/msg/TeleCmd` (`float64 velocity`, `float64 turn`),
  normalized [-1, 1], positive turn = right. Names are relative; do not hardcode a
  leading slash without discovering remaps.
- simulation helper publishes `geometry_msgs/msg/Twist` on relative `cmd_vel`:
  `linear.x = velocity * max_linear_speed`, `angular.z = -turn * max_angular_speed`.
- hardware helper publishes `Float32MultiArray` on `motor_velocity_levels`,
  order front-left, rear-left, front-right, rear-right.
- Helper command timeout default 0.5 s; iBUS frame timeout 0.25 s; this simulator's
  actuator watchdog 0.40 s. Three different failures.
- `odom` — wheel odometry, **not** ground-truth pose.
- `tf` — one publisher owns `odom` → `root`.
- `scan` — labeled 2D fan in frame `lidar` (not a Velodyne driver).
- thermal / probe — local observations; no team message type is claimed here.
- `ground_truth/pose` — oracle only.

Do not also run `mock_components/GenericSystem` open-loop odom against the same joints.
Do not launch CANopen hardware entry points for a laptop demo.

Capstan helper shares `tele_cmd` and uses **velocity only**. This sandbox keeps
capstan on a separate bus so the forward stick does not articulate a test-stand
axis while driving.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RosAdapterContract:
    tele_cmd_topic: str = "tele_cmd"
    tele_cmd_type: str = "embr_interfaces/msg/TeleCmd"
    tele_cmd_normalized: bool = True
    turn_positive: str = "right"
    cmd_vel_topic: str = "cmd_vel"
    cmd_vel_type: str = "geometry_msgs/msg/Twist"
    cmd_vel_stamped: bool = False
    motor_levels_topic: str = "motor_velocity_levels"
    motor_order: tuple[str, ...] = (
        "front_left_maxon",
        "rear_left_maxon",
        "front_right_maxon",
        "rear_right_maxon",
    )
    helper_timeout_s: float = 0.5
    ibus_timeout_s: float = 0.25
    sim_watchdog_s: float = 0.40
    joint_states_topic: str = "joint_states"
    odom_topic: str = "odom"
    scan_topic: str = "scan"
    scan_frame: str = "lidar"
    ground_truth_topic: str = "ground_truth/pose"
    odom_frame: str = "odom"
    base_frame: str = "root"
    control_hz: float = 50.0
    ros_runtime: str = "unverified"


def require_ros_adapter() -> None:
    try:
        import rclpy  # noqa: F401
    except ImportError as exc:
        raise NotImplementedError(
            "ROS runtime unverified: rclpy is not installed. See "
            "embr_local_sim.ros_adapter. Core physics does not depend on ROS."
        ) from exc
    raise NotImplementedError(
        "rclpy imported but no adapter node is implemented in this sandbox. "
        "Do not treat import success as a running /cmd_vel bridge."
    )
