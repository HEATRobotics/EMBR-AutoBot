"""The overlay draws what the planner saw. It does not steer."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from embr_local_sim.mission import MissionView
from embr_local_sim.planners.vfh import vfh_steer
from embr_local_sim.sensor_overlay import heat_rgb, thermal_strip_image, vfh_bars_image, vfh_fan


def _view_with_wall_on_the_right() -> MissionView:
    angles = list(np.linspace(-1.0, 1.0, 21))
    ranges = [0.5 if a < 0.0 else 8.0 for a in angles]
    return MissionView(
        sim_time=0.0,
        stamp_wall=0.0,
        odom_x=0.0,
        odom_y=0.0,
        odom_yaw=0.0,
        lidar_ranges=ranges,
        lidar_angles=angles,
        lidar_stamp_sim=0.0,
        thermal_c=[],
        thermal_angles=[],
        probe_c=20.0,
        probe_stamp_sim=0.0,
        probe_frame="probe",
        lidar_max_range=8.0,
    )


def test_vfh_records_its_bins_and_picks_the_open_side() -> None:
    memory: dict = {}
    vfh_steer(_view_with_wall_on_the_right(), 0.0, wall=0.0, cruise=0.5, memory=memory)
    debug = memory["vfh_debug"]
    assert len(debug["centers"]) == len(debug["density"]) == 15
    right = [d for c, d in zip(debug["centers"], debug["density"]) if c < -0.2]
    left = [d for c, d in zip(debug["centers"], debug["density"]) if c > 0.2]
    assert min(right) > max(left)
    assert debug["heading"] is not None and debug["heading"] > 0.0


def test_the_fan_has_one_spoke_per_sector_plus_goal_and_heading() -> None:
    memory: dict = {}
    vfh_steer(_view_with_wall_on_the_right(), 0.0, wall=0.0, cruise=0.5, memory=memory)
    segs, colors = vfh_fan(np.zeros(3), np.eye(3), memory["vfh_debug"], 2.0)
    assert segs.shape == (17, 2, 3) and colors.shape == (17, 2, 3)
    assert vfh_bars_image(memory["vfh_debug"]).ndim == 3


def test_hotter_bins_are_brighter_and_left_is_drawn_on_the_left() -> None:
    colors = heat_rgb([20.0, 55.0, 90.0], 20.0, 90.0)
    assert colors[0].sum() < colors[1].sum() < colors[2].sum()
    # Bin 0 is the rightmost angle, so it lands on the right of the image.
    obs = SimpleNamespace(thermal_c=[90.0, 20.0, 20.0])
    img = thermal_strip_image(obs, 20.0, 90.0, cell=4)
    assert img[:, -4:].sum() > img[:, :4].sum()
