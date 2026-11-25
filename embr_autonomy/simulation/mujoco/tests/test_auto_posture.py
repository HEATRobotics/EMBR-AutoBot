"""Auto posture: the mission adjusts height, pitch, and roll from the IMU and encoders."""

from __future__ import annotations

import math

import pytest

from embr_local_sim.auto_posture import AutoPosture
from embr_local_sim.body import apply_body
from embr_local_sim.hot_room import begin_hotspot_mission
from embr_local_sim.model import RobotParams, WorldSpec
from embr_local_sim.physics import Simulator

DT = 0.02


def test_levelling_pushes_against_the_measured_tilt() -> None:
    auto = AutoPosture()
    for i in range(50):
        h, p, r = auto.update(t=i * DT, roll_rad=0.0, pitch_rad=math.radians(-6.0), reason="vfh", dt=DT)
    # Nose up on a climb: command nose down (+ pitch), and lift for clearance.
    assert p > 0.1 and r == pytest.approx(0.0)
    assert auto.mode == "slope" and h == pytest.approx(auto.config.slope_height)
    for i in range(50):
        _h, _p, r = auto.update(t=1 + i * DT, roll_rad=math.radians(5.0), pitch_rad=0.0, reason="vfh", dt=DT)
    assert r < -0.1


def test_small_tilts_and_flat_ground_stay_at_cruise() -> None:
    auto = AutoPosture()
    for i in range(100):
        h, p, r = auto.update(t=i * DT, roll_rad=math.radians(0.5), pitch_rad=math.radians(-0.5), reason="vfh", dt=DT)
    assert (h, p, r) == (auto.config.cruise_height, 0.0, 0.0)
    assert auto.mode == "cruise"


def test_a_jam_or_an_encoder_stall_lifts_the_body() -> None:
    auto = AutoPosture()
    auto.update(t=0.0, roll_rad=0.0, pitch_rad=0.0, reason="backing_up", dt=DT)
    assert auto.mode == "unstick"

    auto = AutoPosture()
    for i in range(int(0.9 / DT)):
        auto.update(t=i * DT, roll_rad=0.0, pitch_rad=0.0, reason="vfh", dt=DT, commanded_mps=0.35, wheel_mps=0.02)
    assert auto.mode == "cruise", "a short stall is just acceleration"
    for i in range(int(0.9 / DT), int(1.5 / DT)):
        h, _p, _r = auto.update(t=i * DT, roll_rad=0.0, pitch_rad=0.0, reason="vfh", dt=DT, commanded_mps=0.35, wheel_mps=0.02)
    assert auto.mode == "unstick" and h == pytest.approx(auto.config.stuck_height)


def _peaked_hump_world() -> WorldSpec:
    """A 7 degree hump with no deck: the 11 cm crest is taller than the level belly clearance."""
    walls = [
        {"x": 6.0, "y": -0.05, "size": (6.0, 0.05, 0.2)},
        {"x": 6.0, "y": 4.05, "size": (6.0, 0.05, 0.2)},
        {"x": -0.05, "y": 2.0, "size": (0.05, 2.0, 0.2)},
        {"x": 12.05, "y": 2.0, "size": (0.05, 2.0, 0.2)},
    ]
    return WorldSpec(
        kind="hot-room",
        ambient_c=20.0,
        stones=walls,
        heat=[{"x": 9.5, "y": 2.0, "z": 0.05, "peak_c": 80.0, "sigma": 0.6, "kind": "smolder", "provenance": "assumed"}],
        extras={
            "floor_half": 14.0,
            "spawn": {"x": 1.2, "y": 2.0, "yaw": 0.0},
            "spot": (9.5, 2.0),
            "skip_pass": True,
            "aim_at_spot": True,
            "body_half_width": 0.6,
            "ramps": [{"x": 3.5, "y": 2.0, "width": 3.9, "length": 0.9, "deck": 0.0, "angle_deg": 7.0}],
        },
    )


@pytest.mark.parametrize("auto", [False, True])
def test_auto_posture_clears_a_crest_that_high_centres_the_level_body(auto: bool) -> None:
    sim = Simulator(params=apply_body(RobotParams(morphology="urdf")), world=_peaked_hump_world(), seed=1)
    sim.reset()
    sim.set_auto_posture(auto)
    begin_hotspot_mission(sim, histogram=True)
    belly = False
    while sim.data.time < 60.0 and sim.mission.state not in ("SAMPLE", "COMPLETE", "FAULT"):
        sim.step_control_tick()
        assert sim.finite()
        contacts = sim.data.contact[: sim.data.ncon]
        belly = belly or any("chassis" in (sim.model.geom(c.geom1).name, sim.model.geom(c.geom2).name) for c in contacts)
    if auto:
        assert sim.mission.state in ("SAMPLE", "COMPLETE")
        assert not belly
    else:
        assert sim.mission.state not in ("SAMPLE", "COMPLETE")
        assert belly
