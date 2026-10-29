"""Inspection mission. Tracks are the morphology that can turn and return.

The wheel proxy is checked separately. It is not expected to complete the
return: it loses yaw grip, and the evaluator must not call that a success.
"""

from __future__ import annotations

import math
import random

from embr_local_sim.autonomy import WindowProgress, select_gap
from embr_local_sim.inspection import InspectionConfig, InspectionMachine, classify_probe_c
from embr_local_sim.inspection_run import _stone, run_case
from embr_local_sim.mission import MissionView
from embr_local_sim.model import RobotParams
from embr_local_sim.physics import Simulator, UiRequest
from embr_local_sim.probe import ProbeMachine
from embr_local_sim.tele_cmd import TeleCmd


def _view(**kwargs) -> MissionView:
    base = dict(
        sim_time=0.0,
        stamp_wall=0.0,
        odom_x=0.0,
        odom_y=0.0,
        odom_yaw=0.0,
        lidar_ranges=[2.0] * 11,
        lidar_angles=[(i - 5) * 0.2 for i in range(11)],
        lidar_stamp_sim=0.0,
        thermal_c=[20.0],
        thermal_angles=[0.0],
        probe_c=20.0,
        probe_stamp_sim=0.0,
        probe_frame="probe",
        probe_state="STOWED",
        probe_valid=True,
        lidar_valid=True,
        lidar_max_range=2.0,
    )
    base.update(kwargs)
    return MissionView(**base)


def test_gap_rejects_a_single_long_ray():
    # Same fan density as the simulator: 21 rays across 120 degrees.
    fov = 1.05
    angles = [-fov + 2.0 * fov * i / 20.0 for i in range(21)]
    ranges = [0.22] * 21
    ranges[0] = 5.0
    view = _view(lidar_ranges=ranges, lidar_angles=angles, lidar_max_range=5.0)
    heading, clear, why = select_gap(view, 0.0, 0.09)
    assert why == "gap_selected"
    assert clear < 0.5
    assert heading is None or abs(heading - angles[0]) > 0.15 or clear < 0.5


def test_missing_ranges_are_not_free_space():
    view = _view(lidar_ranges=[], lidar_angles=[], lidar_valid=False)
    heading, clear, why = select_gap(view, 0.0, 0.09)
    assert heading is None
    assert clear == 0.0
    assert why == "range_unobserved"


def test_progress_window_matches_at_two_rates():
    def run(dt: float, moving: bool) -> bool:
        watch = WindowProgress()
        stalled = False
        t = 0.0
        x = 0.0
        while t <= 1.2:
            if moving:
                x = 0.1 * t
            stalled = watch.stalled(
                sim_time=t, x=x, y=0.0, dist_to_goal=1.0 - x, window_s=1.0, active=True
            )
            t += dt
        return stalled

    assert run(0.02, moving=False) is True
    assert run(0.05, moving=False) is True
    assert run(0.02, moving=True) is False
    assert run(0.05, moving=True) is False


def test_sample_dwell_starts_at_sampling_pose_and_cold_is_allowed():
    machine = InspectionMachine(InspectionConfig(goal_x=0.05, goal_y=0.0, sample_dwell_s=0.4))
    machine.start(0.0)
    view = _view(sim_time=0.0, odom_x=0.0, odom_y=0.0)
    # Jump the machine to the deploy state the way a real arrival would.
    machine._remember_start(view)
    machine._enter("DEPLOY_PROBE", 1.0, "stable_stop")
    machine.tick(_view(sim_time=1.0, lidar_stamp_sim=1.0, probe_stamp_sim=1.0, probe_state="DEPLOYING", probe_c=18.0), wall=1.0)
    assert machine.state == "DEPLOY_PROBE"
    assert machine.sample_started is None
    machine.tick(_view(sim_time=1.2, lidar_stamp_sim=1.2, probe_stamp_sim=1.2, probe_state="SAMPLING", probe_c=18.0), wall=1.2)
    assert machine.state == "SAMPLE"
    assert machine.sample_started == 1.2
    machine.tick(_view(sim_time=1.5, lidar_stamp_sim=1.5, probe_stamp_sim=1.5, probe_state="SAMPLING", probe_c=18.0), wall=1.5)
    assert machine.state == "SAMPLE"
    machine.tick(_view(sim_time=1.6, lidar_stamp_sim=1.6, probe_stamp_sim=1.6, probe_state="SAMPLING", probe_c=18.0), wall=1.6)
    assert machine.state == "RETRACT_PROBE"
    assert machine.measurement_class == "cold"
    assert classify_probe_c(18.0) == "cold"


def test_retract_fault_does_not_drive():
    probe = ProbeMachine()
    probe.config.retract_fault = True
    probe.request_deploy(0.0)
    probe.state = "SAMPLING"
    probe.command = "deploy"
    probe.request_stow(1.0)
    assert probe.state == "RETRACTING"
    probe.tick(sim_time=1.05, angle=0.8)
    assert probe.state == "FAULT"
    assert probe.reason == "retract_actuator_fault"

    machine = InspectionMachine(InspectionConfig(goal_x=0.0, goal_y=0.0))
    machine.start(0.0)
    machine._remember_start(_view())
    machine._enter("RETRACT_PROBE", 2.0, "measured_cold")
    tele = machine.tick(_view(sim_time=2.1, lidar_stamp_sim=2.1, probe_stamp_sim=2.1, probe_state="FAULT"), wall=2.1)
    assert machine.state == "FAULT"
    assert machine.reason == "probe_retract_failed"
    assert tele.velocity == 0.0 and tele.turn == 0.0


def test_timeout_is_not_success_and_cancel_does_not_resume():
    machine = InspectionMachine(InspectionConfig(goal_x=2.0, goal_y=0.0, mission_timeout_s=1.0))
    machine.start(0.0)
    machine.tick(_view(sim_time=0.0, lidar_stamp_sim=0.0, probe_stamp_sim=0.0), wall=0.0)
    tele = machine.tick(_view(sim_time=1.2, lidar_stamp_sim=1.2, probe_stamp_sim=1.2), wall=1.2)
    assert machine.state == "FAULT"
    assert machine.reason == "timeout"
    assert tele.velocity == 0.0
    machine2 = InspectionMachine(InspectionConfig(goal_x=1.0, goal_y=0.0))
    machine2.start(0.0)
    machine2.cancel("manual_takeover", 0.4)
    assert machine2.state == "CANCELLED"
    machine2.tick(_view(sim_time=0.8), wall=0.8)
    assert machine2.state == "CANCELLED"


def test_stale_scan_stops():
    machine = InspectionMachine(InspectionConfig(goal_x=1.0, goal_y=0.0))
    machine.start(0.0)
    tele = machine.tick(_view(sim_time=1.0, lidar_stamp_sim=0.0, lidar_valid=True), wall=1.0)
    assert machine.state == "FAULT"
    assert machine.reason == "stale_lidar"
    assert tele.velocity == 0.0


def test_pause_and_takeover_and_reset():
    sim = Simulator(params=RobotParams(morphology="wheels"), seed=1)
    sim.reset({"x": 0.1, "y": -0.2, "yaw": 0.3})
    sim.start_inspection(InspectionConfig(goal_x=1.0, goal_y=0.0))
    for _ in range(15):
        sim.step_control_tick()
    assert sim.inspection.state == "NAVIGATE_TO_TARGET"
    stamp = sim.inspection.state_entered
    sim_t = sim.data.time
    sim.enqueue(UiRequest(kind="pause"))
    for _ in range(20):
        sim.step_control_tick()
    assert sim.data.time == sim_t
    assert sim.inspection.state_entered == stamp
    sim.enqueue(UiRequest(kind="resume"))
    sim.step_control_tick()
    assert sim.data.time > sim_t
    sim.ingest_tele_cmd(TeleCmd(0.0, 0.0, 0.0, source="gui-stop"))
    sim.step_control_tick()
    assert sim.inspection.state == "CANCELLED"
    for _ in range(10):
        sim.step_control_tick()
    assert sim.inspection.state == "CANCELLED"
    sim.reset()
    assert sim.inspection.state == "IDLE"
    assert sim.probe.config.retract_fault is False


def test_five_scenarios_and_a_shifted_target(tmp_path):
    for name in ("clear", "detour", "blocked", "dropout", "retract"):
        report = run_case(name, out_dir=tmp_path)
        assert report["evaluator"]["pass"], (name, report["controller"], report["evaluator"]["failures"])
        assert report["controller"]["used_ground_truth"] is False
    shifted = run_case(
        "clear",
        target_world=(0.48, -0.18),
        spawn={"x": 0.08, "y": 0.12, "yaw": -0.55},
        seed=21,
        out_dir=tmp_path,
    )
    assert shifted["evaluator"]["pass"], shifted["evaluator"]["failures"]
    assert not math.isclose(shifted["spawn"]["yaw"], 0.0)


def _posts_on_one_side(seed: int) -> list[dict]:
    """Posts between the start and the target, all biased to one side.

    The other side stays open. The controller is not given these coordinates.
    """
    rng = random.Random(seed)
    side = 1 if rng.randrange(2) == 0 else -1
    stones = []
    for i in range(1 + seed % 3):
        stones.append(
            _stone(0.32 + i * 0.20, side * rng.uniform(0.0, 0.08), 0.028, 0.038, 0.08)
        )
    return stones


def test_posts_are_passed_from_the_scan_not_a_script(tmp_path):
    for seed in (1, 8, 15, 41):
        yaw = ((seed % 5) - 2) * 0.3
        report = run_case(
            "clear",
            target_world=(1.0, 0.0),
            spawn={"x": 0.0, "y": 0.0, "yaw": yaw},
            seed=seed,
            stones=_posts_on_one_side(seed),
            out_dir=tmp_path,
        )
        assert report["evaluator"]["pass"], (seed, report["controller"], report["evaluator"]["failures"])
        assert report["controller"]["state"] == "COMPLETE"
    # Posts on both sides. The straight line is not open the whole way.
    weaves = [
        [
            _stone(0.38, 0.0, 0.03, 0.05, 0.08),
            _stone(0.62, 0.16, 0.03, 0.06, 0.08),
            _stone(0.84, -0.14, 0.03, 0.05, 0.08),
        ],
        [
            _stone(0.34, -0.06, 0.025, 0.04, 0.08),
            _stone(0.55, 0.10, 0.025, 0.05, 0.08),
            _stone(0.74, -0.02, 0.025, 0.04, 0.08),
        ],
        [
            _stone(0.40, 0.08, 0.03, 0.04, 0.08),
            _stone(0.40, -0.10, 0.03, 0.035, 0.08),
            _stone(0.68, 0.0, 0.03, 0.04, 0.08),
        ],
    ]
    for i, stones in enumerate(weaves):
        report = run_case(
            "clear",
            target_world=(1.05, 0.02),
            spawn={"x": 0.0, "y": 0.0, "yaw": 0.2},
            seed=3 + i,
            stones=stones,
            out_dir=tmp_path,
        )
        assert report["evaluator"]["pass"], (i, report["controller"], report["evaluator"]["failures"])


def test_wheels_proxy_return_is_not_called_success(tmp_path):
    report = run_case("clear", morphology="wheels", timeout_s=25, out_dir=tmp_path)
    metrics = report["evaluator"]["metrics"]
    if report["controller"]["state"] == "COMPLETE":
        assert metrics["return_position_error_m"] <= 0.22
    else:
        assert report["evaluator"]["pass"] is False
