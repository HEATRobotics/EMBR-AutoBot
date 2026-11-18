"""Motor telemetry contract, analytical replay and MuJoCo transform checks."""

from __future__ import annotations

import csv
import json
import math
import subprocess
import sys
from dataclasses import replace

import mujoco
import numpy as np
import pytest

from embr_local_sim.cli import main
from embr_local_sim.motor import MotorParams, MotorSample, MotorTwin, load_motor_params
from embr_local_sim.motor_demo import demo_sample, run_headless


def test_single_hinge_points_along_existing_motor_urdf_axis():
    twin = MotorTwin()
    assert twin.model.nq == twin.model.nv == twin.model.njnt == 1
    assert twin.model.nu == 0
    assert twin.model.joint("motor_shaft").axis == pytest.approx([0, -1, 0])
    assert not twin.model.joint("motor_shaft").limited[0]
    assert twin.model.body("rotor").inertia[1] == pytest.approx(1.7e-5)
    assert sum(twin.model.body_mass) == pytest.approx(.752)


def test_quarter_turn_moves_pointer_up_but_not_housing():
    twin = MotorTwin()
    housing_id = twin.model.geom("housing_shell").id
    pointer_id = twin.model.geom("shaft_pointer").id
    housing_pos = twin.data.geom_xpos[housing_id].copy()
    housing_rot = twin.data.geom_xmat[housing_id].copy()
    initial = twin.data.geom_xpos[pointer_id].copy()
    axis_origin = twin.data.body("rotor").xpos.copy()
    twin.apply(MotorSample(0.0, math.pi / 2, math.pi))
    final = twin.data.geom_xpos[pointer_id]
    assert final[0] == pytest.approx(0, abs=1e-12)
    assert final[1] == pytest.approx(initial[1])
    assert final[2] - axis_origin[2] == pytest.approx(initial[0])
    assert twin.data.geom_xpos[housing_id] == pytest.approx(housing_pos)
    assert twin.data.geom_xmat[housing_id] == pytest.approx(housing_rot)
    assert twin.data.joint("motor_shaft").qvel[0] == pytest.approx(math.pi)


@pytest.mark.parametrize("time_s,pi_units,speed", [
    (0, 0, math.pi), (1, 1, math.pi), (2, 2, 2 * math.pi),
    (3, 4, 2 * math.pi), (4, 6, 0), (5, 6, 0), (6, 6, -math.pi),
    (7, 5, -math.pi), (8, 4, -2 * math.pi), (9, 2, -2 * math.pi),
    (10, 0, 0), (12, 0, 0),
])
def test_demo_matches_independent_analytical_checkpoints(time_s, pi_units, speed):
    _, sample = demo_sample(time_s)
    twin = MotorTwin()
    twin.apply(sample)
    assert twin.data.joint("motor_shaft").qpos[0] == pytest.approx(pi_units * math.pi)
    assert twin.data.joint("motor_shaft").qvel[0] == pytest.approx(speed)


def test_speed_only_uses_previous_speed_and_labels_estimated_angle():
    twin = MotorTwin()
    state = twin.apply(MotorSample(5.0, velocity_rad_s=2.0))
    assert state.position_rad == 0  # Do not integrate across time before the first observation.
    assert state.position_kind == "estimated"
    state = twin.apply(MotorSample(5.5, velocity_rad_s=-4.0))
    assert state.position_rad == pytest.approx(1.0)
    state = twin.apply(MotorSample(6.0, velocity_rad_s=0.0))
    assert state.position_rad == pytest.approx(-1.0)
    state = twin.apply(MotorSample(7.0, velocity_rad_s=0.0))
    assert state.position_rad == pytest.approx(-1.0)


def test_supplied_unwrapped_angle_resynchronizes_estimate_without_modulo():
    twin = MotorTwin()
    twin.apply(MotorSample(0, velocity_rad_s=3))
    state = twin.apply(MotorSample(1, position_rad=8 * math.pi, velocity_rad_s=-2))
    assert state.position_kind == "supplied"
    assert state.position_rad == pytest.approx(8 * math.pi)
    assert twin.apply(MotorSample(2, velocity_rad_s=0)).position_rad == pytest.approx(8 * math.pi - 2)


def test_position_only_clears_old_speed_and_reports_it_unavailable():
    twin = MotorTwin()
    twin.apply(MotorSample(0, 0, 3))
    state = twin.apply(MotorSample(1, position_rad=2))
    assert state.velocity_kind == "unavailable"
    assert twin.data.joint("motor_shaft").qvel[0] == 0
    with pytest.raises(ValueError, match="previous speed"):
        twin.apply(MotorSample(2, velocity_rad_s=3))
    assert twin.state is state


@pytest.mark.parametrize("sample", [
    MotorSample(-1, 0, 0), MotorSample(float("nan"), 0, 0),
    MotorSample(float("inf"), 0, 0), MotorSample(True, 0, 0),
    MotorSample(2), MotorSample(2, float("nan")),
    MotorSample(2, velocity_rad_s=float("inf")), MotorSample(2, True),
    MotorSample(2, "bad"), MotorSample(1, 0, 0), MotorSample(.5, 0, 0),
])
def test_invalid_feedback_is_rejected_without_mutating_previous_state(sample):
    twin = MotorTwin()
    state = twin.apply(MotorSample(1, 4, -2))
    with pytest.raises(ValueError):
        twin.apply(sample)
    assert twin.state is state
    assert twin.data.qpos[0] == 4
    assert twin.data.qvel[0] == -2
    assert twin.data.time == 1


def test_estimate_overflow_is_rejected_and_reset_allows_new_stream():
    twin = MotorTwin()
    state = twin.apply(MotorSample(0, 0, 1e308))
    with pytest.raises(ValueError, match="estimated position"):
        twin.apply(MotorSample(10, velocity_rad_s=0))
    assert twin.state is state
    twin.reset()
    assert twin.state is None
    assert twin.data.time == twin.data.qpos[0] == twin.data.qvel[0] == 0
    assert twin.apply(MotorSample(0, 0, 0)).timestamp_s == 0


@pytest.mark.parametrize("overrides", [
    {"housing_radius_m": 0}, {"shaft_length_m": float("nan")},
    {"motor_mass_kg": True}, {"motor_mass_kg": "heavy"},
    {"rotor_mass_kg": 1}, {"shaft_radius_m": .1},
    {"rotor_axial_inertia_kg_m2": 1},
])
def test_invalid_model_parameters_fail_before_compilation(overrides):
    with pytest.raises(ValueError):
        MotorParams(**overrides)


def test_config_can_override_geometry_and_is_independent_of_working_directory(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert load_motor_params() == MotorParams()
    file = tmp_path / "motor.yaml"
    file.write_text("housing_radius_m: 0.030\nshaft_length_m: 0.04\n", encoding="utf-8")
    params = load_motor_params(file)
    assert params == replace(MotorParams(), housing_radius_m=.030, shaft_length_m=.04)
    twin = MotorTwin(params)
    assert twin.model.geom("housing_shell").size[0] == pytest.approx(.030)
    assert twin.model.geom("shaft").size[1] == pytest.approx(.020)
    assert np.isfinite(twin.data.geom_xpos).all()


@pytest.mark.parametrize("contents", ["[]", "", "unknown_parameter: 1", "housing_radius_m: bad", "[:"])
def test_invalid_config_has_a_clear_error(tmp_path, contents):
    file = tmp_path / "motor.yaml"
    file.write_text(contents, encoding="utf-8")
    with pytest.raises(ValueError):
        load_motor_params(file)


def test_headless_records_every_sample_and_both_stops(tmp_path):
    summary = run_headless(MotorTwin(), tmp_path)
    assert summary["passed"]
    assert summary["samples"] == 601
    assert summary["final_position_rad"] == summary["final_velocity_rad_s"] == 0
    assert summary["max_position_error_rad"] <= 1e-12
    assert summary["max_velocity_error_rad_s"] <= 1e-12
    assert json.loads((tmp_path / "motor_summary.json").read_text()) == summary
    with (tmp_path / "motor_samples.csv").open(newline="") as file:
        rows = list(csv.DictReader(file))
    assert len(rows) == 601
    assert float(rows[200]["qpos_rad"]) == pytest.approx(6 * math.pi)
    assert float(rows[299]["qpos_rad"]) == pytest.approx(6 * math.pi)
    assert all(float(row["qvel_rad_s"]) == 0 for row in rows[200:300])
    assert all(float(row["qpos_rad"]) == 0 for row in rows[500:])


def test_motor_cli_export_and_headless_work_outside_package(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    assert main(["motor", "--headless", "--out", "results"]) == 0
    assert json.loads(capsys.readouterr().out)["passed"]
    assert main(["motor", "--write-model", "model/motor.xml"]) == 0
    model = mujoco.MjModel.from_xml_path(str(tmp_path / "model/motor.xml"))
    assert model.njnt == 1


def test_headless_does_not_import_viser_or_browser_viewer():
    script = (
        "from embr_local_sim.cli import main; import sys; "
        "assert main(['motor', '--headless']) == 0; "
        "assert 'viser' not in sys.modules; "
        "assert 'embr_local_sim.motor_viewer' not in sys.modules"
    )
    subprocess.run([sys.executable, "-c", script], check=True, capture_output=True, text=True)


@pytest.mark.parametrize("args", [
    ["--out", "results"], ["--headless", "--write-model", "motor.xml"],
    ["--config", "missing-motor.yaml"],
])
def test_motor_cli_invalid_options_are_actionable(args, capsys):
    assert main(["motor", *args]) == 2
    assert capsys.readouterr().err
