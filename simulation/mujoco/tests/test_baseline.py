"""Baseline launch contract: paths, pause, headless drive, CLI help."""

from __future__ import annotations

from pathlib import Path

import pytest

from embr_local_sim.cli import main
from embr_local_sim.model import WorldSpec
from embr_local_sim.paths import scenarios_dir
from embr_local_sim.physics import Simulator, UiRequest
from embr_local_sim.scenarios import list_scenarios


def test_scenarios_resolve_outside_the_source_tree(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    paths = list_scenarios()
    assert paths
    assert all(path.is_file() for path in paths)
    assert scenarios_dir() in paths[0].parents


def test_missing_config_is_a_clear_error(capsys):
    code = main(["launch", "--config", "does-not-exist.yaml"])
    assert code == 2
    err = capsys.readouterr().err
    assert "Configuration file not found" in err


def test_missing_scenario_is_a_clear_error(capsys):
    code = main(["headless", "--scenario", "not_a_real_scene"])
    assert code == 2
    err = capsys.readouterr().err
    assert "Scenario file not found" in err


def test_help_lists_scenes(capsys):
    with pytest.raises(SystemExit) as exc:
        main(["--help"])
    assert exc.value.code == 0
    assert "launch" in capsys.readouterr().out
    with pytest.raises(SystemExit) as exc:
        main(["launch", "--help"])
    assert exc.value.code == 0
    launch_help = capsys.readouterr().out
    assert "--scene" in launch_help and "forest" in launch_help
    assert "--port" in launch_help and "--seed" in launch_help
    with pytest.raises(SystemExit) as exc:
        main(["headless", "--help"])
    assert exc.value.code == 0
    assert "--steps" in capsys.readouterr().out


def test_pause_freezes_time_and_drops_drive_commands():
    sim = Simulator(world=WorldSpec(kind="flat"), seed=0)
    sim.reset()
    sim.run_for(0.30)
    sim.drive(0.25, 0.0, source="test")
    sim.step_control_tick()
    assert sim.controller.target_vx == pytest.approx(0.25)
    t0 = float(sim.data.time)
    x0 = float(sim.snapshot()[0].root_pos[0])
    sim.enqueue(UiRequest(kind="pause"))
    sim.drive(0.25, 0.0, source="test")
    for _ in range(8):
        sim.step_control_tick()
    assert sim.data.time == pytest.approx(t0)
    assert sim.snapshot()[0].root_pos[0] == pytest.approx(x0, abs=1e-9)
    assert sim.controller.target_vx == 0.0
    assert sim.snapshot()[0].paused is True
    sim.enqueue(UiRequest(kind="resume"))
    sim.step_control_tick()
    assert sim.controller.target_vx == 0.0
    assert sim.snapshot()[0].paused is False
    sim.drive(0.20, 0.0, source="test")
    sim.step_control_tick()
    assert sim.controller.target_vx == pytest.approx(0.20)


def test_headless_basic_moves_and_stays_finite():
    code = main(["headless", "--steps", "40", "--scene", "basic", "--seed", "1", "--vx", "0.2"])
    assert code == 0


def test_headless_forest_stays_finite():
    code = main(["headless", "--steps", "20", "--scene", "forest", "--seed", "2", "--vx", "0.2"])
    assert code == 0


def test_reset_is_the_only_pose_write_used_by_tests():
    sim = Simulator(world=WorldSpec(kind="flat"), seed=1)
    sim.reset({"x": 0.05, "y": -0.02})
    assert sim.snapshot()[0].root_pos[0] == pytest.approx(0.05, abs=1e-6)
    sim.request_reset()
    sim.step_control_tick()
    assert sim.snapshot()[0].root_pos[0] == pytest.approx(0.0, abs=1e-5)
    text = Path(__file__).resolve().parents[1].joinpath(
        "src", "embr_local_sim", "physics.py"
    ).read_text(encoding="utf-8")
    apply = text.split("def _apply_control")[1].split("def _contacts")[0]
    assert "qpos[adr : adr + 3]" not in apply
