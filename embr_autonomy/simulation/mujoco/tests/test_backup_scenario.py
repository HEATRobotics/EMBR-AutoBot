"""A blocked nose backs up and turns. A clear side can still reach the hotspot."""

import json

from embr_local_sim.scenarios import run_scenario, scenarios_dir


def _velocities(result: dict, tmp_path) -> list[float]:
    path = tmp_path / f"{result['name']}.replay.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    return [float(row["tele_velocity"]) for row in rows]


def test_hotspot_around_backs_up_then_samples(tmp_path):
    result = run_scenario(scenarios_dir() / "hotspot_around.yaml", out_dir=tmp_path)
    assert result["passed"], result["failures"]
    assert result["final"]["mission"]["state"] == "COMPLETE"
    assert result["final"]["mission"]["used_ground_truth"] is False
    assert result["final"]["mission"]["hotspots"]
    went_around = abs(result["final"]["y"]) > 0.05 or min(_velocities(result, tmp_path)) < -0.3
    assert went_around


def test_solid_wall_still_faults_after_the_backup(tmp_path):
    result = run_scenario(scenarios_dir() / "mission_fault.yaml", out_dir=tmp_path)
    assert result["passed"], result["failures"]
    assert result["final"]["mission"]["state"] == "FAULT"
    assert "blocked" in result["final"]["mission"]["reason"]
    assert min(_velocities(result, tmp_path)) < -0.3
