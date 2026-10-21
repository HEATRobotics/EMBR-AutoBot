"""Wildfire / forest / engineering stages. Preserve Milestone A separately."""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from embr_local_sim.assets import cad_status, load_manifest, mjcf_asset_compiler
from embr_local_sim.autonomy import heading_turn
from embr_local_sim.mission import MissionConfig, MissionView
from embr_local_sim.model import RobotParams, WorldSpec, build_mjcf
from embr_local_sim.physics import Simulator
from embr_local_sim.worlds import engineering_world, forest_world, wildfire_world


ROOT = Path(__file__).resolve().parents[1]


def test_cad_archive_still_missing():
    status = cad_status()
    assert status["archive_found"] is False
    assert status["mesh_files"] == []
    assert "STEP" in status["blocker"]
    manifest = load_manifest()
    assert all(not slot.available for slot in manifest.slots.values())
    assert mjcf_asset_compiler(manifest) == ""


def test_asset_pipeline_swaps_when_mesh_appears(tmp_path):
    mesh = tmp_path / "chassis.obj"
    mesh.write_text("# dummy obj\nv 0 0 0\n", encoding="utf-8")
    yaml = tmp_path / "manifest.yaml"
    yaml.write_text(
        "version: 1\nunits: metre\nslots:\n  chassis:\n    visual: chassis.obj\n    scale: 1.0\n",
        encoding="utf-8",
    )
    man = load_manifest(yaml)
    assert man.slots["chassis"].available
    xml = mjcf_asset_compiler(man)
    assert "<mesh name=" in xml and "chassis" in xml


def test_engineering_wall_known_lidar_range():
    sim = Simulator(world=engineering_world(), seed=1)
    sim.reset()
    sim.run_for(0.5)
    _gt, obs = sim.snapshot()
    n = len(obs.lidar_ranges)
    front = min(obs.lidar_ranges[n // 2 - 1 : n // 2 + 2])
    wall_face = sim.world.wall_x - 0.5 * sim.world.wall_thickness
    # lidar site is ~0.02 m forward of root; allow generous proxy tolerance
    expected = wall_face - 0.02
    assert 0.35 < front < 0.55
    assert abs(front - expected) < 0.12
    xml = sim.xml
    assert "name=\"wall\"" in xml
    assert "name=\"step\"" in xml
    assert "name=\"mark_x_0\"" in xml


def test_detailed_forest_has_trees_and_visual_canopy():
    world = forest_world(seed=2, detailed=True)
    xml = build_mjcf(RobotParams(), world)
    assert "name=\"tree_0\"" in xml
    assert "name=\"canopy_0\"" in xml
    assert "contype=\"0\"" in xml  # canopy / visual
    sim = Simulator(world=world, seed=2)
    sim.reset()
    sim.run_for(0.4)
    assert sim.finite()
    names = {c.geom1 for c in sim.snapshot()[0].contacts} | {c.geom2 for c in sim.snapshot()[0].contacts}
    assert "floor" in names


def test_smoke_and_flame_do_not_create_lidar_hits():
    world = wildfire_world(seed=7)
    sim = Simulator(world=world, seed=7)
    sim.reset()
    sim.run_for(0.3)
    xml = sim.xml
    assert "name=\"smoke_0\"" in xml
    assert "name=\"flame_0_0\"" in xml
    assert 'name="smoke_0"' in xml and 'contype="0"' in xml
    # Rays use geom group 0; visual group 3 smoke must not be a hit geom.
    assert "group=\"3\"" in xml


def test_milestone_a_sparse_forest_unchanged():
    world = WorldSpec(kind="forest")
    xml = build_mjcf(RobotParams(), world)
    assert "name=\"stone_0\"" in xml
    assert "name=\"tree_0\"" not in xml


def test_autonomy_modules_do_not_read_root_pos():
    for rel in ("src/embr_local_sim/autonomy.py", "src/embr_local_sim/mission.py"):
        src = (ROOT / rel).read_text(encoding="utf-8")
        tree = ast.parse(src)
        attrs = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
        assert "root_pos" not in attrs
    view_fields = set(MissionView.__dataclass_fields__)
    assert "root_pos" not in view_fields


def test_goto_level2_arrives_on_flat_without_gt():
    sim = Simulator(world=WorldSpec(kind="flat"), seed=3)
    sim.reset()
    sim.run_for(0.4)
    sim.start_goto(0.28, 0.0, level=2)
    for _ in range(int(5.0 * sim.params.control_hz)):
        sim.step_control_tick()
        if sim.mission.state == "COMPLETE":
            break
    assert sim.mission.state == "COMPLETE", sim.mission.snapshot()
    assert sim.mission.used_ground_truth is False
    _gt, obs = sim.snapshot()
    assert abs(obs.wheel_odom_x - 0.28) < 0.14


def test_heading_helper_signs():
    view = MissionView(
        sim_time=0.0,
        stamp_wall=0.0,
        odom_x=0.0,
        odom_y=0.0,
        odom_yaw=0.0,
        lidar_ranges=[2.0] * 5,
        lidar_angles=[-0.4, -0.2, 0.0, 0.2, 0.4],
        lidar_stamp_sim=0.0,
        thermal_c=[20.0] * 5,
        thermal_angles=[-0.4, -0.2, 0.0, 0.2, 0.4],
        probe_c=20.0,
        probe_stamp_sim=0.0,
        probe_frame="probe",
    )
    dist, err, turn = heading_turn(view, 1.0, 0.5)
    assert dist > 0.5
    assert err > 0  # target left of +X
    assert turn < 0  # +turn is right, so left is negative


def test_wildfire_mission_does_not_use_gt_and_stays_finite():
    from embr_local_sim.scenarios import repo_root, run_scenario

    result = run_scenario(repo_root() / "scenarios" / "wildfire.yaml")
    assert result["final"]["mission"]["used_ground_truth"] is False
    assert result["final"]["mission"]["state"] in {"COMPLETE", "FAULT", "SEARCH", "APPROACH", "SAMPLE", "RETREAT"}
    assert not any("NaN" in f for f in result["failures"])


def test_default_wheels_morphology_preserved():
    assert RobotParams().morphology == "wheels"


def test_three_modes_build():
    for world in (engineering_world(), forest_world(seed=1), wildfire_world(seed=1)):
        sim = Simulator(world=world, seed=1)
        sim.reset()
        sim.step_control_tick()
        assert sim.finite()
