"""System-level checks: headless import, no hardware, single physics owner."""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from embr_local_sim.cli import build_parser
from embr_local_sim.physics import Simulator
from embr_local_sim.ros_adapter import require_ros_adapter


def test_headless_does_not_import_viewer():
    import embr_local_sim.cli as cli
    import embr_local_sim.physics as physics
    import embr_local_sim.scenarios as scenarios

    assert "embr_local_sim.viewer" not in __import__("sys").modules
    assert hasattr(cli, "main")
    assert hasattr(physics, "Simulator")
    assert hasattr(scenarios, "run_scenario")


def test_viewer_module_is_optional_until_launch():
    src = Path(__file__).resolve().parents[1] / "src" / "embr_local_sim" / "cli.py"
    tree = ast.parse(src.read_text(encoding="utf-8"))
    top_imports = [
        node.names[0].name
        for node in tree.body
        if isinstance(node, ast.ImportFrom) and node.module
    ]
    assert "embr_local_sim.viewer" not in top_imports
    assert all("viewer" not in name for name in top_imports)


def test_no_hardware_modules_imported():
    import sys

    hardwareish = [name for name in sys.modules if name in {"canopen", "serial", "spidev", "rclpy"}]
    assert hardwareish == []


def test_ros_adapter_is_a_contract_not_a_node():
    with pytest.raises(NotImplementedError):
        require_ros_adapter()


def test_cli_has_launch_headless_test():
    parser = build_parser()
    assert {p.dest for p in parser._subparsers._group_actions}  # noqa: SLF001
    ns = parser.parse_args(["headless", "--all"])
    assert ns.cmd == "headless"


def test_one_simulator_owns_qpos():
    sim = Simulator(seed=0)
    qpos_id = id(sim.data.qpos)
    sim.step_control_tick()
    assert id(sim.data.qpos) == qpos_id
    # UI path cannot write qpos: only reset/teleport does, via Simulator.reset.
    src = Path(__file__).resolve().parents[1] / "src" / "embr_local_sim" / "viewer.py"
    text = src.read_text(encoding="utf-8")
    assert "data.qpos" not in text
    assert "mj_resetData" not in text
