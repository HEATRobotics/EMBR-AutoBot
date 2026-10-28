"""A body and a local policy can be swapped without editing the mission states."""

from embr_local_sim.body import resolve_body
from embr_local_sim.mission import MissionConfig, MissionMachine, MissionView
from embr_local_sim.policy import ClassicalLocalPolicy
from embr_local_sim.tele_cmd import TeleCmd


def _view() -> MissionView:
    return MissionView(
        sim_time=1.0,
        stamp_wall=1.0,
        odom_x=0.2,
        odom_y=0.0,
        odom_yaw=0.0,
        lidar_ranges=[2.0] * 21,
        lidar_angles=[0.0] * 21,
        lidar_stamp_sim=1.0,
        thermal_c=[20.0] * 5,
        thermal_angles=[0.0] * 5,
        probe_c=20.0,
        probe_stamp_sim=1.0,
        probe_frame="probe",
    )


class _Hold:
    name = "hold"

    def search(self, **_kwargs) -> tuple[float, float]:
        return 0.0, 0.0

    def approach(self, **_kwargs) -> tuple[float, float]:
        return 0.0, 0.0

    def flame(self, flame_ang: float, wall: float) -> TeleCmd:
        return TeleCmd(0.0, 0.0, wall, source="mission")

    def goto(self, view, target_x, target_y, *, arrive_m, wall, avoid, cruise):
        return TeleCmd(0.0, 0.0, wall, source="mission"), False, "held"


def test_bodies_share_one_lookup():
    assert resolve_body("wheels").layout_scale == 1.0
    assert resolve_body("tracks").layout_scale == 1.0
    assert resolve_body("articulated").layout_scale == 1.0
    cad = resolve_body("cad")
    assert cad.layout_scale > 4.0
    assert cad.lidar_overhang_m > 0.2
    assert cad.grid_m > resolve_body("wheels").grid_m


def test_classical_search_still_drives():
    machine = MissionMachine(MissionConfig(level=4, detect_c=500.0))
    assert isinstance(machine.local, ClassicalLocalPolicy)
    machine.start(0.0)
    tele = machine.tick(_view(), wall=1.0)
    assert machine.state == "SEARCH"
    assert tele.velocity == 0.55


def test_another_policy_can_hold_the_same_state():
    machine = MissionMachine(MissionConfig(level=4, detect_c=500.0), local=_Hold())
    machine.start(0.0)
    tele = machine.tick(_view(), wall=1.0)
    assert machine.state == "SEARCH"
    assert tele.velocity == 0.0
    assert tele.turn == 0.0


def test_a_custom_planner_replaces_the_histogram():
    class _Right:
        name = "right"

        def steer(self, ctx):
            return TeleCmd(0.4, 0.9, ctx.wall, source="mission"), "custom"

    machine = MissionMachine(MissionConfig(level=4, detect_c=500.0), planner=_Right())
    machine.start(0.0)
    tele = machine.tick(_view(), wall=1.0)
    assert machine.state == "SEARCH"
    assert tele.velocity == 0.4
    assert tele.turn == 0.9
    assert machine.reason == "custom"


def test_steer_planner_assigned_before_start_mission_is_used():
    from embr_local_sim.model import RobotParams, WorldSpec
    from embr_local_sim.physics import Simulator

    class _Right:
        name = "right"

        def steer(self, ctx):
            return TeleCmd(0.4, 0.9, ctx.wall, source="mission"), "custom"

    sim = Simulator(params=RobotParams(), world=WorldSpec(kind="flat"), seed=1)
    sim.reset()
    sim.steer_planner = _Right()
    sim.start_mission()
    sim.run_for(0.3)
    assert sim.mission.reason == "custom"
    assert sim.mission.last_tele.velocity == 0.4
    assert sim.mission.last_tele.turn == 0.9
