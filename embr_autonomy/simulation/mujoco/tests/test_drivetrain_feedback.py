"""Exercise the ROS SDO reader and MuJoCo receiver with four fake drives."""

import json
import math
import socket
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from embr_local_sim import drivetrain_run
from embr_local_sim.drivetrain import DrivetrainTwin, Feedback, load_drivetrain_config
from embr_local_sim.drivetrain_run import Evidence, receive, run_drivetrain


def test_four_named_outputs_and_fixed_chassis():
    twin = DrivetrainTwin(load_drivetrain_config())
    initial = twin.data.body("chassis").xpos.copy()
    pointer = twin.data.geom("front_left_maxon_pointer").xpos.copy()
    for node, counts, rpm in ((1, 20480, 20), (3, 40960, -40), (2, -81920, -80), (4, 61440, 60)):
        assert twin.accept(Feedback("a" * 32, 0, node, 10, .01, 1, counts, counts, 4096, rpm), 10)
    for joint, turns, rpm in (("front_left_maxon", .25, 1), ("rear_left_maxon", .5, -2),
                              ("front_right_maxon", 1, 4), ("rear_right_maxon", -.75, -3)):
        state = twin.data.joint(joint)
        assert state.qpos[0] == pytest.approx(turns * math.tau)
        assert state.qvel[0] == pytest.approx(rpm * math.tau / 60)
    assert twin.data.body("chassis").xpos == pytest.approx(initial)
    assert any(abs(twin.data.geom("front_left_maxon_pointer").xpos - pointer) > .001)
    custom = DrivetrainTwin(replace(twin.config, gear_ratio=10))
    assert custom.accept(twin.states[1].last, 10)
    assert custom.data.joint("front_left_maxon").qpos[0] == pytest.approx(math.pi)


def test_expiry_ordering_and_publisher_restart():
    twin = DrivetrainTwin(load_drivetrain_config())
    packet = Feedback("a" * 32, 4, 1, 10, .01, 1, 20480, 20480, 4096, 20)
    assert twin.accept(packet, 100)
    assert not twin.accept(packet, 100.4)
    assert not twin.accept(replace(packet, seq=3, sample_s=11), 100.4)
    assert not twin.accept(replace(packet, seq=5, sample_s=9), 100.4)
    new_publisher = replace(packet, session="b" * 32, seq=0, sample_s=2, relative_counts=0)
    assert not twin.accept(new_publisher, 100.4)
    twin.expire(100.51)
    assert twin.states[1].status == "Stale"
    joint = twin.data.joint("front_left_maxon")
    assert joint.qpos[0] == pytest.approx(math.pi / 2)
    assert joint.qvel[0] == 0
    assert twin.accept(new_publisher, 100.52)
    assert joint.qpos[0] == 0
    assert not twin.accept(replace(packet, seq=6, sample_s=11), 101.1)


class FakeSdo:
    RESPONSE_TIMEOUT = .05

    def __init__(self):
        self.counts = 2**31 - 4
        self.rpm = -40
        self.unplugged = False
        self.encoder_pulses = 1024

    def upload(self, index, subindex):
        if self.unplugged:
            raise TimeoutError("unplugged")
        if (index, subindex) == (0x60E4, 2):
            return self.counts.to_bytes(4, "little", signed=True)
        if (index, subindex) == (0x606C, 0):
            return self.rpm.to_bytes(4, "little", signed=True)
        objects = {(0x3000, 1): 0x0100, (0x3010, 1): self.encoder_pulses,
                   (0x60A8, 0): 0x00B50000, (0x60A9, 0): 0x00B44700}
        return objects[index, subindex].to_bytes(4, "little")


def test_sdo_to_udp_to_mujoco_and_one_motor_recovery(monkeypatch, tmp_path):
    ros_package = Path(__file__).resolve().parents[4] / "embr_phys/ros2_ws/src/embr_drivetrain"
    monkeypatch.syspath_prepend(str(ros_package))
    from embr_drivetrain import escon_feedback

    clock = SimpleNamespace(now=100.0)
    fake_time = SimpleNamespace(monotonic=lambda: clock.now)
    monkeypatch.setattr(escon_feedback, "time", fake_time)
    monkeypatch.setattr(drivetrain_run, "time", fake_time)
    twin = DrivetrainTwin(load_drivetrain_config())
    motors = [SimpleNamespace(id=node, sdo=FakeSdo()) for node in (1, 3, 2, 4)]
    with ExitStack() as stack:
        receiver = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        receiver.bind(("127.0.0.1", 0))
        receiver.setblocking(False)
        publisher = escon_feedback.FeedbackPublisher(
            motors, [4096] * 4, "127.0.0.1", receiver.getsockname()[1], .015, .5)
        stack.callback(publisher.close)
        evidence = Evidence(twin, stack, tmp_path, live=True)

        def cycle(interval=.02):
            for _ in motors:
                clock.now += interval
                publisher.poll()
                receive(receiver, evidence)

        cycle()
        assert all(state.status == "Live" for state in twin.states.values()), twin.states
        assert all(motor.sdo.RESPONSE_TIMEOUT == .05 for motor in motors)
        for motor in motors:
            motor.sdo.counts = -(2**31) + 16
        for _ in range(10):
            cycle()
        assert evidence.summary()["four_motor_sample_window"]
        assert all(state.last.relative_counts == 20 for state in twin.states.values())

        motors[0].sdo.unplugged = True
        cycle()
        assert twin.states[1].status == "Unavailable"
        assert "unplugged" in twin.states[1].last.error
        assert twin.data.joint("front_left_maxon").qvel[0] == 0
        assert all(twin.states[node].status == "Live" for node in (2, 3, 4))
        for _ in range(7):
            cycle()
        motors[0].sdo.unplugged = False
        motors[0].sdo.counts = 1200
        for _ in range(7):
            cycle()
        assert twin.states[1].last.reference == 2
        assert twin.states[1].status == "Live"
        assert twin.data.joint("front_left_maxon").qpos[0] == 0

        # Malformed packets never refresh a channel or overwrite its angle.
        publisher.socket.sendto(json.dumps({"version": 1, "node_id": 1}).encode(), publisher.destination)
        receive(receiver, evidence)
        assert evidence.ignored == 1
        clock.now += .6
        receive(receiver, evidence)
        assert all(state.status == "Stale" for state in twin.states.values())
        assert all(twin.data.joint(channel.joint).qvel[0] == 0 for channel in twin.channels.values())

    assert "relative_counts" in (tmp_path / "drivetrain_samples.csv").read_text()


def test_packaged_synthetic_replay_reports_every_channel(tmp_path):
    twin = DrivetrainTwin(load_drivetrain_config())
    summary = run_drivetrain(twin, live=False, headless=True, out=tmp_path, duration=15,
                             feedback_bind="127.0.0.1", feedback_port=43153, host="127.0.0.1", port=43148)
    assert summary["source"] == "synthetic"
    assert summary["gear_ratio"] == 20
    assert summary["four_motor_sample_window"]
    for motor in summary["motors"].values():
        assert motor["samples"] == 601
        assert motor["final_qpos_rad"] == motor["final_qvel_rad_s"] == 0
    assert len((tmp_path / "drivetrain_samples.csv").read_text().splitlines()) == 2405
