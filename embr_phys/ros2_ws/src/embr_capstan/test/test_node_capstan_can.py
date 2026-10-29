"""Verify the ROS bridge against real python-can virtual frames."""

import struct
import time

import can
import pytest
import rclpy
from std_msgs.msg import Float32

from embr_capstan.node_capstan_can import CapstanCan


def test_desktop_commands_timeout_and_shutdown():
    rclpy.init(args=["--ros-args", "-p", "simulation:=true",
                      "-p", "interface:=virtual", "-p", "channel:=bridge-test"])
    node = None
    try:
        with can.Bus(channel="bridge-test", interface="virtual", ignore_config=True) as peer:
            node = CapstanCan()
            startup = [peer.recv(0.1) for _ in range(3)]
            assert [frame.arbitration_id for frame in startup] == [0x0E, 0x0B, 0x07]
            assert struct.unpack("<ii", startup[1].data) == (1, 1)
            assert struct.unpack("<I", startup[2].data) == (8,)
            for requested, expected in [(0.05, 0.05), (-0.05, -0.05), (1.0, 0.1),
                                        (float("nan"), 0.0)]:
                node.torque_callback(Float32(data=requested))
                assert struct.unpack("<f", peer.recv(0.1).data)[0] == pytest.approx(expected)
            node.torque_callback(Float32(data=0.05))
            peer.recv(0.1)
            node._last_command = time.monotonic() - 1.0
            node._send_command()
            assert struct.unpack("<f", peer.recv(0.1).data) == (0.0,)
            node.destroy_node()
            node = None
            assert struct.unpack("<f", peer.recv(0.1).data) == (0.0,)
            assert struct.unpack("<I", peer.recv(0.1).data) == (1,)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_real_mode_does_not_open_can(monkeypatch):
    def forbidden_open(*args, **kwargs):
        pytest.fail("Real mode must not open a CAN adapter")

    monkeypatch.setattr("embr_capstan.node_capstan_can.UsbCan.open", forbidden_open)
    rclpy.init(args=[])
    node = CapstanCan()
    try:
        assert node._can is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
