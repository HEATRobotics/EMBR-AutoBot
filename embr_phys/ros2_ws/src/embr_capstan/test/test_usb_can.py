"""Exercise CAN wire messages through python-can's virtual transport."""

import struct

import can
import pytest

from embr_capstan.capstan_hardware.usb_can import UsbCan, can_simple_id


def test_open_is_silent_commands_match_wire_protocol_and_close_reopens():
    helper = UsbCan("capstan-test", interface="virtual")
    with can.Bus(channel="capstan-test", interface="virtual", ignore_config=True) as peer:
        with helper:
            assert helper.is_open
            assert peer.recv(0) is None
            helper.set_axis_state(2, 8)
            helper.set_controller_mode(2, 2, 1)
            helper.set_input_velocity(2, -1.5, 0.25)
            helper.set_input_torque(2, 0.5)
            helper.request_encoder_estimates(2)
            expected = [
                (0x47, struct.pack("<I", 8)),
                (0x4B, struct.pack("<ii", 2, 1)),
                (0x4D, struct.pack("<ff", -1.5, 0.25)),
                (0x4E, struct.pack("<f", 0.5)),
                (0x49, b""),
            ]
            for index, (identifier, payload) in enumerate(expected):
                frame = peer.recv(0.1)
                assert frame is not None
                assert frame.arbitration_id == identifier
                assert bytes(frame.data) == payload
                assert not frame.is_extended_id
                assert frame.is_remote_frame == (index == 4)
                if index == 4:
                    assert frame.dlc == 8
            peer.send(can.Message(arbitration_id=0x49, is_extended_id=False,
                                  data=struct.pack("<ff", 1.0, 2.0)))
            assert struct.unpack("<ff", helper.receive(0.1).data) == (1.0, 2.0)
            assert helper.receive() is None
        assert not helper.is_open
        helper.close()
        with pytest.raises(RuntimeError):
            helper.receive()
        with helper:
            assert helper.is_open


@pytest.mark.parametrize("node,command", [(-1, 1), (64, 1), (1, -1), (1, 32), (1.5, 1)])
def test_invalid_can_simple_ids(node, command):
    with pytest.raises(ValueError):
        can_simple_id(node, command)


def test_invalid_frames_never_reach_bus():
    with UsbCan("invalid-test", interface="virtual") as helper:
        for identifier, payload in [(0x800, b""), (1, b"123456789")]:
            with pytest.raises(ValueError):
                helper.send_frame(identifier, payload)
        with pytest.raises(ValueError):
            helper.send_frame(1, b"x", is_remote_frame=True)
        with pytest.raises(ValueError):
            helper.send_frame(1, b"x", dlc=8)
        with pytest.raises(ValueError):
            helper.set_input_velocity(0, float("nan"))
        with pytest.raises(ValueError):
            helper.receive(float("inf"))


def test_firmware_056_node_63_is_an_addressed_axis():
    with can.Bus(channel="node-63", interface="virtual", ignore_config=True) as peer:
        with UsbCan("node-63", interface="virtual") as helper:
            helper.set_axis_state(63, 1)
            frame = peer.recv(0.1)
            assert frame.arbitration_id == 0x7E7
            assert bytes(frame.data) == b"\x01\x00\x00\x00"
            helper.request_encoder_estimates(63)
            frame = peer.recv(0.1)
            assert frame.arbitration_id == 0x7E9
            assert frame.is_remote_frame and frame.dlc == 8


def test_slcan_configuration_and_cleanup(monkeypatch):
    calls = []

    class Adapter:
        def shutdown(self):
            calls.append("shutdown")

    def factory(**kwargs):
        calls.append(kwargs)
        return Adapter()

    monkeypatch.setattr(can, "Bus", factory)
    helper = UsbCan("/dev/ttyACM0")
    with pytest.raises(RuntimeError):
        with helper:
            helper.open()  # idempotent
            raise RuntimeError("caller failed")
    assert calls == [{"channel": "/dev/ttyACM0", "interface": "slcan",
                      "ignore_config": True, "bitrate": 1_000_000,
                      "tty_baudrate": 115_200}, "shutdown"]
    assert not helper.is_open
