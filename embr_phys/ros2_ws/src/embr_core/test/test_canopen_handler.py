"""CAN transport contract tests, runnable without ROS or physical drives."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, call

import pytest


@pytest.fixture
def handler(monkeypatch):
    for name in ('canopen', 'rclpy', 'rclpy.node', 'rclpy.qos',
                 'embr_interfaces', 'embr_interfaces.msg', 'std_msgs', 'std_msgs.msg'):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    sys.modules['rclpy.node'].Node = object
    sys.modules['rclpy.qos'].QoSProfile = Mock()
    sys.modules['embr_interfaces.msg'].OperationStatus = SimpleNamespace
    sys.modules['std_msgs.msg'].Float32MultiArray = SimpleNamespace
    path = Path(__file__).parents[1] / 'embr/node_canopen_handler.py'
    spec = importlib.util.spec_from_file_location('handler_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = object.__new__(module.CANOpenNetwork)
    node._motors = [SimpleNamespace(id=i, sdo=Mock()) for i in (1, 3, 2, 4)]
    for motor in node._motors:
        motor.sdo.upload.return_value = b'\x27\x00'
    node._fault = None
    node._last_update = 100.0
    node._timeout = 0.5
    node._max_rpm = 1000
    node._directions = [1, -1, 1, -1]
    node._motor_slots = [0, 1, 2, 3]
    node._status_publisher = Mock()
    node.get_logger = Mock()
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.0)
    return node


def test_signed_targets_and_motor_order(handler):
    handler.motor_velocity_callback(SimpleNamespace(data=[1, 0.5, -0.5, -1]))
    assert [m.id for m in handler._motors] == [1, 3, 2, 4]
    for motor, rpm in zip(handler._motors, (1000, -500, -500, 1000)):
        assert motor.sdo.download.call_args_list == [
            call(0x60FF, 0, rpm.to_bytes(4, 'little', signed=True)),
            call(0x6040, 0, b'\x0f\x00'),
        ]
    assert handler._status_publisher.publish.call_args.args[0].success


@pytest.mark.parametrize('data', [[], [0]*3, [0]*5, [float('nan')]*4,
                                 [float('inf')]*4, [1.1, 0, 0, 0]])
def test_invalid_commands_stop_all_and_latch(handler, data):
    handler.motor_velocity_callback(SimpleNamespace(data=data))
    assert handler._fault
    for motor in handler._motors:
        motor.sdo.download.assert_any_call(0x6040, 0, b'\x0b\x00')
        motor.sdo.download.assert_any_call(0x60FF, 0, bytes(4))
        motor.sdo.download.reset_mock()
    handler.motor_velocity_callback(SimpleNamespace(data=[1]*4))
    assert all(not m.sdo.download.called for m in handler._motors)


def test_partial_bus_failure_stops_remaining_motors(handler):
    handler._motors[0].sdo.download.side_effect = TimeoutError('offline')
    handler.motor_velocity_callback(SimpleNamespace(data=[1]*4))
    assert 'stop unconfirmed' in handler._fault
    for motor in handler._motors[1:]:
        motor.sdo.download.assert_any_call(0x6040, 0, b'\x0b\x00')


def test_first_and_subsequent_targets_apply_after_target_write(handler):
    handler._last_update = None
    handler._wait_state = Mock()
    for level in (0.1, 0.2, 0.0):
        handler.motor_velocity_callback(SimpleNamespace(data=[level]*4))
        assert handler._fault is None
        for motor, direction in zip(handler._motors, handler._directions):
            rpm = round(level * direction * handler._max_rpm)
            assert motor.sdo.download.call_args_list[-2:] == [
                call(0x60FF, 0, rpm.to_bytes(4, 'little', signed=True)),
                call(0x6040, 0, b'\x0f\x00'),
            ]


def test_apply_failure_stops_all_and_latches(handler):
    def reject_apply(index, subindex, data):
        if index == 0x6040 and data == b'\x0f\x00':
            raise TimeoutError('apply failed')

    handler._motors[0].sdo.download.side_effect = reject_apply
    handler.motor_velocity_callback(SimpleNamespace(data=[0.1]*4))
    assert 'apply failed' in handler._fault
    assert not handler._status_publisher.publish.call_args.args[0].success
    for motor in handler._motors:
        motor.sdo.download.assert_any_call(0x6040, 0, b'\x0b\x00')
        motor.sdo.download.assert_any_call(0x60FF, 0, bytes(4))
        motor.sdo.download.reset_mock()
    handler.motor_velocity_callback(SimpleNamespace(data=[0.1]*4))
    assert all(not m.sdo.download.called for m in handler._motors)


def test_expired_target_is_not_applied(handler, monkeypatch):
    clock = Mock(return_value=100.0)
    monkeypatch.setattr(handler.motor_velocity_callback.__globals__['time'],
                        'monotonic', clock)

    def delay_target(index, subindex, data):
        if index == 0x60FF:
            clock.return_value = 100.6

    handler._motors[0].sdo.download.side_effect = delay_target
    handler.motor_velocity_callback(SimpleNamespace(data=[0.1]*4))
    assert 'expired before applying' in handler._fault
    for motor in handler._motors:
        assert call(0x6040, 0, b'\x0f\x00') not in motor.sdo.download.call_args_list
        motor.sdo.download.assert_any_call(0x6040, 0, b'\x0b\x00')


def test_watchdog_stops_and_latches(handler):
    handler._last_update = 99.0
    handler._publish_frame()
    assert 'timeout' in handler._fault
    assert handler._last_update is None


def test_disabled_drive_rejects_motion(handler):
    handler._motors[2].sdo.upload.return_value = b'\x08\x00'
    handler.motor_velocity_callback(SimpleNamespace(data=[1]*4))
    assert handler._fault
    handler._motors[3].sdo.download.assert_any_call(0x6040, 0, b'\x0b\x00')


@pytest.mark.parametrize('slot', range(4))
def test_single_motor_uses_original_command_slot(handler, slot):
    motors = handler._motors
    handler._motor_slots = handler._selected_slots([1, 3, 2, 4], [motors[slot].id])
    handler._motors = [motors[slot]]
    levels = [0.1, 0.2, 0.3, 0.4]
    handler.motor_velocity_callback(SimpleNamespace(data=levels))
    rpm = round(levels[slot] * handler._directions[slot] * 1000)
    assert motors[slot].sdo.download.call_args_list == [
        call(0x60FF, 0, rpm.to_bytes(4, 'little', signed=True)),
        call(0x6040, 0, b'\x0f\x00'),
    ]
    handler._last_update = 99.0
    handler._publish_frame()
    motors[slot].sdo.download.assert_any_call(0x6040, 0, b'\x0b\x00')
    for index, motor in enumerate(motors):
        if index != slot:
            motor.sdo.upload.assert_not_called()
            motor.sdo.download.assert_not_called()


@pytest.mark.parametrize('selection', [[], [1, 1], [5], [0]])
def test_invalid_selection(handler, selection):
    with pytest.raises(ValueError):
        handler._selected_slots([1, 3, 2, 4], selection)


def test_selection_order_and_remapped_ids(handler):
    assert handler._selected_slots([11, 13, 12, 14], [14, 13]) == [1, 3]
