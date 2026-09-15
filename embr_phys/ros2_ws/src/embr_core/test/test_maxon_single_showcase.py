"""Single-motor showcase contract tests, runnable without ROS or physical drives."""
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
    path = Path(__file__).parents[1] / 'embr/node_maxon_single_showcase.py'
    spec = importlib.util.spec_from_file_location('handler_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = object.__new__(module.CANOpenNetwork)
    node._motors = [SimpleNamespace(id=i, sdo=Mock()) for i in (1,)]
    for motor in node._motors:
        motor.sdo.upload.return_value = b'\x27\x00'
    node._fault = None
    node._last_update = 100.0
    node._timeout = 0.5
    node._max_rpm = 1000
    node._motor = node._motors[0]
    node._status_publisher = Mock()
    node.get_logger = Mock()
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.0)
    return node


@pytest.mark.parametrize('data', [[-0.5], [-0.5, 1.0, 0.25, -1.0]])
def test_only_showcase_motor_receives_first_level(handler, data):
    handler._last_update = None
    handler._wait_state = Mock()
    handler.motor_velocity_callback(SimpleNamespace(data=data))
    assert handler._fault is None
    assert handler._motor.sdo.download.call_args_list[-2:] == [
        call(0x60FF, 0, (-500).to_bytes(4, 'little', signed=True)),
        call(0x6040, 0, b'\x0f\x00'),
    ]


def test_timeout_stops_showcase_motor(handler):
    handler._last_update = 99.0
    handler._publish_frame()
    assert 'timeout' in handler._fault
    handler._motor.sdo.download.assert_any_call(0x6040, 0, b'\x0b\x00')
    handler._motor.sdo.download.assert_any_call(0x60FF, 0, bytes(4))


def test_showcase_motor_fault_is_monitored(handler):
    handler._network = Mock()
    handler._motor.emcy = SimpleNamespace(active=[])
    handler._motor.sdo.upload.return_value = b'\x08\x00'
    handler._publish_frame()
    assert 'drive fault or disabled' in handler._fault
    handler._motor.sdo.download.assert_any_call(0x60FF, 0, bytes(4))


def prepare_sequence(handler):
    handler._closed = False
    handler._network = Mock()
    handler._motor.emcy = SimpleNamespace(active=[])
    handler._motor.sdo.upload.return_value = bytes(4)
    handler._angle = handler._target_angle = 0.0
    handler._sequence_index = -1
    handler._previous_counts = 0
    handler._relative_counts = 0
    handler._counts_per_rev = 4096.0
    handler._settled_since = None
    handler._move_started = 100.0
    handler._move_timeout = 30.0
    handler._pause = 1.0
    handler._tolerance = 3.0
    handler._showcase_rpm = 10.0
    handler.motor_velocity_callback = Mock()


def test_sequence_repeats_via_startup_center(handler):
    prepare_sequence(handler)
    handler._update_encoder_angle = Mock()
    expected = (180, 90, 180, -90, 180, 180, 90, -90, -90, -90, 180, 90)
    for delta in expected:
        handler._angle = handler._target_angle
        previous = handler._angle
        handler._settled_since = 98.0
        handler._run_sequence()
        assert handler._target_angle == previous + delta
    handler._angle = handler._target_angle
    handler._settled_since = 98.0
    handler._run_sequence()
    assert handler._target_angle == 0.0
    assert handler._sequence_index == -1
    handler._angle = 0.0
    handler._settled_since = 98.0
    handler._run_sequence()
    assert handler._target_angle == 180.0


@pytest.mark.parametrize('counts, angle', [(1024, 90), (2048, 180), (-1024, -90)])
def test_encoder_angle_ignores_velocity_integration(handler, counts, angle):
    prepare_sequence(handler)
    handler._motor.sdo.upload.side_effect = lambda index, sub: (
        counts if (index, sub) == (0x60E4, 2) else 0
    ).to_bytes(4, 'little', signed=True)
    handler._target_angle = -180.0
    handler._run_sequence()
    assert handler._angle == angle
    handler._motor.sdo.upload.assert_any_call(0x60E4, 2)


@pytest.mark.parametrize('before, after, delta', [
    (2147483647, -2147483648, 1), (-2147483648, 2147483647, -1)])
def test_encoder_rollover(handler, before, after, delta):
    prepare_sequence(handler)
    handler._previous_counts = before
    handler._motor.sdo.upload.return_value = after.to_bytes(4, 'little', signed=True)
    handler._update_encoder_angle()
    assert handler._relative_counts == delta


def encoder_objects(handler):
    handler._counts_per_rev = 4096.0
    values = {(0x3000, 1): 0x110, (0x3010, 1): 1024,
              (0x60A8, 0): 0x00B50000, (0x60E4, 2): -1234}
    handler._motor.sdo.upload.side_effect = lambda index, sub: values[index, sub].to_bytes(
        4, 'little', signed=True)
    return values


def test_encoder_startup_captures_nonzero_reference(handler):
    encoder_objects(handler)
    handler._initialize_encoder()
    handler._update_encoder_angle()
    assert handler._previous_counts == -1234
    assert handler._angle == 0
    handler._motor.sdo.download.assert_not_called()


@pytest.mark.parametrize('key, value', [((0x3000, 1), 0x10),
    ((0x3010, 1), 500), ((0x60A8, 0), 0)])
def test_encoder_rejects_wrong_commissioning(handler, key, value):
    values = encoder_objects(handler)
    values[key] = value
    with pytest.raises(ValueError):
        handler._initialize_encoder()
    handler._motor.sdo.download.assert_not_called()


def test_missing_position_object_fails_before_motion(handler):
    values = encoder_objects(handler)
    del values[0x60E4, 2]
    with pytest.raises(KeyError):
        handler._initialize_encoder()
    handler._motor.sdo.download.assert_not_called()


def test_encoder_read_failure_stops_sequence(handler):
    prepare_sequence(handler)
    handler._motor.sdo.upload.side_effect = TimeoutError('encoder unavailable')
    handler._run_sequence()
    assert 'encoder unavailable' in handler._fault
    handler.motor_velocity_callback.assert_not_called()


def test_motion_does_not_settle_while_speed_is_nonzero(handler):
    prepare_sequence(handler)
    handler._settled_since = 98.0
    handler._motor.sdo.upload.side_effect = lambda index, sub: (
        10 if index == 0x606C else 0).to_bytes(4, 'little', signed=True)
    handler._run_sequence()
    assert handler._sequence_index == -1
    assert handler._settled_since is None


def test_stalled_sequence_latches_stop(handler):
    prepare_sequence(handler)
    handler._move_started = 60.0
    handler._run_sequence()
    assert 'move_timeout' in handler._fault
    handler.motor_velocity_callback.assert_not_called()


def test_shutdown_disables_even_when_quick_stop_fails(handler):
    def fail_quick_stop(index, subindex, data):
        if index == 0x6040 and data == b'\x0b\x00':
            raise TimeoutError('quick stop failed')
    for motor in handler._motors:
        motor.sdo.download.side_effect = fail_quick_stop
    assert handler._stop_all(disable=True)
    for motor in handler._motors:
        assert motor.sdo.download.call_args_list[-1] == call(0x6040, 0, bytes(2))
