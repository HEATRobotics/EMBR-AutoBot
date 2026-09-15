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
