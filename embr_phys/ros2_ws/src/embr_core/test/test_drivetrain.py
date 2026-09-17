"""Exercise command routing with actual ROS message types, without DDS."""
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from embr import node_maxon_drivetrain as drivetrain
from embr_interfaces.msg import TeleCmd
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


def controller(simulation):
    return SimpleNamespace(
        _simulation=simulation, _forward=0.0, _turn=0.0,
        _last_command=100.0, _timeout=0.5, _linear_speed=0.8,
        _angular_speed=0.6, _publisher=Mock(), get_logger=Mock(),
    )


@pytest.mark.parametrize('simulation', [False, True])
def test_forward_right_and_timeout(simulation):
    node = controller(simulation)
    node._publish_command = lambda: drivetrain.MaxonTeleopControlSystem._publish_command(node)
    with patch.object(drivetrain.time, 'monotonic', return_value=100.0):
        drivetrain.MaxonTeleopControlSystem.motor_velocity_callback(
            node, TeleCmd(velocity=1.0, turn=1.0))
    command = node._publisher.publish.call_args.args[0]
    if simulation:
        assert isinstance(command, Twist)
        assert command.linear.x == 0.8
        assert command.angular.z == -0.6
    else:
        assert isinstance(command, Float32MultiArray)
        assert list(command.data) == [1.0, 1.0, 0.0, 0.0]
    with patch.object(drivetrain.time, 'monotonic', return_value=100.6):
        node._publish_command()
    command = node._publisher.publish.call_args.args[0]
    if simulation:
        assert command.linear.x == command.angular.z == 0.0
    else:
        assert list(command.data) == [0.0] * 4


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_invalid_command_stops(value):
    node = controller(False)
    node._publish_command = Mock()
    drivetrain.MaxonTeleopControlSystem.motor_velocity_callback(
        node, TeleCmd(velocity=value, turn=1.0))
    assert node._forward == node._turn == 0.0


@pytest.mark.parametrize('flag', ['--sim', '-sim'])
def test_cli_preserves_ros_arguments(flag):
    with patch.object(drivetrain, 'rclpy') as ros, patch.object(
        drivetrain, 'MaxonTeleopControlSystem'
    ) as constructor:
        drivetrain.main([flag, '--ros-args', '-r', 'cmd_vel:=other_cmd'])
    ros.init.assert_called_once_with(args=['--ros-args', '-r', 'cmd_vel:=other_cmd'])
    constructor.assert_called_once_with(simulation=True)
