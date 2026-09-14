"""Route normalized teleoperation commands to CANopen or the RViz drivetrain."""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from embr_interfaces.msg import TeleCmd

from embr.embr_hardware.ibus import mix_four_motor_levels


class MaxonTeleopControlSystem(Node):
    def __init__(self, simulation=False):
        super().__init__('maxon_teleop_control_system')
        self.declare_parameter('simulation', simulation)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_period', 0.05)
        self._simulation = self.get_parameter('simulation').value
        self._linear_speed = self._positive_parameter('max_linear_speed')
        self._angular_speed = self._positive_parameter('max_angular_speed')
        self._timeout = self._positive_parameter('command_timeout')
        period = self._positive_parameter('publish_period')
        if period >= self._timeout:
            raise ValueError('publish_period must be less than command_timeout')
        self._forward = self._turn = 0.0
        self._last_command = None

        message_type = Twist if self._simulation else Float32MultiArray
        topic = 'cmd_vel' if self._simulation else 'motor_velocity_levels'

        self._publisher = self.create_publisher(message_type, topic, 10)
        self._tele_subscriber = self.create_subscription(
            TeleCmd, 'tele_cmd', self.motor_velocity_callback, 10
        )

        self._timer = self.create_timer(period, self._publish_command)
        self.get_logger().info(
            f"Drivetrain {'simulation' if self._simulation else 'real'} mode: {topic}"
        )

    def _positive_parameter(self, name):
        """
        ## Def:
            Helper function to confirm that values given are positive. Usefulas
            a check for values that must be positive, 0 not included. 
        
        ## Args:
            name: The String name of a value declared as a parameter 

        ## Returns:
            value: The value associated with the 'name' parameter

        ## Raises:
            ValueError: Values that are infinite or negative.
        """
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def motor_velocity_callback(self, msg):
        """
        ## Def:
            Checks if velocity and turn msg data is not out of bounds as an infinite value,
            if value is infinite, stop all motors. Otherwise updates _forward & _turn, updates the time of the last 
            command (_last_command) and publishes the command

        ## Args:
            msg: TeleCmd from embr_interfaces.msg

        ## Returns:
            N/A

        ## Raises:
            N/A

        """
        if not math.isfinite(msg.velocity) or not math.isfinite(msg.turn):
            self.get_logger().warn('Invalid tele_cmd: stopping drivetrain')
            self._forward = self._turn = 0.0
        else:
            self._forward = max(-1.0, min(1.0, msg.velocity))
            self._turn = max(-1.0, min(1.0, msg.turn))
        self._last_command = time.monotonic()
        self._publish_command()

    def _publish_command(self):
        """
        ## Def:
            Checkes if time since last command is greater than timeout amount specified and timesout if so.
            If in simulation mode then publishes a `Twist()` type command, else publishes the mixed motor levels in a 
            `Float32MultiArray()`

        ## Args:
            N/A

        ## Returns: 
            N/A

        ## Raises:
            N/A

        """
        if (self._last_command is None
                or time.monotonic() - self._last_command > self._timeout):
            self._forward = self._turn = 0.0
        if self._simulation:
            command = Twist()
            command.linear.x = self._forward * self._linear_speed
            # Teleoperation uses positive turn for right; ROS yaw is positive left.
            command.angular.z = -self._turn * self._angular_speed
        else:
            command = Float32MultiArray()
            # Order: front left, rear left, front right, rear right.
            command.data = mix_four_motor_levels(self._forward, -self._turn)
        self._publisher.publish(command)


def main(args=None):
    cli_args = list(sys.argv[1:] if args is None else args)
    simulation = '--sim' in cli_args or '-sim' in cli_args
    cli_args = [arg for arg in cli_args if arg not in ('--sim', '-sim')]
    rclpy.init(args=cli_args)
    node = None
    try:
        node = MaxonTeleopControlSystem(simulation=simulation)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
