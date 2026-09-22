#!/usr/bin/env python3
"""Route normalized teleoperation commands to a capstan motor or RViz."""

import math
import sys
import time

import rclpy
from embr_interfaces.msg import TeleCmd
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray


class CapstanTeleopControlSystem(Node):
    """A transport-independent bridge from ``tele_cmd`` to one motor."""

    def __init__(self, simulation=False):
        super().__init__("capstan_teleop_control_system")
        self.declare_parameter("simulation", simulation)
        self.declare_parameter("max_velocity", 10.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("publish_period", 0.05)

        self._simulation = bool(self.get_parameter("simulation").value)
        self._max_velocity = self._positive_parameter("max_velocity")
        self._timeout = self._positive_parameter("command_timeout")
        period = self._positive_parameter("publish_period")
        if period >= self._timeout:
            raise ValueError("publish_period must be less than command_timeout")

        self._velocity = 0.0
        self._last_command = None
        message_type = Float64MultiArray if self._simulation else Float32
        topic = (
            "planetary_eagle_controller/commands"
            if self._simulation
            else "capstan_velocity_level"
        )
        self._publisher = self.create_publisher(message_type, topic, 10)
        self._subscriber = self.create_subscription(
            TeleCmd, "tele_cmd", self.teleop_callback, 10
        )
        self._timer = self.create_timer(period, self._publish_command)
        self.get_logger().info(
            f"Capstan {'simulation' if self._simulation else 'real'} mode: {topic}"
        )

    def _positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def teleop_callback(self, message):
        # A single test-stand axis uses the forward channel. Turn is intentionally
        # ignored so this node can share the same TeleCmd source as the drivetrain.
        if math.isfinite(message.velocity):
            self._velocity = max(-1.0, min(1.0, message.velocity))
        else:
            self.get_logger().warn("Invalid tele_cmd: stopping capstan")
            self._velocity = 0.0
        self._last_command = time.monotonic()
        self._publish_command()

    def _publish_command(self):
        if (
            self._last_command is None
            or time.monotonic() - self._last_command > self._timeout
        ):
            self._velocity = 0.0

        if self._simulation:
            command = Float64MultiArray()
            command.data = [self._velocity * self._max_velocity]
        else:
            command = Float32()
            command.data = self._velocity
        self._publisher.publish(command)


def main(args=None):
    cli_args = list(sys.argv[1:] if args is None else args)
    simulation = "--sim" in cli_args or "-sim" in cli_args
    cli_args = [arg for arg in cli_args if arg not in ("--sim", "-sim")]
    rclpy.init(args=cli_args)
    node = None
    try:
        node = CapstanTeleopControlSystem(simulation=simulation)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
