#!/usr/bin/env python3
"""Desktop USB CAN bridge; Raspberry Pi real mode is reserved for later."""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from embr_capstan.capstan_hardware.usb_can import UsbCan, can_simple_id


class CapstanCan(Node):
    """Send the helper's signed torque commands to a configured ODrive axis."""

    def __init__(self, simulation=False):
        super().__init__("capstan_can")
        self._can = None
        self.declare_parameter("simulation", simulation)
        if not self.get_parameter("simulation").value:
            self.get_logger().info("Real mode is reserved for the Raspberry Pi.")
            return

        for name, value in (
            ("channel", ""), ("interface", "slcan"), ("node_id", 0),
            ("bitrate", 1000000), ("tty_baudrate", 115200),
            ("max_torque", 0.1), ("command_timeout", 0.5),
            ("publish_period", 0.05),
        ):
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self._node_id = value("node_id")
        can_simple_id(self._node_id, 0)
        self._max_torque = float(value("max_torque"))
        self._timeout = float(value("command_timeout"))
        period = float(value("publish_period"))
        if not all(math.isfinite(v) and v > 0 for v in
                   (self._max_torque, self._timeout, period)):
            raise ValueError("Torque limit and timer settings must be finite and positive")
        if period >= self._timeout:
            raise ValueError("publish_period must be less than command_timeout")
        self._torque = 0.0
        self._last_command = None
        transport = UsbCan(
            channel=value("channel"), interface=value("interface"),
            bitrate=value("bitrate"), tty_baudrate=value("tty_baudrate"),
        )
        self._can = transport
        try:
            transport.open()
            transport.set_input_torque(self._node_id, 0.0)
            transport.set_controller_mode(self._node_id, 1, 1)
            transport.set_axis_state(self._node_id, 8)
            self.create_subscription(Float32, "capstan_torque", self.torque_callback, 10)
            self.create_timer(period, self._send_command)
        except Exception:
            self.stop()
            raise
        self.get_logger().info(
            f"Desktop CAN mode: {transport.interface} {transport.channel}, "
            f"axis node {self._node_id}; listening on capstan_torque"
        )

    def torque_callback(self, message):
        torque = float(message.data)
        self._torque = (
            max(-self._max_torque, min(self._max_torque, torque))
            if math.isfinite(torque) else 0.0
        )
        self._last_command = time.monotonic()
        self._send_command()

    def _send_command(self):
        if self._last_command is None or time.monotonic() - self._last_command > self._timeout:
            self._torque = 0.0
        try:
            self._can.set_input_torque(self._node_id, self._torque)
        except Exception:
            self.stop()
            raise

    def stop(self):
        """Attempt zero torque and idle independently, then release the adapter."""
        if self._can is None:
            return
        try:
            if self._can.is_open:
                for action in (
                    lambda: self._can.set_input_torque(self._node_id, 0.0),
                    lambda: self._can.set_axis_state(self._node_id, 1),
                ):
                    try:
                        action()
                    except Exception as error:
                        self.get_logger().error(f"Unable to stop CAN axis: {error}")
        finally:
            self._can.close()

    def destroy_node(self):
        try:
            self.stop()
        finally:
            super().destroy_node()


def main(args=None):
    cli_args = list(sys.argv[1:] if args is None else args)
    simulation = any(arg in ("--sim", "-sim") for arg in cli_args)
    cli_args = [arg for arg in cli_args if arg not in ("--sim", "-sim")]
    rclpy.init(args=cli_args)
    node = None
    try:
        node = CapstanCan(simulation=simulation)
        if node._can is not None:
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
