#!/usr/bin/env python3
"""Publish drivetrain levels from an iBUS receiver or terminal controls."""
# TODO: Refactor with a class for sim mode and a class for real mode. Current setup is a quick solution. 

import select
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Float64

from embr_interfaces.msg import TeleCmd 

from embr.embr_hardware.ibus import (
    ChannelCalibration,
    IBusStreamDecoder,
    channels_to_drive,
    mix_four_motor_levels,
)


class Teleoperation(Node):
    """ROS 2 node responsible for EMBR teleoperation."""

    def __init__(self, simulation: bool = False) -> None:
        super().__init__("teleoperation")
        self._simulation = simulation
        self._terminal_settings = None
        self.declare_parameter("serial_port", "/dev/ttyAMA1")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("forward_channel", 1)
        self.declare_parameter("turn_channel", 0)
        self.declare_parameter("channel_min", 1000)
        self.declare_parameter("channel_center", 1500)
        self.declare_parameter("channel_max", 2000)
        self.declare_parameter("deadband", 0.04)
        self.declare_parameter("invert_forward", False)
        self.declare_parameter("invert_turn", False)
        self.declare_parameter("frame_timeout", 0.25)
        self.declare_parameter("poll_period", 0.01)
        self.declare_parameter("frame_display_period", 0.5)
        self.declare_parameter("sim_step", 0.1)

        self._forward_channel = int(self.get_parameter("forward_channel").value)
        self._turn_channel = int(self.get_parameter("turn_channel").value)
        self._invert_forward = bool(self.get_parameter("invert_forward").value)
        self._invert_turn = bool(self.get_parameter("invert_turn").value)
        self._frame_timeout = float(self.get_parameter("frame_timeout").value)
        self._calibration = ChannelCalibration(
            minimum=int(self.get_parameter("channel_min").value),
            center=int(self.get_parameter("channel_center").value),
            maximum=int(self.get_parameter("channel_max").value),
            deadband=float(self.get_parameter("deadband").value),
        )
        self._tele_publisher = self.create_publisher(
            TeleCmd, "tele_cmd", 10
        )
        self._channel_publisher = self.create_publisher(
            Int32MultiArray, "ibus_channels", 10
        )
        poll_period = float(self.get_parameter("poll_period").value)
        if self._simulation:
            self._sim_forward = 0.0
            self._sim_turn = 0.0
            self._sim_step = float(self.get_parameter("sim_step").value)
            if not 0.0 < self._sim_step <= 1.0:
                raise ValueError("sim_step must be in the range (0, 1]")
            self._configure_terminal()
            self._timer = self.create_timer(poll_period, self._poll_terminal)
            self._publish_sim_command()
            self.get_logger().info(
                "Simulation teleoperation started: W/S forward, A/D turn, "
                "space stop, Q quit"
            )
        else:
            self._decoder = IBusStreamDecoder()
            self._last_frame_time: Optional[float] = None
            self._last_frame_display_time: Optional[float] = None
            self._frame_display_period = float(
                self.get_parameter("frame_display_period").value
            )
            if self._frame_display_period < 0.0:
                raise ValueError("frame_display_period must be non-negative")
            self._failsafe_published = False
            self._serial = self._open_serial()
            self._timer = self.create_timer(poll_period, self._poll_receiver)
            self.get_logger().info(
                f"iBUS teleoperation started on {self._serial.port} at "
                f"{self._serial.baudrate} baud; publishing motor order "
                "[front_left, rear_left, front_right, rear_right]"
            )

    def _configure_terminal(self) -> None:
        """
        ## Def:
            Checks if interactive input and connected to terminal, if not, warn developer
            Else, save terminal settings for restoration later, and set terminal to recieve 
            commands immediately, without key presses.

        ## Args:
            N/A

        ## Returns:
            N/A
        ## Raises:
            N/A
        
        """
        """Use single-key input when attached to an interactive terminal."""
        if not sys.stdin.isatty():
            self.get_logger().warn(
                "stdin is not a terminal; commands will be read as they become available"
            )
            return

        import termios
        import tty

        self._terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def _poll_terminal(self) -> None:
        """
        ## Def:
            Callback function for a timer event. Reads terminal inputs when provided
            from `[sys.stdin]`'s first index and navigates specified actions consequently. 
            
        
        #### Commands:
        `w -> Increase forward velocity`  \n
        `s -> Descrease forward velocity` \n
        `a -> Decrease turn step (left)`  \n
        `d -> Increase turn step (right)` \n

        ## Args:
            N/A 

        ## Returns:
            N/A
        
        ## Raises:
            N/A

        """
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1).lower()
            if not key:
                self._sim_forward = 0.0
                self._sim_turn = 0.0
                break
            if key == "w":
                self._sim_forward = min(1.0, self._sim_forward + self._sim_step)
            elif key == "s":
                self._sim_forward = max(-1.0, self._sim_forward - self._sim_step)
            elif key == "a":
                self._sim_turn = max(-1.0, self._sim_turn - self._sim_step)
            elif key == "d":
                self._sim_turn = min(1.0, self._sim_turn + self._sim_step)
            elif key == " ":
                self._sim_forward = 0.0
                self._sim_turn = 0.0
            elif key == "q":
                self._sim_forward = 0.0
                self._sim_turn = 0.0
                self._publish_sim_command()
                rclpy.shutdown()
                return
            else:
                continue
            # Call in loop in order to not flood the terminal
            self._publish_sim_command()
        self._publish(self._sim_forward, self._sim_turn)

    def _publish_sim_command(self) -> None:
        """
        ## Def:
            Helper function for feedback when sending a command in terminal for sim version. Confirms sent command.
        
        ## Args:
            NA

        ## Returns:
            NA

        ## Raises:
            NA

        """
        self._publish(self._sim_forward, self._sim_turn)
        self.get_logger().info(
            f"command: forward={self._sim_forward:+.1f}, turn={self._sim_turn:+.1f}"
        )

    def _open_serial(self):
        """
        ## Def:

        ## Args:

        ## Returns:

        ## Raises:

        """
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required (install the ROS dependency python3-serial)"
            ) from exc

        port = str(self.get_parameter("serial_port").value)
        baud = int(self.get_parameter("baud_rate").value)
        try:
            return serial.Serial(port=port, baudrate=baud, timeout=0)
        except serial.SerialException as exc:
            raise RuntimeError(f"could not open iBUS serial port {port}: {exc}") from exc

    def _poll_receiver(self) -> None:
        """
        ## Def:

        ## Args:

        ## Returns:

        ## Raises:

        """
        try:
            waiting = self._serial.in_waiting
            if waiting:
                for channels in self._decoder.feed(self._serial.read(waiting)):
                    self._publish_channels(channels)
        except Exception as exc:
            # UART errors must lead to a stopped drivetrain. Throttle repeated logs.
            if not self._failsafe_published:
                self.get_logger().error(f"iBUS UART read failed: {exc}")
            self._last_frame_time = None

        now = time.monotonic()
        if (
            (self._last_frame_time is None or now - self._last_frame_time > self._frame_timeout)
            and not self._failsafe_published
        ):
            self._publish(0.0, 0.0)
            self._failsafe_published = True
            self.get_logger().warn("iBUS frame timeout: publishing zero motor levels")

    def _publish_channels(self, channels) -> None:
        """
        ## Def:

        ## Args:

        ## Returns:

        ## Raises:

        """
        forward, turn, motors = channels_to_drive(
            channels,
            self._forward_channel,
            self._turn_channel,
            self._calibration,
            self._invert_forward,
            self._invert_turn,
        )
        now = time.monotonic()
        self._last_frame_time = now
        self._failsafe_published = False

        raw_channels = Int32MultiArray()
        raw_channels.data = [int(value) for value in channels]
        self._channel_publisher.publish(raw_channels)

        if (
            self._last_frame_display_time is None
            or now - self._last_frame_display_time >= self._frame_display_period
        ):
            formatted_channels = ", ".join(
                f"ch{index + 1}={value}" for index, value in enumerate(channels)
            )
            self.get_logger().info(
                f"iBUS frame: [{formatted_channels}] -> "
                f"forward={forward:+.3f}, turn={turn:+.3f}"
            )
            self._last_frame_display_time = now
        self._publish(forward, turn)

    def _publish(self, forward: float, turn: float) -> None:
        """
        ## Def:

        ## Args:

        ## Returns:

        ## Raises:

        """
        command = TeleCmd()
        command.velocity = forward
        command.turn = turn
        self._tele_publisher.publish(command)

    def destroy_node(self) -> None:
        """
        ## Def:

        ## Args:

        ## Returns:

        ## Raises:

        """
        if hasattr(self, "_serial") and self._serial.is_open:
            self._serial.close()
        if self._terminal_settings is not None:
            import termios

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._terminal_settings)
            self._terminal_settings = None
        super().destroy_node()


def main(args=None) -> None:
    cli_args = list(sys.argv[1:] if args is None else args)
    simulation = False
    for flag in ("--sim", "-sim"):
        while flag in cli_args:
            cli_args.remove(flag)
            simulation = True

    rclpy.init(args=cli_args)
    node = Teleoperation(simulation=simulation)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
