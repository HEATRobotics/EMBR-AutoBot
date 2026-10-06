"""RH02 CAN transport and ODrive 0.5.6 CANSimple command helpers.

The adapter must run SLCAN firmware for serial access, or gs_usb/candleLight
firmware for Linux SocketCAN. USB connector shape does not select a protocol.
"""

from __future__ import annotations

import math
import struct
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import can


def can_simple_id(node_id: int, command_id: int) -> int:
    """Encode a 0.5.6 axis node (0–63) and command into an 11-bit CAN ID."""
    if not isinstance(node_id, int) or not 0 <= node_id <= 63:
        raise ValueError("node_id must be an integer in [0, 63]")
    if not isinstance(command_id, int) or not 0 <= command_id <= 31:
        raise ValueError("command_id must be an integer in [0, 31]")
    return (node_id << 5) | command_id


class UsbCan:
    """Explicitly open, send, receive and close a CAN transport.

    Use ``slcan`` with /dev/ttyACM0 (or COM4 on native Windows), ``socketcan``
    with a preconfigured can0, or ``virtual`` for an in-process simulation.
    Construction and opening never send ODrive commands. A CAN node ID names
    an axis, not the board's physical axis number.
    """

    def __init__(
        self,
        channel: str,
        interface: str = "slcan",
        bitrate: int = 1_000_000,
        tty_baudrate: int = 115_200,
        send_timeout: float = 0.1,
    ) -> None:
        if not isinstance(channel, str) or not channel.strip():
            raise ValueError("channel must name a serial port or CAN interface")
        if interface not in ("slcan", "socketcan", "virtual"):
            raise ValueError("interface must be slcan, socketcan or virtual")
        if bitrate not in (10_000, 20_000, 50_000, 100_000, 125_000,
                           250_000, 500_000, 800_000, 1_000_000):
            raise ValueError("unsupported classic CAN bitrate")
        if not isinstance(tty_baudrate, int) or tty_baudrate <= 0:
            raise ValueError("tty_baudrate must be a positive integer")
        self._validate_timeout(send_timeout)
        self.channel = channel
        self.interface = interface
        self.bitrate = bitrate
        self.tty_baudrate = tty_baudrate
        self.send_timeout = send_timeout
        self._bus = None

    @staticmethod
    def _validate_timeout(timeout: float) -> None:
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError("timeout must be finite and nonnegative")

    @property
    def is_open(self) -> bool:
        return self._bus is not None

    def open(self) -> UsbCan:
        """Open the selected adapter; SocketCAN bitrate is configured by Linux."""
        if self.is_open:
            return self
        import can

        options = {}
        if self.interface == "slcan":
            options = {"bitrate": self.bitrate, "tty_baudrate": self.tty_baudrate}
        # Ignore user/global python-can config so it cannot redirect the transport.
        self._bus = can.Bus(
            channel=self.channel, interface=self.interface,
            ignore_config=True, **options,
        )
        return self

    def close(self) -> None:
        """Release the adapter; closing does not command the motor to stop."""
        if self._bus is not None:
            try:
                self._bus.shutdown()
            finally:
                self._bus = None

    def __enter__(self) -> UsbCan:
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def _require_bus(self):
        if self._bus is None:
            raise RuntimeError("CAN transport is closed; call open() first")
        return self._bus

    def send_frame(
        self, arbitration_id: int, data: bytes = b"", *,
        is_remote_frame: bool = False, dlc: int | None = None,
    ) -> None:
        """Send a standard classic CAN frame, optionally an RTR request."""
        bus = self._require_bus()
        import can

        if not isinstance(arbitration_id, int) or not 0 <= arbitration_id <= 0x7FF:
            raise ValueError("arbitration_id must be an integer in [0, 0x7ff]")
        if not isinstance(data, (bytes, bytearray)) or len(data) > 8:
            raise ValueError("data must contain at most eight bytes")
        if is_remote_frame and data:
            raise ValueError("RTR frames cannot carry data")
        length = len(data) if dlc is None else dlc
        if not isinstance(length, int) or not 0 <= length <= 8:
            raise ValueError("dlc must be an integer in [0, 8]")
        if not is_remote_frame and length != len(data):
            raise ValueError("data frame dlc must match payload length")
        message = can.Message(
            arbitration_id=arbitration_id, data=data, dlc=length,
            is_extended_id=False, is_remote_frame=is_remote_frame, check=True,
        )
        bus.send(message, timeout=self.send_timeout)

    def receive(self, timeout: float = 0.0) -> can.Message | None:
        """Receive one frame, returning None on timeout; default is nonblocking."""
        self._validate_timeout(timeout)
        return self._require_bus().recv(timeout=timeout)

    def send_command(self, node_id: int, command_id: int, data: bytes = b"") -> None:
        """Send a CANSimple command to one configured 0.5.6 axis node."""
        self.send_frame(can_simple_id(node_id, command_id), data)

    def request_encoder_estimates(self, node_id: int) -> None:
        """Request command 0x09; reply payload is <ff, turns and turns/second."""
        self.send_frame(can_simple_id(node_id, 0x09), is_remote_frame=True, dlc=8)

    def set_axis_state(self, node_id: int, state: int) -> None:
        """Send a requested state (1=idle, 8=closed loop on ODrive 0.5.6)."""
        self.send_command(node_id, 0x07, struct.pack("<I", state))

    def set_controller_mode(self, node_id: int, control_mode: int, input_mode: int) -> None:
        self.send_command(node_id, 0x0B, struct.pack("<ii", control_mode, input_mode))

    def set_input_velocity(
        self, node_id: int, velocity: float, torque_feedforward: float = 0.0,
    ) -> None:
        """Send velocity in turns/second and feedforward torque in Nm."""
        self.send_command(node_id, 0x0D, self._float_payload(velocity, torque_feedforward))

    def set_input_torque(self, node_id: int, torque: float) -> None:
        """Send input torque in Nm."""
        self.send_command(node_id, 0x0E, self._float_payload(torque))

    @staticmethod
    def _float_payload(*values: float) -> bytes:
        if not all(math.isfinite(value) for value in values):
            raise ValueError("command values must be finite")
        return struct.pack("<" + "f" * len(values), *values)
