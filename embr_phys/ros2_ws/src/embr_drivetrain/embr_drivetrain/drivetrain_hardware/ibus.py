"""FlySky/Turnigy iBUS receiver decoding and drivetrain mixing helpers."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Optional

IBUS_FRAME_LENGTH = 32
IBUS_COMMAND = 0x40
IBUS_CHANNEL_COUNT = 14


def decode_ibus_frame(frame: bytes) -> Optional[list[int]]:
    """Return the 14 channel values from a valid iBUS servo frame.

    iBUS uses a little-endian checksum equal to ``0xffff - sum(payload)``.
    Invalid or incomplete data is deliberately ignored.
    """
    if len(frame) != IBUS_FRAME_LENGTH:
        return None
    if frame[0] != IBUS_FRAME_LENGTH or frame[1] != IBUS_COMMAND:
        return None

    expected_checksum = 0xFFFF - sum(frame[:-2])
    received_checksum = frame[-2] | (frame[-1] << 8)
    if expected_checksum != received_checksum:
        return None

    return [
        frame[index] | (frame[index + 1] << 8)
        for index in range(2, 30, 2)
    ]


class IBusStreamDecoder:
    """Recover complete iBUS frames from arbitrary UART byte chunks."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[list[int]]:
        self._buffer.extend(data)
        decoded: list[list[int]] = []

        while len(self._buffer) >= 2:
            # Re-synchronise on the standard servo-frame header.
            if self._buffer[0] != IBUS_FRAME_LENGTH or self._buffer[1] != IBUS_COMMAND:
                del self._buffer[0]
                continue
            if len(self._buffer) < IBUS_FRAME_LENGTH:
                break

            frame = bytes(self._buffer[:IBUS_FRAME_LENGTH])
            channels = decode_ibus_frame(frame)
            if channels is None:
                # Only discard one byte: a valid header may exist inside corrupt data.
                del self._buffer[0]
            else:
                decoded.append(channels)
                del self._buffer[:IBUS_FRAME_LENGTH]

        return decoded


@dataclass(frozen=True)
class ChannelCalibration:
    minimum: int = 1000
    center: int = 1500
    maximum: int = 2000
    deadband: float = 0.04

    def __post_init__(self) -> None:
        if not self.minimum < self.center < self.maximum:
            raise ValueError("channel minimum < center < maximum is required")
        if not 0.0 <= self.deadband < 1.0:
            raise ValueError("channel deadband must be in [0, 1)")

    def normalize(self, value: int) -> float:
        """Map a calibrated RC value to [-1, 1], including a centre deadband."""
        span = self.maximum - self.center if value >= self.center else self.center - self.minimum
        normalized = max(-1.0, min(1.0, (value - self.center) / span))
        deadband = self.deadband
        if abs(normalized) <= deadband:
            return 0.0
        magnitude = (abs(normalized) - deadband) / (1.0 - deadband)
        return magnitude if normalized > 0.0 else -magnitude


def mix_four_motor_levels(forward: float, turn: float) -> list[float]:
    """Mix forward/turn inputs for [front-left, rear-left, front-right, rear-right]."""
    left = forward - turn
    right = forward + turn
    scale = max(1.0, abs(left), abs(right))
    return [left / scale, left / scale, right / scale, right / scale]


def channels_to_drive(
    channels: Sequence[int],
    forward_channel: int,
    turn_channel: int,
    calibration: ChannelCalibration,
    invert_forward: bool = False,
    invert_turn: bool = False,
) -> tuple[float, float, list[float]]:
    """Convert zero-based iBUS channel indexes into normalized drive commands."""
    if not 0 <= forward_channel < len(channels) or not 0 <= turn_channel < len(channels):
        raise IndexError("configured iBUS channel is not present in the frame")
    forward = calibration.normalize(channels[forward_channel])
    turn = calibration.normalize(channels[turn_channel])
    if invert_forward:
        forward = -forward
    if invert_turn:
        turn = -turn
    return forward, turn, mix_four_motor_levels(forward, turn)
