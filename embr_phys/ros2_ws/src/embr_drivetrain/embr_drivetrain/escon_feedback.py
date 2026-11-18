"""Read-only ESCON2 Sensor 2 telemetry on the drivetrain's SDO connection."""

import ipaddress
import itertools
import json
import socket
import time
import uuid
from dataclasses import dataclass


def read_signed(motor, index, subindex=0):
    """Read a signed INTEGER32 object, rejecting a malformed response."""
    data = motor.sdo.upload(index, subindex)
    if len(data) != 4:
        raise ValueError(f'Expected INTEGER32 at 0x{index:04X}:{subindex:02X}')
    return int.from_bytes(data, 'little', signed=True)


def validate_encoder(read, counts_per_rev):
    """Check commissioned Sensor 2 scaling using SDO uploads only."""
    sensors = read(0x3000, 1)
    if (sensors >> 8) & 0xFF != 1:
        raise ValueError('Configure Sensor 2 as digital incremental encoder (0x3000:01)')
    pulses = read(0x3010, 1)
    if not 16 <= pulses <= 2500000 or pulses * 4 != counts_per_rev:
        raise ValueError(
            f'Sensor 2 has {pulses} pulses/rev (0x3010:01); '
            'encoder_counts_per_rev must match four times this value '
            f'(configured {counts_per_rev:g})')
    if read(0x60A8, 0) != 0x00B50000:
        raise ValueError('Configure position units as increments (0x60A8)')


def count_delta(current, previous):
    """Unwrap adjacent signed 32-bit encoder observations."""
    return (current - previous + (1 << 31)) % (1 << 32) - (1 << 31)


@dataclass
class Encoder:
    motor: object
    counts_per_rev: int
    previous: int | None = None
    relative: int = 0
    reference: int = 0
    sample_s: float = 0.0
    retry_after_s: float = 0.0
    sequence: int = 0


class FeedbackPublisher:
    """Poll one drive per callback and send one versioned observation per datagram."""

    def __init__(self, motors, scales, host, port, timeout, reference_gap):
        if not 1 <= port <= 65535:
            raise ValueError('feedback_port must be in [1, 65535]')
        if any(not 64 <= scale <= 10_000_000 or scale % 4 for scale in scales):
            raise ValueError('feedback_encoder_counts_per_rev must be quadrature counts')
        self.destination = str(ipaddress.IPv4Address(host)), port
        self.encoders = itertools.cycle(
            Encoder(motor, scale) for motor, scale in zip(motors, scales, strict=True))
        self.timeout = timeout
        self.reference_gap = reference_gap
        self.session = uuid.uuid4().hex
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)

    def poll(self):
        """Serialize with the command node's timers. A failing node gets a retry delay."""
        encoder = next(self.encoders)
        started = time.monotonic()
        if started < encoder.retry_after_s:
            return
        motor = encoder.motor
        old_timeout = motor.sdo.RESPONSE_TIMEOUT
        motor.sdo.RESPONSE_TIMEOUT = min(old_timeout, self.timeout)
        try:
            new_reference = (encoder.previous is None
                             or started - encoder.sample_s >= self.reference_gap)
            if new_reference:
                def read(index, subindex):
                    return int.from_bytes(motor.sdo.upload(index, subindex), 'little')

                validate_encoder(read, encoder.counts_per_rev)
                if read(0x60A9, 0) != 0x00B44700:
                    raise ValueError('Configure velocity units as rpm (0x60A9)')
            counts = read_signed(motor, 0x60E4, 2)
            rpm = read_signed(motor, 0x606C)
        except Exception as exc:
            # CANopen transports report upload failures via backend-specific exceptions.
            encoder.previous = None
            encoder.retry_after_s = time.monotonic() + self.reference_gap
            fields = {'error': str(exc)[:256] or type(exc).__name__}
        else:
            if new_reference:
                encoder.reference += 1
                encoder.relative = 0
            else:
                encoder.relative += count_delta(counts, encoder.previous)
            encoder.previous = counts
            fields = {
                'reference': encoder.reference, 'counts': counts,
                'relative_counts': encoder.relative, 'counts_per_rev': encoder.counts_per_rev,
                'rpm': rpm,
            }
        finally:
            motor.sdo.RESPONSE_TIMEOUT = old_timeout
        observed = time.monotonic()
        encoder.sample_s = observed
        packet = {
            'version': 1, 'session': self.session, 'seq': encoder.sequence,
            'node_id': motor.id, 'sample_s': observed, 'poll_s': observed - started,
            **fields,
        }
        encoder.sequence += 1
        self.socket.sendto(json.dumps(packet, separators=(',', ':')).encode('utf-8'),
                           self.destination)

    def close(self):
        self.socket.close()
