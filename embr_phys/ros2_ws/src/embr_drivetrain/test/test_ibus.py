from embr_drivetrain.drivetrain_hardware.ibus import (
    ChannelCalibration,
    IBusStreamDecoder,
    channels_to_drive,
    decode_ibus_frame,
    mix_four_motor_levels,
)


def make_frame(channels):
    payload = bytearray([32, 0x40])
    for channel in channels:
        payload.extend((channel & 0xFF, channel >> 8))
    checksum = 0xFFFF - sum(payload)
    payload.extend((checksum & 0xFF, checksum >> 8))
    return bytes(payload)


def test_decode_valid_frame_and_reject_bad_checksum():
    frame = make_frame([1000, 1500, 2000] + [1500] * 11)
    assert decode_ibus_frame(frame)[:3] == [1000, 1500, 2000]
    assert decode_ibus_frame(frame[:-1] + b"\x00") is None


def test_stream_decoder_handles_noise_and_split_frame():
    frame = make_frame([1500] * 14)
    decoder = IBusStreamDecoder()
    assert decoder.feed(b"noise" + frame[:12]) == []
    assert decoder.feed(frame[12:]) == [[1500] * 14]


def test_normalization_deadband_and_drive_mix():
    calibration = ChannelCalibration(deadband=0.04)
    assert calibration.normalize(1500) == 0.0
    assert calibration.normalize(1510) == 0.0
    forward, turn, motors = channels_to_drive(
        [1750, 1250] + [1500] * 12, 0, 1, calibration
    )
    assert forward > 0.0
    assert turn < 0.0
    assert motors[0] == motors[1]
    assert motors[2] == motors[3]


def test_motor_mix_scales_without_changing_ratio():
    assert mix_four_motor_levels(1.0, 1.0) == [0.0, 0.0, 1.0, 1.0]
    assert mix_four_motor_levels(-1.0, -1.0) == [0.0, 0.0, -1.0, -1.0]
