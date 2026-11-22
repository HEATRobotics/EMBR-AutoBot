"""PureThermal 3 / Lepton frame normalization and the MuJoCo handoff.

Live USB and blackbody checks are skipped at the bottom. The reasons are in
those tests: this package has no recorded Y16 frame from the team's board and
no reference surface temperature.
"""

from __future__ import annotations

import os
import time

import numpy as np
import pytest

from embr_local_sim.lepton import (
    RAD_ENABLE_OFFSET,
    RAD_TLINEAR_ENABLE_OFFSET,
    RAD_TLINEAR_RESOLUTION_OFFSET,
    LeptonCapture,
    centikelvin_to_celsius,
    column_max_celsius,
    normalize_frame,
    rad_control_id,
    tlinear_payload,
    video_plane,
)
from embr_local_sim.physics import Simulator


def _centi(celsius: float, shape: tuple[int, int] = (120, 160)) -> np.ndarray:
    raw = int(round(celsius * 100.0 + 27315.0))
    return np.full(shape, raw, dtype=np.uint16)


def test_rad_selectors_and_payload():
    # GroupGets ctrl_gen.py: selector = (command_offset >> 2) + 1, four LE bytes.
    assert rad_control_id(RAD_ENABLE_OFFSET) == 5
    assert rad_control_id(0x10) == 5
    assert rad_control_id(RAD_TLINEAR_ENABLE_OFFSET) == 49
    assert rad_control_id(0xC0) == 49
    assert rad_control_id(RAD_TLINEAR_RESOLUTION_OFFSET) == 50
    assert rad_control_id(0xC4) == 50
    assert tlinear_payload(1) == b"\x01\x00\x00\x00"


def test_centikelvin_matches_groupgets_ktoc():
    # 30000 centi-Kelvin is the example in the FLIR T-linear note: 26.85 °C.
    assert centikelvin_to_celsius(np.array([30000]))[0] == pytest.approx(26.85)
    assert centikelvin_to_celsius(np.array([27315]))[0] == pytest.approx(0.0)
    assert centikelvin_to_celsius(np.array([26315]))[0] == pytest.approx(-10.0)


def test_tlinear_frame_reports_celsius():
    frame = normalize_frame(_centi(26.85))
    assert frame.valid
    assert frame.celsius_valid
    assert frame.celsius.shape == (120, 160)
    assert frame.mean_c == pytest.approx(26.85, abs=0.02)
    assert frame.display_rgb.shape == (120, 160, 3)
    assert frame.display_rgb.dtype == np.uint8


def test_telemetry_rows_are_not_pixels():
    raw = np.zeros((122, 160), dtype=np.uint16)
    raw[:120, :] = 30000
    raw[120:, :] = 1
    video, note = video_plane(raw)
    assert video.shape == (120, 160)
    assert "telemetry" in note
    frame = normalize_frame(raw)
    assert frame.celsius_valid
    assert frame.mean_c == pytest.approx(26.85, abs=0.02)
    assert frame.telemetry_note


def test_fourteen_bit_raw_is_not_converted():
    raw = np.full((120, 160), 8000, dtype=np.uint16)
    raw[0, 0] = 16383
    frame = normalize_frame(raw)
    assert frame.valid
    assert frame.celsius is None
    assert not frame.celsius_valid
    assert "14-bit" in frame.reason
    assert frame.display_rgb.shape == (120, 160, 3)


def test_eight_bit_gray_is_not_celsius():
    raw = np.full((120, 160), 180, dtype=np.uint8)
    frame = normalize_frame(raw)
    assert frame.valid
    assert not frame.celsius_valid


def test_a_zero_pixel_is_not_absolute_zero():
    raw = _centi(20.0)
    raw[0, 0] = np.uint16(0)
    frame = normalize_frame(raw)
    assert frame.celsius_valid
    assert frame.min_c == pytest.approx(20.0, abs=0.05)
    assert np.isnan(frame.celsius[0, 0])


def test_bgr_uint8_is_not_a_temperature_frame():
    raw = np.zeros((120, 160, 3), dtype=np.uint8)
    raw[..., 1] = 103
    raw[..., 2] = 241
    frame = normalize_frame(raw)
    assert not frame.celsius_valid
    assert frame.celsius is None
    assert "2D Y16" in frame.reason


def test_blank_frame_is_invalid():
    frame = normalize_frame(np.zeros((120, 160), dtype=np.uint16))
    assert not frame.valid
    assert not frame.celsius_valid
    assert "blank" in frame.reason


def test_wrong_shape_is_rejected():
    frame = normalize_frame(np.full((60, 80), 30000, dtype=np.uint16))
    assert not frame.valid
    assert "160" in frame.reason


def test_y16_byte_pairs_unpack():
    pixel = np.uint16(30000)
    lo = np.full((120, 160), pixel & 0xFF, dtype=np.uint8)
    hi = np.full((120, 160), pixel >> 8, dtype=np.uint8)
    packed = np.stack((lo, hi), axis=-1)
    frame = normalize_frame(packed)
    assert frame.celsius_valid
    assert frame.mean_c == pytest.approx(26.85, abs=0.02)


def test_submit_copies_so_the_caller_buffer_can_change():
    capture = LeptonCapture(stale_s=5.0)
    raw = _centi(40.0)
    capture.submit(raw)
    raw[:] = 0
    frame = capture.latest()
    assert frame.celsius_valid
    assert frame.mean_c == pytest.approx(40.0, abs=0.02)


def test_stale_frame_drops_celsius():
    capture = LeptonCapture(stale_s=0.05)
    capture.submit(_centi(30.0))
    time.sleep(0.12)
    frame = capture.latest()
    assert not frame.valid
    assert not frame.celsius_valid
    assert frame.reason.startswith("stale:")


def test_column_max_follows_the_hot_side():
    image = np.full((120, 160), 20.0, dtype=np.float32)
    image[:, 120:] = 80.0
    bins = column_max_celsius(image, 4)
    assert bins[0] == pytest.approx(20.0)
    assert bins[-1] == pytest.approx(80.0)


def test_capture_thread_publishes_without_usb():
    class Scripted:
        def read(self) -> np.ndarray:
            return _centi(22.0)

        def close(self) -> None:
            return None

    capture = LeptonCapture(opener=lambda: Scripted(), stale_s=2.0, source_name="fake")
    capture.start()
    try:
        deadline = time.time() + 2.0
        while time.time() < deadline and capture.latest().sequence < 1:
            time.sleep(0.01)
        frame = capture.latest()
    finally:
        capture.stop()
    assert frame.sequence >= 1
    assert frame.celsius_valid
    assert frame.mean_c == pytest.approx(22.0, abs=0.02)
    assert frame.source == "fake"


def test_missing_camera_does_not_block_the_loop():
    def opener():
        raise RuntimeError("no PureThermal 3 on this bus")

    capture = LeptonCapture(opener=opener, retry_s=0.05)
    capture.start()
    try:
        deadline = time.time() + 1.5
        frame = capture.latest()
        while time.time() < deadline and "no PureThermal" not in frame.reason:
            time.sleep(0.02)
            frame = capture.latest()
    finally:
        capture.stop()
    assert not frame.valid
    assert "no PureThermal" in frame.reason


def test_simulator_starts_and_copies_celsius_without_a_device():
    capture = LeptonCapture(stale_s=5.0)
    capture.submit(_centi(26.85))
    sim = Simulator(seed=0, lepton=capture)
    sim.reset()
    sim.step_control_tick()
    _gt, obs = sim.snapshot()
    assert obs.lepton_celsius_valid
    assert obs.lepton_celsius.shape == (120, 160)
    assert obs.lepton_mean_c == pytest.approx(26.85, abs=0.05)
    # Flat scene ambient stays on the synthetic bins. The laptop image is separate.
    assert max(obs.thermal_c) < 40.0


def test_simulator_without_lepton_keeps_the_old_observation():
    sim = Simulator(seed=0)
    sim.reset()
    sim.step_control_tick()
    _gt, obs = sim.snapshot()
    assert not obs.lepton_valid
    assert obs.lepton_celsius is None
    assert obs.lepton_reason == "camera not attached"
    assert obs.thermal_c


def test_drive_flag_replaces_bins_from_the_image():
    image = _centi(20.0)
    image[:, 120:] = np.uint16(27315 + 8000)
    capture = LeptonCapture(stale_s=5.0)
    capture.submit(image)
    sim = Simulator(seed=0, lepton=capture, lepton_drive_bins=True)
    sim.reset()
    sim.step_control_tick()
    _gt, obs = sim.snapshot()
    assert obs.thermal_c[-1] > 70.0
    assert obs.thermal_c[0] < 30.0
    view_bins = sim._mission_view(obs, _gt).thermal_c
    assert view_bins[-1] == pytest.approx(obs.thermal_c[-1])


def test_hotter_pixel_is_brighter_in_the_display():
    raw = _centi(20.0)
    raw[60, 10] = np.uint16(27315 + 9000)
    frame = normalize_frame(raw)
    cool = int(frame.display_rgb[0, 159, 0])
    hot = int(frame.display_rgb[60, 10, 0])
    assert hot > cool


def test_live_purethermal_open_needs_a_plugged_in_board():
    index = os.environ.get("LEPTON_DEVICE")
    if index is None:
        pytest.skip(
            "No live PureThermal 3 read was run. LEPTON_DEVICE is unset, and this "
            "checkout has no saved Y16 frame from the team's board. Centi-Kelvin "
            "conversion, telemetry stripping, 14-bit rejection, the capture thread, "
            "and the MuJoCo observation copy are covered above with synthetic buffers. "
            "Plug in the board, install the lepton extra, and set LEPTON_DEVICE to the "
            "UVC index (usually 0) to attempt a hardware read."
        )
    pytest.importorskip("cv2")
    from embr_local_sim.lepton import purethermal_opener

    capture = LeptonCapture(opener=purethermal_opener(int(index)), stale_s=2.0)
    capture.start()
    try:
        deadline = time.time() + 5.0
        frame = capture.latest()
        while time.time() < deadline and frame.sequence < 1:
            time.sleep(0.05)
            frame = capture.latest()
    finally:
        capture.stop()
    assert frame.sequence >= 1, frame.reason
    assert frame.raw.shape == (120, 160)


def test_blackbody_accuracy_was_not_measured():
    pytest.skip(
        "No in-scene reference temperature was recorded for this Lepton, so the "
        "FLIR radiometric tolerance (about ±5 °C in high gain for a radiometric "
        "module) cannot be checked against a known surface. The unit conversion "
        "itself is tested in test_centikelvin_matches_groupgets_ktoc."
    )
