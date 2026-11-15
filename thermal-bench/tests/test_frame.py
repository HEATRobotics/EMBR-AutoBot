"""The 160×120 frame matches a candle the camera is aimed at."""

import mujoco
import numpy as np

from thermal_bench.frame import (
    HEIGHT,
    WIDTH,
    colorize,
    hot_pixel_count,
    normalize_fixed,
    normalize_unit,
    render_frame,
    sample_temperature,
    to_tlinear_counts,
)
from thermal_bench.scene import AMBIENT, CANDLE, candle_sources, place_hand, load_model


def _frame(distance: float, hfov: float = 57.0):
    pos = (CANDLE.x - distance, CANDLE.y, CANDLE.z)
    return render_frame(pos, hfov_deg=hfov, sources=candle_sources())


def test_frame_is_lepton_160_by_120():
    frame = _frame(0.45)
    assert frame.shape == (HEIGHT, WIDTH)
    assert frame.shape == (120, 160)


def test_center_sees_the_candle_and_the_corner_does_not():
    frame = _frame(0.45)
    center = frame[HEIGHT // 2, WIDTH // 2]
    corner = frame[0, 0]
    assert center > AMBIENT + 30.0
    assert corner < AMBIENT + 5.0
    assert center > corner


def test_the_hot_spot_shrinks_when_the_camera_moves_back():
    near = _frame(0.28)
    far = _frame(0.95)
    assert hot_pixel_count(near, AMBIENT + 8.0) > hot_pixel_count(far, AMBIENT + 8.0)
    # The ray still goes through the candle, so the center stays hot.
    assert far[HEIGHT // 2, WIDTH // 2] > AMBIENT + 30.0


def test_wider_3_1r_lens_spreads_the_same_candle():
    narrow = _frame(0.45, hfov=57.0)
    wide = _frame(0.45, hfov=95.0)
    assert hot_pixel_count(narrow, AMBIENT + 8.0) > hot_pixel_count(wide, AMBIENT + 8.0)


def test_normalize_is_unit_interval_and_tlinear_matches_the_datasheet_example():
    frame = _frame(0.45)
    unit = normalize_unit(frame)
    assert unit.min() >= 0.0 and unit.max() <= 1.0 + 1e-9
    # Datasheet example: 26.85 °C -> 30000 counts. (26.85 + 273.15) * 100.
    assert int(to_tlinear_counts(np.array([26.85]))[0]) == 30000


def test_fixed_scale_keeps_the_same_temperature_the_same_color():
    room = normalize_fixed(np.array([20.0]))
    top = normalize_fixed(np.array([140.0]))
    assert float(room[0]) == 0.0
    assert float(top[0]) == 1.0
    # A hotter tip does not repaint the room pixel.
    mild = colorize(np.array([[20.0, 80.0]], dtype=float))
    iron = colorize(np.array([[20.0, 400.0]], dtype=float))
    assert np.array_equal(mild[0, 0], iron[0, 0])
    assert not np.array_equal(mild[0, 1], iron[0, 1])
    near = colorize(_frame(0.28))
    far = colorize(_frame(0.95))
    # The corner stays room-colored. A per-frame stretch would repaint it.
    assert int(np.max(np.abs(near[0, 0].astype(int) - far[0, 0].astype(int)))) <= 2


def test_a_lidar_hit_on_the_candle_is_hot_and_the_floor_is_not():
    sources = candle_sources()
    candle = sources[0]
    on_flame = np.array([[candle.x, candle.y, candle.z]])
    on_floor = np.array([[candle.x, candle.y, 0.0]])
    hot = float(sample_temperature(on_flame, sources)[0])
    cold = float(sample_temperature(on_floor, sources)[0])
    assert hot > AMBIENT + 30.0
    assert cold < AMBIENT + 5.0
    hot_rgb = colorize(np.array([[hot]]))[0, 0]
    cold_rgb = colorize(np.array([[cold]]))[0, 0]
    assert int(hot_rgb[0]) > int(cold_rgb[0])


def test_mujoco_knows_the_hand_distance():
    model, data = load_model()
    pos = place_hand(data, 0.50)
    assert abs(pos[0] - (CANDLE.x - 0.50)) < 1e-6
    assert model.ncam >= 0
    assert model.nmocap == 1
    lidar = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "lidar")
    lepton = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "lepton")
    assert lidar >= 0 and lepton >= 0
