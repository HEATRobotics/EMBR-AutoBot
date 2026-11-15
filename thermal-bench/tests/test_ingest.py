"""A phone export and a Y16 frame become the two files the viewer reads."""

import numpy as np
import pytest

from thermal_bench.frame import HEIGHT, WIDTH
from thermal_bench.ingest import (
    MAX_CLOUD_POINTS,
    cloud_from_path,
    load_thermal,
    thermal_celsius,
    write_cloud,
    write_thermal,
)
from thermal_bench.ranges import PHONE, scan
from thermal_bench.scene import load_model


def _ply(path, rows: str) -> None:
    n = len(rows.strip().splitlines())
    path.write_text(
        "ply\nformat ascii 1.0\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "end_header\n" + rows
    )


def test_ply_vertices_become_a_phone_cloud_the_viewer_can_load(tmp_path):
    src = tmp_path / "scan.ply"
    _ply(src, "0.50 0.0 0.0\n0.0 0.20 0.10\n")
    pts = cloud_from_path(src)
    dest = write_cloud(pts, tmp_path / "inbox", PHONE)
    loaded = np.load(dest)
    assert loaded.shape == (2, 3)
    assert abs(float(loaded[0, 0]) - 0.5) < 1e-6

    import mujoco

    model, data = load_model()
    mujoco.mj_forward(model, data)
    cloud = scan(PHONE, model, data, site="lepton", inbox=tmp_path / "inbox")
    assert abs(cloud.forward_m - 0.5) < 1e-6


def test_millimetre_export_is_scaled_to_metres(tmp_path):
    src = tmp_path / "scan.ply"
    _ply(src, "500 0 0\n0 200 100\n")
    pts = cloud_from_path(src)
    assert abs(float(pts[0, 0]) - 0.5) < 1e-6
    assert abs(float(pts[1, 1]) - 0.2) < 1e-6


def test_a_big_scan_is_thinned_to_a_fixed_count(tmp_path):
    src = tmp_path / "room.npy"
    rng = np.random.default_rng(1)
    np.save(src, rng.normal(size=(20_000, 3)))
    pts = cloud_from_path(src)
    assert pts.shape == (MAX_CLOUD_POINTS, 3)
    again = cloud_from_path(src)
    assert np.array_equal(pts, again)


def test_y16_counts_match_the_datasheet_and_a_color_frame_is_refused(tmp_path):
    counts = np.full((HEIGHT, WIDTH), 30000, dtype=np.uint16)
    counts[0, 0] = 29315  # 20.00 °C
    celsius = thermal_celsius(counts)
    assert celsius.shape == (120, 160)
    assert abs(float(celsius[60, 80]) - 26.85) < 1e-6
    assert abs(float(celsius[0, 0]) - 20.0) < 1e-6

    wide = thermal_celsius(np.full((WIDTH, HEIGHT), 30000, dtype=np.uint16))
    assert wide.shape == (120, 160)

    dest = write_thermal(celsius, tmp_path)
    assert np.allclose(load_thermal(dest), celsius)

    color = np.zeros((120, 160, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match="automatic gain"):
        thermal_celsius(color)


def test_a_bad_cloud_shape_is_refused(tmp_path):
    src = tmp_path / "bad.npy"
    np.save(src, np.zeros((4, 4)))
    with pytest.raises(ValueError, match="Nx3"):
        cloud_from_path(src)
