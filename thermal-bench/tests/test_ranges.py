"""Phone depth and the Unitree L2 dome share one cloud, and the phone tracks distance."""

import numpy as np

from thermal_bench.ranges import L2_MIN_M, PHONE, PHONE_MAX_M, UNITREE_L2, scan
from thermal_bench.scene import load_model, place_phone


def _phone_range(distance: float) -> float:
    model, data = load_model()
    place_phone(data, distance)
    import mujoco

    mujoco.mj_forward(model, data)
    cloud = scan(PHONE, model, data, site="lepton")
    assert cloud.name == PHONE
    assert cloud.xyz.shape[1] == 3
    assert cloud.reflectance.shape == cloud.ranges_m.shape
    assert np.all(cloud.reflectance <= 1.0)
    assert cloud.nearest_m <= PHONE_MAX_M
    return cloud.forward_m


def test_phone_reports_a_longer_hit_when_it_moves_back():
    near = _phone_range(0.40)
    far = _phone_range(0.90)
    assert 0.25 < near < 0.50
    assert far > near + 0.30


def test_unitree_dome_is_the_same_cloud_type_and_stays_blind_up_close():
    import mujoco

    model, data = load_model()
    mujoco.mj_forward(model, data)
    cloud = scan(UNITREE_L2, model, data, site="lidar")
    assert cloud.name == UNITREE_L2
    assert cloud.xyz.shape[1] == 3
    if cloud.ranges_m.size:
        assert float(np.min(cloud.ranges_m)) >= L2_MIN_M - 1e-6
    # The dome looks up and around from the roof site, not along the phone rail.
    assert cloud.nearest_m != _phone_range(0.40)


def test_inbox_cloud_replaces_the_rays():
    import tempfile
    from pathlib import Path

    import mujoco

    model, data = load_model()
    mujoco.mj_forward(model, data)

    with tempfile.TemporaryDirectory() as tmp:
        pts = np.array([[0.5, 0.0, 0.0], [0.0, 0.2, 0.1]], dtype=np.float64)
        np.save(Path(tmp) / "phone.npy", pts)
        cloud = scan(PHONE, model, data, site="lepton", inbox=Path(tmp))
    assert cloud.ranges_m.shape == (2,)
    assert abs(float(cloud.ranges_m[0]) - 0.5) < 1e-6
