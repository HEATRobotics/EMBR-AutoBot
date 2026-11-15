"""The rail says whether the phone stepped toward the iron or away from it."""

from thermal_bench.motion import PhoneRail


def test_back_then_forward():
    rail = PhoneRail()
    first = rail.update(0.40)
    assert first.direction == "still"
    assert first.delta_m == 0.0
    back = rail.update(0.80)
    assert back.direction == "farther"
    assert abs(back.delta_m - 0.40) < 1e-9
    forward = rail.update(0.55)
    assert forward.direction == "closer"
    assert abs(forward.delta_m - (-0.25)) < 1e-9
