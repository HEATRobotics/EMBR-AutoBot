"""The CAD mesh lookup is a path, not a username."""

from thermal_bench.scene import cad_mesh


def test_env_override_wins_when_the_file_exists(tmp_path, monkeypatch):
    obj = tmp_path / "body.obj"
    obj.write_text("v 0 0 0\n")
    monkeypatch.setenv("EMBR_CAD_MESH", str(obj))
    assert cad_mesh() == obj


def test_a_missing_override_falls_through(tmp_path, monkeypatch):
    monkeypatch.setenv("EMBR_CAD_MESH", str(tmp_path / "missing.obj"))
    found = cad_mesh()
    assert found is None or found.is_file()
    if found is not None:
        assert found.name == "cad_visual.obj"
