"""Optional NIST FDS slice loader.

No FDS file is shipped. Sampling refuses with a clear path rather than
silently substituting cinematic fire. When a file is present, this module
would replace the analytic Gaussian field at probe/camera pose — not claimed.
"""

from __future__ import annotations

from pathlib import Path


class NoFdsFileError(FileNotFoundError):
    pass


def load_fds_slice(path: Path | None) -> None:
    if path is None or not Path(path).is_file():
        raise NoFdsFileError(
            "No FDS replay file on disk. Thermal uses the analytic Gaussian "
            "field (ASSUMED °C). This is not NIST/FDS validation."
        )
    raise NoFdsFileError(
        f"FDS replay at {path} is not wired. Refuse rather than fake a CFD grid."
    )
