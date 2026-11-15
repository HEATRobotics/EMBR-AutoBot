"""Turn a phone export and a Lepton frame into the two inbox files.

The viewer reads `inbox/phone.npy` (Nx3 metres, +x forward, +z up) and
`inbox/thermal.npy` (120×160 Celsius). This module is the only step between
an AirDrop and those files. A color preview is refused. It is not Celsius.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from thermal_bench.frame import HEIGHT, WIDTH

PHONE = "phone"
UNITREE_L2 = "unitree-l2"
MAX_CLOUD_POINTS = 8000
# A phone export in millimetres has norms of hundreds. A metre cloud of a
# room stays under this. 20 m is past a phone lidar anyway.
_MM_MEDIAN_M = 20.0


def cloud_from_path(path: Path) -> np.ndarray:
    """Nx3 metres. `.npy` is used as-is. A mesh export uses its vertices."""
    path = Path(path)
    if path.suffix.lower() == ".npy":
        pts = np.asarray(np.load(path), dtype=np.float64)
    else:
        import trimesh

        kind = path.suffix.lower().lstrip(".")
        loaded = trimesh.load(path, file_type=kind)
        if isinstance(loaded, trimesh.Scene):
            chunks = [
                np.asarray(geom.vertices, dtype=np.float64)
                for geom in loaded.geometry.values()
                if getattr(geom, "vertices", None) is not None and len(geom.vertices)
            ]
            if not chunks:
                raise ValueError(f"{path.name} has no vertices")
            pts = np.vstack(chunks)
        else:
            pts = np.asarray(loaded.vertices, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] == 0:
        raise ValueError(f"{path.name} must be Nx3 points, got {pts.shape}")
    if not np.isfinite(pts).all():
        raise ValueError(f"{path.name} has a non-finite point")
    norms = np.linalg.norm(pts, axis=1)
    if float(np.median(norms)) > _MM_MEDIAN_M:
        pts = pts / 1000.0
    if pts.shape[0] > MAX_CLOUD_POINTS:
        pick = np.random.default_rng(0).choice(pts.shape[0], MAX_CLOUD_POINTS, replace=False)
        pts = pts[np.sort(pick)]
    return np.ascontiguousarray(pts, dtype=np.float64)


def write_cloud(points: np.ndarray, inbox: Path, name: str) -> Path:
    if name not in (PHONE, UNITREE_L2):
        raise ValueError(f"cloud name must be {PHONE!r} or {UNITREE_L2!r}")
    inbox = Path(inbox)
    inbox.mkdir(parents=True, exist_ok=True)
    dest = inbox / f"{name}.npy"
    np.save(dest, points)
    return dest


def thermal_celsius(frame: np.ndarray) -> np.ndarray:
    """120×160 Celsius.

    A float frame in that shape is already Celsius. Integer counts, or a
    float frame whose values are above 1000, are TLinear: count/100 − 273.15.
    A 3-channel picture is the automatic-gain preview and is rejected.
    """
    arr = np.asarray(frame)
    if arr.ndim == 3 and arr.shape[-1] in (3, 4):
        raise ValueError("color preview is automatic gain, not Celsius. Capture Y16 with gain off.")
    if arr.shape == (WIDTH, HEIGHT):
        arr = np.swapaxes(arr, 0, 1)
    if arr.shape != (HEIGHT, WIDTH):
        raise ValueError(f"thermal frame must be 120×160, got {tuple(arr.shape)}")
    values = arr.astype(np.float64)
    integer = np.issubdtype(arr.dtype, np.integer)
    if integer or float(np.nanmax(values)) > 1000.0:
        values = values / 100.0 - 273.15
    return values


def load_thermal(path: Path) -> np.ndarray:
    return thermal_celsius(np.load(path))


def write_thermal(frame_c: np.ndarray, inbox: Path) -> Path:
    inbox = Path(inbox)
    inbox.mkdir(parents=True, exist_ok=True)
    dest = inbox / "thermal.npy"
    np.save(dest, np.asarray(frame_c, dtype=np.float64))
    return dest


def grab_y16(index: int = 0) -> np.ndarray:
    """One Y16 frame from the PureThermal, as Celsius. Color mode is refused."""
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("OpenCV is not installed, so the USB camera cannot be read.") from exc
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"no camera at index {index}. Plug the PureThermal into this laptop.")
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        raise RuntimeError("the camera opened but returned no frame")
    return thermal_celsius(frame)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Write inbox/phone.npy or inbox/thermal.npy")
    parser.add_argument("--inbox", type=Path, default=Path("inbox"))
    sub = parser.add_subparsers(dest="cmd", required=True)

    cloud = sub.add_parser("cloud", help="phone or lidar export → Nx3 metres")
    cloud.add_argument("path", type=Path)
    cloud.add_argument("--as", dest="name", choices=(PHONE, UNITREE_L2), default=PHONE)

    thermal = sub.add_parser("thermal", help="Y16 counts or a Celsius npy → 120×160 °C")
    thermal.add_argument("path", nargs="?", type=Path)
    thermal.add_argument("--uvc", type=int, default=None, metavar="INDEX")

    args = parser.parse_args(argv)
    if args.cmd == "cloud":
        pts = cloud_from_path(args.path)
        dest = write_cloud(pts, args.inbox, args.name)
        print(f"wrote {dest}  {pts.shape[0]} points  +x forward")
        return
    if args.uvc is not None:
        frame = grab_y16(args.uvc)
    elif args.path is None:
        raise SystemExit("pass a Y16 .npy, or --uvc INDEX with the board plugged in")
    else:
        frame = load_thermal(args.path)
    dest = write_thermal(frame, args.inbox)
    center = float(frame[HEIGHT // 2, WIDTH // 2])
    print(f"wrote {dest}  120×160 °C  center {center:.1f}")


if __name__ == "__main__":
    main()
