"""Read one frame from a PureThermal UVC device, if it is plugged in.

A color frame from the stock firmware is an AGC picture. It is not Celsius.
Y16 with AGC off is raw counts. Those counts are Celsius only when the core
is radiometric (Lepton 3.1R, 3.5, or 2.5) and the firmware is in TLinear.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np


def read_uvc(index: int = 0):
    """Return one frame, or None if OpenCV or the camera is missing."""
    try:
        import cv2
    except ImportError:
        return None
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        cap.release()
        return None
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        return None
    return frame


def save_frame(frame: np.ndarray, path: Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.save(path, frame)


def describe(frame: np.ndarray | None) -> str:
    if frame is None:
        return "No PureThermal frame. The board is unplugged, or OpenCV is not installed."
    return (
        f"Captured shape {frame.shape} dtype {frame.dtype}. "
        "If this is a 3-channel color image it is AGC, not Celsius. "
        "Request Y16 and confirm the core is 3.1R, 3.5, or 2.5 before converting counts."
    )
