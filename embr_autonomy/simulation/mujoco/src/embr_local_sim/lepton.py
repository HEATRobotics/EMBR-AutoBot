"""Live FLIR Lepton frame on a PureThermal 3, read on its own thread.

The capture thread never touches MuJoCo. The physics thread copies the newest
frame when it builds sensor observations.

Celsius uses the GroupGets PureThermal radiometry conversion for a Y16 frame
whose pixels are centi-Kelvin (T-linear):

    celsius = (raw - 27315) / 100

A raw value of 30000 is 26.85 °C. That encoding is what a radiometric Lepton
(3.1R or 3.5) puts on the PureThermal UVC stream. A non-radiometric Lepton 3.1
in 14-bit mode stays at or below 16383 and does not contain Celsius. Those
frames are still stored, with ``celsius_valid`` false and a reason string.
This module does not invent a temperature from 14-bit counts.

No blackbody comparison is in this package. FLIR's own radiometric tolerance
is about ±5 °C in high gain. Nothing here tightens that.
"""

from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Protocol

import numpy as np


VIDEO_WIDTH = 160
VIDEO_HEIGHT = 120
# Extra rows above 120 on a 160-wide frame are VoSPI telemetry, not pixels.
_TELEMETRY_HEIGHTS = {121, 122, 123, 124}
# Centi-Kelvin band covering high gain through low-gain peaks near 450 °C.
_TLINEAR_MEDIAN_MIN = 20000.0
_TLINEAR_MEDIAN_MAX = 75000.0
_RAW14_MAX = 16383
_CENTI_KELVIN_OFFSET = 27315.0
_CENTI_KELVIN_SCALE = 100.0
# GroupGets maps a Lepton command offset to a UVC selector with (offset >> 2) + 1.
# RAD unit: enable 0x10, T-linear enable 0xC0, T-linear resolution 0xC4.
RAD_ENABLE_OFFSET = 0x10
RAD_TLINEAR_ENABLE_OFFSET = 0xC0
RAD_TLINEAR_RESOLUTION_OFFSET = 0xC4


def rad_control_id(command_offset: int) -> int:
    """UVC control selector for a Lepton command offset."""
    return ((int(command_offset) >> 2) & 0xFF) + 1


def tlinear_payload(value: int) -> bytes:
    """Four-byte little-endian value used by the PureThermal RAD unit."""
    return int(value).to_bytes(4, "little", signed=False)


class FrameSource(Protocol):
    def read(self) -> np.ndarray | None: ...

    def close(self) -> None: ...


@dataclass
class LeptonFrame:
    """One normalized frame. Arrays are not written again after publish."""

    raw: np.ndarray
    celsius: np.ndarray | None
    display_rgb: np.ndarray
    stamp_wall: float
    sequence: int
    valid: bool
    celsius_valid: bool
    reason: str
    source: str
    min_c: float | None = None
    max_c: float | None = None
    mean_c: float | None = None
    telemetry_note: str = ""


def absent_frame(reason: str, source: str = "absent") -> LeptonFrame:
    return LeptonFrame(
        raw=np.zeros((0, 0), dtype=np.uint16),
        celsius=None,
        display_rgb=np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8),
        stamp_wall=0.0,
        sequence=0,
        valid=False,
        celsius_valid=False,
        reason=reason,
        source=source,
    )


def centikelvin_to_celsius(raw: np.ndarray) -> np.ndarray:
    """GroupGets ``ktoc``: (centi-Kelvin - 27315) / 100."""
    return (np.asarray(raw, dtype=np.float64) - _CENTI_KELVIN_OFFSET) / _CENTI_KELVIN_SCALE


def _as_uint16_image(frame: np.ndarray) -> np.ndarray:
    arr = np.asarray(frame)
    if arr.ndim == 3 and arr.shape[-1] == 1:
        arr = arr[..., 0]
    if arr.dtype == np.uint8 and arr.ndim == 3 and arr.shape[-1] == 2:
        # Y16 delivered as two little-endian bytes per pixel.
        arr = arr.view(np.uint16).reshape(arr.shape[0], arr.shape[1])
    if arr.ndim != 2:
        raise ValueError(
            f"Lepton frame must be a 2D Y16 image, got shape {tuple(arr.shape)} dtype {arr.dtype}."
        )
    if not np.issubdtype(arr.dtype, np.integer):
        raise ValueError(f"Lepton frame dtype {arr.dtype} is not an integer Y16 buffer.")
    if arr.dtype != np.uint16:
        if int(np.max(arr)) > 65535 or int(np.min(arr)) < 0:
            raise ValueError("Lepton frame does not fit in uint16.")
        arr = arr.astype(np.uint16, copy=False)
    return np.array(arr, dtype=np.uint16, copy=True)


def video_plane(raw: np.ndarray) -> tuple[np.ndarray, str]:
    """Return the 120×160 image and a note when telemetry rows were removed."""
    arr = _as_uint16_image(raw)
    height, width = arr.shape
    note = ""
    if width == VIDEO_WIDTH and height in _TELEMETRY_HEIGHTS:
        note = f"dropped {height - VIDEO_HEIGHT} telemetry row(s); not used as pixels"
        arr = arr[:VIDEO_HEIGHT, :]
    if arr.shape != (VIDEO_HEIGHT, VIDEO_WIDTH):
        raise ValueError(
            f"Expected a {VIDEO_WIDTH}×{VIDEO_HEIGHT} Lepton image "
            f"(or up to 4 telemetry rows). Got {arr.shape[1]}×{arr.shape[0]}."
        )
    return arr, note


def _percent_u8(values: np.ndarray) -> np.ndarray:
    flat = np.asarray(values, dtype=np.float64).reshape(-1)
    finite = flat[np.isfinite(flat)]
    if finite.size == 0:
        return np.zeros(values.shape, dtype=np.uint8)
    lo = float(np.percentile(finite, 1.0))
    hi = float(np.percentile(finite, 99.0))
    if hi <= lo:
        hi = lo + 1.0
    scaled = (np.asarray(values, dtype=np.float64) - lo) / (hi - lo)
    scaled = np.where(np.isfinite(scaled), scaled, 0.0)
    return np.clip(scaled * 255.0, 0.0, 255.0).astype(np.uint8)


def display_rgb(values: np.ndarray) -> np.ndarray:
    """Percentile stretch to a dark-blue / red picture. Display only, not °C."""
    u8 = _percent_u8(values)
    x = u8.astype(np.float32) / 255.0
    red = np.clip(1.6 * x - 0.15, 0.0, 1.0)
    green = np.clip(1.15 - np.abs(2.0 * x - 1.0), 0.0, 1.0) * 0.85
    blue = np.clip(1.15 - 1.7 * x, 0.0, 1.0)
    rgb = np.stack((red, green, blue), axis=-1)
    return (rgb * 255.0).astype(np.uint8)


def normalize_frame(
    raw: np.ndarray,
    *,
    stamp_wall: float | None = None,
    sequence: int = 0,
    source: str = "purethermal3",
) -> LeptonFrame:
    """Copy ``raw``, strip telemetry rows, and convert T-linear pixels to °C."""
    stamp = time.time() if stamp_wall is None else float(stamp_wall)
    try:
        video, note = video_plane(raw)
    except ValueError as exc:
        frame = absent_frame(str(exc), source=source)
        return replace(frame, stamp_wall=stamp, sequence=int(sequence))

    if int(video.max()) == 0 or (int(video.min()) == 65535 and int(video.max()) == 65535):
        return LeptonFrame(
            raw=video,
            celsius=None,
            display_rgb=display_rgb(video),
            stamp_wall=stamp,
            sequence=int(sequence),
            valid=False,
            celsius_valid=False,
            reason="blank frame (dropped USB packet or flat-field shutter)",
            source=source,
            telemetry_note=note,
        )

    median = float(np.median(video))
    peak = int(video.max())
    if _TLINEAR_MEDIAN_MIN <= median <= _TLINEAR_MEDIAN_MAX and peak > _RAW14_MAX:
        celsius = centikelvin_to_celsius(video).astype(np.float32)
        # A raw 0 is an empty sample. (0 - 27315) / 100 is -273.15 °C, not a scene temperature.
        celsius[video == 0] = np.nan
        usable = celsius[np.isfinite(celsius)]
        if usable.size == 0:
            return LeptonFrame(
                raw=video,
                celsius=None,
                display_rgb=display_rgb(video),
                stamp_wall=stamp,
                sequence=int(sequence),
                valid=False,
                celsius_valid=False,
                reason="Y16 frame had no nonzero pixels to convert to °C",
                source=source,
                telemetry_note=note,
            )
        return LeptonFrame(
            raw=video,
            celsius=celsius,
            display_rgb=display_rgb(celsius),
            stamp_wall=stamp,
            sequence=int(sequence),
            valid=True,
            celsius_valid=True,
            reason="",
            source=source,
            min_c=float(np.min(usable)),
            max_c=float(np.max(usable)),
            mean_c=float(np.mean(usable)),
            telemetry_note=note,
        )

    if peak <= _RAW14_MAX:
        reason = (
            "14-bit raw counts (max <= 16383). Celsius needs T-linear centi-Kelvin "
            "from a radiometric Lepton (3.1R or 3.5) on the PureThermal 3. "
            "A non-radiometric Lepton 3.1 does not carry °C in the Y16 frame, "
            "and this package does not invent a scale."
        )
    else:
        reason = (
            f"Y16 median {median:.0f} is outside the centi-Kelvin band "
            f"{_TLINEAR_MEDIAN_MIN:.0f}–{_TLINEAR_MEDIAN_MAX:.0f}, so it was not converted to °C."
        )
    return LeptonFrame(
        raw=video,
        celsius=None,
        display_rgb=display_rgb(video),
        stamp_wall=stamp,
        sequence=int(sequence),
        valid=True,
        celsius_valid=False,
        reason=reason,
        source=source,
        telemetry_note=note,
    )


def column_max_celsius(image: np.ndarray, n_bins: int) -> list[float]:
    """Max °C in each horizontal slice. Image-left is the first bin."""
    if n_bins < 1:
        raise ValueError("n_bins must be at least 1")
    arr = np.asarray(image, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 1:
        raise ValueError("column_max_celsius expects a 2D Celsius image")
    width = arr.shape[1]
    edges = np.linspace(0, width, int(n_bins) + 1)
    bins: list[float] = []
    for i in range(int(n_bins)):
        start = int(edges[i])
        stop = int(edges[i + 1])
        if stop <= start:
            stop = min(width, start + 1)
        slab = arr[:, start:stop]
        if slab.size == 0 or not np.isfinite(slab).any():
            bins.append(float("nan"))
        else:
            bins.append(float(np.nanmax(slab)))
    return bins


def _frame_is_y16(frame: np.ndarray | None) -> bool:
    if frame is None or frame.ndim != 2 or frame.dtype != np.uint16:
        return False
    height, width = frame.shape
    return width == VIDEO_WIDTH and height in {VIDEO_HEIGHT, *_TELEMETRY_HEIGHTS}


def _describe_frame(frame: np.ndarray | None) -> str:
    if frame is None:
        return "no frame"
    return f"shape {tuple(frame.shape)} dtype {frame.dtype}"


def _select_y16(cv2, cap) -> np.ndarray | None:
    """Ask the open capture for Y16. DirectShow ignores this if it is set before open."""
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"Y16 "))
    except Exception:
        pass
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT + 2)
    frame: np.ndarray | None = None
    for _ in range(8):
        ok, frame = cap.read()
        if not ok or frame is None:
            frame = None
            continue
        if not _frame_is_y16(frame):
            return frame
        if float(np.median(frame)) > 0.0:
            return frame
    return frame


def enable_purethermal_tlinear(index: int) -> str:
    """Ask a PureThermal to emit 0.01 K T-linear samples.

    Windows only, and only before OpenCV opens the camera. The DirectShow
    capture filter on this host does not expose ``IKsControl``, so the same
    camera index is opened by its device path, the three RAD controls are
    written with ``KSP_NODE``, read back, and the handle is closed. A refused
    control is reported and does not invent a Celsius scale. Other platforms
    leave the camera as it is.
    """
    if sys.platform != "win32":
        return "Lepton T-linear: not requested (this host is not Windows)"
    try:
        values = _windows_set_tlinear(int(index))
    except Exception as exc:
        return (
            "Lepton T-linear: control failed "
            f"({exc}). The Y16 stream continues. Celsius is not substituted."
        )
    return (
        "Lepton T-linear: "
        f"radiometry={values['radiometry']} "
        f"resolution={values['resolution']} "
        f"tlinear={values['tlinear']} "
        f"(node {values['node']})"
    )


def _windows_set_tlinear(index: int) -> dict[str, int]:
    import ctypes
    from ctypes import (
        POINTER,
        Structure,
        byref,
        c_long,
        c_ubyte,
        c_ulong,
        c_ushort,
        c_void_p,
        cast,
        windll,
    )

    ole32 = windll.ole32
    CLSCTX_INPROC_SERVER = 1
    COINIT_APARTMENTTHREADED = 0x2
    RPC_E_CHANGED_MODE = 0x80010106

    class GUID(Structure):
        _fields_ = (
            ("Data1", c_ulong),
            ("Data2", c_ushort),
            ("Data3", c_ushort),
            ("Data4", c_ubyte * 8),
        )

    def guid(data1: int, data2: int, data3: int, data4: bytes) -> GUID:
        item = GUID()
        item.Data1 = data1
        item.Data2 = data2
        item.Data3 = data3
        item.Data4[:] = data4
        return item

    clsid_dev_enum = guid(0x62BE5D10, 0x60EB, 0x11D0, bytes.fromhex("BD3B00A0C911CE86"))
    clsid_video_input = guid(0x860BB310, 0x5D01, 0x11D0, bytes.fromhex("BD3B00A0C911CE86"))
    iid_create_dev_enum = guid(0x29840822, 0x5B84, 0x11D0, bytes.fromhex("BD3B00A0C911CE86"))
    topology_guid = guid(0x720D4AC0, 0x7533, 0x11D0, bytes.fromhex("A5D628DB04C10000"))
    node_dev_specific = guid(0x941C7AC0, 0xC559, 0x11D0, bytes.fromhex("8A2B00A0C9255AC1"))
    # ASCII "pt1-lep-rad-0000", GUID display order.
    rad_guid = guid(0x2D317470, 0x656C, 0x2D70, bytes.fromhex("7261642D30303030"))

    def method(ptr, slot, *argtypes):
        table = cast(ptr, POINTER(POINTER(c_void_p)))
        return ctypes.WINFUNCTYPE(c_long, c_void_p, *argtypes)(table[0][slot])

    def release(ptr) -> None:
        if ptr:
            method(ptr, 2)(ptr)

    def check(hr: int, what: str) -> None:
        if hr < 0:
            raise OSError(f"{what} failed hr=0x{hr & 0xFFFFFFFF:08X}")

    started = ole32.CoInitializeEx(None, COINIT_APARTMENTTHREADED)
    if started < 0 and (started & 0xFFFFFFFF) != RPC_E_CHANGED_MODE:
        check(started, "CoInitializeEx")
    owned_apartment = started == 0
    enumerator = c_void_p()
    devices = c_void_p()
    bind = c_void_p()
    moniker = c_void_p()
    try:
        check(
            ole32.CoCreateInstance(
                byref(clsid_dev_enum),
                None,
                CLSCTX_INPROC_SERVER,
                byref(iid_create_dev_enum),
                byref(enumerator),
            ),
            "CoCreateInstance",
        )
        check(
            method(enumerator, 3, POINTER(GUID), POINTER(c_void_p), c_ulong)(
                enumerator, byref(clsid_video_input), byref(devices), 0
            ),
            "CreateClassEnumerator",
        )
        seen = -1
        while True:
            fetched = c_ulong()
            nxt = c_void_p()
            hr = method(devices, 3, c_ulong, POINTER(c_void_p), POINTER(c_ulong))(
                devices, 1, byref(nxt), byref(fetched)
            )
            if hr != 0 or fetched.value == 0:
                break
            seen += 1
            if seen == index:
                moniker = nxt
                break
            release(nxt)
        if not moniker:
            raise OSError(f"no DirectShow camera at index {index}")
        check(ole32.CreateBindCtx(0, byref(bind)), "CreateBindCtx")
        name_ptr = c_void_p()
        check(
            method(moniker, 20, c_void_p, c_void_p, POINTER(c_void_p))(
                moniker, bind, None, byref(name_ptr)
            ),
            "GetDisplayName",
        )
        display = ctypes.wstring_at(name_ptr)
        ole32.CoTaskMemFree(name_ptr)
    finally:
        release(moniker)
        release(bind)
        release(devices)
        release(enumerator)
        if owned_apartment:
            ole32.CoUninitialize()

    marker = "\\\\?\\"
    start = display.find(marker)
    if start < 0:
        raise OSError(f"camera index {index} has no device path")
    path = display[start:]
    return _ks_set_tlinear(path, rad_guid, topology_guid, node_dev_specific)


def _ks_set_tlinear(path: str, rad_guid, topology_guid, node_dev_specific) -> dict[str, int]:
    import ctypes
    from ctypes import Structure, byref, c_ubyte, c_ulong, c_ushort, c_void_p, windll

    kernel32 = windll.kernel32
    kernel32.CreateFileW.restype = c_void_p
    kernel32.DeviceIoControl.restype = ctypes.c_int
    generic_read = 0x80000000
    generic_write = 0x40000000
    handle = kernel32.CreateFileW(
        path,
        generic_read | generic_write,
        0x3,
        None,
        3,
        0,
        None,
    )
    invalid = c_void_p(-1).value
    if not handle or handle == invalid:
        raise OSError(f"opening the camera device failed ({kernel32.GetLastError()})")

    class GUID(Structure):
        _fields_ = (
            ("Data1", c_ulong),
            ("Data2", c_ushort),
            ("Data3", c_ushort),
            ("Data4", c_ubyte * 8),
        )

    class KSPROPERTY(Structure):
        _fields_ = (("Set", GUID), ("Id", c_ulong), ("Flags", c_ulong))

    class KSP_NODE(Structure):
        _fields_ = (("Property", KSPROPERTY), ("NodeId", c_ulong), ("Reserved", c_ulong))

    def adopt(src) -> GUID:
        item = GUID()
        item.Data1 = src.Data1
        item.Data2 = src.Data2
        item.Data3 = src.Data3
        item.Data4[:] = src.Data4
        return item

    rad = adopt(rad_guid)
    topology = adopt(topology_guid)
    dev_specific = adopt(node_dev_specific)
    ioctl_ks = 0x002F0003
    get_flag = 0x00000001
    set_flag = 0x00000002
    topology_flag = 0x10000000

    def ioctl(header, out_size: int):
        out = (c_ubyte * out_size)()
        returned = c_ulong()
        ok = kernel32.DeviceIoControl(
            c_void_p(handle),
            ioctl_ks,
            byref(header),
            ctypes.sizeof(header),
            out,
            out_size,
            byref(returned),
            None,
        )
        if not ok:
            return int(kernel32.GetLastError()), b""
        return 0, bytes(out[: int(returned.value)])

    try:
        nodes = KSPROPERTY()
        nodes.Set = topology
        nodes.Id = 2
        nodes.Flags = get_flag
        err, raw = ioctl(nodes, 65536)
        if err or len(raw) < 8:
            raise OSError(f"reading KS topology failed ({err})")
        count = int.from_bytes(raw[4:8], "little")
        if count > 64:
            raise OSError(f"KS topology node count {count} is not usable")
        chosen: int | None = None
        for node_id in range(count):
            offset = 8 + node_id * 16
            chunk = raw[offset : offset + 16]
            if len(chunk) < 16:
                break
            if (
                int.from_bytes(chunk[0:4], "little") != dev_specific.Data1
                or int.from_bytes(chunk[4:6], "little") != dev_specific.Data2
                or int.from_bytes(chunk[6:8], "little") != dev_specific.Data3
            ):
                continue
            probe = KSP_NODE()
            probe.Property.Set = rad
            probe.Property.Id = rad_control_id(RAD_ENABLE_OFFSET)
            probe.Property.Flags = get_flag | topology_flag
            probe.NodeId = node_id
            err, _value = ioctl(probe, 4)
            if err == 0:
                chosen = node_id
                break
        if chosen is None:
            raise OSError("RAD extension unit did not answer radiometry enable")

        def put(control: int, value: int) -> int:
            header = KSP_NODE()
            header.Property.Set = rad
            header.Property.Id = control
            header.Property.Flags = set_flag | topology_flag
            header.NodeId = chosen
            payload = (c_ubyte * 4).from_buffer_copy(tlinear_payload(value))
            returned = c_ulong()
            ok = kernel32.DeviceIoControl(
                c_void_p(handle),
                ioctl_ks,
                byref(header),
                ctypes.sizeof(header),
                payload,
                4,
                byref(returned),
                None,
            )
            if not ok:
                raise OSError(f"SET control {control} failed ({kernel32.GetLastError()})")
            header.Property.Flags = get_flag | topology_flag
            err, raw_value = ioctl(header, 4)
            if err or len(raw_value) < 4:
                raise OSError(f"GET control {control} failed ({err})")
            return int.from_bytes(raw_value[:4], "little")

        return {
            "node": chosen,
            "radiometry": put(rad_control_id(RAD_ENABLE_OFFSET), 1),
            "resolution": put(rad_control_id(RAD_TLINEAR_RESOLUTION_OFFSET), 1),
            "tlinear": put(rad_control_id(RAD_TLINEAR_ENABLE_OFFSET), 1),
        }
    finally:
        kernel32.CloseHandle(c_void_p(handle))


class OpenCvY16:
    """UVC Y16 reader. Requires the optional ``opencv-python`` extra.

    On Windows the PureThermal shows up as a DirectShow camera. Opening it
    and then leaving the default converter on returns a ``(120, 160, 3)``
    ``uint8`` BGR image. Those are not Lepton samples. Y16 has to be selected
    after ``VideoCapture`` has opened the device. The frame is then
    ``uint16``, 160 wide and 120 or 122 tall.
    """

    def __init__(self, index: int) -> None:
        import cv2

        self._cv2 = cv2
        self.tlinear_report = enable_purethermal_tlinear(int(index))
        print(self.tlinear_report, flush=True)
        backends: list[tuple[int, str]] = []
        if sys.platform == "win32":
            backends.append((cv2.CAP_DSHOW, "DirectShow"))
            if hasattr(cv2, "CAP_MSMF"):
                backends.append((cv2.CAP_MSMF, "MediaFoundation"))
        else:
            backends.append((cv2.CAP_ANY, "default"))
        last = "no frame"
        for backend, backend_name in backends:
            cap = cv2.VideoCapture(int(index), backend)
            if not cap.isOpened():
                cap.release()
                last = f"{backend_name} did not open index {index}"
                continue
            frame = _select_y16(cv2, cap)
            if _frame_is_y16(frame):
                video = np.asarray(frame[:VIDEO_HEIGHT])
                usable = video[video > 0]
                median = float(np.median(usable)) if usable.size else float(np.median(video))
                print(
                    f"Lepton Y16 median {median:.0f} -> "
                    f"{(median - _CENTI_KELVIN_OFFSET) / _CENTI_KELVIN_SCALE:.1f} °C",
                    flush=True,
                )
                self._cap = cap
                self._primed: np.ndarray | None = frame
                return
            last = f"{backend_name} returned {_describe_frame(frame)}"
            cap.release()
        raise RuntimeError(
            f"PureThermal at index {index} did not return a Y16 uint16 frame ({last}). "
            "An 8-bit color image is the Windows converter, not the Lepton samples, "
            "and it is not turned into Celsius. "
            "Close any other program using the camera and try another --lepton-index. "
            "Index 0 on this machine is often the laptop webcam, not the PureThermal."
        )

    def read(self) -> np.ndarray | None:
        if self._primed is not None:
            frame = self._primed
            self._primed = None
            return frame
        ok, frame = self._cap.read()
        if not ok or frame is None or not _frame_is_y16(frame):
            return None
        return frame

    def close(self) -> None:
        self._cap.release()


def purethermal_opener(index: int = 0) -> Callable[[], FrameSource]:
    """Build an opener for ``LeptonCapture``. Import of OpenCV is deferred."""

    def open_device() -> FrameSource:
        try:
            import cv2  # noqa: F401
        except ImportError as exc:
            raise RuntimeError(
                "opencv is not installed, so the PureThermal 3 cannot be opened. "
                "From embr_autonomy/simulation/mujoco: "
                "python -m pip install -e \".[lepton]\". "
                "Launch without --lepton does not need it."
            ) from exc
        return OpenCvY16(index)

    return open_device


class LeptonCapture:
    """Background reader. ``latest`` is safe to call from the physics thread."""

    def __init__(
        self,
        opener: Callable[[], FrameSource] | None = None,
        *,
        stale_s: float = 0.50,
        retry_s: float = 2.0,
        source_name: str = "purethermal3",
    ) -> None:
        self._opener = opener
        self.stale_s = float(stale_s)
        self.retry_s = float(retry_s)
        self.source_name = source_name
        self._lock = threading.Lock()
        self._latest = absent_frame("camera not attached", source="absent")
        self._sequence = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def submit(self, raw: np.ndarray, *, source: str | None = None) -> LeptonFrame:
        """Normalize and store a frame. Tests call this without a USB device."""
        with self._lock:
            self._sequence += 1
            sequence = self._sequence
        frame = normalize_frame(
            raw,
            sequence=sequence,
            source=self.source_name if source is None else source,
        )
        with self._lock:
            self._latest = frame
        return frame

    def latest(self) -> LeptonFrame:
        with self._lock:
            frame = self._latest
        if frame.sequence == 0 or not frame.valid:
            return frame
        age = time.time() - frame.stamp_wall
        if age > self.stale_s:
            return replace(
                frame,
                valid=False,
                celsius_valid=False,
                reason=f"stale: last frame is {age:.2f} s old (limit {self.stale_s:.2f} s)",
            )
        return frame

    def start(self) -> None:
        if self._opener is None:
            return
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="embr-lepton", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._thread = None

    def _remember(self, frame: LeptonFrame) -> None:
        with self._lock:
            self._latest = frame

    def _loop(self) -> None:
        source: FrameSource | None = None
        failures = 0
        while not self._stop.is_set():
            if source is None:
                assert self._opener is not None
                try:
                    source = self._opener()
                    failures = 0
                except Exception as exc:
                    self._remember(absent_frame(str(exc), source="absent"))
                    if self._stop.wait(self.retry_s):
                        break
                    continue
            try:
                raw = source.read()
            except Exception as exc:
                self._remember(absent_frame(str(exc), source=self.source_name))
                _close_quiet(source)
                source = None
                if self._stop.wait(self.retry_s):
                    break
                continue
            if raw is None:
                failures += 1
                if failures >= 8:
                    self._remember(
                        absent_frame(
                            "camera opened but returned no frame; will retry",
                            source=self.source_name,
                        )
                    )
                    _close_quiet(source)
                    source = None
                    failures = 0
                    if self._stop.wait(self.retry_s):
                        break
                else:
                    self._stop.wait(0.05)
                continue
            failures = 0
            self.submit(raw)
        if source is not None:
            _close_quiet(source)


def _close_quiet(source: FrameSource) -> None:
    try:
        source.close()
    except Exception:
        pass
