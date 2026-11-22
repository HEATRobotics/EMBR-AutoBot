"""Open the hotspot rooms for a teammate to watch locally.

Room 1 is a clear floor and one hot spot. Room 2 moves the spot and
puts two boxes on the straight line. Room 3 hides the spot behind
two rows of trunks, with two openings in each row. All three use the
CAD body. The histogram drives. This file only starts the existing
viewer. The rooms, the body, and the histogram stay where they already live.
"""

from __future__ import annotations

import subprocess
import sys
import time

# Scene names the viewer already knows. Ports stay off the thermal lab (43150).
ROOM_1 = "hot-room"
ROOM_2 = "hot-room-2"
ROOM_3 = "heat-track"
PORT_1 = 43147
PORT_2 = 43148
PORT_3 = 43149
HOST = "127.0.0.1"


def room_url(port: int) -> str:
    return f"http://{HOST}:{port}"


def launch_command(scene: str, port: int) -> list[str]:
    """One viewer. Histogram on, so the mission builds the chart itself."""
    return [
        sys.executable,
        "-m",
        "embr_local_sim",
        "launch",
        "--scene",
        scene,
        "--mission",
        "--histogram",
        "--host",
        HOST,
        "--port",
        str(port),
    ]


def _show_in_chrome(urls: list[str]) -> None:
    if sys.platform != "darwin":
        return
    try:
        subprocess.Popen(
            ["open", "-a", "Google Chrome", *urls],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError as exc:
        print(f"Chrome did not open ({exc}). Use the URLs above.")


def main() -> int:
    """Start the three rooms and wait. Ctrl-C stops them."""
    rooms = ((ROOM_1, PORT_1), (ROOM_2, PORT_2), (ROOM_3, PORT_3))
    processes: list[subprocess.Popen[bytes]] = []
    try:
        for scene, port in rooms:
            print(f"{scene}  {room_url(port)}")
            processes.append(subprocess.Popen(launch_command(scene, port)))
        time.sleep(2.0)
        _show_in_chrome([room_url(port) for _scene, port in rooms])
        print("Status line says vfh while the histogram drives. Ctrl-C stops all three.")
        while True:
            for proc in processes:
                code = proc.poll()
                if code is not None:
                    print(f"A room exited ({code}). Stopping the others.", file=sys.stderr)
                    return code if code else 1
            time.sleep(0.4)
    except KeyboardInterrupt:
        return 0
    finally:
        for proc in processes:
            if proc.poll() is None:
                proc.terminate()
        for proc in processes:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
