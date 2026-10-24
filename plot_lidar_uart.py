#!/usr/bin/env python3
"""Live XY plot for an M1C1 LiDAR connected to Raspberry Pi UART.

The LiDAR stream is binary, not line-oriented. This script only depends on the
standalone M1C1 protocol parser in this project and keeps serial I/O off the
Matplotlib event loop so the plot remains responsive.

Example:
    python3 tools/plot_lidar_uart.py --port /dev/serial0
"""

from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path
import queue
import sys
import threading

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


# Allow the script to run directly from a source checkout on the Pi.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from lidar_localization.protocol import LidarFrame, M1C1Parser  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot M1C1 LiDAR data from UART")
    parser.add_argument("--port", default="/dev/serial0", help="UART device")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--min-mm", type=int, default=100, help="Ignore shorter returns")
    parser.add_argument("--max-mm", type=int, default=8000, help="Ignore longer returns")
    parser.add_argument("--max-points", type=int, default=12000,
                        help="Maximum displayed points")
    parser.add_argument("--interval-ms", type=int, default=50,
                        help="Plot refresh interval")
    return parser.parse_args()


def frame_to_xy(frame: LidarFrame, minimum_mm: int, maximum_mm: int) -> np.ndarray:
    """Convert one validated polar frame into an N x 2 XY array in millimetres."""

    points = [point for point in frame.points
              if minimum_mm <= point.distance_mm <= maximum_mm]
    if not points:
        return np.empty((0, 2), dtype=np.float64)
    angle = np.radians([point.angle_deg for point in points])
    distance = np.asarray([point.distance_mm for point in points], dtype=np.float64)
    return np.column_stack((distance * np.sin(angle), distance * np.cos(angle)))


class UartReader(threading.Thread):
    def __init__(self, args: argparse.Namespace, frames: queue.Queue[LidarFrame],
                 status: queue.Queue[str], stop: threading.Event) -> None:
        super().__init__(daemon=True)
        self.args = args
        self.frames = frames
        self.status = status
        self.stop = stop

    def set_status(self, text: str) -> None:
        try:
            self.status.put_nowait(text)
        except queue.Full:
            pass

    def run(self) -> None:
        try:
            import serial
        except ImportError:
            self.set_status("pyserial is not installed")
            return

        parser = M1C1Parser()
        try:
            with serial.Serial(self.args.port, self.args.baud, timeout=0.1) as connection:
                self.set_status(f"Connected to {self.args.port} at {self.args.baud} baud")
                while not self.stop.is_set():
                    data = connection.read(1024)
                    if not data:
                        continue
                    for frame in parser.feed(data):
                        try:
                            self.frames.put_nowait(frame)
                        except queue.Full:
                            # The plot is slower than the UART; newest data wins.
                            try:
                                self.frames.get_nowait()
                            except queue.Empty:
                                pass
                            self.frames.put_nowait(frame)
                self.set_status("Stopped")
        except (OSError, serial.SerialException) as exc:
            self.set_status(f"UART error: {exc}")
        finally:
            self.stop.set()


def main() -> int:
    args = parse_args()
    if args.min_mm < 0 or args.max_mm <= args.min_mm:
        raise SystemExit("--max-mm must be greater than --min-mm")
    if args.max_points <= 0 or args.interval_ms <= 0:
        raise SystemExit("--max-points and --interval-ms must be positive")

    frames: queue.Queue[LidarFrame] = queue.Queue(maxsize=64)
    statuses: queue.Queue[str] = queue.Queue(maxsize=8)
    stop = threading.Event()
    reader = UartReader(args, frames, statuses, stop)
    reader.start()

    points = deque(maxlen=args.max_points)
    figure, axis = plt.subplots(figsize=(8, 8))
    scatter = axis.scatter([], [], s=5, c="#26c6da", alpha=0.85)
    axis.set_title("M1C1 LiDAR — live UART scan")
    axis.set_xlabel("X (mm)")
    axis.set_ylabel("Y (mm)")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlim(-args.max_mm, args.max_mm)
    axis.set_ylim(-args.max_mm, args.max_mm)
    axis.grid(True, alpha=0.3)
    status_text = figure.text(0.02, 0.02, "Connecting…", fontsize=9)

    def update(_frame_number: int):
        latest = None
        while True:
            try:
                latest = frames.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            # A frame with bit 0 set starts a fresh scan. Otherwise, retain the
            # current scan until the next scan-start marker arrives.
            if latest.start_of_scan:
                points.clear()
            xy = frame_to_xy(latest, args.min_mm, args.max_mm)
            points.extend(map(tuple, xy))
            if points:
                values = np.asarray(points, dtype=np.float64)
                scatter.set_offsets(values)

        status = None
        while True:
            try:
                status = statuses.get_nowait()
            except queue.Empty:
                break
        if status:
            status_text.set_text(status)
        return scatter, status_text

    def close(_event) -> None:
        stop.set()

    figure.canvas.mpl_connect("close_event", close)
    animation = FuncAnimation(figure, update, interval=args.interval_ms,
                              blit=False, cache_frame_data=False)
    # Keep a strong reference for Matplotlib backends that otherwise collect it.
    figure._lidar_animation = animation  # type: ignore[attr-defined]
    try:
        plt.show()
    finally:
        stop.set()
        reader.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
