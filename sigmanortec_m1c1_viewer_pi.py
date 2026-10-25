#!/usr/bin/env python3
"""Sigmanortec M1C1 Viewer adapted for Raspberry Pi GPIO UART.

This is a separate 2D viewer based on the M1C1 viewer supplied on the
Sigmanortec M1C1 product page. It reads directly from the Pi hardware UART:

    LiDAR TX  -> GPIO15 / physical pin 10 (Pi RXD)
    LiDAR GND -> Pi GND
    LiDAR VCC -> separate regulated 5 V supply

Run:
    python3 sigmanortec_m1c1_viewer_pi.py
    python3 sigmanortec_m1c1_viewer_pi.py /dev/serial0
"""

from __future__ import annotations

import argparse
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import serial


HEADER = b"\xAA\x55"
MAX_SAMPLES = 60
BAUD_CANDIDATES = (115200, 150000)
MIN_DISTANCE_MM = 100
MAX_DISTANCE_MM = 8000
ANGLE_OFFSET_DEG = 12.0
ANGLE_CORRECTION_K1 = 19.16
ANGLE_CORRECTION_K2 = 90.15
SEND_START_CMD = False
START_CMD = bytes((0xAA, 0x55, 0xF0, 0x0F))
STOP_CMD = bytes((0xAA, 0x55, 0xF5, 0x0A))


def u16(data: bytes, offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def angle_correction(distance_mm: int) -> float:
    if distance_mm <= 0:
        return 0.0
    return math.atan(ANGLE_CORRECTION_K1 * (distance_mm - ANGLE_CORRECTION_K2) /
                     (ANGLE_CORRECTION_K2 * distance_mm))


def checksum_ok(packet: bytes) -> bool:
    if len(packet) < 10:
        return False
    c0 = c1 = 0
    for offset in range(0, len(packet), 2):
        if offset == 8:
            continue
        c0 ^= packet[offset]
        c1 ^= packet[offset + 1]
    return c0 == packet[8] and c1 == packet[9]


def decode_packet(packet: bytes) -> tuple[int, list[tuple[float, float]]] | None:
    """Return frame type and corrected polar points from one valid packet."""
    if len(packet) < 10 or packet[:2] != HEADER:
        return None
    frame_type = packet[2]
    count = packet[3]
    if frame_type not in (0, 1) or not 1 <= count <= MAX_SAMPLES:
        return None
    if len(packet) != 10 + count * 2 or not checksum_ok(packet):
        return None

    start = (u16(packet, 4) >> 1) / 64.0
    end = (u16(packet, 6) >> 1) / 64.0
    span = (end - start) % 360.0
    step = span / (count - 1) if count > 1 else 0.0
    points = []
    for index in range(count):
        distance = u16(packet, 10 + index * 2) >> 2
        if not MIN_DISTANCE_MM <= distance <= MAX_DISTANCE_MM:
            continue
        raw_angle = start + index * step
        corrected = math.radians((raw_angle - math.degrees(angle_correction(distance)) +
                                  ANGLE_OFFSET_DEG) % 360.0)
        points.append((corrected, float(distance)))
    return frame_type, points


class M1C1Parser:
    """Resynchronize on AA55 and yield only complete, checksum-valid packets."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.good_packets = 0
        self.bad_packets = 0

    def feed(self, data: bytes) -> list[tuple[int, list[tuple[float, float]]]]:
        self.buffer.extend(data)
        decoded = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                self.buffer[:] = HEADER[:1] if self.buffer[-1:] == HEADER[:1] else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 4:
                break
            count = self.buffer[3]
            if count < 1 or count > MAX_SAMPLES:
                del self.buffer[0]
                self.bad_packets += 1
                continue
            size = 10 + count * 2
            if len(self.buffer) < size:
                break
            packet = bytes(self.buffer[:size])
            item = decode_packet(packet)
            if item is None:
                del self.buffer[0]
                self.bad_packets += 1
                continue
            del self.buffer[:size]
            self.good_packets += 1
            decoded.append(item)
        return decoded


class PiLidarReader(threading.Thread):
    def __init__(self, device: str, events: queue.Queue, stop_event: threading.Event) -> None:
        super().__init__(daemon=True)
        self.device = device
        self.events = events
        self.stop_event = stop_event
        self.connection = None

    def emit(self, event) -> None:
        try:
            self.events.put_nowait(event)
        except queue.Full:
            pass

    def run(self) -> None:
        parser = M1C1Parser()
        baud_index = 0
        try:
            self.connection = serial.Serial(
                self.device, BAUD_CANDIDATES[baud_index],
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.05,
            )
            if SEND_START_CMD:
                self.connection.write(START_CMD)
            self.emit(("connected", self.device, BAUD_CANDIDATES[baud_index]))
            accumulated = []
            last_valid = time.monotonic()
            last_rate_time = time.monotonic()
            while not self.stop_event.is_set():
                data = self.connection.read(4096)
                now = time.monotonic()
                for frame_type, points in parser.feed(data):
                    if frame_type & 0x01 and accumulated:
                        elapsed = max(now - last_rate_time, 1e-3)
                        points_per_second = int(len(accumulated) / elapsed)
                        last_rate_time = now
                        self.emit(("scan", accumulated, parser.good_packets,
                                   parser.bad_packets, points_per_second, now))
                        accumulated = []
                    accumulated.extend(points)
                    last_valid = now

                if (parser.good_packets == 0 and baud_index + 1 < len(BAUD_CANDIDATES)
                        and now - last_valid > 2.0):
                    baud_index += 1
                    self.connection.baudrate = BAUD_CANDIDATES[baud_index]
                    parser = M1C1Parser()
                    last_valid = now
                    self.emit(("baud", BAUD_CANDIDATES[baud_index]))

            if self.connection and self.connection.is_open and SEND_START_CMD:
                self.connection.write(STOP_CMD)
            self.emit(("stopped",))
        except (serial.SerialException, OSError) as exc:
            self.emit(("error", str(exc)))
        finally:
            if self.connection and self.connection.is_open:
                self.connection.close()
            self.connection = None


class LidarViewer:
    def __init__(self, root: tk.Tk, device: str) -> None:
        self.root = root
        self.root.title("Sigmanortec M1C1 — Raspberry Pi GPIO UART")
        self.root.geometry("950x760")
        self.events: queue.Queue = queue.Queue(maxsize=32)
        self.stop_event = threading.Event()
        self.reader = None
        self.current_points: list[tuple[float, float]] = []
        self.latest_scan: list[tuple[float, float]] = []
        self.packet_good = self.packet_bad = 0
        self.points_per_second = 0
        self.last_status = "Disconnected"

        controls = ttk.Frame(root, padding=8)
        controls.pack(fill="x")
        ttk.Label(controls, text="UART device").pack(side="left")
        self.device_var = tk.StringVar(value=device)
        ttk.Entry(controls, textvariable=self.device_var, width=20).pack(side="left", padx=6)
        self.connect_button = ttk.Button(controls, text="Connect", command=self.toggle)
        self.connect_button.pack(side="left", padx=4)
        ttk.Button(controls, text="Clear", command=self.clear).pack(side="left", padx=4)
        ttk.Button(controls, text="Save CSV", command=self.save_csv).pack(side="left", padx=4)
        self.status_var = tk.StringVar(value=self.last_status)
        ttk.Label(controls, textvariable=self.status_var).pack(side="right")

        self.figure = Figure(figsize=(8, 7), dpi=100)
        self.axis = self.figure.add_subplot(111)
        self.axis.set_aspect("equal", adjustable="box")
        self.axis.set_xlim(-MAX_DISTANCE_MM, MAX_DISTANCE_MM)
        self.axis.set_ylim(-MAX_DISTANCE_MM, MAX_DISTANCE_MM)
        self.axis.set_xlabel("X (mm)")
        self.axis.set_ylabel("Y (mm)")
        self.axis.set_title("M1C1 2D scan")
        self.axis.grid(alpha=0.3)
        self.scatter = self.axis.scatter([], [], s=5, c="#177ddc")
        self.info = self.axis.text(0.02, 0.98, "", transform=self.axis.transAxes,
                                   va="top", family="monospace")
        self.canvas = FigureCanvasTkAgg(self.figure, master=root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(50, self.process_events)
        self.root.after(250, self.toggle)

    def toggle(self) -> None:
        if self.reader and self.reader.is_alive():
            self.stop_event.set()
            self.connect_button.configure(state="disabled")
            return
        self.stop_event.clear()
        device = self.device_var.get().strip() or "/dev/serial0"
        self.device_var.set(device)
        self.connect_button.configure(state="disabled")
        self.status_var.set(f"Opening {device}…")
        self.reader = PiLidarReader(device, self.events, self.stop_event)
        self.reader.start()

    def process_events(self) -> None:
        try:
            while True:
                event = self.events.get_nowait()
                if event[0] == "connected":
                    self.status_var.set(f"Connected: {event[1]} @ {event[2]} baud")
                    self.connect_button.configure(text="Disconnect", state="normal")
                elif event[0] == "baud":
                    self.status_var.set(f"Trying {event[1]} baud…")
                elif event[0] == "scan":
                    self.latest_scan = event[1]
                    self.packet_good, self.packet_bad = event[2], event[3]
                    self.points_per_second = event[4]
                    self.redraw()
                elif event[0] == "stopped":
                    self.connect_button.configure(text="Connect", state="normal")
                    self.status_var.set("Disconnected")
                    self.stop_event.clear()
                elif event[0] == "error":
                    self.connect_button.configure(text="Connect", state="normal")
                    self.status_var.set("Connection failed")
                    self.stop_event.clear()
                    messagebox.showerror("LiDAR UART error", event[1])
        except queue.Empty:
            pass
        self.root.after(50, self.process_events)

    def redraw(self) -> None:
        xy = [
            (distance * math.sin(angle), distance * math.cos(angle))
            for angle, distance in self.latest_scan
        ]
        self.scatter.set_offsets(xy)
        self.info.set_text(
            f"Baud         {self.reader.connection.baudrate if self.reader and self.reader.connection else '—'}\n"
            f"Points/s     {self.points_per_second:,}\n"
            f"Points       {len(xy):,}\n"
            f"Packets OK   {self.packet_good:,}\n"
            f"Packets BAD  {self.packet_bad:,}"
        )
        self.canvas.draw_idle()

    def clear(self) -> None:
        self.latest_scan = []
        self.scatter.set_offsets([])
        self.canvas.draw_idle()

    def save_csv(self) -> None:
        if not self.latest_scan:
            messagebox.showinfo("No scan", "No complete scan is available yet.")
            return
        path = filedialog.asksaveasfilename(
            title="Save M1C1 scan", defaultextension=".csv",
            initialfile="m1c1_scan.csv", filetypes=(("CSV files", "*.csv"),),
        )
        if path:
            with open(path, "w", encoding="utf-8") as output:
                output.write("x_mm,y_mm\n")
                for angle, distance in self.latest_scan:
                    output.write(f"{distance * math.sin(angle):.3f},{distance * math.cos(angle):.3f}\n")

    def close(self) -> None:
        self.stop_event.set()
        if self.reader and self.reader.connection and self.reader.connection.is_open:
            self.reader.connection.close()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="Sigmanortec M1C1 viewer on Pi GPIO UART")
    parser.add_argument("device", nargs="?", default="/dev/serial0")
    args = parser.parse_args()
    root = tk.Tk()
    LidarViewer(root, args.device)
    root.mainloop()


if __name__ == "__main__":
    main()
