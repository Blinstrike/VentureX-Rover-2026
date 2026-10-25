# -*- coding: utf-8 -*-
"""M1C1 Mini LiDAR 3D viewer for Raspberry Pi GPIO UART.

This is a separate Pi version of the M1C1 viewer. It reads the LiDAR directly
from the Pi UART device (``/dev/serial0`` by default), which maps to GPIO15 /
physical pin 10 on the standard 40-pin header.

Install on the Pi:
    python3 -m pip install pyserial matplotlib numpy

Run:
    python3 m1c1_viewer_pi.py
    python3 m1c1_viewer_pi.py /dev/serial0
"""

from __future__ import annotations

import math
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


BAUD_CANDIDATES = (115200, 150000)
ANGLE_OFFSET_DEG = 192.0
MIN_RANGE_MM, MAX_RANGE_MM = 80, 8000
MAX_LSN = 100


def u16(data, index):
    return data[index] | data[index + 1] << 8


def angle_correction(distance_mm):
    if not distance_mm:
        return 0.0
    return math.degrees(19.16 * (distance_mm - 90.15) /
                        (90.15 * distance_mm))


def xor_ok(packet):
    count = packet[3]
    checksum = 0x55AA ^ (packet[2] | count << 8) ^ u16(packet, 4) ^ u16(packet, 6)
    for index in range(count):
        checksum ^= u16(packet, 10 + 2 * index)
    return checksum == u16(packet, 8)


def decode_scan(packet):
    count = packet[3]
    first = (u16(packet, 4) >> 1) / 64.0
    last = (u16(packet, 6) >> 1) / 64.0
    span = (last - first) % 360.0
    points = []
    for index in range(count):
        distance = u16(packet, 10 + 2 * index) >> 2
        raw_angle = first + (span * index / (count - 1) if count > 1 else 0.0)
        angle = math.radians((raw_angle - angle_correction(distance) + ANGLE_OFFSET_DEG) % 360.0)
        if MIN_RANGE_MM <= distance <= MAX_RANGE_MM:
            points.append((distance * math.sin(angle), distance * math.cos(angle)))
    return points


class M1C1Parser:
    """Resynchronizing parser for the M1C1 AA 55 UART stream."""

    def __init__(self):
        self.buffer = bytearray()
        self.good = self.bad = 0

    def feed(self, data):
        self.buffer.extend(data)
        packets = []
        while True:
            start = self.buffer.find(b"\xAA\x55")
            if start < 0:
                self.buffer[:] = b"\xAA" if self.buffer[-1:] == b"\xAA" else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 10:
                break
            count = self.buffer[3]
            if count == 0 or count > MAX_LSN:
                del self.buffer[0]
                self.bad += 1
                continue
            size = 10 + count * 2
            if len(self.buffer) < size:
                break
            packet = bytes(self.buffer[:size])
            if not xor_ok(packet):
                del self.buffer[0]
                self.bad += 1
                continue
            del self.buffer[:size]
            self.good += 1
            packets.append((packet[2], decode_scan(packet)))
        return packets


class LidarMapper:
    def __init__(self, root, requested_port="/dev/serial0"):
        self.root = root
        self.root.title("M1C1 Mini LiDAR — Raspberry Pi GPIO UART")
        self.root.geometry("1100x760")
        self.events = queue.Queue()
        self.stop_event = threading.Event()
        self.serial_port = None
        self.worker = None
        self.map_points = []
        self.latest_scan = np.empty((0, 3))
        self.sweep_start_time = None
        self.sweep_rate_deg_s = 10.0
        self.packet_good = self.packet_bad = 0
        self.last_scan_time = None
        self.scan_hz = 0.0
        self.max_map_points = 100000

        toolbar = ttk.Frame(root, padding=8)
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="Pi UART device").pack(side="left")
        self.port_var = tk.StringVar(value=requested_port or "/dev/serial0")
        ttk.Entry(toolbar, textvariable=self.port_var, width=18).pack(side="left", padx=(6, 4))
        self.connect_button = ttk.Button(toolbar, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=8)
        ttk.Button(toolbar, text="Clear map", command=self.clear_map).pack(side="left")
        ttk.Button(toolbar, text="Save map CSV", command=self.save_map).pack(side="left", padx=8)
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(toolbar, textvariable=self.status_var).pack(side="right")

        self.figure = Figure(figsize=(8, 7), dpi=100, facecolor="#101820")
        self.ax = self.figure.add_subplot(111, projection="3d")
        self.ax.set_facecolor("#101820")
        self.ax.set_box_aspect((1, 1, 1))
        self.ax.set_xlim(-6000, 6000)
        self.ax.set_ylim(-6000, 6000)
        self.ax.set_zlim(-6000, 6000)
        self.ax.set_xlabel("X (mm)", color="#c9d5df")
        self.ax.set_ylabel("Y (mm)", color="#c9d5df")
        self.ax.set_zlabel("Z (mm)", color="#c9d5df")
        self.ax.tick_params(colors="#c9d5df")
        self.ax.grid(color="#34414b", alpha=0.7)
        self.map_scatter = self.ax.scatter([], [], [], s=2, c="#53d8a2", alpha=0.65,
                                           label="3D point cloud")
        self.scan_scatter = self.ax.scatter([], [], [], s=5, c="#f4c95d", alpha=0.9,
                                            label="Current scan")
        self.ax.legend(loc="upper right", facecolor="#18232c", labelcolor="white",
                       framealpha=0.85)
        self.info = self.ax.text2D(0.015, 0.985, "", transform=self.ax.transAxes,
                                   va="top", color="white", family="monospace", fontsize=9)
        self.figure.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.figure, master=root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(50, self.drain_events)
        self.root.after(250, self.toggle_connection)

    def toggle_connection(self):
        if self.worker and self.worker.is_alive():
            self.stop_event.set()
            self.connect_button.configure(state="disabled")
            return
        port = self.port_var.get().strip() or "/dev/serial0"
        self.port_var.set(port)
        self.connect_button.configure(state="disabled")
        self.status_var.set(f"Opening {port}…")
        self.worker = threading.Thread(target=self.read_lidar, args=(port,), daemon=True)
        self.worker.start()

    def read_lidar(self, port):
        try:
            ser = serial.Serial(port=port, baudrate=BAUD_CANDIDATES[0], bytesize=8,
                                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                timeout=0.05)
            self.serial_port = ser
            self.events.put(("connected", port, BAUD_CANDIDATES[0]))
            scans = []
            parser = M1C1Parser()
            last_packet = time.perf_counter()
            while not self.stop_event.is_set():
                data = ser.read(4096)
                for flags, points in parser.feed(data):
                    if flags & 0x01 and scans:
                        self.events.put(("scan", np.asarray(scans, dtype=float), parser.good,
                                         parser.bad, time.perf_counter()))
                        scans = []
                    scans.extend(points)
                    last_packet = time.perf_counter()
                if (parser.good == 0 and time.perf_counter() - last_packet > 2.0
                        and ser.baudrate != BAUD_CANDIDATES[1]):
                    ser.baudrate = BAUD_CANDIDATES[1]
                    parser = M1C1Parser()
                    last_packet = time.perf_counter()
                    self.events.put(("baud", port, BAUD_CANDIDATES[1]))
            ser.close()
            self.serial_port = None
            self.events.put(("disconnected",))
        except (serial.SerialException, OSError) as exc:
            self.serial_port = None
            self.events.put(("error", str(exc)))

    def drain_events(self):
        dirty = False
        try:
            while True:
                event = self.events.get_nowait()
                if event[0] == "connected":
                    self.status_var.set(f"Connected: {event[1]} at {event[2]} baud")
                    self.connect_button.configure(text="Disconnect", state="normal")
                elif event[0] == "baud":
                    self.status_var.set(f"Trying {event[2]} baud…")
                elif event[0] == "disconnected":
                    self.connect_button.configure(text="Connect", state="normal")
                    self.status_var.set("Disconnected")
                    self.stop_event.clear()
                elif event[0] == "error":
                    self.connect_button.configure(text="Connect", state="normal")
                    self.status_var.set("Connection failed")
                    self.stop_event.clear()
                    messagebox.showerror("LiDAR connection failed", event[1])
                elif event[0] == "scan":
                    self.add_scan(event[1], event[4])
                    self.packet_good, self.packet_bad = event[2], event[3]
                    dirty = True
        except queue.Empty:
            pass
        if dirty:
            self.redraw()
        self.root.after(50, self.drain_events)

    def add_scan(self, scan, scan_time):
        if len(scan) < 12:
            return
        if self.last_scan_time is not None:
            self.scan_hz = 0.85 * self.scan_hz + 0.15 / max(scan_time - self.last_scan_time, 1e-3)
        self.last_scan_time = scan_time
        if self.sweep_start_time is None:
            self.sweep_start_time = scan_time
        angle = math.radians(self.sweep_rate_deg_s * (scan_time - self.sweep_start_time))
        cosine, sine = math.cos(angle), math.sin(angle)
        cloud = np.column_stack((scan[:, 0] * cosine, scan[:, 1], -scan[:, 0] * sine))
        self.map_points.extend(cloud[::2].tolist())
        self.map_points = self.map_points[-self.max_map_points:]
        self.latest_scan = cloud

    def redraw(self):
        if self.map_points:
            mapped = np.asarray(self.map_points)
            self.map_scatter._offsets3d = (mapped[:, 0], mapped[:, 1], mapped[:, 2])
            extent = max(1000.0, float(np.percentile(np.abs(mapped), 99.5)) * 1.15)
            self.ax.set_xlim(-extent, extent)
            self.ax.set_ylim(-extent, extent)
            self.ax.set_zlim(-extent, extent)
        if len(self.latest_scan):
            self.scan_scatter._offsets3d = (self.latest_scan[:, 0], self.latest_scan[:, 1],
                                             self.latest_scan[:, 2])
        sweep = 0.0
        if self.last_scan_time is not None and self.sweep_start_time is not None:
            sweep = ((self.last_scan_time - self.sweep_start_time) * self.sweep_rate_deg_s) % 360
        self.info.set_text(f"Scan rate   {self.scan_hz:5.2f} Hz\n"
                           f"Y sweep     {self.sweep_rate_deg_s:4.1f} deg/s\n"
                           f"Sweep angle {sweep:5.1f} deg\n"
                           f"Map points  {len(self.map_points):,}\n"
                           f"Packets OK  {self.packet_good:,}\n"
                           f"Packets bad {self.packet_bad:,}")
        self.canvas.draw_idle()

    def clear_map(self):
        self.map_points.clear()
        self.latest_scan = np.empty((0, 3))
        self.sweep_start_time = None
        self.last_scan_time = None
        self.redraw()

    def save_map(self):
        if not self.map_points:
            messagebox.showinfo("Map is empty", "Connect the LiDAR and collect a few scans first.")
            return
        path = filedialog.asksaveasfilename(title="Save LiDAR map", defaultextension=".csv",
                                            filetypes=(("CSV point cloud", "*.csv"),
                                                       ("All files", "*.*")),
                                            initialfile="lidar_map.csv")
        if path:
            np.savetxt(path, np.asarray(self.map_points), delimiter=",")
            self.status_var.set(f"Map saved: {path}")

    def close(self):
        self.stop_event.set()
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass
        self.root.destroy()


def main():
    requested_port = sys.argv[1] if len(sys.argv) > 1 else "/dev/serial0"
    root = tk.Tk()
    LidarMapper(root, requested_port)
    root.mainloop()


if __name__ == "__main__":
    main()
