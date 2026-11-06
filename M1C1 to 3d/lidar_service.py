"""Raspberry Pi LiDAR stream service. Uses a separate port; rover control stays on 5005."""
import threading
import time
import socket
from m1c1 import M1C1Lidar
from servo import ServoController
from scanner3d import Scanner3D
from lidar_net import PORT, send_packet, recv_packet

class LidarService:
    def __init__(self, host="", port=PORT, lidar_port="/dev/serial0",
                 baudrate=115200, servo_pin=17, angle_step=2.0,
                 point_lifetime=30.0, settle_time=0.15):
        self.host, self.port = host, port
        self.lidar_port, self.baudrate, self.servo_pin = lidar_port, baudrate, servo_pin
        self.angle_step = float(angle_step)
        self.point_lifetime = float(point_lifetime)
        self.settle_time = float(settle_time)
        self.running = threading.Event()
        self.scanning = threading.Event()
        self.client_lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.client = None
        self.server = None
        self.worker = None
        self.scanner = None
        self.lidar = None
        self.servo = None
        self.avg_sweep_points = 0.0
        self.avg_sweep_seconds = 0.0

    def start(self):
        self.lidar = M1C1Lidar(port=self.lidar_port, baudrate=self.baudrate,
            angle_offset=192.0, min_range=80, max_range=8000, send_start_cmd=False)
        self.servo = ServoController(pin=self.servo_pin)
        self.scanner = Scanner3D(self.lidar, self.servo, self.angle_step,
            self.settle_time, 5, 20.0)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(1)
        self.running.set()
        threading.Thread(target=self._accept_loop, daemon=True).start()
        self.worker = threading.Thread(target=self._scan_loop, daemon=True)
        self.worker.start()
        print(f"LiDAR data service listening on {self.host or '0.0.0.0'}:{self.port}")

    def start_scan(self):
        self.scanning.set()

    def stop_scan(self):
        self.scanning.clear()

    def _accept_loop(self):
        while self.running.is_set():
            try:
                self.server.settimeout(1.0)
                client, address = self.server.accept()
                client.settimeout(None)
                with self.client_lock:
                    if self.client:
                        try: self.client.close()
                        except OSError: pass
                    self.client = client
                print("LiDAR viewer connected:", address)
                threading.Thread(target=self._receive_loop, args=(client,), daemon=True).start()
                send_packet(client, {"type":"status","status":"connected"}, self.send_lock)
            except socket.timeout:
                continue
            except OSError:
                break

    def _receive_loop(self, client):
        buf = bytearray()
        try:
            while self.running.is_set():
                packet = recv_packet(client, buf)
                if packet is None: break
                if packet.get("type") == "settings":
                    step = float(packet.get("angle_step", self.angle_step))
                    lifetime = float(packet.get("point_lifetime", self.point_lifetime))
                    if not 0 < step <= 180 or lifetime < 0:
                        raise ValueError("Invalid scanner settings")
                    self.angle_step, self.point_lifetime = step, lifetime
                    self.scanner.set_angle_step(step)
                    send_packet(client, {"type":"settings","angle_step":step,
                        "point_lifetime":lifetime}, self.send_lock)
                elif packet.get("type") == "start":
                    self.start_scan()
                elif packet.get("type") == "stop":
                    self.stop_scan()
        except (OSError, ValueError, TypeError) as exc:
            print("LiDAR viewer connection ended:", exc)
        finally:
            with self.client_lock:
                if self.client is client: self.client = None
            try: client.close()
            except OSError: pass

    def _send(self, packet):
        with self.client_lock:
            client = self.client
        if client:
            try: send_packet(client, packet, self.send_lock)
            except OSError:
                with self.client_lock:
                    if self.client is client: self.client = None

    def _scan_loop(self):
        angles_index = 0
        while self.running.is_set():
            if not self.scanning.wait(0.2):
                continue
            angles = self.scanner.get_servo_angles()
            if angles_index >= len(angles): angles_index = 0
            angle = angles[angles_index]
            started = time.monotonic()
            try:
                points = self.scanner.scan_position(angle)
                elapsed = max(time.monotonic() - started, 1e-6)
                n = len(points)
                self.avg_sweep_points = n if not self.avg_sweep_points else 0.8*self.avg_sweep_points+0.2*n
                self.avg_sweep_seconds = elapsed if not self.avg_sweep_seconds else 0.8*self.avg_sweep_seconds+0.2*elapsed
                pmin = self.avg_sweep_points * 60.0 / self.avg_sweep_seconds
                self._send({"type":"points","angle":angle,"points":points,
                    "sweep_seconds":elapsed,"points_per_min_est":pmin,
                    "point_lifetime":self.point_lifetime})
                self._send({"type":"status","status":f"Scanning {angle:+.1f}°"})
                angles_index = (angles_index + 1) % len(angles)
            except TimeoutError:
                self._send({"type":"status","status":"LiDAR sweep timeout; retrying"})
            except Exception as exc:
                self._send({"type":"error","message":str(exc)})
                self.scanning.clear()

    def close(self):
        self.running.clear()
        self.scanning.clear()
        if self.server:
            try: self.server.close()
            except OSError: pass
        with self.client_lock:
            if self.client:
                try: self.client.close()
                except OSError: pass
                self.client = None
        if self.worker and self.worker.is_alive(): self.worker.join(timeout=2)
        if self.lidar: self.lidar.close()
        if self.servo: self.servo.cleanup()
