
import math
import time
from collections import deque

import serial


class M1C1Parser:

    MAX_LSN = 100

    def __init__(self, angle_offset=192.0):
        self.buf = bytearray()
        self.angle_offset = angle_offset
        self.ok = 0
        self.bad = 0

    @staticmethod
    def u16(data, i):
        return data[i] | (data[i + 1] << 8)

    @staticmethod
    def angle_correction(distance):
        if distance == 0:
            return 0.0

        return math.degrees(
            math.atan(
                19.16 * (distance - 90.15) /
                (90.15 * distance)
            )
        )

    def xor_ok(self, pkt):
        lsn = pkt[3]

        cs = (
            0x55AA ^
            (pkt[2] | (lsn << 8)) ^
            self.u16(pkt, 4) ^
            self.u16(pkt, 6)
        )

        for i in range(lsn):
            cs ^= self.u16(pkt, 10 + 2 * i)

        return cs == self.u16(pkt, 8)

    def decode_scan(self, pkt):
        lsn = pkt[3]

        fsa = (self.u16(pkt, 4) >> 1) / 64.0
        lsa = (self.u16(pkt, 6) >> 1) / 64.0

        span = (lsa - fsa) % 360.0

        points = []

        for i in range(lsn):

            distance = self.u16(pkt, 10 + 2 * i) >> 2

            base = fsa + (
                span * i / (lsn - 1)
                if lsn > 1 else 0.0
            )

            angle = (
                base
                - self.angle_correction(distance)
                + self.angle_offset
            ) % 360.0

            points.append((angle, distance))

        return points

    def feed(self, data):
        self.buf.extend(data)
        packets = []

        while True:

            hits = [
                i for i in (
                    self.buf.find(b"\xAA\x55"),
                    self.buf.find(b"\xA5\x5A")
                )
                if i >= 0
            ]

            if not hits:
                if self.buf and self.buf[-1] in (0xAA, 0xA5):
                    self.buf[:] = self.buf[-1:]
                else:
                    self.buf.clear()
                break

            i = min(hits)

            if i:
                del self.buf[:i]

            if self.buf[0] == 0xAA:

                if len(self.buf) < 10:
                    break

                lsn = self.buf[3]

                if lsn == 0 or lsn > self.MAX_LSN:
                    del self.buf[:1]
                    continue

                n = 10 + 2 * lsn

                if len(self.buf) < n:
                    break

                pkt = bytes(self.buf[:n])

                if self.xor_ok(pkt):

                    del self.buf[:n]
                    self.ok += 1

                    packets.append({
                        "ct": pkt[2],
                        "points": self.decode_scan(pkt)
                    })

                else:
                    self.bad += 1
                    del self.buf[:1]

            else:
                # Skip module information frames.
                if len(self.buf) < 7:
                    break

                length = self.u16(self.buf, 2)

                if length > 64:
                    del self.buf[:1]
                    continue

                n = 7 + length

                if len(self.buf) < n:
                    break

                frame = bytes(self.buf[:n])

                valid = (
                    (sum(frame) - frame[4] - frame[5])
                    & 0xFFFF
                ) == self.u16(frame, 4)

                if valid:
                    del self.buf[:n]
                else:
                    del self.buf[:1]

        return packets


class M1C1Lidar:

    CMD_START = bytes([0xAA, 0x55, 0xF0, 0x0F])
    CMD_STOP = bytes([0xAA, 0x55, 0xF5, 0x0A])

    def __init__(
        self,
        port="/dev/serial0",
        baudrate=115200,
        angle_offset=192.0,
        min_range=80,
        max_range=8000,
        send_start_cmd=False
    ):

        self.min_range = min_range
        self.max_range = max_range

        self.parser = M1C1Parser(angle_offset)
        self.pending = deque()
        self.current_sweep = []

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.05
        )

        self.started = False

        if send_start_cmd:
            self.start()

    def start(self):
        if not self.started:
            self.ser.write(self.CMD_START)
            self.started = True

    def stop(self):
        if self.started:
            self.ser.write(self.CMD_STOP)
            self.started = False

    def get_data(self, timeout=None):
        """
        Blocking acquisition of one complete 360-degree sweep.

        Returns:
            [(angle_degrees, distance_mm), ...]

        Raises:
            TimeoutError if a complete sweep isn't received.
        """

        start_time = time.monotonic()

        while True:

            if timeout is not None:
                if time.monotonic() - start_time >= timeout:
                    raise TimeoutError("LiDAR sweep timeout")

            if not self.pending:

                data = self.ser.read(4096)

                if not data:
                    continue

                self.pending.extend(self.parser.feed(data))

            while self.pending:

                packet = self.pending.popleft()

                points = [
                    (a, d)
                    for a, d in packet["points"]
                    if self.min_range <= d <= self.max_range
                ]

                if packet["ct"] & 0x01:

                    if self.current_sweep:
                        result = self.current_sweep
                        self.current_sweep = list(points)
                        return result

                self.current_sweep.extend(points)

    def close(self):
        self.stop()

        if self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
