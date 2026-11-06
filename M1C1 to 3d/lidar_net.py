"""Small newline-delimited JSON transport on a dedicated TCP port (5006)."""
import json
import socket
import threading

PORT = 5006
MAX_LINE = 8 * 1024 * 1024

def send_packet(sock, packet, lock=None):
    raw = (json.dumps(packet, separators=(",", ":")) + "\n").encode("utf-8")
    if lock:
        with lock:
            sock.sendall(raw)
    else:
        sock.sendall(raw)

def recv_packet(sock, buffer):
    while b"\n" not in buffer:
        chunk = sock.recv(65536)
        if not chunk:
            return None
        buffer.extend(chunk)
        if len(buffer) > MAX_LINE:
            raise ValueError("LiDAR network message is too large")
    line, _, rest = buffer.partition(b"\n")
    buffer[:] = rest
    return json.loads(line.decode("utf-8"))
