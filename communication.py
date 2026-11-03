import socket
import json
import struct

#encode data in a json to be sent over TCP to the PI
def send_msg(sock, data_dict):
    serialized = json.dumps(data_dict).encode("utf-8")
    length_prefix = struct.pack('!I', len(serialized))
    sock.sendall(length_prefix + serialized)

def recv_msg(sock):
    raw_msglen = sock.recv(4) #recieve len bytes
    if not raw_msglen:
        return None
    msglen = struct.unpack('!I', raw_msglen)[0]

    data = bytearray()
    while len(data)<msglen:
        packet = sock.recv(msglen-len(data))
        if not packet:
            return None
        data.extend(packet)
    return json.loads(data.decode('utf-8'))
