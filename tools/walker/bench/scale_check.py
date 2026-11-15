# Compares the position decoded from MIT status replies with the motor's own
# mechPos register (0x7019, float radians). Sends only parameter reads.
import math
import socket
import struct
import sys
import time

mid = int(sys.argv[1])
s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind(("can_usb",))
s.settimeout(0.05)


def send(cid, data):
    s.send(struct.pack("=IB3x8s", cid | socket.CAN_EFF_FLAG, 8, bytes(data)))


def recv_until(pred, timeout=0.3):
    end = time.time() + timeout
    while time.time() < end:
        try:
            frame = s.recv(16)
        except socket.timeout:
            continue
        cid, _, data = struct.unpack("=IB3x8s", frame)
        cid &= socket.CAN_EFF_MASK
        result = pred(cid, data)
        if result is not None:
            return result
    return None


def mech_reply(cid, d):
    if (cid >> 24) & 0x1F == 0x11 and (cid >> 8) & 0xFF == mid and d[0] == 0x19 and d[1] == 0x70:
        return struct.unpack("<f", d[4:8])[0]
    return None


def status_reply(cid, d):
    if (cid >> 24) & 0x1F == 2 and (cid >> 8) & 0xFF == mid:
        return ((d[0] << 8 | d[1]) / 32767.0 - 1.0) * 4 * math.pi
    return None


mech, stat = [], []
for _ in range(5):
    send((0x11 << 24) | (0xFF << 8) | mid, [0x19, 0x70, 0, 0, 0, 0, 0, 0])
    mech.append(recv_until(mech_reply))
    stat.append(recv_until(status_reply))
    time.sleep(0.05)
fmt = lambda xs: ["%.4f" % x if x is not None else None for x in xs]
print("motor %d  mechPos register: %s" % (mid, fmt(mech)))
print("motor %d  MIT status pos:   %s" % (mid, fmt(stat)))
