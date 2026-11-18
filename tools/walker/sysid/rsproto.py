"""RobStride private CAN protocol: the frames the sysid tools send and read.

Python port of src/robstride_motor/include/robstride_motor/can_protocol.hpp plus the model
table in robstride_motor_node.cpp. Keep the two in step: test_sysid.py checks this file against
the C++ unit-test vectors.

29-bit extended ID = comm type (bits 28-24) | data field (23-8) | target ID (7-0).
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass

HOST = 0xFF

T_GET_ID, T_MOTION, T_STATUS, T_ENABLE, T_STOP = 0, 1, 2, 3, 4
T_READ, T_WRITE, T_FAULT, T_ACTIVE = 0x11, 0x12, 0x15, 0x18

P_RUN_MODE = 0x7005      # u8: 0 = MIT (operation control), 5 = CSP
P_TORQUE_LIMIT = 0x700B  # float, Nm
P_MECH_POS = 0x7019      # float, rad (true output angle, multi-turn)
P_IQ = 0x701A            # float, A (q-axis current, filtered)
P_MECH_VEL = 0x701B      # float, rad/s
P_VBUS = 0x701C          # float, V
P_CAN_TIMEOUT = 0x7028   # u32, 20000 counts per second

RUN_MODE_MIT = 0
MODE_RESET, MODE_CALIBRATION, MODE_RUNNING = 0, 1, 2
FAULT_BITS = ('undervoltage', 'overcurrent', 'overtemperature', 'magnetic encoder',
              'HALL encoder', 'not calibrated')


@dataclass(frozen=True)
class Ranges:
    """Wire scaling of MIT commands and status frames (RobStride manuals). position, velocity
    and torque map over +/- max; kp and kd over 0..max. A wrong row silently scales gains."""
    position_max: float
    velocity_max: float
    torque_max: float
    kp_max: float
    kd_max: float


RANGES = {
    'RS02': Ranges(4.0 * math.pi, 44.0, 17.0, 500.0, 5.0),
    'RS03': Ranges(4.0 * math.pi, 20.0, 60.0, 5000.0, 100.0),
    'RS06': Ranges(4.0 * math.pi, 50.0, 36.0, 5000.0, 100.0),
}


@dataclass
class Status:
    mode: int
    faults: int     # bit field, see FAULT_BITS
    q: float        # rad
    v: float        # rad/s
    tau: float      # Nm, the drive's estimate from current
    temp: float     # deg C

    def fault_names(self):
        return [n for i, n in enumerate(FAULT_BITS) if self.faults >> i & 1]


def can_id(ctype: int, middle: int, motor: int) -> int:
    return (ctype & 0x1F) << 24 | (middle & 0xFFFF) << 8 | (motor & 0xFF)


def comm_type(cid: int) -> int:
    return cid >> 24 & 0x1F


def frame_motor(cid: int) -> int:
    """Replies carry the sending motor's ID in bits 8-15."""
    return cid >> 8 & 0xFF


def u16_encode(value: float, lo: float, hi: float) -> int:
    x = (min(max(value, lo), hi) - lo) * 65535.0 / (hi - lo)
    return int(math.floor(x + 0.5))  # std::lround for x >= 0


def u16_decode(raw: int, max_abs: float) -> float:
    return (raw / 32767.0 - 1.0) * max_abs


# ----------------------------------------------------------------------------- host -> motor
def motion(motor: int, q: float, v: float, kp: float, kd: float, tau: float, r: Ranges):
    """Type 1 (MIT): the drive applies tau + kp (q - q_meas) + kd (v - v_meas) in its own loop."""
    fields = (u16_encode(q, -r.position_max, r.position_max),
              u16_encode(v, -r.velocity_max, r.velocity_max),
              u16_encode(kp, 0.0, r.kp_max),
              u16_encode(kd, 0.0, r.kd_max))
    return (can_id(T_MOTION, u16_encode(tau, -r.torque_max, r.torque_max), motor),
            struct.pack('>4H', *fields))


def stop(motor: int, clear_faults: bool):
    return can_id(T_STOP, HOST, motor), bytes([1 if clear_faults else 0]) + bytes(7)


def enable(motor: int):
    return can_id(T_ENABLE, HOST, motor), bytes(8)


def active_report(motor: int, on: bool):
    return can_id(T_ACTIVE, HOST, motor), bytes([1, 2, 3, 4, 5, 6, 1 if on else 0, 0])


def read_param(motor: int, index: int):
    return can_id(T_READ, HOST, motor), struct.pack('<H', index) + bytes(6)


def write_f32(motor: int, index: int, value: float):
    return can_id(T_WRITE, HOST, motor), struct.pack('<HHf', index, 0, value)


def write_u8(motor: int, index: int, value: int):
    return can_id(T_WRITE, HOST, motor), struct.pack('<HHB3x', index, 0, value)


def write_u32(motor: int, index: int, value: int):
    return can_id(T_WRITE, HOST, motor), struct.pack('<HHI', index, 0, value)


def can_timeout_counts(seconds: float) -> int:
    return int(math.floor(seconds * 20000.0 + 0.5))


# ----------------------------------------------------------------------------- motor -> host
def decode_status(cid: int, data: bytes, r: Ranges) -> Status:
    """Type 2 (reply to every MIT command) and type 0x18 (active report)."""
    q, v, tau, temp = struct.unpack('>4H', bytes(data[:8]))
    return Status(mode=cid >> 22 & 0x3, faults=cid >> 16 & 0x3F,
                  q=u16_decode(q, r.position_max), v=u16_decode(v, r.velocity_max),
                  tau=u16_decode(tau, r.torque_max), temp=temp * 0.1)


def decode_param_reply(cid: int, data: bytes):
    """Type 0x11 reply -> (motor, index, float value)."""
    index = struct.unpack('<H', bytes(data[:2]))[0]
    return frame_motor(cid), index, struct.unpack('<f', bytes(data[4:8]))[0]


def status_frame(motor: int, st: Status, r: Ranges):
    """What a motor sends back (used by the simulated motor)."""
    def enc(x, m):  # inverse of u16_decode
        return min(max(int(math.floor((x / m + 1.0) * 32767.0 + 0.5)), 0), 65535)
    cid = T_STATUS << 24 | (st.mode & 3) << 22 | (st.faults & 0x3F) << 16 | (motor & 0xFF) << 8 | HOST
    data = struct.pack('>4H', enc(st.q, r.position_max), enc(st.v, r.velocity_max),
                       enc(st.tau, r.torque_max), min(max(int(round(st.temp * 10)), 0), 65535))
    return cid, data


def param_reply(motor: int, index: int, value):
    """`value` is a float, or the 4 raw value bytes (for integer parameters)."""
    raw = bytes(value) if isinstance(value, (bytes, bytearray)) else struct.pack('<f', value)
    return can_id(T_READ, motor, HOST), struct.pack('<HH', index, 0) + raw
