"""Feetech TTL bus protocol — wire-level framing shared by both families.

Both generations Pebble carries speak the same *framing*; they differ in
16-bit byte order and register maps (see registers.py):

  Protocol 0 — SCS series (SCS0009 hand servos): 16-bit values BIG-endian
  Protocol 1 — STS/SMS series (ST3215 / STS3250 leg servos): LITTLE-endian

Instruction packet:  0xFF 0xFF ID LEN INSTR PARAM... CHKSUM
Status packet:       0xFF 0xFF ID LEN ERR   PARAM... CHKSUM
  LEN    = n_params + 2
  CHKSUM = ~(ID + LEN + INSTR/ERR + sum(params)) & 0xFF

Broadcast ID 0xFE: no status reply (except SYNC_READ, protocol 1 only,
where each listed servo replies in listed order).

References: Feetech SCServo SDK memory tables + protocol appnote. Anything
we could not verify from two sources is tagged VERIFY-ON-BENCH in comments;
`bench/register_dump.py` exists to settle those on day one.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum

HEADER = b"\xff\xff"
BROADCAST_ID = 0xFE
MAX_ID = 0xFD


class Instr:
    PING = 0x01
    READ = 0x02
    WRITE = 0x03
    REG_WRITE = 0x04     # buffered write, committed by ACTION
    ACTION = 0x05
    RESET = 0x06         # factory reset — never used by scripts, listed for dumps
    SYNC_READ = 0x82     # protocol 1 only
    SYNC_WRITE = 0x83


class Family(Enum):
    """Servo family = which endianness + register map to use."""
    SCS = 0      # protocol 0 (SCS0009): big-endian words
    STS = 1      # protocol 1 (ST3215/STS3250): little-endian words

    @property
    def little_endian(self) -> bool:
        return self is Family.STS


# --- status packet ERROR bits (protocol 1; VERIFY-ON-BENCH for SCS) --------
ERROR_BITS = {
    0: "voltage",
    1: "sensor",        # angle/magnet fault
    2: "temperature",
    3: "current",
    4: "reserved4",
    5: "overload",
}


def decode_error(err: int) -> list[str]:
    return [name for bit, name in ERROR_BITS.items() if err & (1 << bit)]


# ---------------------------------------------------------------- checksum
def checksum(payload: bytes) -> int:
    """payload = everything between header and checksum (ID..last param)."""
    return (~sum(payload)) & 0xFF


# ---------------------------------------------------------------- words
def encode_u16(value: int, family: Family) -> bytes:
    value &= 0xFFFF
    lo, hi = value & 0xFF, value >> 8
    return bytes((lo, hi)) if family.little_endian else bytes((hi, lo))


def decode_u16(b: bytes, family: Family) -> int:
    if family.little_endian:
        return b[0] | (b[1] << 8)
    return b[1] | (b[0] << 8)


def encode_sm16(value: int, family: Family) -> bytes:
    """Signed-magnitude 16-bit (bit 15 = sign) — STS speed/offset style."""
    v = abs(int(value)) & 0x7FFF
    if value < 0:
        v |= 0x8000
    return encode_u16(v, family)


def decode_sm16(b: bytes, family: Family) -> int:
    v = decode_u16(b, family)
    mag = v & 0x7FFF
    return -mag if v & 0x8000 else mag


# ---------------------------------------------------------------- packets
def build_packet(servo_id: int, instr: int, params: bytes = b"") -> bytes:
    if not (0 <= servo_id <= BROADCAST_ID):
        raise ValueError(f"bad servo id {servo_id}")
    if len(params) > 250:
        raise ValueError("params too long")
    body = bytes((servo_id, len(params) + 2, instr)) + params
    return HEADER + body + bytes((checksum(body),))


def build_status(servo_id: int, error: int = 0, params: bytes = b"") -> bytes:
    body = bytes((servo_id, len(params) + 2, error)) + params
    return HEADER + body + bytes((checksum(body),))


@dataclass
class Packet:
    """A parsed packet. For instruction packets `code` is the instruction;
    for status packets it is the servo's ERROR byte."""
    id: int
    code: int
    params: bytes

    @property
    def error_names(self) -> list[str]:
        return decode_error(self.code)


class ChecksumError(ValueError):
    pass


class FramingError(ValueError):
    pass


def parse_stream(buf: bytearray) -> Packet | None:
    """Pull one complete packet off the front of `buf` (consuming its bytes).

    Returns None if the buffer does not yet hold a complete packet. Garbage
    before a header is discarded (bus noise / partial frames). Raises
    ChecksumError after consuming a complete-but-corrupt packet, so the
    caller can retry.
    """
    # hunt for header
    while len(buf) >= 2 and not (buf[0] == 0xFF and buf[1] == 0xFF):
        buf.pop(0)
    if len(buf) < 5:
        return None
    # tolerate >2 leading 0xFF (idle-high glitch): find the *last* FF pair start
    while len(buf) >= 3 and buf[2] == 0xFF:
        buf.pop(0)
        if len(buf) < 5:
            return None
    pkt_id, length = buf[2], buf[3]
    if length < 2:
        # impossible length — drop the false header and rescan
        buf.pop(0)
        return parse_stream(buf)
    total = 4 + length  # header(2) + id + len + [code + params + chksum]=length
    if len(buf) < total:
        return None
    raw = bytes(buf[:total])
    del buf[:total]
    body, chk = raw[2:-1], raw[-1]
    if checksum(body) != chk:
        raise ChecksumError(f"checksum mismatch id={pkt_id} len={length} "
                            f"got={chk:#04x} want={checksum(body):#04x}")
    return Packet(id=pkt_id, code=raw[4], params=raw[5:-1])


# ---------------------------------------------------------------- helpers
def ping(servo_id: int) -> bytes:
    return build_packet(servo_id, Instr.PING)


def read(servo_id: int, addr: int, nbytes: int) -> bytes:
    return build_packet(servo_id, Instr.READ, bytes((addr, nbytes)))


def write(servo_id: int, addr: int, data: bytes) -> bytes:
    return build_packet(servo_id, Instr.WRITE, bytes((addr,)) + data)


def reg_write(servo_id: int, addr: int, data: bytes) -> bytes:
    return build_packet(servo_id, Instr.REG_WRITE, bytes((addr,)) + data)


def action(servo_id: int = BROADCAST_ID) -> bytes:
    return build_packet(servo_id, Instr.ACTION)


def sync_write(addr: int, data_len: int, entries: list[tuple[int, bytes]]) -> bytes:
    """entries = [(id, data), ...]; every data must be data_len bytes."""
    params = bytearray((addr, data_len))
    for sid, data in entries:
        if len(data) != data_len:
            raise ValueError(f"sync_write id {sid}: data length {len(data)} != {data_len}")
        params.append(sid)
        params += data
    return build_packet(BROADCAST_ID, Instr.SYNC_WRITE, bytes(params))


def sync_read(addr: int, data_len: int, ids: list[int]) -> bytes:
    """Protocol 1 only — SCS-family servos ignore this instruction."""
    return build_packet(BROADCAST_ID, Instr.SYNC_READ,
                        bytes((addr, data_len)) + bytes(ids))
