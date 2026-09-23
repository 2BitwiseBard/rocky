"""Wire-format tests with hand-computed golden vectors.

If any of these fail, nothing else matters — these bytes are the contract
with the physical servos.
"""
import pytest
from rocky_driver import protocol as fp
from rocky_driver.protocol import Family


# ---------------------------------------------------------------- checksum
def test_checksum_known_vector():
    # WRITE goal position 0x0800 (2048) to id 1 @ addr 0x2A, STS little-endian:
    # FF FF 01 05 03 2A 00 08 C4
    body = bytes((0x01, 0x05, 0x03, 0x2A, 0x00, 0x08))
    assert fp.checksum(body) == 0xC4


def test_ping_packet_golden():
    assert fp.ping(1) == bytes.fromhex("ff ff 01 02 01 fb".replace(" ", ""))


def test_write_packet_golden_sts():
    pkt = fp.write(1, 0x2A, fp.encode_u16(2048, Family.STS))
    assert pkt == bytes.fromhex("ffff0105032a0008c4")


def test_write_packet_golden_scs():
    # same logical write to a protocol-0 servo id 16, position 512 (0x0200):
    # params big-endian: 02 00
    pkt = fp.write(16, 0x2A, fp.encode_u16(512, Family.SCS))
    body = bytes((0x10, 0x05, 0x03, 0x2A, 0x02, 0x00))
    assert pkt == fp.HEADER + body + bytes((fp.checksum(body),))


def test_read_packet_golden():
    # READ 2 bytes @ 56 from id 3: FF FF 03 04 02 38 02 BC
    pkt = fp.read(3, 56, 2)
    assert pkt == bytes.fromhex("ffff03040238 02bc".replace(" ", ""))


# ---------------------------------------------------------------- words
@pytest.mark.parametrize("value", [0, 1, 255, 256, 2048, 4095, 65535])
def test_u16_roundtrip_both_families(value):
    for fam in (Family.STS, Family.SCS):
        assert fp.decode_u16(fp.encode_u16(value, fam), fam) == value


def test_endianness_is_actually_different():
    b_sts = fp.encode_u16(0x0102, Family.STS)
    b_scs = fp.encode_u16(0x0102, Family.SCS)
    assert b_sts == bytes((0x02, 0x01))     # lo, hi
    assert b_scs == bytes((0x01, 0x02))     # hi, lo
    assert b_sts != b_scs


@pytest.mark.parametrize("value", [0, 5, -5, 1000, -1000, 32767, -32767])
def test_sm16_roundtrip(value):
    for fam in (Family.STS, Family.SCS):
        assert fp.decode_sm16(fp.encode_sm16(value, fam), fam) == value


def test_sm16_bit15_encoding():
    # -100 on STS: 100|0x8000 = 0x8064 -> little endian 64 80
    assert fp.encode_sm16(-100, Family.STS) == bytes((0x64, 0x80))


# ---------------------------------------------------------------- sync
def test_sync_write_layout():
    pkt = fp.sync_write(0x2A, 2, [(1, b"\x00\x08"), (2, b"\x10\x08")])
    # FF FF FE LEN 83 2A 02 01 00 08 02 10 08 CHK ; LEN = params+2 = 8+2
    assert pkt[2] == 0xFE
    assert pkt[3] == 10
    assert pkt[4] == 0x83
    assert pkt[5:7] == bytes((0x2A, 0x02))
    assert pkt[7:13] == bytes((1, 0x00, 0x08, 2, 0x10, 0x08))


def test_sync_write_length_mismatch_raises():
    with pytest.raises(ValueError):
        fp.sync_write(0x2A, 2, [(1, b"\x00")])


def test_sync_read_layout():
    pkt = fp.sync_read(56, 15, [1, 2, 3])
    assert pkt[2] == 0xFE and pkt[4] == 0x82
    assert pkt[5:7] == bytes((56, 15))
    assert pkt[7:10] == bytes((1, 2, 3))


# ---------------------------------------------------------------- parsing
def test_parse_stream_roundtrip():
    buf = bytearray(fp.build_status(7, 0, b"\x12\x34"))
    pkt = fp.parse_stream(buf)
    assert pkt.id == 7 and pkt.code == 0 and pkt.params == b"\x12\x34"
    assert len(buf) == 0


def test_parse_stream_skips_leading_garbage():
    buf = bytearray(b"\x00\x9a\xff" + fp.build_status(2, 0))
    pkt = fp.parse_stream(buf)
    assert pkt is not None and pkt.id == 2


def test_parse_stream_partial_then_complete():
    full = fp.build_status(4, 0, b"\x01")
    buf = bytearray(full[:4])
    assert fp.parse_stream(buf) is None
    buf += full[4:]
    pkt = fp.parse_stream(buf)
    assert pkt is not None and pkt.id == 4


def test_parse_stream_checksum_error_consumes_packet():
    bad = bytearray(fp.build_status(9, 0, b"\x55"))
    bad[-1] ^= 0xFF
    buf = bytearray(bytes(bad) + fp.build_status(9, 0, b"\x66"))
    with pytest.raises(fp.ChecksumError):
        fp.parse_stream(buf)
    pkt = fp.parse_stream(buf)     # the good one behind it still parses
    assert pkt is not None and pkt.params == b"\x66"


def test_parse_stream_extra_ff_preamble():
    buf = bytearray(b"\xff" + fp.build_status(5, 0))
    pkt = fp.parse_stream(buf)
    assert pkt is not None and pkt.id == 5


def test_error_bit_decode():
    assert fp.decode_error(0b100) == ["temperature"]
    assert set(fp.decode_error(0b100101)) == {"voltage", "temperature", "overload"}
