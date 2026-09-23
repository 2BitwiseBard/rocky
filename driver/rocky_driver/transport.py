"""Byte transports: real serial port or the in-process mock bus.

The bus logic only ever sees this interface, so every test (and every bench
rehearsal with --mock) runs the exact same code path as hardware, down to
the byte level.
"""
from __future__ import annotations


class Transport:
    def write(self, data: bytes) -> None:
        raise NotImplementedError

    def read(self, max_bytes: int, timeout_s: float) -> bytes:
        """Return up to max_bytes; b'' on timeout."""
        raise NotImplementedError

    def flush_input(self) -> None:
        pass

    def close(self) -> None:
        pass

    def set_baud(self, baud: int) -> None:
        pass


class SerialTransport(Transport):
    """pyserial-backed transport (lazy import so the mock path needs no deps).

    Half-duplex TTL note: the Waveshare Bus Servo Adapter does TX/RX
    direction switching in hardware; from the host it is a plain serial port.
    """

    def __init__(self, port: str, baud: int = 1_000_000):
        import serial  # pip install pyserial
        self._serial = serial.Serial(port=port, baudrate=baud, timeout=0)
        self.port = port

    def write(self, data: bytes) -> None:
        self._serial.write(data)
        self._serial.flush()

    def read(self, max_bytes: int, timeout_s: float) -> bytes:
        import time
        end = time.monotonic() + timeout_s
        out = bytearray()
        while len(out) < max_bytes:
            chunk = self._serial.read(max_bytes - len(out))
            if chunk:
                out += chunk
                # keep draining while bytes are flowing
                continue
            if time.monotonic() >= end:
                break
            time.sleep(0.0002)
        return bytes(out)

    def flush_input(self) -> None:
        self._serial.reset_input_buffer()

    def close(self) -> None:
        self._serial.close()

    def set_baud(self, baud: int) -> None:
        self._serial.baudrate = baud
