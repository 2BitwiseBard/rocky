"""Mock Feetech bus — byte-level servo simulator behind the Transport API.

Faithful enough that every bench script runs unmodified with --mock:
  * full instruction parsing (PING/READ/WRITE/REG_WRITE/ACTION/SYNC_*)
  * per-family endianness + register maps, EEPROM LOCK semantics
  * response_level, broadcast silence, SYNC_READ replies in listed order
  * first-order motion model (position chases goal at goal speed, never
    faster than the datasheet no-load speed — D052: the mock cannot teleport)
  * MIN/MAX_ANGLE_LIMIT clamp the goal (D052 bench/apply_limits.py; the real
    STS behaviour at a limit is VERIFY-ON-BENCH — clamp is our assumption)
  * thermal model (heats with load^2 when torque on, cools toward ambient)
  * fault injection: corrupt_next_checksum, drop_next_response, extra noise
    bytes, baud mismatch (servo silent unless bauds agree), and per-servo
    `silent` (a brown-out / broken return wire: writes land, nothing answers)

The clock is manual (`advance(dt)`) plus an automatic per-transaction tick,
so tests are deterministic while long soaks can be fast-forwarded.
"""
from __future__ import annotations
import time
from dataclasses import dataclass, field
from .protocol import (HEADER, BROADCAST_ID, Instr, Family, checksum,
                       build_status, encode_u16, decode_u16)
from .registers import (MAPS, COUNTS, CENTER, BAUD_CODES, Reg, max_speed_cps)
from .transport import Transport

MEM_SIZE = 96


def _default_memory(family: Family, servo_id: int) -> bytearray:
    m = bytearray(MEM_SIZE)
    R = MAPS[family]

    def put(name, value):
        r = R[name]
        if r.nbytes == 1:
            m[r.addr] = value & 0xFF
        else:
            m[r.addr:r.addr + 2] = encode_u16(value, family)

    put("MODEL", 0x0309 if family is Family.STS else 0x0505)  # arbitrary mock IDs
    put("ID", servo_id)
    put("BAUD", 0)                     # 1 Mbps
    put("RETURN_DELAY", 250)           # 500 us, Feetech default
    put("RESPONSE_LEVEL", 1)
    put("MIN_ANGLE_LIMIT", 0)
    put("MAX_ANGLE_LIMIT", COUNTS[family] - 1)
    put("MAX_TEMP", 70)
    put("MAX_VOLT", 140 if family is Family.STS else 65)
    put("MIN_VOLT", 90 if family is Family.STS else 45)
    put("MAX_TORQUE", 1000)
    put("LOCK", 1)
    put("PRESENT_POSITION", CENTER[family])
    put("GOAL_POSITION", CENTER[family])
    put("PRESENT_VOLTAGE", 121 if family is Family.STS else 60)
    put("PRESENT_TEMP", 26)
    if family is Family.STS:
        put("TORQUE_LIMIT", 1000)
    return m


@dataclass
class MockServo:
    servo_id: int
    family: Family = Family.STS
    baud: int = 1_000_000
    ambient_c: float = 26.0
    # thermal model: dT/dt = heat_k * (load_frac^2) - cool_k * (T - ambient)
    # equilibria: 60% load -> 53 C (safe), 85% -> 80 C (crosses the 70 C
    # self-cut in ~3 min) — rough shape of D015's "43-65% is a thermal no-go"
    heat_k: float = 0.75       # degC/s at 100% load
    cool_k: float = 0.010      # 1/s
    external_load_pct: float = 0.0    # test hook: simulated mechanical load, %
    stalled: bool = False             # test hook: motion blocked
    silent: bool = False              # test hook: applies writes, never replies
    mem: bytearray = field(default_factory=bytearray)
    _temp_f: float = 26.0
    _pos_f: float = 0.0
    _reg_write: tuple[int, bytes] | None = None

    def __post_init__(self):
        if not self.mem:
            self.mem = _default_memory(self.family, self.servo_id)
        self._pos_f = float(CENTER[self.family])
        self._temp_f = self.ambient_c

    # -- register helpers ---------------------------------------------------
    def _reg(self, name) -> Reg:
        return MAPS[self.family][name]

    def get(self, name: str) -> int:
        r = self._reg(name)
        if r.nbytes == 1:
            return self.mem[r.addr]
        return decode_u16(bytes(self.mem[r.addr:r.addr + 2]), self.family)

    def put(self, name: str, value: int) -> None:
        r = self._reg(name)
        if r.nbytes == 1:
            self.mem[r.addr] = value & 0xFF
        else:
            self.mem[r.addr:r.addr + 2] = encode_u16(value, self.family)

    # -- dynamics -----------------------------------------------------------
    def advance(self, dt: float) -> None:
        """Integrate motion + thermals by dt seconds."""
        goal = self.get("GOAL_POSITION")
        lo, hi = self.get("MIN_ANGLE_LIMIT"), self.get("MAX_ANGLE_LIMIT")
        if hi > lo:                          # 0/0 would be wheel mode; 0/4095 is a no-op
            goal = min(max(goal, lo), hi)
        speed = self.get("GOAL_SPEED") & 0x7FFF if "GOAL_SPEED" in MAPS[self.family] else 0
        vmax = max_speed_cps(self.family)    # D052: 0 = "servo max" = the no-load speed
        max_step_s = min(speed, vmax) if speed > 0 else vmax
        torque_on = self.get("TORQUE_ENABLE") == 1
        # torque-limit vs load: if the allowed torque can't hold the external
        # load, the servo can't chase its goal — position sags away instead
        # (this is exactly what torque_step.py measures)
        under_torqued = False
        if "TORQUE_LIMIT" in MAPS[self.family]:
            tl_pct = self.get("TORQUE_LIMIT") / 10.0
            under_torqued = tl_pct < self.external_load_pct
        moving = 0
        if torque_on and not self.stalled and not under_torqued:
            delta = goal - self._pos_f
            step = max_step_s * dt
            if abs(delta) <= step:
                self._pos_f = float(goal)
            else:
                self._pos_f += step if delta > 0 else -step
                moving = 1
        elif torque_on and under_torqued:
            self._pos_f -= (self.external_load_pct - tl_pct) * 20.0 * dt
            self._pos_f = max(0.0, self._pos_f)
        self.put("PRESENT_POSITION", int(round(self._pos_f)))
        if "MOVING" in MAPS[self.family]:
            self.put("MOVING", moving)
        # load: external plus a bump while moving
        load_pct = min(100.0, self.external_load_pct + (18.0 if moving else 0.0)) \
            if torque_on else 0.0
        self.put("PRESENT_LOAD", int(load_pct * 10))          # 0.1% units
        if "PRESENT_CURRENT" in MAPS[self.family]:
            amps = 0.06 + 2.6 * (load_pct / 100.0)            # rough ST3215 curve
            self.put("PRESENT_CURRENT", int(amps / 0.0065))
        # thermals
        frac = load_pct / 100.0
        self._temp_f += (self.heat_k * frac * frac
                         - self.cool_k * (self._temp_f - self.ambient_c)) * dt
        self.put("PRESENT_TEMP", int(round(self._temp_f)))
        # over-temperature fault -> torque cut + status bit (protocol 1)
        if self._temp_f >= self.get("MAX_TEMP"):
            if "STATUS" in MAPS[self.family]:
                self.put("STATUS", self.get("STATUS") | 0b100)
            self.put("TORQUE_ENABLE", 0)

    # -- protocol -----------------------------------------------------------
    def error_byte(self) -> int:
        return self.get("STATUS") if "STATUS" in MAPS[self.family] else 0

    def handle(self, instr: int, params: bytes, broadcast: bool) -> bytes | None:
        """Return status packet bytes, or None for silence."""
        if self.silent:
            if instr in (Instr.WRITE,):
                self._apply_write(params[0], params[1:])
            return None
        level = self.get("RESPONSE_LEVEL")
        err = self.error_byte()

        def reply(payload: bytes = b"") -> bytes | None:
            if broadcast:
                return None
            if level == 0 and instr not in (Instr.PING, Instr.READ):
                return None
            return build_status(self.servo_id, err, payload)

        if instr == Instr.PING:
            return None if broadcast else build_status(self.servo_id, err)
        if instr == Instr.READ:
            addr, n = params[0], params[1]
            return None if broadcast else \
                build_status(self.servo_id, err, bytes(self.mem[addr:addr + n]))
        if instr in (Instr.WRITE, Instr.REG_WRITE):
            addr, data = params[0], params[1:]
            if instr == Instr.REG_WRITE:
                self._reg_write = (addr, bytes(data))
                return reply()
            self._apply_write(addr, data)
            return reply()
        if instr == Instr.ACTION:
            if self._reg_write:
                self._apply_write(*self._reg_write)
                self._reg_write = None
            return None if broadcast else reply()
        return None

    def _apply_write(self, addr: int, data: bytes) -> None:
        lock_reg = self._reg("LOCK")
        locked = self.mem[lock_reg.addr] == 1
        for i, b in enumerate(data):
            a = addr + i
            if a >= MEM_SIZE:
                break
            # EEPROM area writable only when unlocked (LOCK itself always writable)
            if a < 40 and locked and a != lock_reg.addr:
                continue
            self.mem[a] = b
        # an accepted ID write means the servo now answers on the new ID
        id_addr = self._reg("ID").addr
        if addr <= id_addr < addr + len(data):
            self.servo_id = self.mem[id_addr]
        # side effects
        ge = self._reg("GOAL_POSITION")
        if addr <= ge.addr < addr + len(data) and self.get("TORQUE_ENABLE") == 0:
            # Feetech: writing goal with torque off leaves position; we mirror that
            pass
        te = self._reg("TORQUE_ENABLE")
        if addr <= te.addr < addr + len(data) and self.mem[te.addr] == 1:
            # enabling torque snaps goal to present (avoid jump) — real STS
            # keeps last goal; we keep last goal too (no snap) for realism
            pass
        if addr <= lock_reg.addr < addr + len(data):
            pass  # lock state simply updated


class MockTransport(Transport):
    """A wire with N servos on it. Parses outgoing packets, queues replies."""

    def __init__(self, servos: list[MockServo], baud: int = 1_000_000,
                 auto_advance_s: float = 0.0006):
        self.servos = {s.servo_id: s for s in servos}
        self.baud = baud
        self.auto_advance_s = auto_advance_s   # sim time per transaction
        self.time_s = 0.0
        self._rx = bytearray()   # host->bus accumulation
        self._tx = bytearray()   # bus->host queue
        # fault injection
        self.corrupt_next_checksum = 0
        self.drop_next_response = 0
        self.noise_before_next = b""
        self.log: list[tuple[str, bytes]] = []

    # -- test/bench hooks ---------------------------------------------------
    def advance(self, dt: float) -> None:
        self.time_s += dt
        for s in self.servos.values():
            s.advance(dt)

    def servo(self, servo_id: int) -> MockServo:
        return self.servos[servo_id]

    # -- Transport API ------------------------------------------------------
    def set_baud(self, baud: int) -> None:
        self.baud = baud

    def write(self, data: bytes) -> None:
        self.log.append(("tx", data))
        self._rx += data
        self._process()

    def read(self, max_bytes: int, timeout_s: float) -> bytes:
        out = bytes(self._tx[:max_bytes])
        del self._tx[:max_bytes]
        if out:
            self.log.append(("rx", out))
        elif timeout_s > 0:
            # nothing queued: yield like a real port would instead of letting the
            # bus spin its deadline loop hot (the bridge thread shares the GIL
            # with the sim thread)
            time.sleep(min(timeout_s, 0.0005))
        return out

    def flush_input(self) -> None:
        self._tx.clear()

    # -- wire emulation -----------------------------------------------------
    def _process(self) -> None:
        while True:
            pkt = self._extract_instruction()
            if pkt is None:
                return
            servo_id, instr, params = pkt
            self.advance(self.auto_advance_s)
            responses: list[bytes] = []
            if instr == Instr.SYNC_WRITE:
                addr, dlen = params[0], params[1]
                body = params[2:]
                step = 1 + dlen
                for k in range(0, len(body) - step + 1, step):
                    sid = body[k]
                    if sid in self.servos:
                        self.servos[sid]._apply_write(addr, body[k + 1:k + 1 + dlen])
                # sync write: never answered
            elif instr == Instr.SYNC_READ:
                addr, dlen = params[0], params[1]
                for sid in params[2:]:
                    s = self.servos.get(sid)
                    if s and s.family is Family.STS and not s.silent:   # protocol 1 only
                        responses.append(build_status(
                            sid, s.error_byte(), bytes(s.mem[addr:addr + dlen])))
            elif servo_id == BROADCAST_ID:
                for sid in sorted(self.servos):
                    r = self.servos[sid].handle(instr, params, broadcast=True)
                    if r:
                        responses.append(r)
            else:
                s = self.servos.get(servo_id)
                if s and s.baud == self.baud:
                    r = s.handle(instr, params, broadcast=False)
                    if r:
                        responses.append(r)
            # ID writes may have re-keyed a servo
            self.servos = {s.servo_id: s for s in self.servos.values()}
            for r in responses:
                if self.drop_next_response > 0:
                    self.drop_next_response -= 1
                    continue
                if self.corrupt_next_checksum > 0:
                    self.corrupt_next_checksum -= 1
                    r = r[:-1] + bytes(((r[-1] ^ 0x5A),))
                if self.noise_before_next:
                    self._tx += self.noise_before_next
                    self.noise_before_next = b""
                self._tx += r

    def _extract_instruction(self) -> tuple[int, int, bytes] | None:
        buf = self._rx
        while len(buf) >= 2 and not (buf[0] == 0xFF and buf[1] == 0xFF):
            buf.pop(0)
        if len(buf) < 6:
            return None
        length = buf[3]
        total = 4 + length
        if len(buf) < total:
            return None
        raw = bytes(buf[:total])
        del buf[:total]
        body, chk = raw[2:-1], raw[-1]
        if checksum(body) != chk:
            return self._extract_instruction()   # servos ignore corrupt frames
        return raw[2], raw[4], raw[5:total - 1]


def make_pebble_mock(n_legs: int = 5) -> MockTransport:
    """The full 20-servo robot: IDs 1..15 STS legs, 16..20 SCS hands."""
    servos = [MockServo(i, Family.STS) for i in range(1, 3 * n_legs + 1)]
    servos += [MockServo(15 + i, Family.SCS) for i in range(1, n_legs + 1)]
    return MockTransport(servos)


def make_factory_fresh_mock(n: int = 1, family: Family = Family.STS) -> MockTransport:
    """What a bag of new servos looks like: everyone is ID 1 (why the runbook
    says ONE SERVO AT A TIME during assignment)."""
    return MockTransport([MockServo(1, family) for _ in range(n)][:1])
