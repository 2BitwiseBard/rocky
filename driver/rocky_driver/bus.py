"""FeetechBus — the one bus object everything above it talks to.

Speaks BOTH families on one wire (D016): protocol 1 (ST3215/STS3250 legs,
IDs 1–15) and protocol 0 (SCS0009 hands, IDs 16–20). Family is chosen per
call from the servo map you hand the constructor, so callers never think
about endianness again.

Safety posture baked in (master plan §7):
  * every transaction retries on checksum error / timeout (bus noise)
  * telemetry reads decode fault bits and raise them to the caller
  * `SafetyMonitor` polls temp/volt/load and torque-cuts past limits
"""
from __future__ import annotations
import time
from dataclasses import dataclass, field
from . import protocol as fp
from .protocol import Family, Instr, ChecksumError
from .registers import (MAPS, BAUD_CODES, BAUD_TO_CODE, CURRENT_LSB_A,
                        VOLTAGE_LSB_V, LOAD_LSB_PCT, counts_to_deg,
                        deg_to_counts, Reg)
from .transport import Transport


class BusError(RuntimeError):
    pass


class NoResponse(BusError):
    pass


@dataclass
class Telemetry:
    servo_id: int
    family: Family
    position_counts: int
    position_deg: float
    speed_counts: int
    load_pct: float
    voltage_v: float
    temp_c: int
    current_a: float | None     # STS only
    moving: bool
    faults: list[str] = field(default_factory=list)

    def __str__(self):
        cur = f" {self.current_a:5.2f} A" if self.current_a is not None else ""
        fl = f"  FAULTS: {','.join(self.faults)}" if self.faults else ""
        return (f"id {self.servo_id:2d} [{self.family.name}] "
                f"pos {self.position_counts:4d} ({self.position_deg:+7.2f} deg)  "
                f"load {self.load_pct:5.1f}%  {self.voltage_v:4.1f} V "
                f"{self.temp_c:2d} C{cur}{fl}")


class FeetechBus:
    def __init__(self, transport: Transport, families: dict[int, Family] | None = None,
                 timeout_s: float = 0.02, retries: int = 2):
        """families: id -> Family map. Unknown IDs default to STS (legs)."""
        self.t = transport
        self.families = dict(families or {})
        self.timeout_s = timeout_s
        self.retries = retries
        self.stats = {"tx": 0, "rx": 0, "retries": 0, "checksum_errors": 0,
                      "timeouts": 0}
        self._rxbuf = bytearray()

    # ------------------------------------------------------------ low level
    def family_of(self, servo_id: int) -> Family:
        return self.families.get(servo_id, Family.STS)

    def _reg(self, servo_id: int, name: str) -> Reg:
        m = MAPS[self.family_of(servo_id)]
        if name not in m:
            raise KeyError(f"register {name} not defined for "
                           f"{self.family_of(servo_id).name} (id {servo_id})")
        return m[name]

    def _transact(self, pkt: bytes, expect_ids: list[int] | None) -> list[fp.Packet]:
        """Send one instruction packet; collect replies for expect_ids (in order).
        expect_ids=None means fire-and-forget."""
        last_err: Exception | None = None
        for attempt in range(self.retries + 1):
            if attempt:
                self.stats["retries"] += 1
                self.t.flush_input()
                self._rxbuf.clear()
            self.t.write(pkt)
            self.stats["tx"] += 1
            if not expect_ids:
                return []
            try:
                return self._collect(list(expect_ids))
            except (NoResponse, ChecksumError) as e:
                last_err = e
                continue
        raise NoResponse(f"no valid reply after {self.retries + 1} tries: {last_err}")

    def _collect(self, want: list[int]) -> list[fp.Packet]:
        out: list[fp.Packet] = []
        deadline = time.monotonic() + self.timeout_s * len(want)
        while want:
            chunk = self.t.read(256, self.timeout_s)
            if chunk:
                self._rxbuf += chunk
            while True:
                try:
                    pkt = fp.parse_stream(self._rxbuf)
                except ChecksumError:
                    self.stats["checksum_errors"] += 1
                    raise
                if pkt is None:
                    break
                self.stats["rx"] += 1
                if pkt.id in want:
                    want.remove(pkt.id)
                    out.append(pkt)
                # else: stale/echo packet — discard
            if not want:
                break
            if not chunk and time.monotonic() > deadline:
                self.stats["timeouts"] += 1
                raise NoResponse(f"timeout waiting for ids {want}")
        return out

    # ------------------------------------------------------------ commands
    def ping(self, servo_id: int) -> bool:
        try:
            self._transact(fp.ping(servo_id), [servo_id])
            return True
        except NoResponse:
            return False

    def read_raw(self, servo_id: int, addr: int, nbytes: int) -> bytes:
        pkt = self._transact(fp.read(servo_id, addr, nbytes), [servo_id])[0]
        if len(pkt.params) != nbytes:
            raise BusError(f"short read from {servo_id}: {pkt.params!r}")
        return pkt.params

    def write_raw(self, servo_id: int, addr: int, data: bytes,
                  await_status: bool = True) -> int:
        expect = [servo_id] if (await_status and servo_id != fp.BROADCAST_ID) else None
        pkts = self._transact(fp.write(servo_id, addr, data), expect)
        return pkts[0].code if pkts else 0

    def read_reg(self, servo_id: int, name: str) -> int:
        r = self._reg(servo_id, name)
        raw = self.read_raw(servo_id, r.addr, r.nbytes)
        fam = self.family_of(servo_id)
        if r.nbytes == 1:
            return raw[0]
        return fp.decode_sm16(raw, fam) if r.kind == "sm16" else fp.decode_u16(raw, fam)

    def write_reg(self, servo_id: int, name: str, value: int,
                  await_status: bool = True) -> None:
        r = self._reg(servo_id, name)
        fam = self.family_of(servo_id)
        if r.nbytes == 1:
            data = bytes((value & 0xFF,))
        else:
            data = (fp.encode_sm16(value, fam) if r.kind == "sm16"
                    else fp.encode_u16(value, fam))
        if r.eeprom:
            self._eeprom_write(servo_id, r, data)
        else:
            self.write_raw(servo_id, r.addr, data, await_status)

    def _eeprom_write(self, servo_id: int, r: Reg, data: bytes) -> None:
        """EEPROM writes need the LOCK released; always re-lock after."""
        self.write_reg(servo_id, "LOCK", 0)
        try:
            self.write_raw(servo_id, r.addr, data)
        finally:
            self.write_reg(servo_id, "LOCK", 1)

    # ------------------------------------------------------------ position
    def torque(self, servo_id: int, enable: bool) -> None:
        self.write_reg(servo_id, "TORQUE_ENABLE", 1 if enable else 0)

    def torque_all(self, ids: list[int], enable: bool) -> None:
        for fam in (Family.STS, Family.SCS):
            group = [i for i in ids if self.family_of(i) is fam]
            if group:
                r = MAPS[fam]["TORQUE_ENABLE"]
                self._transact(fp.sync_write(
                    r.addr, 1, [(i, bytes((1 if enable else 0,))) for i in group]),
                    None)

    def set_position(self, servo_id: int, deg: float, speed_cps: int = 0,
                     acc: int | None = None) -> None:
        fam = self.family_of(servo_id)
        if acc is not None and "ACC" in MAPS[fam]:
            self.write_reg(servo_id, "ACC", acc)
        counts = deg_to_counts(deg, fam)
        r = MAPS[fam]["GOAL_POSITION"]
        data = fp.encode_u16(counts, fam)
        if "GOAL_SPEED" in MAPS[fam] and speed_cps:
            # write pos+time+speed as one block (addr 42..47) — atomic move
            data = data + fp.encode_u16(0, fam) + fp.encode_u16(speed_cps, fam)
        self.write_raw(servo_id, r.addr, data)

    def sync_positions(self, targets: dict[int, float], speed_cps: int = 0) -> None:
        """One SYNC_WRITE per family. targets: id -> degrees (center-relative)."""
        for fam in (Family.STS, Family.SCS):
            group = {i: d for i, d in targets.items() if self.family_of(i) is fam}
            if not group:
                continue
            r = MAPS[fam]["GOAL_POSITION"]
            if speed_cps:
                entries = [(i, fp.encode_u16(deg_to_counts(d, fam), fam)
                            + fp.encode_u16(0, fam)
                            + fp.encode_u16(speed_cps, fam))
                           for i, d in group.items()]
                self._transact(fp.sync_write(r.addr, 6, entries), None)
            else:
                entries = [(i, fp.encode_u16(deg_to_counts(d, fam), fam))
                           for i, d in group.items()]
                self._transact(fp.sync_write(r.addr, 2, entries), None)

    # ------------------------------------------------------------ telemetry
    def telemetry(self, servo_id: int) -> Telemetry:
        fam = self.family_of(servo_id)
        # one block read 56..70 (STS) / 56..66 (SCS) — cheap on the wire
        span = 15 if fam is Family.STS else 11
        raw = self.read_raw(servo_id, 56, span)
        pos = fp.decode_u16(raw[0:2], fam)
        spd = fp.decode_sm16(raw[2:4], fam) if fam is Family.STS \
            else fp.decode_u16(raw[2:4], fam)
        load = fp.decode_sm16(raw[4:6], fam) if fam is Family.STS \
            else fp.decode_u16(raw[4:6], fam)
        volt, temp = raw[6], raw[7]
        moving = bool(raw[10])
        current = None
        faults = []
        if fam is Family.STS:
            faults = fp.decode_error(raw[9])           # STATUS @65
            current = fp.decode_sm16(raw[13:15], fam) * CURRENT_LSB_A  # @69-70
        return Telemetry(servo_id, fam, pos, counts_to_deg(pos, fam), spd,
                         abs(load) * LOAD_LSB_PCT, volt * VOLTAGE_LSB_V,
                         temp, current, moving, faults)

    def sync_telemetry(self, ids: list[int]) -> dict[int, Telemetry]:
        """SYNC_READ for STS ids (one bus transaction), loop for SCS ids."""
        out: dict[int, Telemetry] = {}
        sts = [i for i in ids if self.family_of(i) is Family.STS]
        if sts:
            pkts = self._transact(fp.sync_read(56, 15, sts), sts)
            for p in pkts:
                fam = Family.STS
                raw = p.params
                pos = fp.decode_u16(raw[0:2], fam)
                out[p.id] = Telemetry(
                    p.id, fam, pos, counts_to_deg(pos, fam),
                    fp.decode_sm16(raw[2:4], fam),
                    abs(fp.decode_sm16(raw[4:6], fam)) * LOAD_LSB_PCT,
                    raw[6] * VOLTAGE_LSB_V, raw[7],
                    fp.decode_sm16(raw[13:15], fam) * CURRENT_LSB_A,
                    bool(raw[10]), fp.decode_error(raw[9]))
        for i in ids:
            if self.family_of(i) is Family.SCS:
                out[i] = self.telemetry(i)
        return out

    # ------------------------------------------------------------ admin
    def scan(self, id_range=range(1, 31), bauds=(1_000_000, 500_000, 115_200),
             progress=None) -> dict[int, dict]:
        """Find every servo: try each baud, ping, read MODEL + basics.
        Returns {id: {baud, model_le, model_be, family_guess, volt, temp}}."""
        found: dict[int, dict] = {}
        for baud in bauds:
            self.t.set_baud(baud)
            for sid in id_range:
                if sid in found:
                    continue
                if progress:
                    progress(baud, sid)
                if not self.ping(sid):
                    continue
                raw = self.read_raw(sid, 3, 2)
                info = dict(
                    baud=baud,
                    model_le=fp.decode_u16(raw, Family.STS),
                    model_be=fp.decode_u16(raw, Family.SCS),
                    family_cfg=self.family_of(sid).name,
                )
                try:
                    tel = self.telemetry(sid)
                    info.update(volt=tel.voltage_v, temp=tel.temp_c,
                                pos=tel.position_counts)
                except BusError:
                    pass
                found[sid] = info
        self.t.set_baud(bauds[0])
        return found

    def set_id(self, old_id: int, new_id: int) -> None:
        """EEPROM ID change with read-back verification."""
        if not (0 <= new_id <= fp.MAX_ID):
            raise ValueError(f"bad new id {new_id}")
        fam = self.family_of(old_id)
        self.write_reg(old_id, "LOCK", 0)
        r = MAPS[fam]["ID"]
        # after this write the servo answers only on new_id
        self.write_raw(old_id, r.addr, bytes((new_id,)), await_status=False)
        self.families.pop(old_id, None)
        self.families[new_id] = fam
        self.write_reg(new_id, "LOCK", 1)
        if self.read_reg(new_id, "ID") != new_id:
            raise BusError(f"id change {old_id}->{new_id} failed verification")

    def set_baud_reg(self, servo_id: int, baud: int) -> None:
        self.write_reg(servo_id, "BAUD", BAUD_TO_CODE[baud])

    def set_position_offset(self, servo_id: int, offset_counts: int) -> None:
        """STS only: burn center-calibration offset into EEPROM (sm16)."""
        if self.family_of(servo_id) is not Family.STS:
            raise BusError("SCS0009 has no POSITION_OFFSET register; "
                           "store a software offset in calibration.yaml instead")
        self.write_reg(servo_id, "POSITION_OFFSET", offset_counts)


# ---------------------------------------------------------------- monitor
@dataclass
class SafetyLimits:
    temp_warn_c: int = 60
    temp_cut_c: int = 65          # master plan §7: auto-sit at 65 C
    volt_min_v: float = 9.9       # 3S at 3.3 V/cell under load
    volt_max_v: float = 12.9
    load_warn_pct: float = 60.0   # sustained-load warning (D015 margins)


class SafetyMonitor:
    """Poll telemetry; torque-cut on violations. Call step() at 1–5 Hz."""

    def __init__(self, bus: FeetechBus, ids: list[int],
                 limits: SafetyLimits | None = None, on_event=None):
        self.bus = bus
        self.ids = list(ids)
        self.limits = limits or SafetyLimits()
        self.on_event = on_event or (lambda kind, tel: None)
        self.tripped: set[int] = set()

    def step(self) -> dict[int, Telemetry]:
        tels = self.bus.sync_telemetry(self.ids)
        for sid, tel in tels.items():
            L = self.limits
            if tel.temp_c >= L.temp_cut_c or tel.faults:
                if sid not in self.tripped:
                    self.tripped.add(sid)
                    self.bus.torque(sid, False)
                    self.on_event("cut", tel)
            elif tel.temp_c >= L.temp_warn_c:
                self.on_event("temp_warn", tel)
            if tel.voltage_v and not (L.volt_min_v <= tel.voltage_v <= L.volt_max_v):
                self.on_event("volt", tel)
            if tel.load_pct >= L.load_warn_pct:
                self.on_event("load_warn", tel)
        return tels
