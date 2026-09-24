"""FeetechBus — the one bus object everything above it talks to.

Speaks BOTH families on one wire (D016): protocol 1 (ST3215/STS3250 legs,
IDs 1–15) and protocol 0 (SCS0009 hands, IDs 16–20). Family is chosen per
call from the servo map you hand the constructor, so callers never think
about endianness again.

Safety posture baked in (master plan §7):
  * every transaction retries on checksum error / timeout (bus noise)
  * telemetry reads decode fault bits and raise them to the caller
  * `SafetyMonitor` polls temp/volt/load and torque-cuts past limits

D052: `read_telemetry` is the tolerant read — one SYNC_READ, collect until the
deadline, one single-read retry of whoever stayed silent, and return what
answered PLUS the missing list. One browned-out servo no longer blanks the
whole group (the old all-or-nothing `sync_telemetry` did, and a caller that
swallowed the NoResponse silently kept stale positions). `sync_telemetry`
keeps the strict contract for the bench scripts on top of it.
"""
from __future__ import annotations
import time
from dataclasses import dataclass, field
from . import protocol as fp
from .protocol import Family, Instr, ChecksumError
from .registers import (MAPS, BAUD_CODES, BAUD_TO_CODE, CURRENT_LSB_A,
                        VOLTAGE_LSB_V, LOAD_LSB_PCT, counts_to_deg,
                        deg_to_counts, Reg, COUNTS as MAPS_COUNTS)
from .transport import Transport


class BusError(RuntimeError):
    pass


class NoResponse(BusError):
    pass


# SYNC_READ collection budget: base timeout + this much per listed servo
# (a reply is ~22 bytes = 0.22 ms at 1 Mbps plus the 0.5 ms default return
# delay, so 1 ms/servo is ~30 % headroom). The old timeout_s * n budget was
# 0.3 s for 15 servos — longer than seven bridge ticks.
SYNC_REPLY_S = 0.001


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

    def _transact(self, pkt: bytes, expect_ids: list[int] | None,
                  retries: int | None = None) -> list[fp.Packet]:
        """Send one instruction packet; collect replies for expect_ids (in order).
        expect_ids=None means fire-and-forget. retries overrides self.retries
        (the tolerant telemetry path uses 0/1 to keep a bridge tick bounded)."""
        last_err: Exception | None = None
        n_retry = self.retries if retries is None else retries
        for attempt in range(n_retry + 1):
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
        raise NoResponse(f"no valid reply after {n_retry + 1} tries: {last_err}")

    def _collect(self, want: list[int], budget_s: float | None = None,
                 partial: bool = False) -> list[fp.Packet]:
        """Gather replies for `want`. partial=True never raises: a corrupt
        frame is counted and skipped, the deadline ends the wait, and the
        caller sees who is still missing in `want` (mutated in place)."""
        out: list[fp.Packet] = []
        deadline = time.monotonic() + (self.timeout_s * len(want) if budget_s is None else budget_s)
        while want:
            chunk = self.t.read(256, self.timeout_s)
            if chunk:
                self._rxbuf += chunk
            while True:
                try:
                    pkt = fp.parse_stream(self._rxbuf)
                except ChecksumError:
                    self.stats["checksum_errors"] += 1
                    if partial:
                        continue            # parse_stream consumed the bad frame
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
                if partial:
                    break
                raise NoResponse(f"timeout waiting for ids {want}")
        return out

    # ------------------------------------------------------------ commands
    def ping(self, servo_id: int) -> bool:
        try:
            self._transact(fp.ping(servo_id), [servo_id])
            return True
        except NoResponse:
            return False

    def read_raw(self, servo_id: int, addr: int, nbytes: int,
                 retries: int | None = None) -> bytes:
        pkt = self._transact(fp.read(servo_id, addr, nbytes), [servo_id], retries)[0]
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

    @staticmethod
    def _speed_block(fam: Family, speed_cps: int) -> bool:
        """Write pos+time+speed as one block? Always on STS (D052): speed 0
        means "servo max" and must be WRITTEN, or a slow speed left by a jog
        or a sim2real entry lingers in GOAL_SPEED forever. SCS keeps the old
        rule (speed only when asked) until its speed-0 semantics are bench-
        verified."""
        return "GOAL_SPEED" in MAPS[fam] and (fam is Family.STS or bool(speed_cps))

    def set_position(self, servo_id: int, deg: float, speed_cps: int = 0,
                     acc: int | None = None) -> None:
        fam = self.family_of(servo_id)
        if acc is not None and "ACC" in MAPS[fam]:
            self.write_reg(servo_id, "ACC", acc)
        counts = deg_to_counts(deg, fam)
        r = MAPS[fam]["GOAL_POSITION"]
        data = fp.encode_u16(counts, fam)
        if self._speed_block(fam, speed_cps):
            # write pos+time+speed as one block (addr 42..47) — atomic move
            data = data + fp.encode_u16(0, fam) + fp.encode_u16(int(speed_cps), fam)
        self.write_raw(servo_id, r.addr, data)

    def sync_write_reg(self, name: str, values: dict[int, int]) -> None:
        """One SYNC_WRITE per family of one SRAM register (TORQUE_LIMIT, ACC,
        ...). Ids whose family lacks the register are skipped. EEPROM
        registers are refused: they need the LOCK dance per servo."""
        for fam in (Family.STS, Family.SCS):
            if name not in MAPS[fam]:
                continue
            r = MAPS[fam][name]
            if r.eeprom:
                raise BusError(f"{name} is EEPROM; use write_reg per servo")
            group = [(i, int(v)) for i, v in values.items() if self.family_of(i) is fam]
            if not group:
                continue
            if r.nbytes == 1:
                entries = [(i, bytes((v & 0xFF,))) for i, v in group]
            else:
                enc = fp.encode_sm16 if r.kind == "sm16" else fp.encode_u16
                entries = [(i, enc(v, fam)) for i, v in group]
            self._transact(fp.sync_write(r.addr, r.nbytes, entries), None)

    def sync_positions(self, targets: dict[int, float], speed_cps: int = 0) -> None:
        """One SYNC_WRITE per family. targets: id -> degrees (center-relative).
        speed_cps 0 = servo max (written explicitly on STS, see _speed_block)."""
        for fam in (Family.STS, Family.SCS):
            group = {i: d for i, d in targets.items() if self.family_of(i) is fam}
            if not group:
                continue
            r = MAPS[fam]["GOAL_POSITION"]
            if self._speed_block(fam, speed_cps):
                entries = [(i, fp.encode_u16(deg_to_counts(d, fam), fam)
                            + fp.encode_u16(0, fam)
                            + fp.encode_u16(int(speed_cps), fam))
                           for i, d in group.items()]
                self._transact(fp.sync_write(r.addr, 6, entries), None)
            else:
                entries = [(i, fp.encode_u16(deg_to_counts(d, fam), fam))
                           for i, d in group.items()]
                self._transact(fp.sync_write(r.addr, 2, entries), None)

    # ------------------------------------------------------------ telemetry
    def telemetry(self, servo_id: int, retries: int | None = None) -> Telemetry:
        fam = self.family_of(servo_id)
        # one block read 56..70 (STS) / 56..66 (SCS) — cheap on the wire
        span = 15 if fam is Family.STS else 11
        raw = self.read_raw(servo_id, 56, span, retries)
        return self._decode_tel(servo_id, fam, raw)

    @staticmethod
    def _decode_tel(servo_id: int, fam: Family, raw: bytes) -> Telemetry:
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

    def read_telemetry(self, ids: list[int], single_retries: int = 0
                       ) -> tuple[dict[int, Telemetry], list[int]]:
        """Tolerant telemetry (D052). One SYNC_READ for the STS ids, collected
        until ONE deadline (timeout_s + SYNC_REPLY_S per id); every STS id that
        stayed silent gets one single READ (plus single_retries extra tries); SCS
        ids are read one by one (they ignore SYNC_READ) with one retry.
        Never raises NoResponse/ChecksumError — returns (answered, missing).
        A transport failure (OSError from a pulled adapter) still raises."""
        out: dict[int, Telemetry] = {}
        sts = [i for i in ids if self.family_of(i) is Family.STS]
        if sts:
            self.t.flush_input()             # a late reply from a timed-out read must
            self._rxbuf.clear()              # not pass for this frame's telemetry
            self.t.write(fp.sync_read(56, 15, sts))
            self.stats["tx"] += 1
            for p in self._collect(list(sts), self.timeout_s + SYNC_REPLY_S * len(sts),
                                   partial=True):
                if len(p.params) == 15:
                    out[p.id] = self._decode_tel(p.id, Family.STS, p.params)
        for i in ids:
            if i in out:
                continue
            fam = self.family_of(i)
            n = single_retries if fam is Family.STS else single_retries + 1
            try:
                out[i] = self.telemetry(i, retries=n)
            except (NoResponse, ChecksumError, BusError):
                pass
        return out, [i for i in ids if i not in out]

    def sync_telemetry(self, ids: list[int]) -> dict[int, Telemetry]:
        """Strict: SYNC_READ for STS ids (one bus transaction), loop for SCS
        ids; NoResponse if anyone stays silent after the full retries. For
        bench scripts that want all-or-nothing; live loops use read_telemetry."""
        tels, missing = self.read_telemetry(ids, single_retries=self.retries)
        if missing:
            raise NoResponse(f"timeout waiting for ids {missing}")
        return tels

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

    def set_angle_limits(self, servo_id: int, min_counts: int, max_counts: int,
                         verify: bool = True) -> tuple[int, int]:
        """EEPROM MIN/MAX_ANGLE_LIMIT (D052 hardware backstop), read back.
        Returns the read-back pair; BusError if it does not match."""
        fam = self.family_of(servo_id)
        top = MAPS_COUNTS[fam] - 1
        if not (0 <= min_counts < max_counts <= top):
            raise ValueError(f"id {servo_id}: bad angle limits {min_counts}..{max_counts} "
                             f"(need 0 <= min < max <= {top}; 0/0 would be wheel mode)")
        self.write_reg(servo_id, "MIN_ANGLE_LIMIT", int(min_counts))
        self.write_reg(servo_id, "MAX_ANGLE_LIMIT", int(max_counts))
        if not verify:
            return int(min_counts), int(max_counts)
        rb = (self.read_reg(servo_id, "MIN_ANGLE_LIMIT"), self.read_reg(servo_id, "MAX_ANGLE_LIMIT"))
        if rb != (int(min_counts), int(max_counts)):
            raise BusError(f"id {servo_id}: angle limits read back {rb}, wrote "
                           f"{(int(min_counts), int(max_counts))}")
        return rb

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
    # D052: the hands (SCS0009) are monitored too now, on their own 5-6 V rail
    # (D016) — the 3S window above would flag every healthy hand every poll.
    scs_volt_min_v: float = 4.5
    scs_volt_max_v: float = 7.0   # above this it has probably met the 12 V rail

    def volt_window(self, family: Family) -> tuple[float, float]:
        if family is Family.SCS:
            return self.scs_volt_min_v, self.scs_volt_max_v
        return self.volt_min_v, self.volt_max_v


def silent_telemetry(servo_id: int, family: Family) -> Telemetry:
    """Placeholder for a servo that did not answer (the "lost" event): same
    shape as a real reading so on_event callbacks that print tel fields keep
    working; position is NaN and faults says why."""
    return Telemetry(servo_id, family, -1, float("nan"), 0, 0.0, 0.0, 0, None,
                     False, ["no_reply"])


class SafetyMonitor:
    """Poll telemetry; torque-cut on violations. Call step() at 1-5 Hz.

    D052: step() uses the tolerant read, so a silent servo no longer hides the
    others. `check(tels, missing)` is the same logic on telemetry someone else
    already read (the hardware bridge reads at 25 Hz and feeds every frame in,
    so a fault bit cuts within one tick, not one poll). A servo missing from
    `lost_after` consecutive POLLS (missing passed, not None) is tripped with
    reason "lost" and a "lost" event (tel = silent_telemetry). `cut(sid, tel)`
    is the action on a trip; default torque-off that one servo — the bridge
    replaces it to limp the whole leg. Nothing but `rearm(ids)` clears a trip.
    """

    def __init__(self, bus: FeetechBus, ids: list[int],
                 limits: SafetyLimits | None = None, on_event=None,
                 lost_after: int = 3):
        self.bus = bus
        self.ids = list(ids)
        self.limits = limits or SafetyLimits()
        self.on_event = on_event or (lambda kind, tel: None)
        self.tripped: set[int] = set()
        self.reason: dict[int, str] = {}      # sid -> why it tripped
        self.lost: set[int] = set()           # tripped because it stopped answering
        self.missed: dict[int, int] = {}      # sid -> consecutive polls missed
        self.lost_after = int(lost_after)
        self.missing: list[int] = []          # who was silent in the last step()
        self.cut = lambda sid, tel: self.bus.torque(sid, False)

    def step(self) -> dict[int, Telemetry]:
        tels, missing = self.bus.read_telemetry(self.ids)
        self.missing = missing
        self.check(tels, missing)
        return tels

    def trip(self, sid: int, tel: Telemetry, reason: str) -> None:
        self.tripped.add(sid)
        self.reason[sid] = reason
        try:
            self.cut(sid, tel)
        except BusError:
            pass                              # the event still fires; torque state unknown
        self.on_event("cut", tel)

    def check(self, tels: dict[int, Telemetry], missing: list[int] | None = None,
              warn: bool = True) -> None:
        L = self.limits
        for sid, tel in tels.items():
            self.missed[sid] = 0
            if tel.temp_c >= L.temp_cut_c or tel.faults:
                if sid not in self.tripped:
                    why = ("fault " + ",".join(tel.faults)) if tel.faults else f"temp {tel.temp_c} C"
                    self.trip(sid, tel, why)
            elif warn and tel.temp_c >= L.temp_warn_c:
                self.on_event("temp_warn", tel)
            v_lo, v_hi = L.volt_window(tel.family)
            if warn and tel.voltage_v and not (v_lo <= tel.voltage_v <= v_hi):
                self.on_event("volt", tel)
            if warn and tel.load_pct >= L.load_warn_pct:
                self.on_event("load_warn", tel)
        for sid in (missing or ()):
            n = self.missed.get(sid, 0) + 1
            self.missed[sid] = n
            if n >= self.lost_after and sid not in self.tripped:
                self.tripped.add(sid)
                self.lost.add(sid)
                self.reason[sid] = "lost"
                self.on_event("lost", silent_telemetry(sid, self.bus.family_of(sid)))

    def rearm(self, ids) -> list[int]:
        """Clear trips on these ids (the only way — D052). Returns the re-armed."""
        out = [i for i in ids if i in self.tripped]
        for i in ids:
            self.tripped.discard(i)
            self.lost.discard(i)
            self.reason.pop(i, None)
            self.missed[i] = 0
        return out
