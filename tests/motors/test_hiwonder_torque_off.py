#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Verified torque-off for HiwonderMotorsBus (issue #2) and the SDK reply-length check."""

import logging
import re
import time
from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass, field

import pytest
import serial

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.hiwonder import hiwonder as hiwonder_mod, hiwonder_sdk as hw
from lerobot.motors.hiwonder.hiwonder import HiwonderMotorsBus
from lerobot.utils.errors import DeviceNotConnectedError

pytestmark = pytest.mark.timeout(10)

TE_ADDR, GOAL_ADDR, LOCK_ADDR, PRESENT_ADDR, STATUS_ADDR = 40, 42, 55, 56, 65


def _checksum(packet: list[int]) -> int:
    return ~sum(packet[2:-1]) & 0xFF


def status_packet(hw_id: int, params: list[int], error: int = 0, length: int | None = None) -> bytes:
    """A status packet; `length` overrides the LEN byte (default len(params) + 2)."""
    packet = [0xFF, 0xFF, hw_id, len(params) + 2 if length is None else length, error, *params, 0]
    packet[-1] = _checksum(packet)
    return bytes(packet)


class ByteServoLine:
    """Byte-level fake of the serial line: parses instruction packets and answers like HX-30HM servos.

    `memory[id][addr]` holds register bytes. `queued` replies (bytes) are served before the servo's own
    reply, which models a stale reply left in the input buffer.
    """

    def __init__(self, memory: dict[int, dict[int, int]]):
        self.memory = memory
        self.rx = bytearray()
        self.queued: list[bytes] = []
        self.errors: dict[int, int] = {}
        self.raise_on_read: list[BaseException] = []
        self.on_write = None

    # pyserial surface used by PortHandler
    def write(self, data) -> int:
        data = list(data)
        if self.on_write is not None:
            self.on_write(data)
        hw_id, instr, params = data[2], data[4], data[5:-1]
        reply = b""
        if instr == hw.INST_READ:
            addr, n = params
            regs = self.memory[hw_id]
            reply = status_packet(hw_id, [regs.get(addr + i, 0) for i in range(n)], self.errors.get(hw_id, 0))
        elif instr == hw.INST_WRITE:
            addr, values = params[0], params[1:]
            for i, v in enumerate(values):
                self.memory[hw_id][addr + i] = v
            reply = status_packet(hw_id, [], self.errors.get(hw_id, 0))
        for stale in self.queued:
            self.rx += stale
        self.queued.clear()
        self.rx += reply
        return len(data)

    def read(self, n: int) -> bytes:
        if self.raise_on_read:
            raise self.raise_on_read.pop(0)
        out, self.rx = bytes(self.rx[:n]), self.rx[n:]
        return out

    def reset_input_buffer(self) -> None:
        self.rx.clear()

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass

    @property
    def in_waiting(self) -> int:
        return len(self.rx)


def make_packet_handler(line: ByteServoLine) -> hw.PacketHandler:
    port = hw.PortHandler("/dev/fake")
    port.ser = line
    port.is_open = True
    port.tx_time_per_byte = (1000.0 / port.baudrate) * 10.0
    return hw.PacketHandler(port)


# --------------------------------------------------------------------------------------------------------------
# T1 / R4: readData rejects a reply whose LEN byte does not match the request
# --------------------------------------------------------------------------------------------------------------


def test_read_reply_with_matching_len_is_accepted():
    line = ByteServoLine({6: {TE_ADDR: 1, PRESENT_ADDR: 0x02, PRESENT_ADDR + 1: 0x08}})
    ph = make_packet_handler(line)
    assert ph.read1ByteData(6, TE_ADDR) == (1, hw.COMM_SUCCESS, 0)
    assert ph.read2ByteData(6, PRESENT_ADDR) == (0x0802, hw.COMM_SUCCESS, 0)


def test_stale_write_ack_is_not_decoded_as_a_one_byte_read():
    """A late write ack (LEN=2, no data) for the same ID would otherwise be read as Torque_Enable."""
    line = ByteServoLine({6: {TE_ADDR: 1}})
    ph = make_packet_handler(line)
    line.queued.append(status_packet(6, []))  # stale ack of an earlier write
    value, comm, _ = ph.read1ByteData(6, TE_ADDR)
    assert comm == hw.COMM_RX_CORRUPT
    assert value == 0  # SDK convention on failure; callers must look at comm


def test_one_byte_reply_to_a_two_byte_read_is_rejected():
    line = ByteServoLine({6: {}})
    ph = make_packet_handler(line)
    line.queued.append(status_packet(6, [0x00]))  # stale Torque_Enable reply (LEN=3)
    _, comm, _ = ph.read2ByteData(6, PRESENT_ADDR)
    assert comm == hw.COMM_RX_CORRUPT


def test_wrong_len_reply_is_rejected_by_read_data():
    line = ByteServoLine({6: {}})
    ph = make_packet_handler(line)
    line.queued.append(status_packet(6, [0x10, 0x20]))  # 2-byte reply to a 1-byte read
    data, comm, _ = ph.readData(6, TE_ADDR, 1)
    assert comm == hw.COMM_RX_CORRUPT
    assert data == []


# --------------------------------------------------------------------------------------------------------------
# Fake servo bus for HiwonderMotorsBus.disable_torque / disconnect
# --------------------------------------------------------------------------------------------------------------

NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
REG = {TE_ADDR: "TE", GOAL_ADDR: "Goal", LOCK_ADDR: "Lock", PRESENT_ADDR: "Present", STATUS_ADDR: "Status"}
OVERLOAD = hw.ERRBIT_OVERLOAD
TIMEOUT = (0, hw.COMM_RX_TIMEOUT, 0)  # what the SDK returns for a read that got no reply
FOREVER = 10**9


def timeout_read(servo, n, t):
    return TIMEOUT


@dataclass
class Op:
    t: float  # seconds since the test started (fake clock), taken when the op completed
    kind: str  # read | write | flush | clearPort | closePort | sleep | disable_torque
    id: int | None = None
    reg: str | None = None
    value: int | None = None
    comm: int = hw.COMM_SUCCESS
    error: int = 0

    @property
    def valid_zero(self) -> bool:
        return self.kind == "read" and self.comm == hw.COMM_SUCCESS and self.value == 0


class FakeClock:
    def __init__(self):
        self.t0 = 1000.0
        self.t = self.t0
        self.on_sleep = None

    def monotonic(self) -> float:
        return self.t

    def sleep(self, s: float) -> None:
        self.t += s
        if self.on_sleep is not None:
            self.on_sleep(s)

    @property
    def rel(self) -> float:
        return self.t - self.t0


@dataclass
class FakeServo:
    id: int
    te: int = 1
    lock: int = 1
    present: int = 2048
    goal: int = 2048
    overload: bool = False  # every reply flags Overload; Torque_Enable=0 does not apply until Goal≈Present
    ignore_te0: int = 0  # number of Torque_Enable=0 writes acknowledged OK but not applied
    goal_enables_torque: bool = False
    sticky_overload: bool = False  # Goal=Present does not clear Overload
    extra_status: int = 0  # other Status / error-byte fault bits, e.g. hw.ERRBIT_OVERHEAT
    absent: bool = False  # never replies
    te_read: Callable | None = None  # (servo, n, t) -> (value, comm, error)
    present_read: Callable | None = None  # (servo, n, t) -> (value, comm, error)
    write_reply: Callable | None = None  # (servo, reg, value, n) -> (comm, error, applied) | None
    op_cost: float | None = None
    n_reads: Counter = field(default_factory=Counter)
    n_writes: Counter = field(default_factory=Counter)

    @property
    def status(self) -> int:
        return (OVERLOAD if self.overload else 0) | self.extra_status


class FakeSer:
    def __init__(self, fake: "FakeServoBus"):
        self.fake = fake

    def reset_input_buffer(self) -> None:
        self.fake.flush()


class FakePort:
    def __init__(self, fake: "FakeServoBus"):
        self.fake = fake
        self.is_open = True
        self.is_using = False
        self.ser = FakeSer(fake)
        self.clear_raises: list[BaseException] = []
        self.close_raises: list[BaseException] = []

    def clearPort(self):  # noqa: N802
        self.fake.record("clearPort")
        if self.clear_raises:
            raise self.clear_raises.pop(0)

    def closePort(self):  # noqa: N802
        self.fake.record("closePort")
        if self.close_raises:
            raise self.close_raises.pop(0)
        self.is_open = False


class FakeServoBus:
    """Stands in for bus.packet_handler and bus.port_handler; every op costs fake time and is logged."""

    getTxRxResult = hw.PacketHandler.getTxRxResult  # noqa: N815
    getRxPacketError = hw.PacketHandler.getRxPacketError  # noqa: N815

    def __init__(self, clock: FakeClock, servos: dict[int, FakeServo], op_cost=0.005, timeout_cost=0.06):
        self.clock = clock
        self.servos = servos
        self.op_cost = op_cost
        self.timeout_cost = timeout_cost
        self.log: list[Op] = []
        self.port = FakePort(self)
        self.raises: dict[
            tuple, list[BaseException]
        ] = {}  # (kind, id, reg) -> exceptions raised before the op
        self.flush_raises_first_of_pass: BaseException | None = None
        self.on_write = None  # (op) -> None, called after a write is logged
        self._first_flush_of_pass = True
        clock.on_sleep = self._on_sleep

    def _on_sleep(self, s):
        self._first_flush_of_pass = True
        self.record("sleep", value=s)

    def record(self, kind, **kw) -> Op:
        op = Op(self.clock.rel, kind, **kw)
        self.log.append(op)
        return op

    def _maybe_raise(self, kind, id_, reg):
        exc = self.raises.get((kind, id_, reg))
        if exc:
            raise exc.pop(0)

    def flush(self):
        first, self._first_flush_of_pass = self._first_flush_of_pass, False
        self.record("flush")
        if first and self.flush_raises_first_of_pass is not None:
            raise self.flush_raises_first_of_pass

    def _cost(self, servo, comm):
        if comm != hw.COMM_SUCCESS:
            return self.timeout_cost
        return servo.op_cost if servo.op_cost is not None else self.op_cost

    def _read(self, id_, addr):
        s, reg = self.servos[id_], REG[addr]
        self._maybe_raise("read", id_, reg)
        n = s.n_reads[reg]
        s.n_reads[reg] += 1
        if s.absent:
            value, comm, error = TIMEOUT
        elif reg == "Status":
            value, comm, error = s.status, 0, s.status
        elif reg == "TE":
            value, comm, error = s.te_read(s, n, self.clock.rel) if s.te_read else (s.te, 0, s.status)
        elif reg == "Present":
            value, comm, error = (
                s.present_read(s, n, self.clock.rel) if s.present_read else (s.present, 0, s.status)
            )
        else:
            value, comm, error = 0, 0, s.status
        self.clock.t += self._cost(s, comm)
        self.record("read", id=id_, reg=reg, value=value, comm=comm, error=error)
        return value, comm, error

    def read1ByteData(self, id_, addr):  # noqa: N802
        return self._read(id_, addr)

    def read2ByteData(self, id_, addr):  # noqa: N802
        return self._read(id_, addr)

    def ping(self, id_):
        s = self.servos[id_]
        comm, error = (hw.COMM_RX_TIMEOUT, 0) if s.absent else (hw.COMM_SUCCESS, s.status)
        self.clock.t += self._cost(s, comm)
        self.record("ping", id=id_, comm=comm, error=error)
        return [], comm, error

    def writeReadData(self, id_, addr, length, data):  # noqa: N802
        s, reg = self.servos[id_], REG[addr]
        value = data[0] | (data[1] << 8) if length == 2 else data[0]
        self._maybe_raise("write", id_, reg)
        n = s.n_writes[reg]
        s.n_writes[reg] += 1
        error = s.status
        reply = s.write_reply(s, reg, value, n) if s.write_reply else None
        if reply is None:
            comm, applied = hw.COMM_SUCCESS, True
        else:
            comm, error, applied = reply
        if applied:
            self._apply(s, reg, value)
        self.clock.t += self._cost(s, comm)
        op = self.record("write", id=id_, reg=reg, value=value, comm=comm, error=error)
        if self.on_write is not None:
            self.on_write(op)
        return comm, error

    @staticmethod
    def _apply(s: FakeServo, reg, value):
        if reg == "TE":
            if value == 0 and s.overload:
                return
            if value == 0 and s.ignore_te0 > 0:
                s.ignore_te0 -= 1
                return
            s.te = value
        elif reg == "Lock":
            s.lock = value
        elif reg == "Goal":
            s.goal = value
            if s.overload and not s.sticky_overload and abs(value - s.present) <= 20:
                s.overload = False  # load dropped
            if s.goal_enables_torque:
                s.te = 1

    def ops(self, kind=None, id_=None, reg=None) -> list[Op]:
        return [
            o
            for o in self.log
            if (kind is None or o.kind == kind)
            and (id_ is None or o.id == id_)
            and (reg is None or o.reg == reg)
        ]

    def te_reads(self, id_):
        return self.ops("read", id_, "TE")

    def goal_writes(self, id_=None):
        return self.ops("write", id_, "Goal")

    def kinds(self):
        return [o.kind for o in self.log]


@pytest.fixture
def clock(monkeypatch):
    c = FakeClock()
    # FA-15: patch the driver module's own names, never time.monotonic / time.sleep globally.
    monkeypatch.setattr(hiwonder_mod, "monotonic", c.monotonic, raising=False)
    monkeypatch.setattr(hiwonder_mod, "sleep", c.sleep, raising=False)
    return c


def make_bus(clock, servo_kw: dict[int, dict] | None = None, calibration=None, op_cost=0.005):
    servo_kw = servo_kw or {}
    servos = {i: FakeServo(i, **servo_kw.get(i, {})) for i in range(1, 7)}
    motors = {n: Motor(i + 1, "hx30hm", MotorNormMode.RANGE_M100_100) for i, n in enumerate(NAMES)}
    bus = HiwonderMotorsBus("/dev/fake", motors, calibration)
    fake = FakeServoBus(clock, servos, op_cost=op_cost)
    bus.packet_handler = fake
    bus.port_handler = fake.port
    return bus, fake


def assert_confirmed(fake: FakeServoBus, id_: int):
    """Confirmed off = after the last non-zero/failed read, >= 2 valid 0 reads spanning >= 0.5 s."""
    reads = fake.te_reads(id_)
    last_bad = max((i for i, o in enumerate(reads) if not o.valid_zero), default=-1)
    zeros = reads[last_bad + 1 :]
    assert len(zeros) >= 2, f"id {id_}: TE reads {[(round(o.t, 3), o.value, o.comm) for o in reads]}"
    gap = zeros[-1].t - zeros[0].t
    assert gap >= 0.5 - 1e-9, f"id {id_}: zero reads only {gap:.3f} s apart"


def assert_te0_after_each_bad_read(fake: FakeServoBus, id_: int):
    """Every non-zero or failed TE read of id_ is followed by a Torque_Enable=0 write before the next TE read
    (the last one may be cut off by the deadline)."""
    ops = [o for o in fake.log if o.id == id_]
    bad = [i for i, o in enumerate(ops) if o.kind == "read" and o.reg == "TE" and not o.valid_zero]
    assert len(bad) >= 2, f"id {id_}: fewer than 2 non-zero/failed TE reads happened"
    missing = []
    for i in bad[:-1]:
        nxt = next(j for j in range(i + 1, len(ops)) if ops[j].kind == "read" and ops[j].reg == "TE")
        if not any(o.kind == "write" and o.reg == "TE" and o.value == 0 for o in ops[i + 1 : nxt]):
            missing.append(round(ops[i].t, 3))
    assert not missing, f"id {id_}: no Torque_Enable=0 write after bad reads at t={missing}"


def assert_all_off(fake: FakeServoBus, ids):
    for i in ids:
        assert fake.servos[i].te == 0, f"id {i} ended with Torque_Enable={fake.servos[i].te}"


def assert_names_only(msg: str, *names: str):
    for n in NAMES:
        assert (n in msg) == (n in names), f"{n!r} {'missing from' if n in names else 'wrongly in'}: {msg}"
    assert "Cut servo power before touching the arm." in msg


def calibration_for(names, lo=0, hi=4095):
    return {
        n: MotorCalibration(id=NAMES.index(n) + 1, drive_mode=0, homing_offset=0, range_min=lo, range_max=hi)
        for n in names
    }


# --------------------------------------------------------------------------------------------------------------
# disable_torque: acceptance criteria AC1-AC5 and review findings (C1, R2)
# --------------------------------------------------------------------------------------------------------------


def test_ignored_first_torque_off_is_retried_and_confirmed(clock):
    """AC1: a TE=0 write acknowledged OK but not applied is retried; every motor confirmed by two 0 reads."""
    bus, fake = make_bus(clock, {3: {"ignore_te0": 1}})
    bus.disable_torque()
    assert_all_off(fake, range(1, 7))
    for i in range(1, 7):
        assert_confirmed(fake, i)


def test_overload_flag_on_every_reply_clears_after_goal_and_other_motors_still_written(clock):
    """AC2 + C1: Overload on every reply of motor 1; TE=0 only applies after Goal=Present drops the load."""
    bus, fake = make_bus(clock, {1: {"overload": True, "present": 1500, "goal": 1700}})
    bus.disable_torque()
    assert_all_off(fake, range(1, 7))
    assert {o.id for o in fake.ops("write", reg="TE") if o.value == 0} == set(range(1, 7))
    goals = [o.value for o in fake.goal_writes(1)]
    assert goals and set(goals) == {1500}
    assert_confirmed(fake, 1)


def test_overload_status_on_three_writes_then_success(clock):
    """AC2 as filed: motor 1's first 3 writes reply with Overload and do not apply, then writes succeed."""

    def reply(s, reg, value, n):
        return (hw.COMM_SUCCESS, OVERLOAD, False) if sum(s.n_writes.values()) <= 3 else None

    bus, fake = make_bus(clock, {1: {"write_reply": reply}})
    bus.disable_torque()
    assert_all_off(fake, range(1, 7))
    for i in range(1, 7):
        assert_confirmed(fake, i)


def test_motor_never_off_raises_within_deadline_naming_only_it(clock):
    """AC3: one motor never reads 0 -> RuntimeError within 2.5 s naming only that motor."""
    bus, fake = make_bus(clock, {6: {"ignore_te0": FOREVER}})
    with pytest.raises(RuntimeError) as e:
        bus.disable_torque()
    assert clock.rel <= 2.5
    msg = str(e.value)
    assert_names_only(msg, "gripper")
    assert re.search(r"gripper \(id 6\): ON \(read 1\)", msg), msg
    assert_all_off(fake, range(1, 6))
    assert_te0_after_each_bad_read(fake, 6)


def test_slow_timeouts_still_raise_within_deadline(clock):
    """AC3 / A3: ~60 ms per timed-out op still raises within 2.5 s."""
    bus, fake = make_bus(clock, {6: {"ignore_te0": FOREVER, "te_read": timeout_read}})
    with pytest.raises(RuntimeError) as e:
        bus.disable_torque()
    assert clock.rel <= 2.5
    assert_names_only(str(e.value), "gripper")
    assert "no reply" in str(e.value)
    assert_all_off(fake, range(1, 6))


def test_stale_zero_between_ones_never_confirms(clock):
    """AC5: 0,1,0,1,... never confirms (a non-zero read clears the first-0 stamp)."""
    bus, fake = make_bus(clock, {6: {"ignore_te0": FOREVER, "te_read": lambda s, n, t: (n % 2, 0, 0)}})
    with pytest.raises(RuntimeError) as e:
        bus.disable_torque()
    assert_names_only(str(e.value), "gripper")
    assert_all_off(fake, range(1, 6))


def test_stale_zero_before_real_one_needs_a_fresh_gap(clock):
    """AC5: a stale 0 then the true 1 -> TE=0 re-sent; confirmed only by fresh 0 reads >= 0.5 s apart."""

    def stale_first(s, n, t):
        return (0, 0, 0) if n == 0 else (s.te, 0, 0)

    bus, fake = make_bus(clock, {6: {"ignore_te0": 2, "te_read": stale_first}})
    bus.disable_torque()
    assert fake.servos[6].te == 0
    assert any(not o.valid_zero for o in fake.te_reads(6))
    assert_confirmed(fake, 6)


@pytest.mark.parametrize("failure", ["timeout", "raise"])
def test_present_read_failures_still_rewrite_torque_off_every_pass(clock, failure):
    """AC5 variant: Present_Position reads fail -> no Goal write, Torque_Enable=0 still sent every pass."""
    kw = {"ignore_te0": FOREVER}
    if failure == "timeout":
        kw["present_read"] = timeout_read
    bus, fake = make_bus(clock, {6: kw})
    if failure == "raise":
        fake.raises[("read", 6, "Present")] = [IndexError("list index out of range") for _ in range(100)]
    with pytest.raises(RuntimeError) as e:
        bus.disable_torque()
    assert_names_only(str(e.value), "gripper")
    assert fake.goal_writes(6) == []
    assert_te0_after_each_bad_read(fake, 6)
    assert_all_off(fake, range(1, 6))


def test_disagreeing_present_reads_do_not_write_goal(clock):
    """R2: two Present reads more than 20 ticks apart -> no Goal write."""
    kw = {"ignore_te0": FOREVER, "present_read": lambda s, n, t: (2048 if n % 2 == 0 else 2300, 0, 0)}
    bus, fake = make_bus(clock, {6: kw})
    with pytest.raises(RuntimeError):
        bus.disable_torque()
    assert len(fake.ops("read", 6, "Present")) >= 2
    assert fake.goal_writes(6) == []
    assert_te0_after_each_bad_read(fake, 6)
    assert_all_off(fake, range(1, 6))  # FA-21: healthy motors really ended off


def test_short_reply_index_error_does_not_stop_the_loop(clock):
    """C1: an IndexError from a short reply is caught; the motor is read again and confirmed."""
    bus, fake = make_bus(clock)
    fake.raises[("read", 6, "TE")] = [IndexError("list index out of range")]
    bus.disable_torque()
    assert_all_off(fake, range(1, 7))
    for i in range(1, 7):
        assert_confirmed(fake, i)


# --------------------------------------------------------------------------------------------------------------
# disable_torque: failure-analysis findings (FA-xx)
# --------------------------------------------------------------------------------------------------------------


def test_timeouts_never_confirm_off(clock):
    """FA-01: a timed-out read returns value 0; it must never count as a 0 readback."""
    bus, fake = make_bus(clock, {3: {"ignore_te0": FOREVER, "te_read": timeout_read}})
    with pytest.raises(RuntimeError) as e:
        bus.disable_torque()
    msg = str(e.value)
    assert_names_only(msg, "elbow_flex")
    assert re.search(r"elbow_flex \(id 3\): no reply", msg), msg
    assert_all_off(fake, [1, 2, 4, 5, 6])


def test_failed_present_reads_never_write_goal(clock):
    """FA-02: two failed Present reads (both 0) must not produce Goal_Position=0."""
    bus, fake = make_bus(clock, {6: {"ignore_te0": FOREVER, "present_read": timeout_read}})
    with pytest.raises(RuntimeError):
        bus.disable_torque()
    assert len(fake.ops("read", 6, "Present")) >= 2
    assert fake.goal_writes(6) == []
    assert_te0_after_each_bad_read(fake, 6)
    assert_all_off(fake, range(1, 6))  # FA-21: healthy motors really ended off


def test_goal_written_raw_equals_present_raw(clock):
    """FA-03: Goal is written at the raw layer with the raw Present value."""
    bus, fake = make_bus(clock, {6: {"ignore_te0": 1, "present": 2050}})
    bus.disable_torque()
    assert [o.value for o in fake.goal_writes(6)] == [2050]
    assert_confirmed(fake, 6)


def test_present_with_sign_bit_is_not_written_as_goal(clock):
    """FA-03: raw Present with bit 15 set (sign-magnitude negative) is rejected before the range check."""
    bus, fake = make_bus(clock, {6: {"ignore_te0": FOREVER, "present": 0x8010}})
    with pytest.raises(RuntimeError):
        bus.disable_torque()
    assert len(fake.ops("read", 6, "Present")) >= 2
    assert fake.goal_writes(6) == []
    assert_all_off(fake, range(1, 6))  # FA-21: healthy motors really ended off


@pytest.mark.parametrize(
    "motors, expected",
    [(6, {6}), ("gripper", {6}), (["gripper", 1], {1, 6}), ([], set())],
    ids=["int", "str", "mixed-list", "empty"],
)
def test_disable_torque_respects_motor_subset(clock, motors, expected):
    """FA-04: only the named motors get I/O; [] does no I/O."""
    bus, fake = make_bus(clock)
    bus.disable_torque(motors)
    assert {o.id for o in fake.log if o.id is not None} == expected
    for i in expected:
        assert_confirmed(fake, i)
    if not expected:
        assert [o for o in fake.log if o.kind in ("read", "write", "flush")] == []


def test_no_goal_write_when_torque_read_failed(clock):
    """FA-05: Goal=Present only after a TE read that succeeded and returned non-zero."""
    bus, fake = make_bus(clock, {6: {"ignore_te0": FOREVER, "te_read": timeout_read}})
    with pytest.raises(RuntimeError):
        bus.disable_torque()
    assert len(fake.te_reads(6)) >= 3
    assert fake.goal_writes(6) == []
    assert_te0_after_each_bad_read(fake, 6)
    assert_all_off(fake, range(1, 6))  # FA-21: healthy motors really ended off


def test_torque_off_follows_goal_even_past_deadline(clock):
    """FA-05: once Goal is written, its TE=0 and Lock=0 follow-ups are sent even past the deadline."""
    bus, fake = make_bus(clock, {6: {"ignore_te0": FOREVER}})

    def jump_past_deadline(op):
        if op.reg == "Goal":
            clock.t += 3.0

    fake.on_write = jump_past_deadline
    with pytest.raises(RuntimeError):
        bus.disable_torque()
    i = next(i for i, o in enumerate(fake.log) if o.reg == "Goal")
    after = [(o.kind, o.id, o.reg, o.value) for o in fake.log[i + 1 : i + 3]]
    assert after == [("write", 6, "TE", 0), ("write", 6, "Lock", 0)]
    assert fake.ops("read")[-1].t < fake.log[i].t  # nothing is read after the deadline passed
    assert_all_off(fake, range(1, 6))  # FA-21: healthy motors really ended off


def test_goal_write_that_reenables_torque_is_followed_by_torque_off(clock):
    """FA-05: if a Goal write turns torque back on (unverified on HX-30HM), the TE=0 after it turns it off."""
    bus, fake = make_bus(clock, {6: {"ignore_te0": 1, "goal_enables_torque": True}})
    bus.disable_torque()
    assert fake.goal_writes(6)
    assert fake.servos[6].te == 0
    assert_confirmed(fake, 6)


def test_transient_serial_exception_does_not_poison_later_ops(clock):
    """FA-06: an exception mid-packet leaves is_using=True; later ops must not all return COMM_PORT_BUSY.
    Real PacketHandler/PortHandler over a byte-level fake line."""
    memory = {i: {TE_ADDR: 1, PRESENT_ADDR: 0x00, PRESENT_ADDR + 1: 0x08} for i in range(1, 7)}
    line = ByteServoLine(memory)
    line.raise_on_read = [serial.SerialException("device reports readiness to read but returned no data")]
    line.on_write = lambda data: setattr(clock, "t", clock.t + 0.002)
    motors = {n: Motor(i + 1, "hx30hm", MotorNormMode.RANGE_M100_100) for i, n in enumerate(NAMES)}
    bus = HiwonderMotorsBus("/dev/fake", motors)
    bus.port_handler.ser = line
    bus.port_handler.is_open = True
    bus.port_handler.tx_time_per_byte = (1000.0 / bus.port_handler.baudrate) * 10.0
    results = []
    real_tx = bus.packet_handler.txPacket

    def tx(packet):
        results.append(real_tx(packet))
        return results[-1]

    bus.packet_handler.txPacket = tx
    bus.disable_torque()
    assert all(memory[i][TE_ADDR] == 0 for i in range(1, 7))
    assert hw.COMM_PORT_BUSY not in results
    assert bus.port_handler.is_using is False


def test_error_message_distinguishes_pending_on_and_no_reply(clock):
    """FA-07, FA-29: per-motor state in the error: ON (read 1) / no reply / pending."""

    def late_zero(s, n, t):
        return (1, 0, 0) if t < 1.7 else (0, 0, 0)

    bus, fake = make_bus(
        clock,
        {
            4: {"ignore_te0": FOREVER},
            5: {"ignore_te0": FOREVER, "te_read": timeout_read},
            6: {"ignore_te0": FOREVER, "te_read": late_zero},
        },
    )
    with pytest.raises(RuntimeError) as e:
        bus.disable_torque()
    assert clock.rel <= 2.5
    msg = str(e.value)
    assert_names_only(msg, "wrist_flex", "wrist_roll", "gripper")
    assert re.search(r"wrist_flex \(id 4\): ON \(read 1\)", msg), msg
    assert re.search(r"wrist_roll \(id 5\): no reply", msg), msg
    assert re.search(r"gripper \(id 6\): pending", msg), msg
    assert_all_off(fake, [1, 2, 3])


def test_flush_error_does_not_skip_other_motors(clock):
    """FA-09: an input-flush error for motor 1 does not stop motors 2-6 from being verified."""
    bus, fake = make_bus(clock)
    fake.flush_raises_first_of_pass = OSError(5, "Input/output error")
    with pytest.raises(RuntimeError) as e:
        bus.disable_torque()
    assert_names_only(str(e.value), "shoulder_pan")
    assert "OSError" in str(e.value)
    for i in range(2, 7):
        assert_confirmed(fake, i)


def test_initial_torque_off_reaches_every_motor_even_when_slow(clock):
    """FA-17: step 1 (TE=0, Lock=0 to every motor) is unconditional even when it overruns the deadline."""
    bus, fake = make_bus(clock, op_cost=0.3)
    with pytest.raises(RuntimeError) as e:
        bus.disable_torque()
    first_read = next((i for i, o in enumerate(fake.log) if o.kind == "read"), len(fake.log))
    te0 = [o.id for o in fake.log[:first_read] if o.kind == "write" and o.reg == "TE" and o.value == 0]
    assert te0 == [1, 2, 3, 4, 5, 6]
    assert_names_only(str(e.value), *NAMES)


def test_one_warning_per_affected_motor(clock, caplog):
    """FA-19: one WARNING per affected motor per call, not one per failed op."""
    kw = {"ignore_te0": FOREVER, "te_read": timeout_read}
    bus, fake = make_bus(clock, {5: kw, 6: dict(kw)})
    caplog.set_level(logging.DEBUG, logger=hiwonder_mod.logger.name)
    with pytest.raises(RuntimeError):
        bus.disable_torque()
    warnings = [
        r for r in caplog.records if r.levelno == logging.WARNING and r.name == hiwonder_mod.logger.name
    ]
    assert len(warnings) == 2, [r.getMessage() for r in warnings]
    assert {"wrist_roll", "gripper"} == {n for n in NAMES for r in warnings if n in r.getMessage()}


def test_goal_guard_range_calibrated_and_uncalibrated(clock):
    """FA-23: calibrated range rejects Present outside it; a motor without calibration uses 0-4095, no KeyError."""
    bus, fake = make_bus(
        clock,
        {6: {"ignore_te0": FOREVER, "present": 3500}},
        calibration=calibration_for(["gripper"], 1000, 3000),
    )
    with pytest.raises(RuntimeError):
        bus.disable_torque()
    assert len(fake.ops("read", 6, "Present")) >= 2
    assert fake.goal_writes(6) == []
    assert_all_off(fake, range(1, 6))  # FA-21: healthy motors really ended off

    bus, fake = make_bus(
        clock, {6: {"ignore_te0": 1, "present": 3500}}, calibration=calibration_for(NAMES[:5])
    )
    bus.disable_torque()
    assert [o.value for o in fake.goal_writes(6)] == [3500]


def test_fake_clock_is_module_local(clock):
    """FA-15: the driver uses its module-level monotonic/sleep; time.monotonic stays the real function."""
    real = time.monotonic
    seen = []
    bus, fake = make_bus(clock, {6: {"ignore_te0": FOREVER}})
    fake.on_write = lambda op: seen.append(time.monotonic is real)
    with pytest.raises(RuntimeError):
        bus.disable_torque()
    assert seen and all(seen)
    assert hiwonder_mod.monotonic == clock.monotonic
    assert fake.ops("sleep")
    assert clock.rel <= 2.5


def test_raise_tests_prove_healthy_motors_reached_zero(clock, caplog):
    """FA-21: a programming-type error inside an op is caught, logged with a traceback at DEBUG, and the
    motor is still verified afterwards; healthy motors end at 0."""
    bus, fake = make_bus(clock, {6: {"ignore_te0": FOREVER}})
    fake.raises[("read", 2, "TE")] = [TypeError("injected")]
    caplog.set_level(logging.DEBUG, logger=hiwonder_mod.logger.name)
    with pytest.raises(RuntimeError) as e:
        bus.disable_torque()
    assert_names_only(str(e.value), "gripper")
    assert_all_off(fake, range(1, 6))
    assert_confirmed(fake, 2)
    debug = [
        r for r in caplog.records if r.levelno == logging.DEBUG and r.exc_info and r.exc_info[0] is TypeError
    ]
    assert debug


def test_flush_precedes_every_loop_read(clock):
    """FA-25: the input buffer is flushed immediately before every TE and Present read."""
    bus, fake = make_bus(clock, {6: {"ignore_te0": 1}})
    bus.disable_torque()
    reads = [i for i, o in enumerate(fake.log) if o.kind == "read"]
    assert fake.ops("read", 6, "Present")
    assert all(fake.log[i - 1].kind == "flush" for i in reads)


# --------------------------------------------------------------------------------------------------------------
# disconnect(): AC4, A4, R3 regression contract, FA-09
# --------------------------------------------------------------------------------------------------------------


def disconnect_bus(clock, monkeypatch, effect: BaseException | None = None):
    """Bus whose disable_torque is replaced by a recorder (optionally raising `effect`)."""
    bus, fake = make_bus(clock)
    seen = []

    def disable_torque(motors=None, num_retry=0):
        fake.record("disable_torque", value=num_retry)
        seen.append(fake.port.is_using)
        if effect is not None:
            raise effect

    monkeypatch.setattr(bus, "disable_torque", disable_torque)
    return bus, fake, seen


def test_disconnect_runs_verified_torque_off_and_closes(clock):
    bus, fake = make_bus(clock, {6: {"ignore_te0": 1}})
    bus.disconnect()
    assert_all_off(fake, range(1, 7))
    assert_confirmed(fake, 6)
    assert fake.kinds().count("closePort") == 1


def test_disconnect_closes_port_then_reraises_torque_error(clock, monkeypatch):
    """AC4."""
    err = RuntimeError("Torque-off not confirmed")
    bus, fake, _ = disconnect_bus(clock, monkeypatch, err)
    with pytest.raises(RuntimeError) as e:
        bus.disconnect()
    assert e.value is err
    k = fake.kinds()
    assert k.count("closePort") == 1
    assert k.index("disable_torque") < k.index("closePort")


def test_torque_error_wins_over_close_error(clock, monkeypatch, caplog):
    """AC4: if closePort also raises, the torque error is raised and the close error is logged."""
    err = RuntimeError("Torque-off not confirmed")
    bus, fake, _ = disconnect_bus(clock, monkeypatch, err)
    fake.port.close_raises = [OSError("close failed")]
    with pytest.raises(RuntimeError) as e:
        bus.disconnect()
    assert e.value is err
    assert any(r.levelno == logging.ERROR and "close failed" in r.getMessage() for r in caplog.records)


def test_keyboard_interrupt_during_torque_off_still_closes_port(clock, monkeypatch):
    """A4."""
    bus, fake, _ = disconnect_bus(clock, monkeypatch, KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        bus.disconnect()
    assert fake.kinds().count("closePort") == 1


def test_close_error_raised_when_torque_off_succeeded(clock, monkeypatch):
    bus, fake, _ = disconnect_bus(clock, monkeypatch)
    fake.port.close_raises = [OSError("close failed")]
    with pytest.raises(OSError, match="close failed"):
        bus.disconnect()


def test_disconnect_without_torque_off_only_closes(clock, monkeypatch, caplog):
    """R3-1 (regression contract; green against the inherited method too)."""
    caplog.set_level(logging.DEBUG)
    bus, fake, seen = disconnect_bus(clock, monkeypatch)
    bus.disconnect(disable_torque=False)
    assert fake.kinds() == ["closePort"]
    assert seen == []
    assert "disconnected" in caplog.text


def test_disconnect_when_not_connected_touches_nothing(clock, monkeypatch):
    """R3-2."""
    bus, fake, seen = disconnect_bus(clock, monkeypatch)
    fake.port.is_open = False
    with pytest.raises(DeviceNotConnectedError):
        bus.disconnect()
    assert fake.log == []
    assert seen == []


def test_disconnect_happy_path_order(clock, monkeypatch):
    """R3-3: clearPort, disable_torque(num_retry=5) with is_using reset, closePort once, no raise."""
    bus, fake, seen = disconnect_bus(clock, monkeypatch)
    fake.port.is_using = True
    bus.disconnect()
    assert fake.kinds() == ["clearPort", "disable_torque", "closePort"]
    assert fake.ops("disable_torque")[0].value == 5
    assert seen == [False]


def test_torque_disabled_body_skipped_when_torque_off_raises(clock, monkeypatch):
    """R3-4: torque_disabled() runs no body and no enable_torque when disable_torque raises."""
    bus, fake, _ = disconnect_bus(clock, monkeypatch, RuntimeError("Torque-off not confirmed"))
    enabled, body = [], []
    monkeypatch.setattr(bus, "enable_torque", lambda *a, **k: enabled.append(1))
    with pytest.raises(RuntimeError), bus.torque_disabled():
        body.append(1)
    assert body == []
    assert enabled == []


def test_clearport_error_still_runs_torque_off(clock, monkeypatch):
    """FA-09: a clearPort error does not skip torque-off; the port is still closed once."""
    bus, fake, seen = disconnect_bus(clock, monkeypatch)
    fake.port.is_using = True
    fake.port.clear_raises = [OSError("tcdrain failed")]
    bus.disconnect()
    assert seen == [False]
    assert fake.kinds().count("closePort") == 1


# --------------------------------------------------------------------------------------------------------------
# Overload latch on connect (LA-01..LA-14): a stall cut off by torque-off leaves Status 0x20 set with torque off;
# every reply then carries the flag, so ping() reports the motor missing and reads raise.
# --------------------------------------------------------------------------------------------------------------

# Hardware 2026-10-08: gripper latched with TE 0 at 1491; a Goal_Position write turns torque on (LESSONS 12).
LATCHED = {"te": 0, "overload": True, "goal_enables_torque": True, "present": 1491, "goal": 1456}


def latch_warnings(caplog):
    return [r for r in caplog.records if r.levelno == logging.WARNING and "Overload latched" in r.getMessage()]


def test_latched_gripper_is_cleared_and_connect_succeeds(clock, caplog):
    """LA-01, LA-02, LA-11, LA-12: Goal=Present clears the latch, then verified torque-off; pings all succeed."""
    bus, fake = make_bus(clock, {6: dict(LATCHED)})
    bus._handshake()

    s = fake.servos[6]
    assert not s.overload
    assert s.te == 0
    goals = fake.goal_writes()
    assert [(o.id, o.value) for o in goals] == [(6, 1491)]
    goal_at = fake.log.index(goals[0])
    assert any(o.kind == "write" and o.reg == "TE" and o.value == 0 and o.id == 6 for o in fake.log[goal_at:])
    assert_confirmed(fake, 6)
    assert {o.id for o in fake.ops("write")} == {6}
    pings = fake.ops("ping")
    assert sorted(o.id for o in pings) == list(range(1, 7))
    assert all(o.comm == hw.COMM_SUCCESS and o.error == 0 for o in pings)
    status_reads = [i for i, o in enumerate(fake.log) if o.kind == "read" and o.reg == "Status"]
    assert status_reads
    assert all(fake.log[i - 1].kind == "flush" for i in status_reads)
    warns = latch_warnings(caplog)
    assert len(warns) == 1
    msg = warns[0].getMessage()
    assert "gripper" in msg and "torque briefly on" in msg


def test_handshake_without_latch_clear_reports_gripper_missing(clock, monkeypatch):
    """Negative control: the inherited handshake fails on a latched gripper with a misleading 'Missing' error."""
    bus, fake = make_bus(clock, {6: dict(LATCHED)})
    monkeypatch.setattr(bus, "_clear_overload_latches", lambda: None, raising=False)
    with pytest.raises(RuntimeError, match=r"Missing motor IDs:\n  - 6"):
        bus._handshake()


def test_healthy_handshake_writes_nothing(clock):
    """LA-09: no latch, no writes (healthy follower and leader connects are unchanged)."""
    bus, fake = make_bus(clock)
    bus._handshake()
    assert fake.ops("write") == []
    assert {o.reg for o in fake.ops("read")} <= {"Status"}


@pytest.mark.parametrize(
    "bit", [hw.ERRBIT_OVERHEAT, hw.ERRBIT_VOLTAGE, hw.ERRBIT_ANGLE, hw.ERRBIT_CURRENT, hw.ERRBIT_SENSOR, 0x40, 0x80]
)
def test_other_fault_bit_is_never_written(clock, bit):
    """LA-03: Overload plus any other fault bit gets no torque-on write; the error names the motor."""
    bus, fake = make_bus(clock, {6: {**LATCHED, "extra_status": bit}})
    with pytest.raises(RuntimeError) as e:
        bus._handshake()
    assert fake.ops("write") == []
    msg = str(e.value)
    assert "gripper" in msg and "shoulder_pan" not in msg
    assert "power-cycle" in msg


def disagreeing(s, n, t):
    return (1491 if n % 2 == 0 else 1600), 0, s.status


@pytest.mark.parametrize(
    "servo_kw, calibration",
    [
        ({"present": 0x8000 | 5}, None),
        ({"present_read": disagreeing}, None),
        ({"present": 1000}, calibration_for(["gripper"], lo=1416, hi=2100)),
        ({"present_read": timeout_read}, None),
    ],
    ids=["sign-bit", "reads-disagree", "outside-calibration", "read-fails"],
)
def test_invalid_present_gets_no_goal_write(clock, servo_kw, calibration):
    """LA-04: no trustworthy Present_Position, no Goal write; torque confirmed off and the error names the motor."""
    bus, fake = make_bus(clock, {6: {**LATCHED, **servo_kw}}, calibration=calibration)
    with pytest.raises(RuntimeError) as e:
        bus._handshake()
    assert fake.goal_writes() == []
    assert fake.servos[6].te == 0
    assert_confirmed(fake, 6)
    msg = str(e.value)
    assert "gripper" in msg and "power-cycle" in msg


def test_latch_not_cleared_raises_after_torque_off(clock):
    """LA-05, LA-14: Goal written, latch stays; torque-off still confirmed, then a bounded 'still latched' error."""
    bus, fake = make_bus(clock, {6: {**LATCHED, "sticky_overload": True}})

    def te_zero_applies(op):
        if op.id == 6 and op.reg == "TE" and op.value == 0:
            fake.servos[6].te = 0

    fake.on_write = te_zero_applies
    with pytest.raises(RuntimeError) as e:
        bus._handshake()
    assert [o.value for o in fake.goal_writes(6)][:1] == [1491]
    assert fake.servos[6].te == 0
    assert_confirmed(fake, 6)
    msg = str(e.value)
    assert "gripper" in msg and "still latched" in msg and "power-cycle" in msg
    assert clock.rel <= 2.5 + 2.5 + 0.5


def test_unconfirmed_torque_off_after_clear_propagates(clock):
    """LA-06: if torque-off after the Goal write cannot be confirmed, that error is raised."""
    bus, fake = make_bus(clock, {6: {**LATCHED, "sticky_overload": True}})
    with pytest.raises(RuntimeError) as e:
        bus._handshake()
    msg = str(e.value)
    assert "Torque-off not confirmed" in msg
    assert_names_only(msg, "gripper")


def test_interrupt_after_goal_write_still_turns_torque_off(clock):
    """LA-07: Ctrl-C right after the Goal write (torque now on) still runs the verified torque-off."""
    bus, fake = make_bus(clock, {6: dict(LATCHED)})
    fired = []

    def interrupt_once(op):
        if op.reg == "Goal" and not fired:
            fired.append(op)
            fake.port.is_using = True  # the SDK's port lock, left set by an interrupt mid-packet
            raise KeyboardInterrupt

    fake.on_write = interrupt_once
    with pytest.raises(KeyboardInterrupt):
        bus._handshake()
    assert fired
    assert fake.port.is_using is False
    assert fake.servos[6].te == 0
    assert_confirmed(fake, 6)


def test_absent_motor_is_untouched(clock):
    """LA-08: a motor that never replies gets no writes and is still reported missing."""
    bus, fake = make_bus(clock, {6: {"absent": True}})
    with pytest.raises(RuntimeError, match=r"Missing motor IDs:\n  - 6"):
        bus._handshake()
    assert fake.ops("write") == []


def test_two_latched_motors_both_cleared(clock, caplog):
    """LA-10."""
    bus, fake = make_bus(clock, {5: {**LATCHED, "present": 2000, "goal": 2100}, 6: dict(LATCHED)})
    bus._handshake()
    assert sorted((o.id, o.value) for o in fake.goal_writes()) == [(5, 2000), (6, 1491)]
    for i in (5, 6):
        assert not fake.servos[i].overload
        assert fake.servos[i].te == 0
        assert_confirmed(fake, i)
    assert len(latch_warnings(caplog)) == 2
