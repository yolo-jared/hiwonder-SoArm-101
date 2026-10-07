"""Memory-only HX-30HM bus emulator for offline audits.

Written from the HX-30HM workbook (Register List V3.7) and the shared
FF FF / ID / LEN / INST / PARAMS / ~SUM framing. It does NOT import or reuse
any lerobot or hiwonder_sdk code, so a shared mistake in the driver cannot be
copied into the expected values. It never opens a real device.
"""

from __future__ import annotations

import time

# Workbook register map: address -> (bytes, name). Only documented addresses.
WORKBOOK = {
    0: (1, "Firmware major version"),
    1: (1, "Firmware minor version"),
    3: (1, "Servo major version"),
    4: (1, "Servo minor version"),
    5: (1, "ID"),
    6: (1, "Baud rate"),
    8: (1, "Response status level"),
    13: (1, "Overtemperature threshold"),
    14: (1, "Overvoltage threshold"),
    15: (1, "Undervoltage threshold"),
    16: (2, "Maximum torque limit"),
    19: (1, "Protection control"),
    20: (1, "LED alarm condition"),
    21: (1, "Position loop P gain"),
    22: (1, "Position loop D gain"),
    23: (1, "Position loop I gain"),
    24: (2, "Minimum startup torque"),
    26: (1, "Clockwise deadband"),
    27: (1, "Counterclockwise deadband"),
    28: (2, "Overcurrent threshold (1 mA)"),
    31: (2, "Position calibration (bit11 sign)"),
    33: (1, "Operating mode"),
    34: (1, "Overload torque limit"),
    35: (1, "Overload trigger time"),
    36: (1, "Overload torque threshold"),
    37: (1, "Speed loop P gain"),
    38: (1, "Overcurrent protection time"),
    39: (1, "Speed loop I gain"),
    40: (1, "Torque switch"),
    41: (1, "Acceleration"),
    42: (2, "Target position (bit15 sign)"),
    44: (2, "PWM open-loop speed"),
    46: (2, "Motion speed"),
    55: (1, "Lock flag"),
    56: (2, "Current position"),
    58: (2, "Current speed"),
    60: (2, "Current load (bit10 sign)"),
    62: (1, "Current voltage"),
    63: (1, "Current temperature"),
    64: (1, "Async write flag"),
    65: (1, "Servo status"),
    66: (1, "Moving flag"),
    69: (2, "Present current"),
}

INST_NAMES = {1: "PING", 2: "READ", 3: "WRITE", 4: "REG_WRITE", 5: "ACTION", 6: "RESET", 130: "SYNC_READ", 131: "SYNC_WRITE"}


def checksum(body: list[int]) -> int:
    """~(sum of ID..last param) & 0xFF, per the protocol framing."""
    return (~sum(body)) & 0xFF


def frame(id_: int, inst_or_err: int, params: list[int]) -> list[int]:
    body = [id_, len(params) + 2, inst_or_err, *params]
    return [0xFF, 0xFF, *body, checksum(body)]


def sm_encode(value: int, sign_bit: int) -> int:
    return (abs(value) | (1 << sign_bit)) if value < 0 else value


def sm_decode(raw: int, sign_bit: int) -> int:
    return -(raw & ~(1 << sign_bit)) if raw & (1 << sign_bit) else raw


class RealPortForbidden(RuntimeError):
    pass


class FakeHXBus:
    """Six emulated servos sharing one memory-only 'wire'."""

    def __init__(self, ids=(1, 2, 3, 4, 5, 6)):
        self.mem = {i: bytearray(128) for i in ids}
        for i in ids:
            m = self.mem[i]
            m[0], m[1] = 3, 15  # firmware 3.15 as read from the real follower
            m[3], m[4] = 9, 3  # servo version 9/3 -> little-endian word 777
            m[5] = i
            m[8] = 1  # respond to all instructions (workbook default)
            m[21], m[22], m[23] = 32, 32, 0  # workbook defaults P32 D32 I0
            m[24] = 16  # minimum startup torque 16
            m[62] = 123
        self.packets: list[dict] = []  # every host->servo packet, decoded independently
        self.hold_next_read_reply = False
        self._held: list[int] = []
        self.on_read = None  # optional callback(id, addr) before a read is served

    # -- register helpers (little-endian, workbook row 4) --
    def put(self, id_, addr, nbytes, value):
        for k in range(nbytes):
            self.mem[id_][addr + k] = (value >> (8 * k)) & 0xFF

    def get(self, id_, addr, nbytes):
        return sum(self.mem[id_][addr + k] << (8 * k) for k in range(nbytes))

    # -- wire --
    def handle(self, data: list[int]) -> list[int]:
        out: list[int] = []
        if self._held:
            out += self._held  # a late reply from the previous exchange lands now
            self._held = []
        i = 0
        while i < len(data):
            assert data[i] == 0xFF and data[i + 1] == 0xFF, f"bad header at {i}: {data}"
            id_, ln, inst = data[i + 2], data[i + 3], data[i + 4]
            params = list(data[i + 5 : i + 3 + ln])
            cks = data[i + 3 + ln]
            ok = cks == checksum([id_, ln, inst, *params])
            rec = {"t": time.monotonic(), "id": id_, "inst": INST_NAMES.get(inst, inst), "params": params,
                   "bytes": list(data[i : i + 4 + ln]), "checksum_ok": ok}
            self.packets.append(rec)
            i += 4 + ln
            if not ok:
                continue
            reply = self._exec(id_, inst, params, rec)
            if reply and inst == 2 and self.hold_next_read_reply:
                self.hold_next_read_reply = False
                self._held = reply
                rec["reply_delayed"] = True
                continue
            out += reply
        return out

    def _exec(self, id_, inst, params, rec):
        if inst == 1:
            return frame(id_, 0, []) if id_ in self.mem else []
        if inst == 2:
            addr, n = params
            if id_ not in self.mem:
                return []
            if self.on_read:
                self.on_read(id_, addr)
            return frame(id_, 0, list(self.mem[id_][addr : addr + n]))
        if inst == 3:
            addr, data = params[0], params[1:]
            rec["writes"] = [self._record_write(id_, addr, data)]
            if id_ in self.mem:
                self.mem[id_][addr : addr + len(data)] = bytes(data)
                return frame(id_, 0, []) if self.mem[id_][8] else []
            return []
        if inst == 130:
            addr, n, ids = params[0], params[1], params[2:]
            out = []
            for j in ids:
                if j in self.mem:
                    if self.on_read:
                        self.on_read(j, addr)
                    out += frame(j, 0, list(self.mem[j][addr : addr + n]))
            return out
        if inst == 131:
            addr, n = params[0], params[1]
            rest = params[2:]
            rec["writes"] = []
            for k in range(0, len(rest), n + 1):
                j, data = rest[k], rest[k + 1 : k + 1 + n]
                rec["writes"].append(self._record_write(j, addr, data))
                if j in self.mem:
                    self.mem[j][addr : addr + n] = bytes(data)
            return []
        rec["unsupported"] = True
        return []

    @staticmethod
    def _record_write(id_, addr, data):
        value = sum(b << (8 * k) for k, b in enumerate(data))
        doc = WORKBOOK.get(addr)
        return {"id": id_, "addr": addr, "nbytes": len(data), "raw": value,
                "workbook": doc[1] if doc else None,
                "documented_width": (doc[0] == len(data)) if doc else None}


class FakeSerial:
    """pyserial-shaped object bound to one FakeHXBus. Refuses non-fake ports."""

    bus: FakeHXBus | None = None
    opened: list[str] = []

    def __init__(self, port=None, baudrate=None, bytesize=None, timeout=None, **_):
        if not str(port).startswith("fake://"):
            raise RealPortForbidden(f"refusing to open non-fake port {port!r}")
        assert FakeSerial.bus is not None
        FakeSerial.opened.append(port)
        self.port, self.baudrate, self.is_open = port, baudrate, True
        self._rx: list[int] = []

    def write(self, data):
        data = list(data)
        self._rx += FakeSerial.bus.handle(data)
        return len(data)

    def read(self, n):
        chunk, self._rx = self._rx[:n], self._rx[n:]
        return bytes(chunk)

    @property
    def in_waiting(self):
        return len(self._rx)

    def flush(self):  # pyserial flush(): waits for OUTPUT to drain; input untouched
        pass

    def deliver_held(self):
        """Late reply arrives in the idle gap BEFORE the host's next transmit."""
        self._rx += FakeSerial.bus._held
        FakeSerial.bus._held = []

    def reset_input_buffer(self):
        self._rx = []

    def close(self):
        self.is_open = False
