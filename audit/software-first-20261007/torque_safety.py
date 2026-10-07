"""Readback-driven torque-off for HX-30HM buses.

Observed 2026-10-07 (gripper run G): after a ~2 s stall the servo flags Overload in every reply status. LeRobot's
bus.write() then raises whether or not the write took effect (a Goal write raised but moved the jaw; four
Torque_Enable=0 writes raised and did NOT take effect). So success is judged only by reading Torque_Enable back.
Run T1 (15:11): this helper reported off, torque was found ON 83 s later with no other writer. A late reply can be
parsed as register data (bundled SDK has no LEN check, FINDINGS F3), so each read is preceded by an input flush and
"off" requires two consecutive 0 readbacks CONFIRM_GAP apart. Every attempt is logged.
"""

from __future__ import annotations

import time

CONFIRM_GAP = 0.5


def _flush(bus):
    try:
        bus.port_handler.ser.reset_input_buffer()
    except Exception:  # noqa: BLE001
        pass


def _read(bus, reg, n):
    _flush(bus)
    try:
        return bus.read(reg, n, normalize=False)
    except Exception as e:  # noqa: BLE001
        return f"ERR {str(e)[-50:]}"


def torque_off_verified(bus, names, timeout=6.0, pause=0.3, log=None):
    """Goal=Present (drops stall load), then retry Torque_Enable=0 until each joint reads 0 twice, CONFIRM_GAP apart.

    Returns (verified, {name: last readback}). Never raises. `log` (list) receives one dict per attempt.
    """
    log = log if log is not None else []
    for n in names:
        pos = _read(bus, "Present_Position", n)
        err = None
        if isinstance(pos, int):
            try:
                bus.write("Goal_Position", n, pos, normalize=False)
            except Exception as e:  # noqa: BLE001
                err = str(e)[-50:]
        log.append({"wall": time.strftime("%H:%M:%S"), "t": time.monotonic(), "joint": n, "op": "goal=present",
                    "value": pos, "write_error": err})
    deadline = time.monotonic() + timeout
    zero_since: dict = {}
    rb: dict = {}
    while True:
        for n in names:
            if n in zero_since and time.monotonic() - zero_since[n] >= CONFIRM_GAP:
                continue
            err = None
            if n not in zero_since:
                try:
                    bus.write("Torque_Enable", n, 0, normalize=False)
                except Exception as e:  # noqa: BLE001
                    err = str(e)[-50:]
            rb[n] = _read(bus, "Torque_Enable", n)
            status = _read(bus, "Status", n)
            log.append({"wall": time.strftime("%H:%M:%S"), "t": time.monotonic(), "joint": n, "op": "torque=0",
                        "write_error": err, "readback": rb[n], "status": status})
            if rb[n] == 0:
                zero_since.setdefault(n, time.monotonic())
            else:
                zero_since.pop(n, None)
        if all(n in zero_since and time.monotonic() - zero_since[n] >= CONFIRM_GAP for n in names):
            return True, rb
        if time.monotonic() >= deadline:
            return False, rb
        time.sleep(min(pause, CONFIRM_GAP))
