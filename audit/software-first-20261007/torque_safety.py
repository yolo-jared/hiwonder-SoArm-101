"""Readback-driven torque-off for HX-30HM buses.

Observed 2026-10-07 (gripper run G): after a ~2 s stall the servo flags Overload in every reply status. LeRobot's
bus.write() then raises whether or not the write took effect (a Goal write raised but moved the jaw; four
Torque_Enable=0 writes raised and did NOT take effect). So success is judged only by reading Torque_Enable back.
"""

from __future__ import annotations

import time


def torque_off_verified(bus, names, timeout=5.0, pause=0.3):
    """Command Goal=Present (drops stall load), then retry Torque_Enable=0 until every readback is 0.

    Returns (verified, {name: last readback or None}). Never raises.
    """
    for n in names:
        try:
            bus.write("Goal_Position", n, bus.read("Present_Position", n, normalize=False), normalize=False)
        except Exception:  # noqa: BLE001
            pass
    deadline = time.monotonic() + timeout
    rb: dict = {}
    while True:
        for n in names:
            if rb.get(n) == 0:
                continue
            try:
                bus.write("Torque_Enable", n, 0, normalize=False)
            except Exception:  # noqa: BLE001
                pass
            try:
                rb[n] = bus.read("Torque_Enable", n, normalize=False)
            except Exception:  # noqa: BLE001
                rb[n] = None
        if all(rb.get(n) == 0 for n in names):
            return True, rb
        if time.monotonic() >= deadline:
            return False, rb
        time.sleep(pause)
