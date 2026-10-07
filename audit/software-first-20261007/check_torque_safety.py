"""Fake-bus checks for torque_safety.torque_off_verified, with negative controls.

    python -I check_torque_safety.py      (exit 0 = all checks behaved as expected)
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "src"))
sys.path.insert(0, str(HERE))

import serial  # noqa: E402

import fake_hx_bus as fk  # noqa: E402
from torque_safety import torque_off_verified  # noqa: E402

OVERLOAD = 0x20


def make(refuse_torque_off: int):
    """Gripper with torque on; first N Torque_Enable=0 writes reply Overload and are NOT applied (run G behaviour)."""
    serial.Serial = fk.FakeSerial
    fake = fk.FakeHXBus()
    fk.FakeSerial.bus = fake
    fake.mem[6][40] = 1
    left = {"n": refuse_torque_off}

    def on_write(id_, addr, data):
        if addr == 40 and data == [0] and left["n"] > 0:
            left["n"] -= 1
            return False, OVERLOAD
        return True, OVERLOAD if left["n"] > 0 else 0  # other writes apply but still report overload

    fake.on_write = on_write
    from lerobot.motors.hiwonder import HiwonderMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorNormMode

    bus = HiwonderMotorsBus(port="fake://t", motors={"gripper": Motor(6, "hx30hm", MotorNormMode.DEGREES)})
    bus.port_handler.openPort()
    return fake, bus


results = []

# Negative control: the old cleanup path (bus.write with retries) fails exactly like run G.
fake, bus = make(refuse_torque_off=10)
try:
    bus.write("Torque_Enable", "gripper", 0, normalize=False, num_retry=3)
    old = "no raise"
except RuntimeError as e:
    old = "raised: " + str(e)[:60]
results.append(("old path raises and leaves torque on", old.startswith("raised") and fake.mem[6][40] == 1, old))

# Helper recovers when the servo refuses the first 6 torque-off writes.
fake, bus = make(refuse_torque_off=6)
ok, rb = torque_off_verified(bus, ["gripper"], timeout=5.0, pause=0.05)
results.append(("helper recovers after 6 refusals", ok and rb == {"gripper": 0} and fake.mem[6][40] == 0, rb))

# Helper reports failure (warning path) when torque-off never takes.
fake, bus = make(refuse_torque_off=10**6)
ok, rb = torque_off_verified(bus, ["gripper"], timeout=0.5, pause=0.05)
results.append(("helper reports failure if never cleared", (not ok) and fake.mem[6][40] == 1, rb))

for name, passed, detail in results:
    print("PASS" if passed else "FAIL", name, detail)
sys.exit(0 if all(p for _, p, _ in results) else 1)
