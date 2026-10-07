"""Read-only identity probe: pings IDs 1-6 and reads settings/state. Sends NO write packets."""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from lerobot.motors.hiwonder import HiwonderMotorsBus  # noqa: E402
from lerobot.motors.motors_bus import Motor, MotorNormMode  # noqa: E402

port = sys.argv[1]
names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
bus = HiwonderMotorsBus(port=port, motors={n: Motor(i + 1, "hx30hm", MotorNormMode.DEGREES) for i, n in enumerate(names)})


def no_write(*a, **k):
    raise RuntimeError("write blocked in read-only probe")


bus._write = no_write
bus.packet_handler.writeReadData = no_write
bus.port_handler.openPort()
out = {"port": port}
for n in names:
    i = names.index(n) + 1
    row = {}
    for reg in ["P_Coefficient", "Torque_Enable", "Present_Position", "Goal_Position", "Present_Voltage", "Status",
                "Homing_Offset", "Min_Position_Limit", "Max_Position_Limit", "Lock"]:
        try:
            row[reg] = bus.read(reg, n, normalize=False)
        except Exception as e:  # noqa: BLE001
            row[reg] = f"ERR {type(e).__name__}"
    out[f"{i}:{n}"] = row
bus.port_handler.closePort()
print(json.dumps(out, indent=1))
