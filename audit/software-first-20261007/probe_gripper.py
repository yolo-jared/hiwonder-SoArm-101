"""Read-only gripper protection probe (ID 6). Sends NO write packets.

    python -I probe_gripper.py /dev/cu.usbmodemXXXX
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from lerobot.motors.hiwonder import HiwonderMotorsBus  # noqa: E402
from lerobot.motors.motors_bus import Motor, MotorNormMode  # noqa: E402

REGS = ["P_Coefficient", "Max_Torque_Limit", "Torque_Limit", "Protection_Current", "Protective_Torque", "Protection_Time",
        "Overload_Torque", "Over_Current_Protection_Time", "Unloading_Condition", "Max_Temperature_Limit",
        "Present_Temperature", "Present_Current", "Present_Load", "Present_Position", "Torque_Enable", "Status"]

bus = HiwonderMotorsBus(port=sys.argv[1], motors={"gripper": Motor(6, "hx30hm", MotorNormMode.DEGREES)})


def no_write(*a, **k):
    raise RuntimeError("write blocked in read-only probe")


bus._write = no_write
bus.packet_handler.writeReadData = no_write
bus.port_handler.openPort()
out = {}
for r in REGS:
    try:
        out[r] = bus.read(r, "gripper", normalize=False)
    except Exception as e:  # noqa: BLE001
        out[r] = f"ERR {type(e).__name__}"
bus.port_handler.closePort()
print(json.dumps(out, indent=1))
