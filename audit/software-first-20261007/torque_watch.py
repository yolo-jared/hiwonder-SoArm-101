"""Read-only torque watch: poll Torque_Enable/Status/Goal/Present on IDs 1-6 and log every change. NO writes.

    python -I torque_watch.py /dev/cu.usbmodemXXXX OUT.jsonl SECONDS
"""

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from lerobot.motors.hiwonder import HiwonderMotorsBus  # noqa: E402
from lerobot.motors.motors_bus import Motor, MotorNormMode  # noqa: E402

N = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
REGS = ["Torque_Enable", "Status", "Goal_Position", "Present_Position"]
port, out, secs = sys.argv[1], Path(sys.argv[2]), float(sys.argv[3])
bus = HiwonderMotorsBus(port=port, motors={n: Motor(i + 1, "hx30hm", MotorNormMode.DEGREES) for i, n in enumerate(N)})


def no_write(*a, **k):
    raise RuntimeError("write blocked in read-only watch")


bus._write = no_write
bus._sync_write = no_write
bus.packet_handler.writeReadData = no_write
bus.port_handler.openPort()
t_end, last = time.monotonic() + secs, None
with out.open("w") as f:
    while time.monotonic() < t_end:
        snap = {}
        for n in N:
            try:
                snap[n] = {r: bus.read(r, n, normalize=False) for r in REGS}
            except Exception as e:  # noqa: BLE001
                snap[n] = f"ERR {e}"
        key = json.dumps({n: (v["Torque_Enable"], v["Status"]) if isinstance(v, dict) else v for n, v in snap.items()})
        if key != last:
            row = {"wall": time.strftime("%H:%M:%S"), "snap": snap}
            f.write(json.dumps(row) + "\n")
            f.flush()
            print(row["wall"], key, flush=True)
            last = key
        time.sleep(0.5)
bus.port_handler.closePort()
print("watch done", time.strftime("%H:%M:%S"))
