"""Read-only probe for the SDK reply-length check (issue #2, FA-10). Sends NO write packets.

For every register LeRobot reads one motor at a time (calibration, model/firmware checks, the verified
torque-off loop and its hardware harness), sends one read per motor and records the reply's LEN byte.
The SDK now rejects a reply whose LEN != data length + 2; this probe shows whether real HX-30HM replies
satisfy that. Any mismatch blocks merging the SDK change.

Usage: python probe_reply_len.py /dev/cu.usbmodemXXXX   (prints JSON; exit 1 on any mismatch or no reply)
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from lerobot.motors.hiwonder import HiwonderMotorsBus  # noqa: E402
from lerobot.motors.motors_bus import Motor, MotorNormMode, get_address  # noqa: E402

REGISTERS = [
    "Model_Number", "Firmware_Major_Version", "Firmware_Minor_Version", "Homing_Offset",
    "Min_Position_Limit", "Max_Position_Limit", "Torque_Enable", "Lock", "Present_Position",
    "Goal_Position", "Present_Temperature", "Status",
]  # fmt: skip
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def main(port: str) -> int:
    bus = HiwonderMotorsBus(
        port=port, motors={n: Motor(i + 1, "hx30hm", MotorNormMode.DEGREES) for i, n in enumerate(NAMES)}
    )

    def no_write(*a, **k):
        raise RuntimeError("write blocked in read-only probe")

    bus._write = no_write
    bus.packet_handler.writeReadData = no_write
    bus.packet_handler.writeDataOnly = no_write

    # Record the raw reply of every txRxPacket, before readData's LEN check looks at it.
    raw = {}
    real_txrx = bus.packet_handler.txRxPacket

    def recording_txrx(txpacket):
        rxpacket, result, error = real_txrx(txpacket)
        raw["rx"] = list(rxpacket) if rxpacket else []
        raw["result"] = result
        return rxpacket, result, error

    bus.packet_handler.txRxPacket = recording_txrx

    bus.port_handler.openPort()
    rows, mismatches, no_reply = [], 0, 0
    try:
        for i, name in enumerate(NAMES):
            for reg in REGISTERS:
                addr, length = get_address(bus.model_ctrl_table, "hx30hm", reg)
                raw.clear()
                bus.port_handler.ser.reset_input_buffer()
                _, comm, error = bus._read(addr, length, i + 1, raise_on_error=False)
                rx = raw.get("rx", [])
                reply_len = rx[3] if len(rx) > 3 else None
                ok = reply_len == length + 2
                if raw.get("result") != 0:
                    no_reply += 1
                elif not ok:
                    mismatches += 1
                rows.append({
                    "id": i + 1, "register": reg, "addr": addr, "length": length, "reply_len": reply_len,
                    "expected_len": length + 2, "match": ok, "txrx_result": raw.get("result"), "comm": comm,
                    "status": error,
                })  # fmt: skip
    finally:
        bus.port_handler.closePort()

    print(json.dumps({"rows": rows, "mismatches": mismatches, "no_reply": no_reply}, indent=1))
    print(f"SUMMARY: {len(rows)} reads, {mismatches} LEN mismatches, {no_reply} without a valid reply", file=sys.stderr)
    return 0 if mismatches == 0 and no_reply == 0 else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
