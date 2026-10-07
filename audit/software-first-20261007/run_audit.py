"""Offline software-first audit of the Hiwonder SO-101 follower command path.

Usage (no hardware; refuses any non fake:// port):
    <venv>/bin/python -I run_audit.py --src <lerobot src dir> --label <name> --out <dir>

Runs the REAL driver code (HiwonderMotorsBus, hiwonder_sdk, SOFollower) against
fake_hx_bus.FakeHXBus and compares emitted bytes with hand-derived golden packets.
"""

from __future__ import annotations

import argparse
import builtins
import hashlib
import json
import platform
import shutil
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REAL_CAL = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/hiwonder_follower.json"


def sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def hx(bs):
    return [f"{b:02X}" for b in bs]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--label", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    src = Path(args.src).resolve()
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)

    sys.path.insert(0, str(HERE))
    sys.path.insert(0, str(src))
    import serial  # noqa: E402

    import fake_hx_bus as fk  # noqa: E402

    # Safety: every pyserial open in this process goes through FakeSerial, which refuses real ports.
    serial.Serial = fk.FakeSerial
    builtins.input = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("input() blocked in offline audit"))
    cal_hash_before = sha256(REAL_CAL) if REAL_CAL.exists() else None

    import lerobot  # noqa: E402
    import lerobot.motors.hiwonder.hiwonder as hwmod  # noqa: E402
    from lerobot.motors.hiwonder import HiwonderMotorsBus  # noqa: E402
    from lerobot.motors.motors_bus import Motor, MotorNormMode  # noqa: E402

    res: dict = {"label": args.label}
    sdk_dir = Path(hwmod.__file__).parent / "hiwonder_sdk"
    res["provenance"] = {
        "executable": sys.executable,
        "python": platform.python_version(),
        "platform": platform.platform(),
        "pyserial": getattr(serial, "__version__", "?"),
        "lerobot_file": lerobot.__file__,
        "hiwonder_file": hwmod.__file__,
        "src_requested": str(src),
        "imports_from_requested_src": str(Path(lerobot.__file__).resolve()).startswith(str(src)),
        "sha256": {p.name: sha256(p) for p in [Path(hwmod.__file__), *sorted(sdk_dir.glob("*.py"))]},
        "serial_Serial_is_fake": serial.Serial is fk.FakeSerial,
    }
    assert res["provenance"]["imports_from_requested_src"], res["provenance"]

    # ---------------- B. golden bytes through the real serializer ----------------
    golden = json.loads((HERE / "golden.json").read_text())
    names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

    def make_bus(tag):
        fake = fk.FakeHXBus()
        fk.FakeSerial.bus = fake
        motors = {n: Motor(i + 1, "hx30hm", MotorNormMode.DEGREES) for i, n in enumerate(names)}
        bus = HiwonderMotorsBus(port=f"fake://{tag}", motors=motors)
        assert bus.port_handler.openPort()
        return bus, fake

    def run_call(bus, call):
        kind = call[0]
        if kind == "write":
            _, id_, addr, n, val = call
            bus._write(addr, n, id_, val)
        elif kind == "write_signed":
            _, id_, data_name, val = call
            bus.write(data_name, names[id_ - 1], val, normalize=False)
        elif kind == "read":
            _, id_, addr, n = call
            bus._read(addr, n, id_)
        elif kind == "ping":
            bus.ping(call[1])
        elif kind == "sync_write":
            _, addr, n, vals = call
            bus._sync_write(addr, n, {int(k): v for k, v in vals.items()})
        elif kind == "sync_read":
            _, addr, n, ids = call
            bus._sync_read(addr, n, ids)

    def golden_check(mutator=None):
        bus, fake = make_bus("golden")
        if mutator:
            mutator(bus)
        rows = []
        for g in golden["packets"]:
            before = len(fake.packets)
            try:
                run_call(bus, g["call"])
                err = None
            except Exception as e:  # noqa: BLE001
                err = repr(e)
            emitted = fake.packets[before]["bytes"] if len(fake.packets) > before else []
            rows.append({"name": g["name"], "match": hx(emitted) == g["bytes"], "emitted": hx(emitted), "error": err})
        bus.port_handler.closePort()
        return rows

    res["golden_green"] = golden_check()

    # Reply decoding: fake must reproduce the hand-derived reply bytes, real SDK must decode the value.
    reply_rows = []
    for g, raw in zip(golden["replies"], [2543, fk.sm_encode(-5, 15)]):
        bus, fake = make_bus("reply")
        fake.put(3, 56, 2, raw)
        fake_reply = fk.frame(3, 0, list(fake.mem[3][56:58]))
        got = bus.read("Present_Position", "elbow_flex", normalize=False)
        reply_rows.append({"name": g["name"], "fake_reply_matches_golden": hx(fake_reply) == g["bytes"],
                           "decoded": got, "expected": g["expect_value"], "match": got == g["expect_value"]})
        bus.port_handler.closePort()
    res["golden_replies"] = reply_rows

    # Negative controls: break one thing in the REAL code path; the golden check must go red.
    def big_endian(bus):
        bus._split_into_byte_chunks = lambda v, n: [(v >> 8) & 0xFF, v & 0xFF] if n == 2 else [v]

    def wrong_sign_bit(bus):
        bus.model_encoding_table = {"hx30hm": {**bus.model_encoding_table["hx30hm"], "Goal_Position": 11}}

    def wrong_id(bus):
        orig = bus._write
        bus._write = lambda addr, n, id_, v, **kw: orig(addr, n, 4 if id_ == 3 else id_, v, **kw)

    def wrong_addr(bus):
        t = dict(bus.model_ctrl_table["hx30hm"])
        t["Goal_Position"] = (44, 2)
        bus.model_ctrl_table = {"hx30hm": t}

    def bad_checksum(bus):
        import lerobot.motors.hiwonder.hiwonder_sdk.packet_handler as ph

        orig = bus.packet_handler.port_handler.writePort
        bus.packet_handler.port_handler.writePort = lambda pkt: orig(pkt[:-1] + [(pkt[-1] + 1) & 0xFF])
        _ = ph

    negs = {}
    for name, mut in [("big_endian_serializer", big_endian), ("sign_bit_11_for_goal", wrong_sign_bit),
                      ("elbow_id_3_to_4", wrong_id), ("goal_addr_42_to_44", wrong_addr), ("checksum_plus_1", bad_checksum)]:
        rows = golden_check(mut)
        negs[name] = {"red_cases": [r["name"] for r in rows if not r["match"]], "went_red": any(not r["match"] for r in rows)}
    res["negative_controls"] = negs

    # ---------------- C. real follower: connect -> loop -> disconnect write ledger ----------------
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    from lerobot.robots.so_follower.so_follower import SOFollower

    tmp = Path(tempfile.mkdtemp(prefix="audit-cal-"))
    shutil.copy(REAL_CAL, tmp / "audit_follower.json")
    cal = json.loads(REAL_CAL.read_text())

    def seeded_fake():
        fake = fk.FakeHXBus()
        for n, c in cal.items():
            i = c["id"]
            fake.put(i, 31, 2, fk.sm_encode(c["homing_offset"], 11))
            fake.put(i, 9, 2, c["range_min"])
            fake.put(i, 11, 2, c["range_max"])
            mid = (c["range_min"] + c["range_max"]) // 2
            fake.put(i, 56, 2, mid)
            fake.put(i, 42, 2, mid)
        fake.put(3, 42, 2, (cal["elbow_flex"]["range_min"] + cal["elbow_flex"]["range_max"]) // 2 - 150)  # stale goal
        fake.mem[3][55] = 1  # locked at start, as after a previous enable_torque()
        return fake

    def ledger(fake, start, end, phase, lock_track):
        rows = []
        for p in fake.packets[start:end]:
            for w in p.get("writes", []):
                i = w["id"]
                row = {"phase": phase, "inst": p["inst"], **w}
                row["region"] = "NVS" if w["addr"] < 40 else "SRAM"
                row["lock_before"] = lock_track.get(i)
                row["persists_if_firmware_honours_lock"] = row["region"] == "NVS" and lock_track.get(i) == 0
                if w["addr"] == 55:
                    lock_track[i] = w["raw"]
                rows.append(row)
        return rows

    fake = seeded_fake()
    fk.FakeSerial.bus = fake
    cfg = SOFollowerRobotConfig(port="fake://follower", id="audit_follower", calibration_dir=tmp, max_relative_target=2.0)
    robot = SOFollower(cfg)
    lock = {i: 1 for i in range(1, 7)}
    p0 = len(fake.packets)
    robot.connect()
    p1 = len(fake.packets)
    led = ledger(fake, p0, p1, "connect+configure", lock)
    res["torque_enable_goal_at_enable"] = {"elbow_goal_register": fake.get(3, 42, 2), "elbow_present": fake.get(3, 56, 2),
                                           "note": "goal register value when configure() re-enables torque"}

    # Loop with a stationary elbow and a leader 10 deg away (re-anchoring check).
    loop_rows = []
    for k in range(5):
        obs = robot.get_observation()
        action = {f"{n}.pos": obs[f"{n}.pos"] for n in names}
        action["elbow_flex.pos"] = obs["elbow_flex.pos"] + 10.0
        before = len(fake.packets)
        sent = robot.send_action(action)
        goal_raw = fake.get(3, 42, 2)
        loop_rows.append({"iter": k, "present_raw": fake.get(3, 56, 2), "goal_raw": goal_raw,
                          "commanded_error_ticks": goal_raw - fake.get(3, 56, 2),
                          "requested_deg_delta": 10.0, "sent_deg_delta": sent["elbow_flex.pos"] - obs["elbow_flex.pos"],
                          "packets_in_cycle": [p["inst"] for p in fake.packets[before:]]})
    p2 = len(fake.packets)
    led += ledger(fake, p1, p2, "teleop-loop", lock)
    res["clamp_loop_stationary_elbow"] = loop_rows

    # Negative controls for the clamp: other caps must yield different commanded errors.
    def clamp_variant(mrt):
        f = seeded_fake()
        fk.FakeSerial.bus = f
        c = SOFollowerRobotConfig(port="fake://clamp", id="audit_follower", calibration_dir=tmp, max_relative_target=mrt)
        r = SOFollower(c)
        r.connect()
        obs = r.get_observation()
        action = {f"{n}.pos": obs[f"{n}.pos"] for n in names}
        action["elbow_flex.pos"] = obs["elbow_flex.pos"] + 10.0
        r.send_action(action)
        err = f.get(3, 42, 2) - f.get(3, 56, 2)
        r.disconnect()
        return err

    full_dict = {n: 2.0 for n in names} | {"elbow_flex": 5.0}
    variants = {"None": None, "2.0": 2.0, "5.0": 5.0, "dict_elbow_5_others_2": full_dict}
    res["clamp_variants_commanded_error_ticks"] = {k: clamp_variant(v) for k, v in variants.items()}
    try:
        clamp_variant({"elbow_flex": 5.0})
        res["elbow_only_dict"] = "accepted"
    except Exception as e:  # noqa: BLE001
        res["elbow_only_dict"] = f"{type(e).__name__}: {e}"
    fk.FakeSerial.bus = fake

    # CLI parse through the real teleoperate config (draccus), no devices constructed.
    import draccus

    from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig

    base = ["--robot.type=so101_follower", "--robot.port=fake://f", "--robot.id=x", "--teleop.type=so101_leader",
            "--teleop.port=fake://l", "--teleop.id=y", "--fps=10"]
    cli = {}
    for arg in ["--robot.max_relative_target=2.0", "--robot.max_relative_target=" + json.dumps(full_dict)]:
        try:
            v = draccus.parse(TeleoperateConfig, args=base + [arg]).robot.max_relative_target
            cli[arg] = repr(v)
        except BaseException as e:  # noqa: BLE001
            cli[arg] = f"ERROR {type(e).__name__}: {e}"
    res["cli_parse"] = cli

    robot.disconnect()
    p3 = len(fake.packets)
    led += ledger(fake, p2, p3, "disconnect", lock)
    res["write_ledger"] = led
    res["packet_counts"] = {"connect": p1 - p0, "loop": p2 - p1, "disconnect": p3 - p2}
    res["undocumented_writes"] = sorted({(w["addr"], w["nbytes"]) for w in led if w["workbook"] is None})
    res["final_lock_state"] = {i: fake.mem[i][55] for i in range(1, 7)}
    res["final_torque_state"] = {i: fake.mem[i][40] for i in range(1, 7)}

    # ---------------- D. stale-reply injection (late status packet) ----------------
    def stale_case(clear_input: bool, arrives: str):
        fake = seeded_fake()
        fk.FakeSerial.bus = fake
        fake.mem[3][40] = 1  # torque truly ON
        motors = {n: Motor(i + 1, "hx30hm", MotorNormMode.DEGREES) for i, n in enumerate(names)}
        bus = HiwonderMotorsBus(port="fake://stale", motors=motors)
        bus.port_handler.openPort()
        if clear_input:
            bus.port_handler.clearPort = lambda: bus.port_handler.ser.reset_input_buffer()
        fake.hold_next_read_reply = True
        t0 = time.monotonic()
        try:
            bus.read("Present_Position", "elbow_flex", normalize=False)
            first = "returned"
        except Exception as e:  # noqa: BLE001
            first = type(e).__name__
        dt = time.monotonic() - t0
        if arrives == "in_gap_before_next_tx":
            bus.port_handler.ser.deliver_held()
        try:
            torque = bus.read("Torque_Enable", "elbow_flex", normalize=False)
        except Exception as e:  # noqa: BLE001
            torque = repr(e)
        bus.port_handler.closePort()
        return {"clear_input_before_tx": clear_input, "late_reply_arrives": arrives, "first_read": first, "first_read_wait_s": round(dt, 3),
                "torque_enable_read": torque, "true_torque_enable": 1,
                "elbow_position_low_byte": fake.get(3, 56, 2) & 0xFF}

    res["stale_reply_injection"] = [
        stale_case(c, a) for a in ("in_gap_before_next_tx", "after_next_tx_starts") for c in (False, True)
    ]

    # ---------------- E. arithmetic (model, from prior fitted logs; not a measurement) ----------------
    tpd = 4095 / 360
    res["clamp_arithmetic"] = {
        "ticks_per_degree": tpd,
        "max_relative_target_deg": 2.0,
        "max_commanded_error_ticks": int(2.0 * tpd),
        "fit_from_independent_audit": "abs(Present_Load) ~= 16 + 3.5*error_ticks (error>=2), both adapters, P16",
        "max_drive_under_clamp_raw": 16 + 3.5 * int(2.0 * tpd),
        "observed_hold_drive_raw": [132, 136],
        "observed_hold_error_ticks": [33, 34],
    }

    cal_hash_after = sha256(REAL_CAL) if REAL_CAL.exists() else None
    res["calibration_file_unchanged"] = cal_hash_before == cal_hash_after
    res["real_ports_opened"] = [p for p in fk.FakeSerial.opened if not p.startswith("fake://")]
    (out / f"audit-{args.label}.json").write_text(json.dumps(res, indent=2, default=str))
    print(json.dumps({k: res[k] for k in ["label", "calibration_file_unchanged", "real_ports_opened"]}))
    print("golden green:", sum(r["match"] for r in res["golden_green"]), "/", len(res["golden_green"]))
    print("replies:", [(r["fake_reply_matches_golden"], r["match"]) for r in res["golden_replies"]])
    print("negative controls went red:", {k: v["went_red"] for k, v in negs.items()})
    print("clamp variants (commanded elbow error, ticks, for +10 deg):", res["clamp_variants_commanded_error_ticks"])
    print("elbow-only dict:", res["elbow_only_dict"])
    print("cli parse:", res["cli_parse"])


if __name__ == "__main__":
    main()
