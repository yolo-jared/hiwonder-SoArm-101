"""Gripper-only squeeze test: torque on the gripper ONLY, close toward fully closed, hold, reopen, torque off.

Closing = decreasing ticks (verified on video 2026-10-07: +ticks lifts the moving jaw away). Target = range_min + 40.
Writes only Goal_Position (42) / Torque_Enable (40) on ID 6, plus P_Coefficient (21) on ID 6 with --set-p
(torque off, restored to the preflight value after torque-off). Logs position, load, current, temperature, status.
Guards: status fault, temperature >= 60 C. The servo's own overload protection is left as configured and observed.

    python -I gripper_grasp.py --port /dev/cu.usbmodemXXXX --out DIR [--set-p 32] [--hold 6]
    python -I gripper_grasp.py --port fake://dry --out DIR --dry-run [--stop 1900]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "src"))
sys.path.insert(0, str(HERE))

from torque_safety import torque_off_verified  # noqa: E402

CAL = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/hiwonder_follower.json"
GID = 6
TEMP_ABORT_C = 60
LOG_REGS = ["Present_Position", "Present_Load", "Present_Current", "Present_Temperature", "Status"]
PRE_REGS = ["P_Coefficient", "Torque_Enable", "Status", "Present_Voltage", "Present_Temperature", "Homing_Offset",
            "Min_Position_Limit", "Max_Position_Limit", "Max_Torque_Limit", "Torque_Limit", "Protection_Current",
            "Protective_Torque", "Protection_Time", "Overload_Torque", "Present_Position"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--set-p", type=int, default=None)
    ap.add_argument("--hold", type=float, default=6.0)
    ap.add_argument("--ramp", type=float, default=2.0)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--stop", type=int, default=1900, help="dry-run object position")
    ap.add_argument("--no-speech", action="store_true")
    ap.add_argument("--watch", type=float, default=180.0, help="read-only torque watch after torque-off (s)")
    a = ap.parse_args()
    if a.set_p is not None and not 16 <= a.set_p <= 32:
        sys.exit("refusing: --set-p must be 16..32")
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    log = open(out / "samples.jsonl", "w")
    c = json.loads(CAL.read_text())["gripper"]

    import serial

    if a.dry_run:
        import fake_hx_bus as fk

        serial.Serial = fk.FakeSerial
        fake = fk.FakeHXBus()
        fk.FakeSerial.bus = fake
        fake.put(GID, 31, 2, fk.sm_encode(c["homing_offset"], 11))
        fake.put(GID, 9, 2, c["range_min"])
        fake.put(GID, 11, 2, c["range_max"])
        fake.put(GID, 56, 2, 2186)
        fake.put(GID, 62, 1, 123)
        fake.put(GID, 63, 1, 45)
        fake.put(GID, 16, 2, 500)
        fake.mem[GID][21] = 16

        def plant(id_, addr):  # tracks goal, blocked by an object at --stop, drive capped at 500
            if addr == 56 and fake.mem[id_][40]:
                g, p = fake.get(id_, 42, 2), fake.get(id_, 56, 2)
                p = max(a.stop, int(p + 0.5 * (g - p)))
                fake.put(id_, 56, 2, p)
                err = p - g
                drive = min(500, int(16 + fake.mem[id_][21] / 16 * 3.5 * abs(err))) if err else 0
                fake.put(id_, 60, 2, drive | (0x400 if err < 0 else 0))

        fake.on_read = plant
    elif not a.port.startswith("/dev/cu."):
        sys.exit("refusing: real runs need a /dev/cu.* port")

    def say(msg):
        print(f"[cue] {msg}", flush=True)
        if not a.no_speech and not a.dry_run:
            import subprocess

            subprocess.run(["/usr/bin/say", msg], check=False)

    from lerobot.motors.hiwonder import HiwonderMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorCalibration, MotorNormMode

    bus = HiwonderMotorsBus(port=a.port, motors={"gripper": Motor(GID, "hx30hm", MotorNormMode.DEGREES)},
                            calibration={"gripper": MotorCalibration(**c)})
    sent: list[tuple] = []
    orig_write = bus._write
    p_phase = {"on": False}

    def guarded_write(addr, n, id_, v, **kw):
        assert id_ == GID, f"blocked write to id {id_}"
        assert addr in (40, 42) or (p_phase["on"] and addr == 21), f"blocked write addr {addr}"
        sent.append((addr, v))
        return orig_write(addr, n, id_, v, **kw)

    def blocked_sync(*_a, **_k):
        raise AssertionError("sync write blocked")

    bus._write, bus._sync_write = guarded_write, blocked_sync
    t0 = time.monotonic()

    def rec(event, **kw):
        log.write(json.dumps({"t": round(time.monotonic() - t0, 4), "event": event, **kw}) + "\n")
        log.flush()

    def rd(r):
        return bus.read(r, "gripper", normalize=False, num_retry=3)

    def set_p(v):
        p_phase["on"] = True
        try:
            bus.write("P_Coefficient", "gripper", v, normalize=False, num_retry=3)
            return rd("P_Coefficient")
        finally:
            p_phase["on"] = False

    summary = {"port": a.port, "dry_run": a.dry_run, "set_p": a.set_p, "hold_s": a.hold}
    torque_on, p_prior = False, None
    bus.port_handler.openPort()
    try:
        pre = {r: rd(r) for r in PRE_REGS}
        summary["preflight"] = pre
        rec("preflight", snap=pre)
        problems = []
        if pre["P_Coefficient"] != 16:
            problems.append(f"P={pre['P_Coefficient']} (gripper expected 16)")
        if (pre["Homing_Offset"], pre["Min_Position_Limit"], pre["Max_Position_Limit"]) != (
                c["homing_offset"], c["range_min"], c["range_max"]):
            problems.append("calibration mismatch vs file")
        if pre["Status"]:
            problems.append(f"status {pre['Status']}")
        if pre["Torque_Enable"]:
            problems.append("torque already on")
        if not 90 <= pre["Present_Voltage"] <= 140:
            problems.append(f"voltage raw {pre['Present_Voltage']}")
        if pre["Present_Temperature"] >= TEMP_ABORT_C:
            problems.append(f"temperature {pre['Present_Temperature']} C")
        if problems:
            summary["aborted"] = problems
            print("PREFLIGHT FAIL:", problems)
            return
        start = pre["Present_Position"]
        closed = c["range_min"] + 40
        summary.update(start=start, closed_target=closed)

        if a.set_p is not None:
            p_prior = pre["P_Coefficient"]
            rb = set_p(a.set_p)
            summary["p_set_readback"] = rb
            print("P set readback:", rb, flush=True)
            if rb != a.set_p:
                raise RuntimeError(f"P readback {rb}")

        say(f"Gripper squeeze at P {a.set_p or pre['P_Coefficient']}. Hands clear of the jaws.")
        bus.write("Goal_Position", "gripper", start, normalize=False, num_retry=3)
        torque_on = True
        bus.write("Torque_Enable", "gripper", 1, normalize=False, num_retry=3)
        rec("torque_on", readback=rd("Torque_Enable"))

        frm = start
        for label, to, ramp, hold in [("close", closed, a.ramp, a.hold), ("open", start, 1.5, 1.5)]:
            seg_t0, rows, contact_t = time.monotonic(), [], None
            while (el := time.monotonic() - seg_t0) <= ramp + hold:
                f = 0.5 - 0.5 * math.cos(math.pi * min(1.0, el / ramp))
                goal = int(round(frm + (to - frm) * f))
                try:
                    bus.write("Goal_Position", "gripper", goal, normalize=False)
                except RuntimeError as e:  # overload status in the reply; the write may still have applied
                    summary.setdefault("write_errors", []).append({"seg": label, "el": round(el, 2), "err": str(e)[-40:]})
                    rec("write_error", seg=label, el=round(el, 3), err=str(e))
                    if label == "close":
                        summary["overload_at_s"] = round(el, 2)
                        summary["overload_after_contact_s"] = round(el - contact_t, 2) if contact_t else None
                        print(f"overload flagged at {el:.2f} s (contact at {contact_t})", flush=True)
                        break
                try:
                    s = {r: rd(r) for r in LOG_REGS}
                except RuntimeError as e:
                    rec("read_error", seg=label, el=round(el, 3), err=str(e))
                    time.sleep(0.05)
                    continue
                rec("sample", seg=label, goal=goal, el=round(el, 3), **s)
                rows.append((el, goal, s))
                if label == "close" and contact_t is None and len(rows) >= 4 and \
                        abs(rows[-1][2]["Present_Position"] - rows[-4][2]["Present_Position"]) <= 1 and \
                        abs(goal - s["Present_Position"]) > 20:
                    contact_t = rows[-4][0]  # jaw stopped while still commanded to close
                    summary["contact_at_s"] = round(contact_t, 2)
                    summary["contact_pos"] = s["Present_Position"]
                if s["Status"]:
                    summary.setdefault("status_events", []).append({"seg": label, "el": round(el, 2), "status": s["Status"]})
                if s["Present_Temperature"] >= TEMP_ABORT_C:
                    raise RuntimeError(f"temperature {s['Present_Temperature']} C")
                time.sleep(0.02)
            frm = rows[-1][2]["Present_Position"] if rows else to
            hold_rows = [r for r in rows if r[0] > ramp]

            def med(k, rs=hold_rows):
                v = sorted(r[2][k] for r in rs)
                return v[len(v) // 2] if v else None

            early = [r for r in hold_rows if r[0] <= ramp + 1.0]
            late = [r for r in hold_rows if r[0] >= ramp + hold - 1.0]
            seg = {"target": to, "hold_pos_median": med("Present_Position"),
                   "hold_load_first_s": med("Present_Load", early), "hold_load_last_s": med("Present_Load", late),
                   "hold_current_first_s": med("Present_Current", early),
                   "hold_current_last_s": med("Present_Current", late),
                   "peak_load": max((abs(r[2]["Present_Load"]) for r in rows), default=None),
                   "peak_current": max((r[2]["Present_Current"] for r in rows), default=None),
                   "temp_end": rows[-1][2]["Present_Temperature"] if rows else None}
            summary[label] = seg
            print(label, json.dumps(seg), flush=True)
    except BaseException as e:  # noqa: BLE001
        summary["error"] = repr(e)
        print("ERROR:", repr(e), flush=True)
    finally:
        if torque_on:
            tlog: list = []
            ok, rb = torque_off_verified(bus, ["gripper"], log=tlog)
            summary["torque_off_verified"], summary["torque_off_readback"] = ok, rb
            summary["torque_off_log"] = [{k: v for k, v in e.items() if k != "t"} for e in tlog]
            say("Torque off, confirmed. Watching it." if ok else "Warning. Gripper torque did not turn off.")
            # read-only tail watch in the same process: does torque come back on by itself?
            w_t0, w_last, changes = time.monotonic(), None, []
            while time.monotonic() - w_t0 < a.watch:
                s = {}
                for r in ["Torque_Enable", "Status", "Goal_Position", "Present_Position"]:
                    try:
                        bus.port_handler.ser.reset_input_buffer()
                    except Exception:  # noqa: BLE001
                        pass
                    try:
                        s[r] = rd(r)
                    except Exception as e:  # noqa: BLE001
                        s[r] = f"ERR {str(e)[-40:]}"
                key = (s["Torque_Enable"], s["Status"])
                if key != w_last:
                    row = {"wall": time.strftime("%H:%M:%S"), "after_s": round(time.monotonic() - w_t0, 2), **s}
                    changes.append(row)
                    rec("watch", **row)
                    print("watch:", row, flush=True)
                    if s["Torque_Enable"] == 1 and "torque_recurred_after_s" not in summary:
                        summary["torque_recurred_after_s"] = row["after_s"]
                        say("Gripper torque came back on.")
                    w_last = key
                time.sleep(0.2)
            summary["watch_s"], summary["watch_changes"] = a.watch, changes
        if p_prior is not None:
            try:
                summary["p_restored_readback"] = set_p(p_prior)
                summary["p_restored_verified"] = summary["p_restored_readback"] == p_prior
            except Exception as e:  # noqa: BLE001
                summary["p_restore_error"] = repr(e)
            print("P restored readback:", summary.get("p_restored_readback"), flush=True)
        bus.port_handler.closePort()
        summary["write_addrs_used"] = sorted({w[0] for w in sent})
        (out / "summary.json").write_text(json.dumps(summary, indent=1, default=str))
        if torque_on:
            recurred = "torque_recurred_after_s" in summary
            say("Warning. Gripper torque turned back on during the watch." if recurred else
                "Gripper test finished. Torque stayed off." if summary["torque_off_verified"]
                else "Warning. Gripper torque did not turn off. Unplug the 12 volt power.")
        print("torque_off_verified:", summary.get("torque_off_verified"), "write addrs:", summary["write_addrs_used"])
        if a.dry_run:
            print("dry-run fake writes (id, addr):", sorted({(w["id"], w["addr"]) for p in fake.packets for w in p.get("writes", [])}))


if __name__ == "__main__":
    main()
