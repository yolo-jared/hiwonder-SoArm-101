"""Powered follower joint sweep: move each joint +/-A degrees from its start, others hold.

Writes ONLY Goal_Position (42) and Torque_Enable (40). No limit/lock/calibration writes, no clamp.
Goal is preloaded to Present before torque enable. Returns to start before torque-off.
Optional --set-p N --set-p-joints a,b: writes P_Coefficient (21) on those joints only, torque off, before enable;
restores each joint's preflight P after torque-off and reads it back (LeRobot configure() also rewrites P on every connect).

    python -I follower_sweep.py --port /dev/cu.usbmodemXXXX --out DIR [--amp 15] [--joints a,b]
    python -I follower_sweep.py --port fake://dry --out DIR --dry-run
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

NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
DEFAULT_ORDER = ["wrist_roll", "wrist_flex", "gripper", "elbow_flex", "shoulder_pan", "shoulder_lift"]
CAL = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/hiwonder_follower.json"
TPD = 4095 / 360
ALLOWED_WRITE_ADDRS = {40, 42}
ACCEPTED_P = {n: ({16, 32} if n in ("shoulder_lift", "elbow_flex", "wrist_flex") else {16}) for n in NAMES}  # configure()
P_ADDR = 21
OSC_PTP_TICKS = 60  # hold-phase peak-to-peak over 1 s above this = oscillation abort


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--amp", type=float, default=15.0)
    ap.add_argument("--joints", default=",".join(DEFAULT_ORDER))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--no-speech", action="store_true")
    ap.add_argument("--direction", choices=["both", "plus", "minus"], default="both")
    ap.add_argument("--set-p", type=int, default=None)
    ap.add_argument("--set-p-joints", default="")
    a = ap.parse_args()
    p_joints = [j for j in a.set_p_joints.split(",") if j]
    if a.set_p is not None:
        if not 16 <= a.set_p <= 32:
            sys.exit("refusing: --set-p must be 16..32")
        if not p_joints or any(j not in NAMES for j in p_joints):
            sys.exit(f"refusing: --set-p-joints must name joints from {NAMES}")
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    log = open(out / "samples.jsonl", "w")

    import serial

    if a.dry_run:
        import fake_hx_bus as fk

        serial.Serial = fk.FakeSerial
        fake = fk.FakeHXBus()
        fk.FakeSerial.bus = fake
        cal0 = json.loads(CAL.read_text())
        for n, c in cal0.items():
            fake.put(c["id"], 31, 2, fk.sm_encode(c["homing_offset"], 11))
            fake.put(c["id"], 9, 2, c["range_min"])
            fake.put(c["id"], 11, 2, c["range_max"])
            mid = (c["range_min"] + c["range_max"]) // 2
            fake.put(c["id"], 56, 2, mid)
            fake.put(c["id"], 42, 2, mid - 300)  # stale goal
            fake.mem[c["id"]][21] = 16

        def plant(id_, addr):  # first-order tracking, elbow sags 30 ticks
            if addr == 56 and fake.mem[id_][40]:
                g, p = fake.get(id_, 42, 2), fake.get(id_, 56, 2)
                target = g + (30 if id_ == 3 else 0)
                fake.put(id_, 56, 2, int(p + 0.5 * (target - p)))

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

    cal = {n: MotorCalibration(**v) for n, v in json.loads(CAL.read_text()).items()}
    bus = HiwonderMotorsBus(port=a.port, motors={n: Motor(i + 1, "hx30hm", MotorNormMode.DEGREES) for i, n in enumerate(NAMES)},
                            calibration=cal)
    sent: list[tuple] = []
    orig_write, orig_sync = bus._write, bus._sync_write
    p_phase = {"on": False}
    p_ids = {NAMES.index(j) + 1 for j in p_joints}

    def guarded_write(addr, n, id_, v, **kw):
        ok_p = p_phase["on"] and addr == P_ADDR and id_ in p_ids
        assert addr in ALLOWED_WRITE_ADDRS or ok_p, f"blocked write addr {addr} id {id_}"
        sent.append(("w", addr, id_, v))
        return orig_write(addr, n, id_, v, **kw)

    def guarded_sync(addr, n, ids_values, **kw):
        assert addr in ALLOWED_WRITE_ADDRS, f"blocked sync write addr {addr}"
        sent.append(("s", addr, dict(ids_values)))
        return orig_sync(addr, n, ids_values, **kw)

    bus._write, bus._sync_write = guarded_write, guarded_sync
    t0 = time.monotonic()

    def rec(event, **kw):
        log.write(json.dumps({"t": round(time.monotonic() - t0, 4), "event": event, **kw}) + "\n")
        log.flush()

    def read_all():
        pos = bus.sync_read("Present_Position", normalize=False)
        load = bus.sync_read("Present_Load", normalize=False)
        status = bus.sync_read("Status", normalize=False)
        return pos, load, status

    def torque(val):
        for n in NAMES:
            bus.write("Torque_Enable", n, val, normalize=False, num_retry=3)
        return {n: bus.read("Torque_Enable", n, normalize=False, num_retry=3) for n in NAMES}

    def set_p(val):  # int, or {joint: value}
        p_phase["on"] = True
        try:
            for j in p_joints:
                bus.write("P_Coefficient", j, val[j] if isinstance(val, dict) else val, normalize=False, num_retry=3)
            return {j: bus.read("P_Coefficient", j, normalize=False, num_retry=3) for j in p_joints}
        finally:
            p_phase["on"] = False

    summary = {"port": a.port, "amp_deg": a.amp, "dry_run": a.dry_run, "set_p": a.set_p, "set_p_joints": p_joints,
               "joints": {}}
    torque_on = False
    p_changed = False
    bus.port_handler.openPort()
    try:
        # ---- preflight (reads only) ----
        snap = {n: {r: bus.read(r, n, normalize=False) for r in
                    ["P_Coefficient", "D_Coefficient", "Torque_Enable", "Status", "Present_Voltage", "Homing_Offset",
                     "Min_Position_Limit", "Max_Position_Limit", "Goal_Position", "Present_Position"]} for n in NAMES}
        summary["preflight"] = snap
        rec("preflight", snap=snap)
        problems = []
        for n in NAMES:
            s, c = snap[n], cal[n]
            if s["P_Coefficient"] not in ACCEPTED_P[n]:
                problems.append(f"{n} P={s['P_Coefficient']} (follower expects {sorted(ACCEPTED_P[n])}; is this the leader?)")
            if (s["Homing_Offset"], s["Min_Position_Limit"], s["Max_Position_Limit"]) != (c.homing_offset, c.range_min, c.range_max):
                problems.append(f"{n} calibration mismatch vs file")
            if s["Status"]:
                problems.append(f"{n} status {s['Status']}")
            if not 90 <= s["Present_Voltage"] <= 140:
                problems.append(f"{n} voltage raw {s['Present_Voltage']}")
        if problems:
            summary["aborted"] = problems
            print("PREFLIGHT FAIL:", problems)
            return
        start = {n: snap[n]["Present_Position"] for n in NAMES}
        summary["start"] = start

        if a.set_p is not None:  # torque is off here (preflight verified via snapshot)
            assert all(snap[n]["Torque_Enable"] == 0 for n in NAMES), "torque must be off to change P"
            p_changed = True
            rb = set_p(a.set_p)
            summary["p_set_readback"] = rb
            rec("p_set", readback=rb)
            print("P set readback:", rb, flush=True)
            if any(v != a.set_p for v in rb.values()):
                raise RuntimeError(f"P readback mismatch {rb}")

        # ---- enable with goal preloaded to present ----
        say("Follower test starting. Stand clear.")
        bus.sync_write("Goal_Position", start, normalize=False)
        torque_on = True
        rec("torque_on", readback=torque(1))
        # settle: 2 s at start pose, record jitter
        st0, spos = time.monotonic(), {n: [] for n in NAMES}
        while time.monotonic() - st0 < 2.0:
            pos, load, status = read_all()
            rec("settle", pos=pos, load=load, status=status)
            for n in NAMES:
                spos[n].append(pos[n])
            time.sleep(0.02)
        summary["settle_ptp"] = {n: max(v) - min(v) for n, v in spos.items()}
        print("settle peak-to-peak ticks:", summary["settle_ptp"], flush=True)

        for joint in [j for j in a.joints.split(",") if j]:
            c = cal[joint]
            amp = int(a.amp * TPD)
            lo, hi = c.range_min + 40, c.range_max - 40
            s0 = start[joint]
            plus, minus = min(hi, s0 + amp), max(lo, s0 - amp)
            # (target, ramp seconds, hold seconds, label)
            if a.direction == "minus":
                plan = [(minus, 2.0, 3.0, "minus"), (s0, 2.0, 1.5, "return")]
            elif a.direction == "plus":
                plan = [(plus, 2.0, 3.0, "plus"), (s0, 2.0, 1.5, "return")]
            else:
                plan = [(plus, 1.5, 2.0, "plus"), (minus, 3.0, 2.0, "minus"), (s0, 1.5, 1.5, "return")]
            say(f"Moving {joint.replace('_', ' ')}.")
            jr = {"start": s0, "range": [c.range_min, c.range_max], "segments": []}
            stalled = False
            goal = s0
            for target, ramp, hold, label in plan:
                g_from = goal
                seg_t0 = time.monotonic()
                peak_err, holds, hold_loads, win = 0, [], [], []
                stall_since = None
                while True:
                    el = time.monotonic() - seg_t0
                    if el > ramp + hold:
                        break
                    f = min(1.0, el / ramp)
                    f = 0.5 - 0.5 * math.cos(math.pi * f)
                    goal = int(round(g_from + (target - g_from) * f))
                    bus.sync_write("Goal_Position", {**start, joint: goal}, normalize=False)
                    pos, load, status = read_all()
                    err = goal - pos[joint]
                    rec("sample", joint=joint, seg=label, goal=goal, pos=pos, load=load, status=status)
                    if any(status.values()):
                        raise RuntimeError(f"servo status fault {status}")
                    drift = {n: pos[n] - start[n] for n in NAMES if n != joint and abs(pos[n] - start[n]) > 8 * TPD}
                    if drift:
                        raise RuntimeError(f"holding joint drift {drift}")
                    if el > ramp:
                        holds.append(pos[joint])
                        hold_loads.append(load[joint])
                    if el > ramp + 0.5:
                        now = time.monotonic()
                        win = [(t, p) for t, p in win if now - t <= 1.0] + [(now, pos[joint])]
                        ptp = max(p for _, p in win) - min(p for _, p in win)
                        if ptp > OSC_PTP_TICKS:
                            raise RuntimeError(f"oscillation on {joint}: {ptp} ticks peak-to-peak in 1 s")
                    peak_err = max(peak_err, abs(err))
                    if abs(err) > 120 and abs(load[joint]) > 600:  # load already sign-decoded by sync_read
                        stall_since = stall_since or time.monotonic()
                        if time.monotonic() - stall_since > 1.5:
                            stalled = True
                            break
                    else:
                        stall_since = None
                    time.sleep(0.02)
                hold_med = sorted(holds)[len(holds) // 2] if holds else None
                jr["segments"].append({"label": label, "target": target, "requested_ticks": target - g_from,
                                       "hold_median": hold_med,
                                       "hold_error_ticks": (target - hold_med) if hold_med is not None else None,
                                       "hold_ptp": (max(holds) - min(holds)) if holds else None,
                                       "hold_load_median": sorted(hold_loads)[len(hold_loads) // 2] if hold_loads else None,
                                       "hold_load_ptp": (max(hold_loads) - min(hold_loads)) if hold_loads else None,
                                       "peak_abs_error": peak_err})
                if stalled:
                    jr["stalled_in"] = label
                    say(f"{joint.replace('_', ' ')} stalled. Returning.")
                    bus.sync_write("Goal_Position", start, normalize=False)
                    time.sleep(1.5)
                    goal = s0
                    break
            summary["joints"][joint] = jr
            print(joint, json.dumps(jr["segments"]), "STALLED" if stalled else "", flush=True)

        bus.sync_write("Goal_Position", start, normalize=False)
        time.sleep(1.0)
    except BaseException as e:  # noqa: BLE001
        summary["error"] = repr(e)
        print("ERROR:", repr(e), flush=True)
        if torque_on:
            try:  # attempt a gentle return before releasing
                bus.sync_write("Goal_Position", summary.get("start", {}), normalize=False)
                time.sleep(1.0)
            except Exception:  # noqa: BLE001
                pass
    finally:
        if torque_on:
            try:
                rb = torque(0)
                summary["final_torque_readback"] = rb
                summary["torque_off_verified"] = all(v == 0 for v in rb.values())
            except Exception as e:  # noqa: BLE001
                summary["torque_off_error"] = repr(e)
        if p_changed:
            try:
                prior = {j: summary["preflight"][j]["P_Coefficient"] for j in p_joints}
                rb = set_p(prior)
                summary["p_restored_readback"] = rb
                summary["p_restored_verified"] = rb == prior
            except Exception as e:  # noqa: BLE001
                summary["p_restore_error"] = repr(e)
            print("P restored readback:", summary.get("p_restored_readback"), summary.get("p_restore_error", ""), flush=True)
        bus.port_handler.closePort()
        summary["write_addrs_used"] = sorted({w[1] for w in sent})
        (out / "summary.json").write_text(json.dumps(summary, indent=1, default=str))
        if torque_on:
            say("Follower test finished. Torque off.")
        print("torque_off_verified:", summary.get("torque_off_verified"), "write addrs:", summary["write_addrs_used"])
        if a.dry_run:
            print("dry-run fake write addrs:", sorted({w["addr"] for p in fake.packets for w in p.get("writes", [])}))
            print("dry-run P writes (id, value):",
                  [(w["id"], w["raw"]) for p in fake.packets for w in p.get("writes", []) if w["addr"] == P_ADDR])


if __name__ == "__main__":
    main()
