"""AC7/AC8 hardware harness for verified torque-off (issue #2): gripper overload, then LeRobot shutdown, then a
separate read-only watch of all six motors.

Each cycle: (1) a child process stalls the gripper against the closed position until the servo flags Overload
(or the hold runs out), then shuts the bus down with the method under test and exits; (2) after the child has
exited and nothing holds the port, this process opens it exclusively and watches Torque_Enable on IDs 1-6 every
0.5 s (input flushed before each read). Any Torque_Enable != 0 fails the cycle, stops the watch and runs the
verified torque-off at once; a read error is "unknown" and fails the cycle unless that motor's next read is a valid 0.
The gripper temperature is read every sample; >= 60 C aborts. Between cycles it waits until the gripper is within
5 C of its starting temperature. Only cycles where Overload was actually observed count as exercised.

    --method new   AC7: HiwonderMotorsBus.disconnect() (verified torque-off)
    --method old   AC8: FeetechMotorsBus.disable_torque(bus, num_retry=5) then closePort() (the inherited method)

    python -I lerobot_disconnect_trials.py --port /dev/cu.usbmodemXXXX --out DIR --method new [--cycles 5]
    python -I lerobot_disconnect_trials.py --port /dev/cu.usbmodemXXXX --self-check    (needs nothing moving)
    python -I lerobot_disconnect_trials.py --port /dev/cu.usbmodemXXXX --goal-probe --out DIR   (FA-05, no motion)

Raw output stays in DIR (use the main checkout's gitignored test-output/); only RESULT.md tables are committed.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from functools import partial
from pathlib import Path

HERE = Path(__file__).resolve().parent
WORKTREE_SRC = HERE.parents[1] / "src"
sys.path.insert(0, str(WORKTREE_SRC))
sys.path.insert(0, str(HERE))

CAL = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/hiwonder_follower.json"
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
GID = 6
TE, GOAL, LOCK, STATUS_REG, TEMP, VOLT, PRESENT = 40, 42, 55, 65, 63, 62, 56
OVERLOAD = 0x20
TEMP_ABORT_C = 60
COOLDOWN_MARGIN_C = 5


# ---------------------------------------------------------------------------------------------------------------
# Pure logic (checked offline by check_disconnect_trials.py)
# ---------------------------------------------------------------------------------------------------------------


def import_guard() -> list[str]:
    """SC1: refuse to run unless lerobot comes from this worktree and the verified override is active."""
    import lerobot
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors.hiwonder import HiwonderMotorsBus

    problems = []
    if not Path(lerobot.__file__).resolve().is_relative_to(WORKTREE_SRC.resolve()):
        problems.append(f"lerobot imported from {lerobot.__file__}, not {WORKTREE_SRC}")
    if HiwonderMotorsBus.disable_torque is FeetechMotorsBus.disable_torque:
        problems.append("HiwonderMotorsBus.disable_torque is the inherited Feetech method")
    return problems


def watch(
    sample, duration_s, interval_s=0.5, now=time.monotonic, sleep=time.sleep, temp_abort_c=TEMP_ABORT_C
):
    """Poll `sample()` -> {id: {"te": int|None, "status": int|None, "err": str|None}, "temp": int|None}.

    Returns {"verdict": PASS|FAIL|ABORT, "rows": [...], "first_on": {...}|None, "unknown": [...], "samples": n}.
    Stops at the first Torque_Enable != 0 (FAIL) or temperature >= temp_abort_c (ABORT).
    """
    rows, pending_unknown, t0, n = [], set(), now(), 0
    while now() - t0 < duration_s:
        snap = sample()
        n += 1
        t = round(now() - t0, 2)
        temp = snap.get("temp")
        for id_ in sorted(k for k in snap if k != "temp"):
            s = snap[id_]
            rows.append(
                {
                    "t": t,
                    "id": id_,
                    "te": s["te"],
                    "status": s["status"],
                    "err": s.get("err"),
                    "temp": temp if id_ == GID else None,
                }
            )
            if s["te"] is None:
                pending_unknown.add(id_)
            elif s["te"] == 0:
                pending_unknown.discard(id_)
            else:
                return {
                    "verdict": "FAIL",
                    "rows": rows,
                    "first_on": {"t": t, "id": id_, "te": s["te"]},
                    "unknown": sorted(pending_unknown),
                    "samples": n,
                }
        if temp is not None and temp >= temp_abort_c:
            return {
                "verdict": "ABORT",
                "rows": rows,
                "first_on": None,
                "unknown": sorted(pending_unknown),
                "samples": n,
                "abort": f"temperature {temp} C",
            }
        sleep(interval_s)
    verdict = "FAIL" if pending_unknown else "PASS"
    return {
        "verdict": verdict,
        "rows": rows,
        "first_on": None,
        "unknown": sorted(pending_unknown),
        "samples": n,
    }


def cycle_failed(cycle: dict) -> bool:
    """A cycle fails if the shutdown raised, the watch saw torque on / unknown, or it aborted."""
    return bool(cycle.get("disconnect_error")) or cycle.get("watch_verdict") in ("FAIL", "ABORT")


def summarize(cycles: list[dict], wanted: int, method: str) -> tuple[str, int]:
    """FA-30/FA-31: count only Overload-exercised cycles; state the 95% upper bound for 0 failures."""
    exercised = [c for c in cycles if c.get("overload_seen")]
    failed = [c for c in exercised if cycle_failed(c)]
    n = len(exercised)
    lines = [f"method={method} cycles_run={len(cycles)} exercised={n} failed={len(failed)}"]
    if n:
        lines.append(
            f"95% upper bound on recurrence rate: {1 - 0.05 ** (1 / n):.3f} (0 failures in {n})"
            if not failed
            else f"failures in exercised cycles: {[c['cycle'] for c in failed]}"
        )
    if method == "old":
        lines.append(f"negative control failed at least once: {'yes' if failed else 'no'}")
        code = 0 if failed else 3  # 3 = never observed failing (open risk, AC8)
    elif failed:
        lines.append("RESULT: FAIL")
        code = 1
    elif n < wanted:
        lines.append(f"{n} exercised cycles — INCONCLUSIVE (need {wanted})")
        code = 2
    else:
        lines.append("RESULT: PASS")
        code = 0
    return "\n".join(lines), code


def clear_overload_latch(
    io,
    open_pos,
    now=time.monotonic,
    sleep=time.sleep,
    settle_s=2.0,
    interval_s=0.25,
    wait_s=900,
    notify=print,
) -> dict:
    """Clear the gripper's latched Overload status (0x20) so the next cycle starts clean.

    A stall cut off mid-squeeze leaves Status 0x20 set with torque off, and every LeRobot read raises while it is
    set (2026-10-08). Tries Goal=Present (the write re-enables torque, FA-05), then Goal=open_pos (jaw off the
    object); any write is followed by a verified gripper torque-off whose failure propagates. If the writes do not
    clear it, asks for a 12 V power cycle and waits up to wait_s. ok means Status 0 and all six Torque_Enable 0.
    io: status(), present(), write_goal(v), torque_off() (verified, raises), all_te() -> {id: te}.
    """
    res: dict = {"status_before": io.status(), "writes": [], "cleared_by": None}

    def clear_within(secs):
        t0 = now()
        while now() - t0 < secs:
            if io.status() == 0:
                return True
            sleep(interval_s)
        return False

    if res["status_before"] == 0:
        res["cleared_by"] = "already clear"
    else:
        try:
            for label, goal in (("goal=present", io.present()), ("goal=open", open_pos)):
                if goal is None or goal & 0x8000:
                    continue
                res["writes"].append(
                    (label, goal)
                )  # before the write: a raise mid-write still turns torque off
                io.write_goal(goal)
                if clear_within(settle_s):
                    res["cleared_by"] = label
                    break
        finally:
            if res["writes"]:
                io.torque_off()
        if res["cleared_by"] is None:
            notify(
                f"LATCHED: gripper Overload flag did not clear; power-cycle the 12 V servo supply "
                f"(waiting up to {wait_s / 60:.0f} min)"
            )
            if clear_within(wait_s):
                res["cleared_by"] = "power cycle"
    res["te_after"] = io.all_te()
    res["ok"] = res["cleared_by"] is not None and all(v == 0 for v in res["te_after"].values())
    return res


def port_holders(port: str) -> list[str]:
    """FA-16: PIDs holding the port (macOS has no exclusive serial lock by default)."""
    r = subprocess.run(["lsof", "-t", port], capture_output=True, text=True)
    return r.stdout.split()


# ---------------------------------------------------------------------------------------------------------------
# Hardware
# ---------------------------------------------------------------------------------------------------------------


def say(msg: str, quiet: bool = False):
    print(f"[cue] {msg}", flush=True)
    if not quiet:
        subprocess.run(["/usr/bin/say", msg], check=False)


def make_bus(port: str):
    from lerobot.motors.hiwonder import HiwonderMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorCalibration, MotorNormMode

    cal = json.loads(CAL.read_text())
    return HiwonderMotorsBus(
        port=port,
        motors={n: Motor(i + 1, "hx30hm", MotorNormMode.DEGREES) for i, n in enumerate(NAMES)},
        calibration={n: MotorCalibration(**cal[n]) for n in NAMES},
    )


def read_raw(bus, addr, length, id_):
    """Flush, then one read. Returns (value | None, status byte | None, error text | None)."""
    try:
        bus.port_handler.ser.reset_input_buffer()
        value, comm, error = bus._read(addr, length, id_, raise_on_error=False)
    except Exception as e:  # noqa: BLE001
        bus.port_handler.is_using = False
        return None, None, f"{type(e).__name__}: {e}"
    if comm != 0:
        return None, None, bus.packet_handler.getTxRxResult(comm)
    return value, error, None


def phase_stall(port: str, method: str, out: Path, ramp: float = 2.0, hold_max: float = 6.0) -> dict:
    """Child process: gripper stall, then shutdown by `method`. Writes out/stall.json."""
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors.hiwonder import HiwonderMotorsBus

    res: dict = {"method": method, "port": port}
    bus = make_bus(port)
    real_write = bus._write

    def stall_write(addr, n, id_, v, **kw):
        assert id_ == GID and addr in (TE, GOAL), f"blocked write id {id_} addr {addr}"
        return real_write(addr, n, id_, v, **kw)

    bus._write = stall_write
    bus.port_handler.openPort()
    torque_on = False
    t0 = time.monotonic()
    try:
        pre = {i: read_raw(bus, TE, 1, i) for i in range(1, 7)}
        res["preflight_te"] = {i: v[0] for i, v in pre.items()}
        temp0, _, _ = read_raw(bus, TEMP, 1, GID)
        volt, _, _ = read_raw(bus, VOLT, 1, GID)
        status0, _, _ = read_raw(bus, STATUS_REG, 1, GID)
        res.update(temp_start=temp0, voltage_raw=volt, status_start=status0)
        problems = [f"id {i} torque {v[0]}" for i, v in pre.items() if v[0] != 0]
        if temp0 is None or temp0 >= TEMP_ABORT_C or volt is None or not 90 <= volt <= 140 or status0:
            problems.append(f"preflight temp={temp0} volt={volt} status={status0}")
        # Wrong arm or stale calibration file -> wrong close target: the servo must match the file.
        if status0:
            problems.append(
                "overload latched (Status 0x20): reads raise; clear_overload_latch should have run"
            )
            res["aborted"] = problems
            return res
        cal = bus.calibration["gripper"]
        servo_cal = {}
        for reg in ("Homing_Offset", "Min_Position_Limit", "Max_Position_Limit"):
            try:
                servo_cal[reg] = bus.read(reg, "gripper", normalize=False, num_retry=3)
            except Exception as e:  # noqa: BLE001
                servo_cal[reg] = f"ERR {type(e).__name__}"
        res["servo_calibration"] = servo_cal
        if (servo_cal["Homing_Offset"], servo_cal["Min_Position_Limit"], servo_cal["Max_Position_Limit"]) != (
            cal.homing_offset,
            cal.range_min,
            cal.range_max,
        ):
            problems.append(f"calibration mismatch: servo {servo_cal} vs file {cal}")
        if problems:
            res["aborted"] = problems
            return res
        start, _, _ = read_raw(bus, PRESENT, 2, GID)
        closed = bus.calibration["gripper"].range_min + 40
        res.update(start=start, closed_target=closed)
        real_write(GOAL, 2, GID, start, raise_on_error=False)
        torque_on = True
        real_write(TE, 1, GID, 1, raise_on_error=False)
        t_ramp = time.monotonic()
        while (el := time.monotonic() - t_ramp) <= ramp + hold_max:
            f = 0.5 - 0.5 * math.cos(math.pi * min(1.0, el / ramp))
            goal = int(round(start + (closed - start) * f))
            comm, error = bus._write(GOAL, 2, GID, goal, raise_on_error=False)
            if error & OVERLOAD:
                res.update(overload_seen=True, overload_at_s=round(el, 2))
                break
            temp, _, _ = read_raw(bus, TEMP, 1, GID)
            if temp is not None and temp >= TEMP_ABORT_C:
                res["aborted"] = [f"temperature {temp} C during stall"]
                break
            time.sleep(0.02)
        res.setdefault("overload_seen", False)
        _, status_err, _ = read_raw(bus, STATUS_REG, 1, GID)
        res["reply_status_before_disconnect"] = status_err
    except BaseException as e:  # noqa: BLE001
        res["stall_error"] = repr(e)
    finally:
        bus._write = real_write
        t_d = time.monotonic()
        if method == "new":
            try:
                bus.disconnect()
            except BaseException as e:  # noqa: BLE001
                res["disconnect_error"] = repr(e)
                if bus.port_handler.is_open:
                    bus.port_handler.closePort()
        else:
            try:
                FeetechMotorsBus.disable_torque(bus, num_retry=5)
            except BaseException as e:  # noqa: BLE001
                res["disconnect_error"] = repr(e)
            if res.get("disconnect_error") and torque_on:
                try:  # FA-18: never leave the stalled gripper powered after the old method raised
                    HiwonderMotorsBus.disable_torque(bus)
                    res["safety_off"] = "verified"
                except Exception as e:  # noqa: BLE001
                    res["safety_off"] = repr(e)
            bus.port_handler.closePort()
        res["disconnect_s"] = round(time.monotonic() - t_d, 3)
        res["total_s"] = round(time.monotonic() - t0, 2)
        (out / "stall.json").write_text(json.dumps(res, indent=1, default=str))
    return res


def open_watch_bus(port: str):
    """Read-only bus on an exclusively opened port; returns (bus, enable_writes)."""
    import serial

    bus = make_bus(port)
    real_write = bus._write

    def no_write(*a, **k):
        raise RuntimeError("write blocked in watch")

    bus._write = no_write
    bus.port_handler.ser = serial.Serial(
        port=port, baudrate=1_000_000, bytesize=serial.EIGHTBITS, timeout=0, exclusive=True
    )
    bus.port_handler.is_open = True
    bus.port_handler.tx_time_per_byte = (1000.0 / 1_000_000) * 10.0

    def enable_writes():
        bus._write = real_write

    return bus, enable_writes


def sample_bus(bus) -> dict:
    snap = {}
    for i in range(1, 7):
        te, status, err = read_raw(bus, TE, 1, i)
        snap[i] = {"te": te, "status": status, "err": err}
    snap["temp"] = read_raw(bus, TEMP, 1, GID)[0]
    return snap


class BusLatchIO:
    """clear_overload_latch() adapter for the real gripper (writes: Goal_Position on ID 6, verified torque-off)."""

    def __init__(self, bus):
        self.bus = bus

    def status(self):
        return read_raw(self.bus, STATUS_REG, 1, GID)[0]

    def present(self):
        return read_raw(self.bus, PRESENT, 2, GID)[0]

    def write_goal(self, v):
        self.bus._write(GOAL, 2, GID, v, raise_on_error=False)

    def torque_off(self):
        self.bus.disable_torque(["gripper"])

    def all_te(self):
        return {i: read_raw(self.bus, TE, 1, i)[0] for i in range(1, 7)}


def reset_latch(port: str, d: Path) -> dict:
    """Before each cycle: clear a latched Overload (open position = the last cycle's start, else 2039)."""
    starts = sorted(d.parent.glob("*-cycle*/stall.json"), key=lambda p: p.stat().st_mtime)
    open_pos = next((s for p in reversed(starts) if (s := json.loads(p.read_text()).get("start"))), 2039)
    bus = make_bus(port)
    bus.port_handler.openPort()
    try:
        res = clear_overload_latch(BusLatchIO(bus), open_pos)
    finally:
        bus.port_handler.closePort()
    (d / "latch_reset.json").write_text(json.dumps(res, indent=1, default=str))
    print("LATCH", json.dumps(res, default=str), flush=True)
    return res


def run_cycles(a) -> int:
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    cycles = []
    base_temp = None
    for k in range(1, a.cycles + 1):
        d = out / f"{a.method}-cycle{k}"
        d.mkdir(exist_ok=True)
        if holders := port_holders(a.port):
            sys.exit(f"refusing: port held by PIDs {holders} before cycle {k}")
        if not reset_latch(a.port, d)["ok"]:
            print(f"stopping: overload latch not cleared before cycle {k}", flush=True)
            break
        say(f"Cycle {k} of {a.cycles}, {a.method} method. Hands clear of the gripper.", a.quiet)
        subprocess.run(
            [
                sys.executable,
                "-I",
                __file__,
                "--phase",
                "stall",
                "--port",
                a.port,
                "--method",
                a.method,
                "--out",
                str(d),
            ],
            check=False,
        )
        stall = json.loads((d / "stall.json").read_text())
        if holders := port_holders(a.port):
            sys.exit(f"refusing: port still held by PIDs {holders} after the shutdown process exited")
        cyc = {
            "cycle": k,
            **{
                key: stall.get(key)
                for key in (
                    "overload_seen",
                    "overload_at_s",
                    "disconnect_error",
                    "disconnect_s",
                    "safety_off",
                    "temp_start",
                    "aborted",
                    "stall_error",
                )
            },
        }
        if stall.get("aborted") and not stall.get("overload_seen"):
            cycles.append(cyc)
            print("CYCLE", json.dumps(cyc), flush=True)
            print("stopping: preflight or stall aborted", flush=True)
            break
        base_temp = base_temp if base_temp is not None else stall.get("temp_start")
        bus, enable_writes = open_watch_bus(a.port)
        try:
            w = watch(partial(sample_bus, bus), a.watch_s)
            if w["verdict"] in ("FAIL", "ABORT"):
                enable_writes()
                try:
                    bus.disable_torque()
                    cyc["safety_off"] = "verified"
                except Exception as e:  # noqa: BLE001
                    cyc["safety_off"] = repr(e)
                say(
                    "Warning. Torque came back on. Turned it off."
                    if w["verdict"] == "FAIL"
                    else "Temperature abort.",
                    a.quiet,
                )
        finally:
            bus.port_handler.closePort()
        (d / "watch.jsonl").write_text("\n".join(json.dumps(r) for r in w["rows"]) + "\n")
        cyc.update(
            watch_verdict=w["verdict"],
            first_on=w["first_on"],
            unknown=w["unknown"],
            watch_samples=w["samples"],
            temp_end=w["rows"][-1]["temp"] if w["rows"] else None,
        )
        cycles.append(cyc)
        print("CYCLE", json.dumps(cyc), flush=True)
        if w["verdict"] == "ABORT":
            break
        if k < a.cycles and base_temp is not None:
            wait_cooldown(a.port, base_temp + COOLDOWN_MARGIN_C)
    final = out / f"{a.method}-final"
    final.mkdir(exist_ok=True)
    if not port_holders(a.port):
        reset_latch(a.port, final)  # leave the gripper unlatched and verified off, readable by LeRobot
    (out / f"{a.method}-cycles.json").write_text(json.dumps(cycles, indent=1))
    text, code = summarize(cycles, a.cycles, a.method)
    print(text, flush=True)
    (out / f"{a.method}-summary.txt").write_text(text + "\n")
    say("Trials finished.", a.quiet)
    return code


def wait_cooldown(port: str, max_temp: int, limit_s: float = 600):
    bus, _ = open_watch_bus(port)
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < limit_s:
            temp = read_raw(bus, TEMP, 1, GID)[0]
            if temp is not None and temp <= max_temp:
                return
            print(f"cooling: gripper {temp} C > {max_temp} C", flush=True)
            time.sleep(5)
        sys.exit("refusing: gripper did not cool within 10 min")
    finally:
        bus.port_handler.closePort()


def self_check(port: str) -> int:
    """FA-16: show the harness refuses when another process holds the port. Opens the port only; sends nothing."""
    holder = subprocess.Popen(
        [sys.executable, "-c", f"import serial, time; s = serial.Serial({port!r}); time.sleep(5)"]
    )
    time.sleep(1.5)
    try:
        holders = port_holders(port)
        refused = bool(holders)
        print(
            f"self-check: holders={holders} -> {'refused (good)' if refused else 'NOT refused (bad)'}",
            flush=True,
        )
        return 0 if refused else 1
    finally:
        holder.terminate()
        holder.wait()


def goal_probe(port: str, out: Path) -> int:
    """FA-05: does a Goal_Position write re-enable torque on HX-30HM? Gripper only, Goal = current position."""
    out.mkdir(parents=True, exist_ok=True)
    bus = make_bus(port)
    bus.port_handler.openPort()
    res = {}
    try:
        te = [read_raw(bus, TE, 1, GID)[0] for _ in range(2)]
        res["te_before"] = te
        if te != [0, 0]:
            res["aborted"] = "gripper torque not confirmed off before the probe"
            return 1
        present = read_raw(bus, PRESENT, 2, GID)[0]
        res["present"] = present
        if present is None or present & 0x8000:
            res["aborted"] = f"bad Present_Position {present}"
            return 1
        res["goal_write"] = bus._write(GOAL, 2, GID, present, raise_on_error=False)
        readings = []
        for _ in range(3):
            readings.append(read_raw(bus, TE, 1, GID)[0])
            time.sleep(0.25)
        res["te_after_goal"] = readings
        res["goal_write_reenables_torque"] = any(v == 1 for v in readings)
        bus.disable_torque(["gripper"])
        res["verified_off_after"] = True
        return 0
    except Exception as e:  # noqa: BLE001
        res["error"] = repr(e)
        return 1
    finally:
        bus.port_handler.closePort()
        (out / "goal_probe.json").write_text(json.dumps(res, indent=1, default=str))
        print(json.dumps(res), flush=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out")
    ap.add_argument("--method", choices=["new", "old"], default="new")
    ap.add_argument("--cycles", type=int, default=5)
    ap.add_argument("--watch-s", type=float, default=90)
    ap.add_argument("--phase", choices=["stall"])
    ap.add_argument("--self-check", action="store_true")
    ap.add_argument("--goal-probe", action="store_true")
    ap.add_argument("--quiet", action="store_true", help="no spoken cues")
    a = ap.parse_args()
    if problems := import_guard():
        sys.exit("refusing: " + "; ".join(problems))
    if not a.port.startswith("/dev/cu."):
        sys.exit("refusing: real runs need a /dev/cu.* port")
    if a.self_check:
        return self_check(a.port)
    if not a.out:
        sys.exit("--out is required")
    if a.goal_probe:
        return goal_probe(a.port, Path(a.out))
    if a.phase == "stall":
        phase_stall(a.port, a.method, Path(a.out))
        return 0
    return run_cycles(a)


if __name__ == "__main__":
    sys.exit(main())
