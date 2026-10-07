"""Repeat gripper overload cycles, each followed by a read-only torque watch, to measure torque-recurrence rate.

    python -I recurrence_trials.py --port /dev/cu.usbmodemXXXX --out DIR --n 5 --watch 90 [--hold 5]
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

ap = argparse.ArgumentParser()
ap.add_argument("--port", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--n", type=int, default=5)
ap.add_argument("--watch", type=float, default=90)
ap.add_argument("--hold", type=float, default=5)
a = ap.parse_args()
out = Path(a.out)
rows = []
for k in range(1, a.n + 1):
    d = out / f"trial{k}"
    subprocess.run(["/usr/bin/say", f"Trial {k} of {a.n}."], check=False)
    subprocess.run([sys.executable, "-I", str(HERE / "gripper_grasp.py"), "--port", a.port, "--out", str(d),
                    "--hold", str(a.hold), "--watch", str(a.watch)], check=False)
    s = json.loads((d / "summary.json").read_text())
    first = [e for e in s.get("torque_off_log", []) if e["op"] == "torque=0"]
    row = {"trial": k, "aborted": s.get("aborted"), "overload_at_s": s.get("overload_at_s"),
           "torque_off_verified": s.get("torque_off_verified"), "attempts": len(first),
           "first_readback": first[0]["readback"] if first else None,
           "recurred_after_s": s.get("torque_recurred_after_s"), "temp_end": s.get("open", {}).get("temp_end")}
    rows.append(row)
    print("TRIAL", json.dumps(row), flush=True)
    if s.get("aborted") or not s.get("torque_off_verified") or row["recurred_after_s"] is not None:
        print("stopping: abort, unverified torque-off, or recurrence", flush=True)
        break
(out / "trials.json").write_text(json.dumps(rows, indent=1))
