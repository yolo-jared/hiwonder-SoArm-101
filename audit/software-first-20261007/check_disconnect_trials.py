"""Offline checks for lerobot_disconnect_trials.py (no serial port). Run before any hardware cycle.

python -I check_disconnect_trials.py      (exit 0 = every check behaved as expected)
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import lerobot_disconnect_trials as h  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += s


def snaps(script):
    """script: list of (te_by_id or None-for-error, temp) per sample; ids 1-6."""
    it = iter(script)

    def sample():
        te, temp = next(it)
        snap = {
            i: {
                "te": te.get(i, 0) if te is not None else None,
                "status": 0,
                "err": None if te is not None else "timeout",
            }
            for i in range(1, 7)
        }
        snap["temp"] = temp
        return snap

    return sample


results = []


def check(name, ok, detail=""):
    results.append((name, bool(ok), detail))


# FA-18: AC8 watch stops at the first Torque_Enable=1 (sample 3), not after 90 s.
c = Clock()
script = [({}, 40), ({}, 41), ({6: 1}, 42)] + [({}, 43)] * 500
w = h.watch(snaps(script), 90, now=c.now, sleep=c.sleep)
check(
    "test_ac8_stops_watch_on_first_torque_on",
    w["verdict"] == "FAIL" and w["samples"] == 3 and w["first_on"]["id"] == 6,
    w["first_on"],
)

# FA-18: temperature >= 60 C aborts the watch.
c = Clock()
w = h.watch(snaps([({}, 55), ({}, 58), ({}, 60)] + [({}, 61)] * 500), 90, now=c.now, sleep=c.sleep)
check("watch aborts at 60 C", w["verdict"] == "ABORT" and w["samples"] == 3, w.get("abort"))

# AC7: a read error is unknown; it passes only if that motor's next read is a valid 0.
c = Clock()
w = h.watch(snaps([({}, 40), (None, 40)] + [({}, 40)] * 500), 2, now=c.now, sleep=c.sleep)
check("read error then valid 0 passes", w["verdict"] == "PASS" and w["unknown"] == [], w["verdict"])
c = Clock()
w = h.watch(snaps([({}, 40)] * 3 + [(None, 40)] * 500), 2, now=c.now, sleep=c.sleep)
check(
    "read error at the end fails", w["verdict"] == "FAIL" and w["unknown"] == [1, 2, 3, 4, 5, 6], w["unknown"]
)
c = Clock()
w = h.watch(snaps([({}, 40)] * 500), 90, now=c.now, sleep=c.sleep)
check(
    "clean 90 s watch passes with ~180 samples",
    w["verdict"] == "PASS" and 170 <= w["samples"] <= 181,
    w["samples"],
)

# FA-30: a cycle that never overloaded does not count; 0 exercised = INCONCLUSIVE, non-zero exit.
cycles = [{"cycle": k, "overload_seen": False, "watch_verdict": "PASS"} for k in range(1, 6)]
text, code = h.summarize(cycles, 5, "new")
check(
    "test_cycle_without_overload_is_not_counted",
    "0 exercised cycles — INCONCLUSIVE" in text and code == 2,
    text,
)

# FA-31: the bound line for N exercised cycles with 0 failures.
cycles = [{"cycle": k, "overload_seen": True, "watch_verdict": "PASS"} for k in range(1, 6)]
text, code = h.summarize(cycles, 5, "new")
check(
    "bound line for 0/5",
    "95% upper bound on recurrence rate: 0.451" in text and "RESULT: PASS" in text and code == 0,
    text,
)
text, _ = h.summarize(
    [{"cycle": k, "overload_seen": True, "watch_verdict": "PASS"} for k in range(1, 10)], 9, "new"
)
check("bound line for 0/9", "0.283" in text, text)

# A failed exercised cycle fails AC7; a raise from the shutdown counts as a failure.
cycles[2] = {
    "cycle": 3,
    "overload_seen": True,
    "watch_verdict": "PASS",
    "disconnect_error": "RuntimeError(...)",
}
text, code = h.summarize(cycles, 5, "new")
check("shutdown raise fails the run", "RESULT: FAIL" in text and code == 1, text)

# AC8: the old method never failing is reported as an open risk (exit 3), failing once as falsifiable (exit 0).
text, code = h.summarize([{"cycle": 1, "overload_seen": True, "watch_verdict": "PASS"}], 5, "old")
check(
    "AC8 never failing -> open risk", "negative control failed at least once: no" in text and code == 3, text
)
text, code = h.summarize([{"cycle": 1, "overload_seen": True, "watch_verdict": "FAIL"}], 5, "old")
check(
    "AC8 failing once -> falsifiable",
    "negative control failed at least once: yes" in text and code == 0,
    text,
)

# SC1: the import guard passes here and refuses when lerobot comes from elsewhere.
check("import guard passes in this worktree", h.import_guard() == [], h.import_guard())
real_src = h.WORKTREE_SRC
h.WORKTREE_SRC = Path("/nonexistent/src")
check(
    "import guard refuses another checkout",
    any("not /nonexistent/src" in p for p in h.import_guard()),
    h.import_guard(),
)
h.WORKTREE_SRC = real_src

for name, ok, detail in results:
    print("PASS" if ok else "FAIL", name, detail if not ok else "")
sys.exit(0 if all(ok for _, ok, _ in results) else 1)
