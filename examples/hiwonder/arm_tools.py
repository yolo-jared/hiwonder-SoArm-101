"""Pure helpers for the Hiwonder SO-ARM101 teleop start and arm check scripts. No hardware access here.

Units: arm joints in degrees (LeRobot DEGREES mode: 0 = middle of the calibrated range), gripper in 0-100.
Self-test plans are in raw encoder ticks.
"""

import math

GRIPPER = "gripper"
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", GRIPPER]
TICKS_PER_DEG = 4095 / 360

PASS, FAIL, INCOMPLETE = "PASS", "FAIL", "INCOMPLETE"
STATUS_READ_ERROR = -1  # a Status read that raised (comm), recorded and counted, not a motor fault

SPOKEN = {
    "shoulder_pan": "base rotation",
    "shoulder_lift": "shoulder",
    "elbow_flex": "elbow",
    "wrist_flex": "wrist bend",
    "wrist_roll": "wrist roll",
    GRIPPER: "gripper",
}

# Start guard: leader within this of the follower before the first send (deg; gripper in percent).
MATCH_TOL = {
    "shoulder_pan": 5.0,
    "shoulder_lift": 5.0,
    "elbow_flex": 5.0,
    "wrist_flex": 8.0,
    "wrist_roll": 10.0,
    GRIPPER: 10.0,
}

# Guided joint check: leader travel needed for a verdict, and follower error allowed once the leader is still.
MIN_TRAVEL = {
    "shoulder_pan": 30.0,
    "shoulder_lift": 30.0,
    "elbow_flex": 30.0,
    "wrist_flex": 30.0,
    "wrist_roll": 90.0,
    GRIPPER: 50.0,
}
SETTLE_TOL = {
    "shoulder_pan": 2.0,
    "shoulder_lift": 4.0,
    "elbow_flex": 4.0,
    "wrist_flex": 2.0,
    "wrist_roll": 2.0,
    GRIPPER: 5.0,
}
FOLLOWER_TRAVEL_FRACTION = 0.9
STILL_RANGE = 0.5  # leader moved less than this ...
STILL_S = 1.0  # ... over this long counts as still

# Self-test planner (raw ticks).
RANGE_MARGIN_DEG = 2.0
MIN_ROOM_DEG = 5.0
GRIPPER_CLOSED_MARGIN = 0.10  # never close below range_min + 10 % of the range (jaws meet before range_min)


def spoken_name(joint):
    return SPOKEN[joint]


def unit(joint):
    return "percent" if joint == GRIPPER else "degrees"


def calibration_offsets(leader_cal, follower_cal):
    """Leader minus follower range midpoint per joint, in degrees (None for the gripper).

    DEGREES mode puts 0 at the middle of each arm's calibrated range, so a midpoint gap means equal numbers are
    that many degrees apart physically.
    """
    for joint in set(leader_cal) ^ set(follower_cal):
        raise ValueError(f"{joint} is calibrated on only one arm")
    out = {}
    for joint, lc in leader_cal.items():
        if joint == GRIPPER:
            out[joint] = None
            continue
        fc = follower_cal[joint]
        lmid = (lc["range_min"] + lc["range_max"]) / 2
        fmid = (fc["range_min"] + fc["range_max"]) / 2
        out[joint] = (lmid - fmid) / TICKS_PER_DEG
    return out


class MatchGuide:
    """Turns leader-minus-follower gaps into one short spoken cue at a time, one joint at a time, in order."""

    def __init__(self, order, tol, min_step=1.0):
        self.order, self.tol, self.min_step = list(order), dict(tol), min_step
        self.focus, self.ref = None, None

    def update(self, diff):
        gaps = {j: abs(diff[j]) for j in self.order}
        unmatched = [j for j in self.order if gaps[j] > self.tol[j]]
        if not unmatched:
            self.focus = self.ref = None
            return True, "All joints matched."
        joint, gap = unmatched[0], gaps[unmatched[0]]
        name = SPOKEN[joint]
        if joint != self.focus:
            prev = self.focus
            prefix = f"{SPOKEN[prev].capitalize()} matched. " if prev and gaps[prev] <= self.tol[prev] else ""
            self.focus, self.ref = joint, gap
            return False, f"{prefix}{name.capitalize()}: {gap:.0f} {unit(joint)} off. Move the leader {name}."
        if gap <= self.ref - self.min_step:
            self.ref = gap
            return False, f"{name.capitalize()} closer, {gap:.0f}."
        if gap >= self.ref + self.min_step:
            self.ref = gap
            return False, f"{name.capitalize()} wrong way, go back. {gap:.0f}."
        return False, None


def _still_errors(t, leader, follower):
    errors = []
    j = 0
    for k in range(len(t)):
        while j < k and t[k] - t[j + 1] >= STILL_S:
            j += 1
        if t[k] - t[j] < STILL_S:
            continue
        seg = leader[j : k + 1]
        if max(seg) - min(seg) < STILL_RANGE:
            errors.append(abs(leader[k] - follower[k]))
    return errors


def evaluate_joint(joint, t, leader, follower, status):
    """PASS / FAIL / INCOMPLETE for one joint the operator moved through the leader."""
    travel = max(leader) - min(leader)
    f_travel = max(follower) - min(follower)
    metrics = {
        "leader_travel": round(travel, 1),
        "follower_travel": round(f_travel, 1),
        "peak_gap": round(max(abs(a - b) for a, b in zip(leader, follower, strict=True)), 1),
    }
    if travel < MIN_TRAVEL[joint]:
        reason = f"leader moved {travel:.0f} {unit(joint)}, needs {MIN_TRAVEL[joint]:.0f}"
        return {"joint": joint, "verdict": INCOMPLETE, "reasons": [reason], "metrics": metrics}
    reasons = []
    if f_travel < FOLLOWER_TRAVEL_FRACTION * travel:
        reasons.append(f"follower moved {f_travel:.0f} of the leader's {travel:.0f} {unit(joint)}")
    faults = sorted({s for s in status if s > 0})
    metrics["status_read_errors"] = sum(s == STATUS_READ_ERROR for s in status)
    if faults:
        reasons.append(f"status flags {[hex(s) for s in faults]}")
    errors = _still_errors(t, leader, follower)
    if errors:
        metrics["settle_max"] = round(max(errors), 2)
        if max(errors) > SETTLE_TOL[joint]:
            reasons.append(f"settle error {max(errors):.1f} > {SETTLE_TOL[joint]:.1f} {unit(joint)}")
    if reasons:
        return {"joint": joint, "verdict": FAIL, "reasons": reasons, "metrics": metrics}
    if not errors:
        return {
            "joint": joint,
            "verdict": INCOMPLETE,
            "reasons": ["leader never held still for 1 s"],
            "metrics": metrics,
        }
    return {"joint": joint, "verdict": PASS, "reasons": [], "metrics": metrics}


def find_grasp(leader, delta=30.0):
    """Index of the lowest leader gripper reading that has an open (>= delta higher) both before and after it."""
    pre, suf = [], []
    for v in leader:
        pre.append(max(v, pre[-1]) if pre else v)
    for v in reversed(leader):
        suf.append(max(v, suf[-1]) if suf else v)
    suf.reverse()
    found = [k for k, v in enumerate(leader) if pre[k] - v >= delta and suf[k] - v >= delta]
    return min(found, key=leader.__getitem__) if found else None


def evaluate_grasp(t, leader, follower, status):
    """Gripper closes on an object (follower blocked short of the leader) and must open again afterwards."""
    k_min = find_grasp(leader)
    metrics = {
        "status_seen": sorted({s for s in status if s > 0}),
        "status_read_errors": sum(s == STATUS_READ_ERROR for s in status),
    }
    if k_min is None:
        return {
            "joint": GRIPPER,
            "verdict": INCOMPLETE,
            "metrics": metrics,
            "reasons": ["leader gripper must close at least 30 percent, then open at least 30 percent"],
        }
    closed = leader[k_min]
    lo = hi = k_min  # the contiguous closed stretch around the grasp
    while lo > 0 and leader[lo - 1] - closed <= 2.0:
        lo -= 1
    while hi + 1 < len(leader) and leader[hi + 1] - closed <= 2.0:
        hi += 1
    held = [follower[i] - leader[i] for i in range(lo, hi + 1)]
    metrics["blocked_gap"] = round(max(held), 1)
    final_gap = abs(leader[-1] - follower[-1])
    metrics["final_gap"] = round(final_gap, 1)
    if final_gap > SETTLE_TOL[GRIPPER]:
        return {
            "joint": GRIPPER,
            "verdict": FAIL,
            "metrics": metrics,
            "reasons": [f"follower did not reopen: {final_gap:.0f} percent from the leader at the end"],
        }
    return {"joint": GRIPPER, "verdict": PASS, "reasons": [], "metrics": metrics}


def clamp_goal(value, calibration):
    """A goal inside the calibrated range (the servo rejects one outside it with an Angle error)."""
    return min(max(value, calibration["range_min"]), calibration["range_max"])


def plan_self_move(joint, start, calibration, amp_deg):
    """Raw-tick targets for a follower self-test of one joint: out and back on each side that has room."""
    margin = round(RANGE_MARGIN_DEG * TICKS_PER_DEG)
    lo = calibration["range_min"] + margin
    hi = calibration["range_max"] - margin
    if joint == GRIPPER:
        span = calibration["range_max"] - calibration["range_min"]
        lo = max(lo, calibration["range_min"] + GRIPPER_CLOSED_MARGIN * span)
    amp = round(amp_deg * TICKS_PER_DEG)
    min_room = MIN_ROOM_DEG * TICKS_PER_DEG
    start = clamp_goal(
        start, calibration
    )  # a joint can rest a few ticks past its range; the servo refuses that goal
    sides = []  # (room, out_target): a short side is near a range end, where the arm may touch itself or the base
    if hi - start >= min_room:
        sides.append((hi - start, start + min(amp, math.floor(hi - start))))
    if start - lo >= min_room:
        sides.append((start - lo, start - min(amp, math.floor(start - lo))))
    full = [s for s in sides if s[0] >= amp]
    plan = [x for _, out in (full or sides) for x in (out, start)]
    if not plan:
        raise ValueError(
            f"{joint}: no room to move at least {MIN_ROOM_DEG:.0f} degrees inside its calibrated range"
        )
    return plan


def probe_ok(start, target, after):
    """A probe passed if the follower covered at least half the commanded move, in the commanded direction."""
    commanded = target - start
    return (after - start) * math.copysign(1, commanded) >= 0.5 * abs(commanded)


def ramp(start, target, max_step):
    """Intermediate goal positions from start to target, each step at most max_step."""
    n = math.ceil(abs(target - start) / max_step)
    if n == 0:
        return [target]
    return [round(start + (target - start) * i / n) for i in range(1, n + 1)]
