"""Check a Hiwonder SO-ARM101 pair after setup or recalibration. Three modes, run in this order:

  calibration  No motion. Compares leader and follower calibration files joint by joint.
  self         Follower only. The script moves each follower joint out and back on its own (15 deg, gripper
               opens only) after a small probe move, and checks it arrives. Hands clear of the follower.
  teleop       Both arms. Guided pose match, then one spoken step per joint: you move the leader joint through
               a wide range and hold still; the script checks the follower followed. Ends with a gripper grasp
               on a soft object resting in the follower's open jaws.

Every joint gets PASS, FAIL or INCOMPLETE (not enough movement to judge). Results go to --out as CSV and JSON.
Ctrl+C, SIGTERM and SIGHUP turn torque off.

    PYTHONPATH=src uv run --no-sync python examples/hiwonder/arm_check.py MODE \
        --leader-port=/dev/cu.usbmodemLEADER --leader-id=my_leader \
        --follower-port=/dev/cu.usbmodemFOLLOWER --follower-id=my_follower
"""

import argparse
import csv
import json
import logging
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from examples.hiwonder.arm_tools import (  # noqa: E402
    FAIL,
    GRIPPER,
    INCOMPLETE,
    JOINTS,
    MIN_TRAVEL,
    PASS,
    STILL_RANGE,
    TICKS_PER_DEG,
    calibration_offsets,
    clamp_goal,
    evaluate_grasp,
    evaluate_joint,
    plan_self_move,
    probe_ok,
    ramp,
    spoken_name,
)
from examples.hiwonder.session import (  # noqa: E402
    Speaker,
    guided_match,
    make_pair,
    require_calibrated,
    require_this_checkout,
)

CALIBRATION_LIMIT_DEG = 5.0
SELF_ORDER = ["wrist_roll", "wrist_flex", GRIPPER, "elbow_flex", "shoulder_pan", "shoulder_lift"]
SELF_AMP_DEG = 15.0
PROBE_DEG = 6.0
SELF_SPEED_DEG_S = 20.0
SELF_PERIOD_S = 1 / 30
STALL_DEG = 8.0
STALL_S = 0.5
SELF_TOL_DEG = {"shoulder_lift": 4.0, "elbow_flex": 4.0}  # gravity droop; others 3
TELEOP_ORDER = ["shoulder_pan", "wrist_flex", "wrist_roll", GRIPPER, "shoulder_lift", "elbow_flex"]
STEP_TIMEOUT_S = 45.0
STEP_STILL_S = 1.5


def cal_dict(cal):
    return {j: {"range_min": m.range_min, "range_max": m.range_max} for j, m in cal.items()}


def print_table(results):
    print(f"\n{'joint':<14} {'verdict':<11} detail", flush=True)
    for r in results:
        print(
            f"{r['joint']:<14} {r['verdict']:<11} {'; '.join(r['reasons']) or r.get('metrics', '')}",
            flush=True,
        )


def summarize(results, speaker):
    print_table(results)
    bad = [r for r in results if r["verdict"] != PASS]
    if bad:
        speaker.say(
            "Check done. "
            + ". ".join(f"{spoken_name(r['joint'])} {r['verdict'].lower()}" for r in bad)
            + ".",
            wait=True,
        )
    else:
        speaker.say("Check done. Every joint passed.", wait=True)
    return 0 if not bad else 1


# ---------- calibration ----------


def check_calibration(args, out):
    teleop, robot = make_pair(args)  # constructing reads the calibration files; no port is opened
    offsets = calibration_offsets(cal_dict(teleop.calibration), cal_dict(robot.calibration))
    results = []
    for joint in JOINTS:
        off = offsets[joint]
        if off is None:
            results.append(
                {"joint": joint, "verdict": PASS, "reasons": [], "metrics": "0-100 units, not compared"}
            )
        elif abs(off) > CALIBRATION_LIMIT_DEG:
            results.append(
                {
                    "joint": joint,
                    "verdict": FAIL,
                    "metrics": {"offset_deg": round(off, 1)},
                    "reasons": [
                        f"calibrations {off:+.1f} deg apart: equal numbers point the arms that far "
                        "apart. Recalibrate both arms"
                    ],
                }
            )
        else:
            results.append(
                {"joint": joint, "verdict": PASS, "reasons": [], "metrics": {"offset_deg": round(off, 1)}}
            )
    (out / "calibration.json").write_text(json.dumps(results, indent=2))
    return results


# ---------- self ----------


def move_joint(bus, joint, target, calibration, log):
    """Ramp one joint's goal to target (kept inside the calibrated range); return (reached, stalled)."""
    start = bus.read("Present_Position", joint, normalize=False)
    target = clamp_goal(target, calibration)
    over_since = None
    for goal in ramp(start, target, max_step=max(1, round(SELF_SPEED_DEG_S * SELF_PERIOD_S * TICKS_PER_DEG))):
        goal = clamp_goal(goal, calibration)
        t0 = time.monotonic()
        bus.write("Goal_Position", joint, goal, normalize=False)
        present = bus.read("Present_Position", joint, normalize=False)
        log.append({"t": round(time.monotonic(), 3), "joint": joint, "goal": goal, "present": present})
        if abs(goal - present) > STALL_DEG * TICKS_PER_DEG:
            over_since = over_since or t0
            if t0 - over_since > STALL_S:
                return present, True
        else:
            over_since = None
        time.sleep(max(0.0, SELF_PERIOD_S - (time.monotonic() - t0)))
    time.sleep(0.7)  # settle
    return bus.read("Present_Position", joint, normalize=False), False


def self_test_joint(bus, joint, calibration, log):
    start = bus.read("Present_Position", joint, normalize=False)
    home = clamp_goal(start, calibration)
    tol = SELF_TOL_DEG.get(joint, 3.0) * TICKS_PER_DEG
    plan = plan_self_move(joint, start, calibration, SELF_AMP_DEG)
    sides = [plan[i : i + 2] for i in range(0, len(plan), 2)]
    done, reasons, metrics = 0, [], {"start": start, "home": home, "sides": []}
    for out_target, back in sides:
        probe = home + round(PROBE_DEG * TICKS_PER_DEG) * (1 if out_target > home else -1)
        after, stalled = move_joint(bus, joint, probe, calibration, log)
        if stalled or not probe_ok(home, probe, after):
            move_joint(bus, joint, home, calibration, log)
            metrics["sides"].append(
                {"target": out_target, "probe_after": after, "skipped": "blocked or did not move"}
            )
            continue
        reached, stalled = move_joint(bus, joint, out_target, calibration, log)
        returned, stalled_back = move_joint(bus, joint, back, calibration, log)
        status = read_status(bus, joint)
        side = {
            "target": out_target,
            "reached": reached,
            "returned": returned,
            "status": status,
            "error_deg": round(abs(reached - out_target) / TICKS_PER_DEG, 2),
            "return_error_deg": round(abs(returned - back) / TICKS_PER_DEG, 2),
        }
        metrics["sides"].append(side)
        if stalled or stalled_back:
            reasons.append(f"stalled moving to {out_target if stalled else back}")
        elif abs(reached - out_target) > tol or abs(returned - back) > tol:
            reasons.append(f"missed target by {side['error_deg']} / {side['return_error_deg']} deg")
        if status:
            reasons.append(f"status flag {hex(status)}")
        done += 1
    if not done:
        reasons.append("did not move as commanded in either direction")
    return {"joint": joint, "verdict": FAIL if reasons else PASS, "reasons": reasons, "metrics": metrics}


def check_self(args, out, speaker):
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    from lerobot.robots.utils import make_robot_from_config
    from lerobot.utils.robot_utils import disconnect_all, exit_on_termination_signals

    robot = make_robot_from_config(
        SOFollowerRobotConfig(port=args.follower_port, id=args.follower_id, max_relative_target=None)
    )
    results, log, connected = [], [], False
    with exit_on_termination_signals() as signals:
        try:
            robot.connect(calibrate=False)
            connected = True
            require_calibrated(robot, "follower")
            bus = robot.bus
            faults = {j: s for j, s in bus.sync_read("Status", normalize=False).items() if s}
            if faults:
                raise RuntimeError(f"motor fault before moving: {faults}")
            speaker.say(
                "Self test. The follower moves on its own. Hands clear. Starting in five seconds.", wait=True
            )
            time.sleep(5)
            for joint in SELF_ORDER:
                speaker.say(f"Testing {spoken_name(joint)}.")
                results.append(self_test_joint(bus, joint, cal_dict(robot.calibration)[joint], log))
                print(results[-1], flush=True)
        except KeyboardInterrupt:
            results.append({"joint": "-", "verdict": INCOMPLETE, "reasons": ["stopped with Ctrl+C"]})
        finally:
            signals.hold()
            disconnect_all(robot.disconnect if connected else None, signals.release)
            (out / "self.json").write_text(json.dumps(results, indent=2))
            if log:
                with open(out / "self.csv", "w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=list(log[0]))
                    w.writeheader()
                    w.writerows(log)
    return results


# ---------- teleop ----------


READ_ERROR = -1


def read_status(bus, joint):
    """Status register, or READ_ERROR: a motor flagging Overload can make the read itself raise."""
    try:
        return bus.read("Status", joint, normalize=False)
    except Exception:  # noqa: BLE001 - recorded as a fault, the step continues
        return READ_ERROR


def run_step(teleop, robot, joint, prompt, speaker, rows, done_when, fps=30):
    """Teleoperate (no cap) while the operator does one step; record until done_when(trace) or timeout."""
    speaker.say(prompt, wait=True)
    trace = {"t": [], "leader": [], "follower": [], "status": []}
    t0 = time.monotonic()
    while time.monotonic() - t0 < STEP_TIMEOUT_S:
        loop = time.monotonic()
        obs = robot.get_observation()
        act = teleop.get_action()
        robot.send_action(act)
        status = read_status(robot.bus, joint) if len(rows) % 10 == 0 else 0
        t = loop - t0
        trace["t"].append(t)
        trace["leader"].append(act[f"{joint}.pos"])
        trace["follower"].append(obs[f"{joint}.pos"])
        trace["status"].append(status)
        rows.append(
            {
                "step": prompt[:20],
                "joint": joint,
                "t": round(t, 3),
                "status": status,
                **{f"{j}.leader": round(act[f"{j}.pos"], 2) for j in JOINTS},
                **{f"{j}.follower": round(obs[f"{j}.pos"], 2) for j in JOINTS},
            }
        )
        if done_when(trace):
            break
        time.sleep(max(0.0, 1 / fps - (time.monotonic() - loop)))
    return trace


def still_for(trace, seconds):
    t, lead = trace["t"], trace["leader"]
    if not t or t[-1] - t[0] < seconds:
        return False
    seg = [v for tt, v in zip(t, lead, strict=True) if t[-1] - tt <= seconds]
    return max(seg) - min(seg) < STILL_RANGE


def joint_done(joint):
    def done(trace):
        lead = trace["leader"]
        return max(lead) - min(lead) >= MIN_TRAVEL[joint] and still_for(trace, STEP_STILL_S)

    return done


def grasp_done(trace):
    lead = trace["leader"]
    k = min(range(len(lead)), key=lead.__getitem__)
    return (
        max(lead[: k + 1]) - lead[k] >= 30
        and max(lead[k:]) - lead[k] >= 30
        and still_for(trace, STEP_STILL_S)
    )


def check_teleop(args, out, speaker):
    from lerobot.utils.robot_utils import disconnect_all, exit_on_termination_signals

    teleop, robot = make_pair(args)
    results, rows = [], []
    robot_connected = teleop_connected = False
    with exit_on_termination_signals() as signals:
        try:
            teleop.connect(calibrate=False)
            teleop_connected = True
            require_calibrated(teleop, "leader")
            robot.connect(calibrate=False)
            robot_connected = True
            require_calibrated(robot, "follower")
            guided_match(teleop, robot, speaker)
            for joint in TELEOP_ORDER:
                name = spoken_name(joint)
                if joint == GRIPPER:
                    prompt = (
                        f"Next: {name}. Open and close the leader gripper fully, twice, then leave it half "
                        "open and hold still."
                    )
                else:
                    prompt = (
                        f"Next: {name}. Move the leader {name} slowly far one way, then far the other way, "
                        "then back to the middle, and hold still."
                    )
                tr = run_step(teleop, robot, joint, prompt, speaker, rows, joint_done(joint))
                results.append(evaluate_joint(joint, tr["t"], tr["leader"], tr["follower"], tr["status"]))
                speaker.say(f"{name} {results[-1]['verdict'].lower()}.")
            prompt = (
                "Gripper grasp. Open the leader gripper. Lay a soft object in the follower's open jaws, then "
                "go back to the leader. Close the leader gripper fully, hold three seconds, open it fully, "
                "and hold still."
            )
            tr = run_step(teleop, robot, GRIPPER, prompt, speaker, rows, grasp_done)
            grasp = evaluate_grasp(tr["t"], tr["leader"], tr["follower"], tr["status"])
            grasp["joint"] = "gripper grasp"
            results.append(grasp)
        except KeyboardInterrupt:
            results.append({"joint": "-", "verdict": INCOMPLETE, "reasons": ["stopped with Ctrl+C"]})
        finally:
            signals.hold()
            disconnect_all(
                robot.disconnect if robot_connected else None,
                teleop.disconnect if teleop_connected else None,
                signals.release,
            )
            (out / "teleop.json").write_text(json.dumps(results, indent=2))
            if rows:
                with open(out / "teleop.csv", "w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=list(rows[0]))
                    w.writeheader()
                    w.writerows(rows)
    return results


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("mode", choices=["calibration", "self", "teleop"])
    parser.add_argument("--leader-port", default="")
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--leader-id", required=True)
    parser.add_argument("--follower-id", required=True)
    parser.add_argument(
        "--out", default=None, help="results folder (default outputs/arm_check/<time>-<mode>)"
    )
    parser.add_argument("--quiet", action="store_true", help="no speech; cues are still printed")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(asctime)s %(message)s")
    if args.mode == "teleop" and not args.leader_port:
        parser.error("teleop mode needs --leader-port")
    require_this_checkout()
    out = Path(args.out or f"outputs/arm_check/{datetime.now():%Y%m%d-%H%M%S}-{args.mode}")
    out.mkdir(parents=True, exist_ok=True)
    speaker = Speaker(args.quiet)
    if args.mode == "calibration":
        results = check_calibration(args, out)
    elif args.mode == "self":
        results = check_self(args, out, speaker)
    else:
        results = check_teleop(args, out, speaker)
    print(f"Results: {out}", flush=True)
    raise SystemExit(summarize(results, speaker))


if __name__ == "__main__":
    main()
