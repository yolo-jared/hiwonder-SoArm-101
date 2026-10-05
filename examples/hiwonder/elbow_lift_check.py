"""Small, operator-supervised elbow check; NOT general-purpose teleoperation.

Run from this checkout with `uv run --no-sync python examples/hiwonder/elbow_lift_check.py`.
The leader teaches an encoder direction while BOTH arms remain torque-off. A second
Enter authorizes a five-degree follower attempt and a controlled return to rest.
An encoder direction is not proof of upward Cartesian motion: watch the follower.
No calibration, PID, torque limit, or other persistent motor setting is changed.
"""

import argparse
import json
import math
import subprocess
import time
from contextlib import suppress
from datetime import datetime
from pathlib import Path

ELBOW = "elbow_flex"
TICKS_PER_DEGREE = 4095 / 360
LIFT_TICKS = 56  # Less than five degrees; fixed, deliberately not a CLI option.
END_SECONDS = 9.0  # Three up, one hold, three return, two settling.


def infer_direction(before, after):
    delta = after[ELBOW] - before[ELBOW]
    if not 3 * TICKS_PER_DEGREE <= abs(delta) <= 20 * TICKS_PER_DEGREE:
        raise ValueError("Rehearsal needs a small, clear elbow lift (3–20 degrees). No follower motion.")
    if any(abs(after[k] - v) > 3 * TICKS_PER_DEGREE for k, v in before.items() if k != ELBOW):
        raise ValueError("Other leader joints moved too much. Repeat with shoulder and wrist steady.")
    return 1 if delta > 0 else -1


def target_at(baseline, direction, elapsed):
    if direction not in (-1, 1) or not math.isfinite(elapsed):
        raise ValueError("Invalid trajectory input")
    if elapsed < 3:
        fraction = max(0, elapsed) / 3
    elif elapsed < 4:
        fraction = 1
    else:
        fraction = max(0, 1 - (elapsed - 4) / 3)
    return {**baseline, ELBOW: baseline[ELBOW] + direction * round(LIFT_TICKS * fraction)}


def validate_path(baseline, direction, calibration):
    if any(not isinstance(v, int) or not 0 <= v <= 4095 for v in baseline.values()):
        raise ValueError("Encoder position outside the supported single-turn range; diagnose before motion.")
    endpoint = target_at(baseline, direction, 3)[ELBOW]
    if not all(calibration.range_min <= p <= calibration.range_max for p in (baseline[ELBOW], endpoint)):
        raise ValueError("Lift would exceed the follower's recorded elbow range. No motion.")


def preload_and_enable(bus, baseline):
    present = bus.sync_read("Present_Position", normalize=False)
    if any(abs(present[k] - v) > 2 for k, v in baseline.items()):
        raise RuntimeError("Follower moved during preparation. No torque enabled.")
    # Never enable against a stale target left by an earlier program.
    bus.sync_write("Goal_Position", baseline, normalize=False)
    if bus.sync_read("Goal_Position", normalize=False) != baseline:
        raise RuntimeError("Current-position goal read-back failed. No torque enabled.")
    bus.enable_torque()


def check_feedback(baseline, target, actual, direction, status):
    if any(status.values()):
        raise RuntimeError(f"Motor fault: {status}")
    if direction * (actual[ELBOW] - baseline[ELBOW]) < -TICKS_PER_DEGREE:
        raise RuntimeError("Elbow moved opposite the taught encoder direction.")
    if any(abs(actual[k] - v) > 2 * TICKS_PER_DEGREE for k, v in baseline.items() if k != ELBOW):
        raise RuntimeError("A joint that should be holding still drifted more than two degrees.")
    if any(abs(actual[k] - v) > 4 * TICKS_PER_DEGREE for k, v in target.items()):
        raise RuntimeError("Follower lag exceeded four degrees. Stop and diagnose; do not raise the limits.")
    if abs(actual[ELBOW] - baseline[ELBOW]) > LIFT_TICKS + TICKS_PER_DEGREE:
        raise RuntimeError("Elbow exceeded the bounded travel envelope.")


def say(message):
    print(message, flush=True)
    subprocess.run(["/usr/bin/say", message], check=True, timeout=15)


def require_rest(bus):
    if any(bus.sync_read("Torque_Enable", normalize=False).values()):
        raise RuntimeError("Motors are already enabled. Stop the other program and settle the arm first.")
    if not bus.is_calibrated:
        raise RuntimeError(
            "Saved calibration differs from motor state. No automatic recalibration or motion."
        )
    if any(bus.sync_read("Operating_Mode", normalize=False).values()):
        raise RuntimeError("Motors are not all in position mode. No motion.")
    if any(bus.sync_read("Status", normalize=False).values()):
        raise RuntimeError("A motor reports a fault. No motion.")


def run(args):
    # Lazy imports let --help and software-only tests run without touching hardware.
    from serial.tools.list_ports import comports

    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    from lerobot.robots.so_follower.so_follower import SOFollower
    from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig
    from lerobot.teleoperators.so_leader.so_leader import SOLeader

    ports = {p.device: p.serial_number for p in comports()}
    if args.leader_port == args.follower_port or any(
        p not in ports for p in (args.leader_port, args.follower_port)
    ):
        raise RuntimeError("Two distinct, currently connected ports are required. Re-identify the arms.")
    print(f"Leader:   {args.leader_port}, USB serial {ports[args.leader_port]}")
    print(f"Follower: {args.follower_port}, USB serial {ports[args.follower_port]}")
    print("Confirm those physical identities; USB serial alone does not identify an arm's role.")
    print("Keep both bases clamped. Both arms start gently resting, with room to lift above the table.")
    print("Servo power connected; cutoff reachable. No hand on the follower during powered motion.")
    print("Normal finish returns to rest. An error/Ctrl+C releases torque; the arm can settle abruptly.")
    input("Press Enter for LEADER-ONLY rehearsal; the follower stays torque-off: ")

    leader = SOLeader(SOLeaderTeleopConfig(port=args.leader_port, id=args.leader_id))
    follower = SOFollower(SOFollowerRobotConfig(port=args.follower_port, id=args.follower_id))
    if not leader.calibration or not follower.calibration:
        raise RuntimeError("Both named calibration files must already exist. No motion.")
    buses = [leader.bus, follower.bus]
    attempted_enable = False
    cleanup_errors = []
    log_dir = Path("test-logs")
    log_dir.mkdir(exist_ok=True)
    log_path = log_dir / f"elbow-lift-{datetime.now():%Y%m%d-%H%M%S}.log"
    print(f"Diagnostic log: {log_path.resolve()}")
    try:
        for bus in buses:
            # Do NOT call follower.connect(): its configure() re-enables torque.
            bus.connect()
            require_rest(bus)
        say("Rehearsal only. Both motors are off. Leave the leader resting until I say lift.")
        before = leader.bus.sync_read("Present_Position", normalize=False)
        say(
            "Lift the leader gripper slowly by bending only its elbow. Keep the shoulder and wrist steady. Hold there."
        )
        time.sleep(15)
        after = leader.bus.sync_read("Present_Position", normalize=False)
        direction = infer_direction(before, after)
        say(
            "Rehearsal complete. Put the leader back down. Then press Enter with your hands clear of the follower."
        )
        print("The follower will attempt less than five degrees in the taught elbow direction.")
        print("That direction is NOT a verified upward path. Cut power if it presses down or binds.")
        input("Press Enter to authorize the small follower lift and return: ")
        say("Follower test in ten seconds. Leave both arms resting. Keep hands clear. Watch the follower.")
        time.sleep(7)
        for word in ("Three", "Two", "One"):
            say(word)
            time.sleep(1)
        for bus in buses:
            require_rest(bus)
        baseline = follower.bus.sync_read("Present_Position", normalize=False)
        validate_path(baseline, direction, follower.calibration[ELBOW])
        say("Starting the small lift. It will return to rest automatically. Keep clear.")
        with log_path.open("w") as log:
            log.write(
                json.dumps(
                    {
                        "baseline": baseline,
                        "leader_before": before,
                        "leader_after": after,
                        "direction": direction,
                    }
                )
                + "\n"
            )
            log.flush()
            attempted_enable = True  # Covers a partially successful enable, too.
            preload_and_enable(follower.bus, baseline)
            if any(v != 1 for v in follower.bus.sync_read("Torque_Enable", normalize=False).values()):
                raise RuntimeError("Not all follower motors enabled.")
            start = time.monotonic()
            last_elapsed = 0.0
            while True:
                elapsed = time.monotonic() - start
                if elapsed - last_elapsed > 0.5:
                    raise RuntimeError("Control loop stalled. No catch-up jump will be commanded.")
                last_elapsed = elapsed
                target = target_at(baseline, direction, elapsed)
                actual = follower.bus.sync_read("Present_Position", normalize=False)
                status = follower.bus.sync_read("Status", normalize=False)
                log.write(
                    json.dumps({"t": round(elapsed, 3), "target": target, "actual": actual, "status": status})
                    + "\n"
                )
                log.flush()
                check_feedback(baseline, target, actual, direction, status)
                follower.bus.sync_write("Goal_Position", target, normalize=False)
                if elapsed >= END_SECONDS:
                    if any(abs(actual[k] - v) > 2 * TICKS_PER_DEGREE for k, v in baseline.items()):
                        raise RuntimeError("Follower did not return close enough to its starting position.")
                    break
                time.sleep(0.1)
    finally:
        if attempted_enable and follower.bus.is_connected:
            try:
                follower.bus.disable_torque(num_retry=3)  # Includes motor-by-motor read-back.
            except Exception as exc:
                cleanup_errors.append(str(exc))
        for bus in reversed(buses):
            if bus.is_connected:
                try:
                    bus.disconnect(disable_torque=False)
                except Exception as exc:
                    cleanup_errors.append(str(exc))
        if cleanup_errors:
            print(f"CUT SERVO POWER NOW. Shutdown could not be verified: {cleanup_errors}", flush=True)
            say("Cut servo power now. Shutdown could not be verified.")
            raise RuntimeError("Unverified shutdown")
    say("Test finished. Follower motor torque verified off. You can check the terminal.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--leader-id", required=True)
    parser.add_argument("--follower-id", required=True)
    args = parser.parse_args()
    try:
        run(args)
    except (Exception, KeyboardInterrupt) as exc:
        print(f"STOPPED: {exc or 'Ctrl+C'}", flush=True)
        with suppress(Exception):
            say("Test stopped. Check the terminal before trying again.")
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
