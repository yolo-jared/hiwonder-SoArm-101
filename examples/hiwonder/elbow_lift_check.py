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

# Support both direct script invocation and package imports without importing hardware.
if __package__:
    from . import elbow_evidence as evidence, elbow_session as session
else:
    import elbow_evidence as evidence
    import elbow_session as session

ELBOW = "elbow_flex"
TICKS_PER_DEGREE = 4095 / 360
LIFT_TICKS = 56  # Less than five degrees; fixed, deliberately not a CLI option.
END_SECONDS = 9.0  # Three up, one hold, three return, two settling.


class RehearsalError(ValueError):
    """An operator-actionable read-only rehearsal result to speak verbatim."""


def require_joint(joint):
    if joint not in ("elbow_flex", "shoulder_lift"):
        raise ValueError("Only elbow_flex or shoulder_lift is supported")


def infer_direction(before, after, *, joint=ELBOW):
    require_joint(joint)
    label = joint.replace("_flex", "").replace("_", " ")
    delta = after[joint] - before[joint]
    if not 3 * TICKS_PER_DEGREE <= abs(delta) <= 20 * TICKS_PER_DEGREE:
        amount = "too little" if abs(delta) < 3 * TICKS_PER_DEGREE else "too much"
        raise RehearsalError(
            f"Rehearsal incomplete: {amount} {label} movement. "
            f"Measured {abs(delta) / TICKS_PER_DEGREE:.1f} degrees; required 3 to 20 degrees. "
            f"Try a small {label} bend, not the middle-range calibration pose. The follower was not enabled."
        )
    if any(abs(after[k] - v) > 3 * TICKS_PER_DEGREE for k, v in before.items() if k != joint):
        raise RehearsalError(
            "Rehearsal incomplete: other joints moved too much. "
            "Repeat with the other joints steady. The follower was not enabled."
        )
    return 1 if delta > 0 else -1


def target_at(baseline, direction, elapsed, *, joint=ELBOW):
    require_joint(joint)
    if direction not in (-1, 1) or not math.isfinite(elapsed):
        raise ValueError("Invalid trajectory input")
    if elapsed < 3:
        fraction = max(0, elapsed) / 3
    elif elapsed < 4:
        fraction = 1
    else:
        fraction = max(0, 1 - (elapsed - 4) / 3)
    return {**baseline, joint: baseline[joint] + direction * round(LIFT_TICKS * fraction)}


def validate_path(baseline, direction, calibration, *, joint=ELBOW):
    require_joint(joint)
    if any(not isinstance(v, int) or not 0 <= v <= 4095 for v in baseline.values()):
        raise ValueError("Encoder position outside the supported single-turn range; diagnose before motion.")
    endpoint = target_at(baseline, direction, 3, joint=joint)[joint]
    if not all(calibration.range_min <= p <= calibration.range_max for p in (baseline[joint], endpoint)):
        raise ValueError(f"Lift would exceed the follower's recorded {joint} range. No motion.")


def preload_and_enable(bus, baseline):
    present = bus.sync_read("Present_Position", normalize=False)
    if any(abs(present[k] - v) > 2 for k, v in baseline.items()):
        raise RuntimeError("Follower moved during preparation. No torque enabled.")
    # Never enable against a stale target left by an earlier program.
    bus.sync_write("Goal_Position", baseline, normalize=False)
    if bus.sync_read("Goal_Position", normalize=False) != baseline:
        raise RuntimeError("Current-position goal read-back failed. No torque enabled.")
    bus.enable_torque()


def check_feedback(baseline, target, actual, direction, status, *, joint=ELBOW):
    require_joint(joint)
    if any(status.values()):
        raise RuntimeError(f"Motor fault: {status}")
    if direction * (actual[joint] - baseline[joint]) < -TICKS_PER_DEGREE:
        raise RuntimeError(f"{joint} moved opposite the taught encoder direction.")
    if any(abs(actual[k] - v) > 2 * TICKS_PER_DEGREE for k, v in baseline.items() if k != joint):
        raise RuntimeError("A joint that should be holding still drifted more than two degrees.")
    if any(abs(actual[k] - v) > 4 * TICKS_PER_DEGREE for k, v in target.items()):
        raise RuntimeError("Follower lag exceeded four degrees. Stop and diagnose; do not raise the limits.")
    if abs(actual[joint] - baseline[joint]) > LIFT_TICKS + TICKS_PER_DEGREE:
        raise RuntimeError(f"{joint} exceeded the bounded travel envelope.")


def movement_report(peak_ticks):
    passed = peak_ticks >= 0.8 * LIFT_TICKS
    label = "ENCODER TRAVEL CHECK PASSED" if passed else "INCOMPLETE"
    return passed, (
        f"{label}: requested {LIFT_TICKS / TICKS_PER_DEGREE:.2f} degrees; "
        f"measured peak {peak_ticks / TICKS_PER_DEGREE:.2f} degrees "
        f"({100 * peak_ticks / LIFT_TICKS:.0f}% of requested travel). "
        "An upward physical lift still needs visual confirmation."
    )


def say(message):
    print(message, flush=True)
    subprocess.run(["/usr/bin/say", message], check=True, timeout=45)


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


def make_devices(args):
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    from lerobot.robots.so_follower.so_follower import SOFollower
    from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig
    from lerobot.teleoperators.so_leader.so_leader import SOLeader

    return (
        SOLeader(SOLeaderTeleopConfig(port=args.leader_port, id=args.leader_id)),
        SOFollower(SOFollowerRobotConfig(port=args.follower_port, id=args.follower_id)),
    )


def run(args):
    from serial.tools.list_ports import comports

    identities = session.identify_ports(args.leader_port, args.follower_port, comports())
    print(f"Leader: {args.leader_port}, USB serial {identities[0]}")
    print(f"Follower: {args.follower_port}, USB serial {identities[1]}")
    print("Confirm physical identities; USB serial alone does not establish arm role.")
    print("Close other serial tools. Locks only exclude cooperating diagnostics.")
    leader, follower = make_devices(args)
    if not leader.calibration or not follower.calibration:
        raise RuntimeError("Both named calibration files must exist. No motion.")
    buses = [session.BoundedBus(leader.bus), session.BoundedBus(follower.bus)]
    inspect_only = getattr(args, "inspect", False)
    log_dir = Path("test-logs")
    log_dir.mkdir(exist_ok=True)
    log_path = log_dir / f"elbow-lift-{datetime.now():%Y%m%d-%H%M%S-%f}.log"
    print(f"Diagnostic log: {log_path.resolve()}")
    records = []
    attempted_enable = False
    execution, shutdown_state = "rejected", "not attempted"
    guard_events, errors = [], []
    return_error = None
    hardware = session.HardwareSession(buses, identities)
    with evidence.Recorder(log_path) as recorder:

        def emit(event, **fields):
            with session.Deadline(session.IO_SECONDS):
                record = recorder.emit(event, **fields)
            records.append(record)

        emit(
            "start",
            adapter_serials=identities,
            mode="inspect" if inspect_only else "baseline",
            cleanup_budget_seconds=session.cleanup_budget(buses[1]),
        )
        try:
            with hardware:
                snapshots = [
                    session.snapshot(bus, device.calibration)
                    for bus, device in zip(buses, (leader, follower), strict=False)
                ]
                emit("configuration", snapshots=snapshots)
                if inspect_only:
                    for snapshot in snapshots:
                        for name, fields in snapshot.get("motors", {}).items():
                            for field, value in fields.items():
                                if isinstance(value, dict) and value.get("state") == "error":
                                    errors.append(
                                        f"Inspection read failed: {name}.{field}: {value.get('message')}"
                                    )
                    execution = "completed"
                else:
                    for bus in buses:
                        require_rest(bus)
                    print(
                        "Keep bases clamped, comparable resting poses, clear lift path, servo power on, cutoff reachable."
                    )
                    print(
                        "No hand on the powered follower. An error releases torque; it may settle abruptly."
                    )
                    input("Press Enter for LEADER-ONLY rehearsal; follower stays torque-off: ")
                    say("Rehearsal only. Both motors are off. Leave the leader resting until I say lift.")
                    before = buses[0].sync_read("Present_Position", normalize=False)
                    say(
                        "Lift the leader gripper slowly by bending only its elbow. Keep the shoulder and wrist steady. Hold there."
                    )
                    time.sleep(15)
                    after = buses[0].sync_read("Present_Position", normalize=False)
                    emit("rehearsal", before=before, after=after)
                    direction = infer_direction(before, after)
                    say(
                        "Rehearsal complete. Put the leader back down. Then press Enter with your hands clear of the follower."
                    )
                    print(
                        "Follower attempts less than five degrees. Direction is not a verified upward path."
                    )
                    print("Cut power if it presses down, binds, or oscillates.")
                    input("Press Enter to authorize the small follower lift and return: ")
                    say(
                        "Follower test in ten seconds. Leave both arms resting. Keep hands clear. Watch the follower."
                    )
                    time.sleep(7)
                    for word in ("Three", "Two", "One"):
                        say(word)
                        time.sleep(1)
                    say(
                        "Starting the small lift after configuration checks. Keep clear. It returns automatically."
                    )
                    for i, (bus, device) in enumerate(zip(buses, (leader, follower), strict=False)):
                        current = session.snapshot(bus, device.calibration)
                        emit("configuration_recheck", role=i, snapshot=current)
                        session.require_unchanged(snapshots[i], current)
                        require_rest(bus)
                    baseline = buses[1].sync_read("Present_Position", normalize=False)
                    validate_path(baseline, direction, follower.calibration[ELBOW])
                    emit("baseline", baseline=baseline, direction=direction)
                    try:
                        # Covers partial enable. Deadline covers preload and all torque writes.
                        attempted_enable = True
                        with session.Deadline(0.5):
                            preload_and_enable(buses[1], baseline)
                        if any(v != 1 for v in buses[1].sync_read("Torque_Enable", normalize=False).values()):
                            raise RuntimeError("Not all follower motors enabled")
                        start = time.monotonic()
                        previous_loop = start
                        last_sent = baseline[ELBOW]
                        while True:
                            loop_start = time.monotonic()
                            session.require_fresh(previous_loop, loop_start)
                            previous_loop = loop_start
                            elapsed = loop_start - start
                            target = target_at(baseline, direction, elapsed)
                            read_start = time.monotonic()
                            actual = buses[1].sync_read("Present_Position", normalize=False)
                            read_end = time.monotonic()
                            status = buses[1].sync_read("Status", normalize=False)
                            telemetry = {
                                name: buses[1].read(name, ELBOW, normalize=False)
                                for name in (
                                    "Goal_Position",
                                    "Torque_Enable",
                                    "Present_Load",
                                    "Present_Current",
                                    "Present_Voltage",
                                    "Present_Temperature",
                                )
                            }
                            emit(
                                "readback",
                                t=time.monotonic(),
                                goal=telemetry["Goal_Position"],
                                expected=last_sent,
                            )
                            emit(
                                "sample",
                                read_start=read_start,
                                read_end=read_end,
                                actual=actual,
                                intended_target=target,
                                previous_sent_elbow=last_sent,
                                status=status,
                                elbow_telemetry_raw=telemetry,
                            )
                            if telemetry["Torque_Enable"] != 1:
                                raise RuntimeError("Elbow torque became disabled")
                            if telemetry["Goal_Position"] != last_sent:
                                raise RuntimeError("Foreign or missing goal readback")
                            check_feedback(baseline, target, actual, direction, status)
                            # Includes feedback, retries, serialization and log flush, not just loop entry.
                            session.require_fresh(read_start, time.monotonic())
                            buses[1].sync_write("Goal_Position", target, normalize=False)
                            last_sent = target[ELBOW]
                            emit("command", t=time.monotonic(), goal=last_sent)
                            if elapsed >= END_SECONDS:
                                return_error = max(abs(actual[k] - v) for k, v in baseline.items())
                                if return_error > 2 * TICKS_PER_DEGREE:
                                    raise RuntimeError("Return error exceeds two degrees")
                                break
                            time.sleep(0.1)
                        execution = "completed"
                    finally:
                        shutdown_state = session.shutdown(buses[1])
        except (Exception, KeyboardInterrupt) as exc:
            execution = "aborted" if attempted_enable else "rejected"
            message = str(exc) or "Ctrl+C"
            guard_events.append(message)
            print(f"STOPPED: {message}", flush=True)
            # Do not synthesize speech until cleanup and finalization have been attempted.
        if hardware.close_errors:
            errors.extend(hardware.close_errors)
        final_fields = {
            "execution": execution,
            "shutdown": shutdown_state,
            "restoration": "not applicable",
            "return_error_ticks": return_error,
            "guard_events": guard_events,
            "errors": errors,
        }
        provisional = dict(final_fields, event="final", version=1, run_id=recorder.run_id, seq=recorder.seq)
        measured = evidence.analyze([*records, provisional])
        for field in (
            "encoder",
            "requested_ticks",
            "peak_ticks",
            "hold_median_ticks",
            "tracking_error_ticks",
        ):
            final_fields[field] = measured[field]
        try:
            emit("final", **final_fields)
        except Exception as exc:
            errors.append(f"Final record could not be persisted: {exc}")
        result = evidence.analyze(records)
        # Live knowledge remains valid if disk finalization failed, but the file stays unknown.
        if not any(e["event"] == "final" for e in records):
            result.update(execution=execution, shutdown=shutdown_state, physical_accepted=False)
            result["errors"].extend(errors)
        if inspect_only:
            result.update(execution=execution, encoder="not attempted", shutdown="not attempted")
            print(json.dumps(result, indent=2))
            return result
        if shutdown_state == "unverified":
            print("CUT SERVO POWER NOW. Shutdown could not be verified.", flush=True)
        message = guard_events[-1] if guard_events else evidence.format_result(result)
        if shutdown_state == "unverified":
            message = "Cut servo power now. Shutdown could not be verified. " + message
        try:
            say(message)
        except Exception as exc:
            result["errors"].append(f"Speech failed after cleanup: {exc}")
            with suppress(Exception):
                emit("communication_error", message=str(exc))
            print("Speech failed; read the terminal result.", flush=True)
        if execution == "completed" and shutdown_state == "verified off":
            try:
                choice = (
                    input(
                        "Torque verified off. Observation: [u] upward/no binding/no oscillation, [n] no lift, [d] downward/binding, [o] oscillation, Enter unobserved: "
                    )
                    .strip()
                    .lower()
                )
                observation = {
                    "u": evidence.OBSERVATIONS[0],
                    "n": "no lift",
                    "d": "downward/binding",
                    "o": "oscillation",
                }.get(choice, "unobserved")
                emit("observation", value=observation)
                final_result = evidence.analyze(records)
                final_result["errors"].extend(e for e in result["errors"] if e not in final_result["errors"])
                if not any(e["event"] == "final" for e in records):
                    final_result.update(execution=execution, shutdown=shutdown_state, physical_accepted=False)
                result = final_result
            except (EOFError, KeyboardInterrupt):
                pass
            except Exception as exc:
                result["errors"].append(f"Observation reporting failed: {exc}")
        result["physical_accepted"] = result["physical_accepted"] and not result["errors"]
        if result["observation"] != "unobserved":
            try:
                say(evidence.format_result(result))
            except Exception as exc:
                result["errors"].append(f"Speech failed after observation: {exc}")
                result["physical_accepted"] = False
                with suppress(Exception):
                    emit("communication_error", message=str(exc))
        else:
            print(evidence.format_result(result), flush=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--leader-id", required=True)
    parser.add_argument("--follower-id", required=True)
    parser.add_argument(
        "--inspect", action="store_true", help="Capture configuration without torque or setting writes"
    )
    args = parser.parse_args()
    try:
        result = run(args)
        if isinstance(result, dict):
            code = (
                0
                if args.inspect and result["execution"] == "completed" and not result["errors"]
                else evidence.exit_code(result)
            )
            raise SystemExit(code)
        if result is False:
            raise SystemExit(2)
    except RehearsalError as exc:
        say(str(exc))
        raise SystemExit(1) from exc
    except (Exception, KeyboardInterrupt) as exc:
        print(f"STOPPED: {exc or 'Ctrl+C'}", flush=True)
        with suppress(Exception):
            say("Test stopped. Check the terminal before trying again.")
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
